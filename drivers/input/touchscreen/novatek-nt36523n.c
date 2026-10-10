// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Novatek NT36523N SPI touchscreen controller.
 *
 * The touch MCU shares power/reset with the display and boots from volatile
 * SRAM. Follow the panel so its firmware is loaded after every power cycle.
 * Register and packet formats are based on Novatek's GPL NT36xxx driver.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#define NVT_EVENT		0x2fd00
#define NVT_HOST_CMD		(NVT_EVENT + 0x50)
#define NVT_RESET_COMPLETE	(NVT_EVENT + 0x60)
#define NVT_FW_INFO		(NVT_EVENT + 0x78)
#define NVT_TRIM			0x3f004
#define NVT_SW_RESET		0x3f0fe
#define NVT_BOOT_READY		0x3f10d
#define NVT_ACI_CLEAR		0x3f705
#define NVT_AUTO_COPY		0x3f7e8
#define NVT_DMA_STATUS		0x3f7f1
#define NVT_DMA_CONTROL		0x3f7d0
#define NVT_SRAM_SIZE		0x40000
#define NVT_TRANSFER_SIZE	1024
#define NVT_FIRMWARE_BURST_SIZE	(63 * 1024)
#define NVT_STARTUP_DELAY_MS	7000
#define NVT_REPORT_SIZE		108
#define NVT_CHECKSUM_SIZE	65
#define NVT_MAX_CONTACTS		10
#define NVT_COORD_SCALE		10
#define NVT_MAX_PARTITIONS	64
#define NVT_FW_SECTOR_SIZE	4096

static bool pen_enabled;
module_param(pen_enabled, bool, 0644);
MODULE_PARM_DESC(pen_enabled, "Enable firmware pen mode instead of stock initial touch mode");

struct nvt_partition {
	u32 offset;
	u32 address;
	u32 size;
	u32 crc;
};

struct nvt_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct input_dev *pen;
	struct touchscreen_properties prop;
	struct drm_panel_follower follower;
	const struct firmware *firmware;
	struct nvt_partition *partitions;
	unsigned int num_partitions;
	struct mutex state_lock;
	struct mutex io_lock;
	struct delayed_work release_work;
	struct delayed_work startup_work;
	unsigned long active_slots;
	bool invalid_report;
	bool prepared;
	bool enabled;
	bool suspended;
	bool running;
	bool faulted;
	bool cascade;
	bool dma_setup;
	u8 recovery_frames;
	u8 recovery_attempts;
	u8 io_failures;
	u8 *tx;
	u8 *rx;
	size_t tx_size;
};

static int nvt_setup_transfer_buffers(struct nvt_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	size_t size;

	size = max_t(size_t, NVT_FIRMWARE_BURST_SIZE + 1, NVT_TRANSFER_SIZE + 2);
	if (spi_max_transfer_size(ts->spi) < size ||
	    spi_max_message_size(ts->spi) < size)
		return dev_err_probe(dev, -EINVAL, "SPI cannot preserve the firmware burst\n");

	ts->tx = devm_kmalloc(dev, size, GFP_KERNEL);
	if (!ts->tx)
		return -ENOMEM;
	ts->rx = devm_kmalloc(dev, NVT_TRANSFER_SIZE + 2, GFP_KERNEL);
	if (!ts->rx)
		return -ENOMEM;
	ts->tx_size = size;

	return 0;
}

static int nvt_transfer(struct nvt_ts *ts, size_t length, bool receive)
{
	struct spi_transfer transfer = {
		.tx_buf = ts->tx,
		.rx_buf = receive ? ts->rx : NULL,
		.len = length,
	};

	return spi_sync_transfer(ts->spi, &transfer, 1);
}

static int nvt_set_page(struct nvt_ts *ts, u32 address)
{
	ts->tx[0] = 0xff;
	ts->tx[1] = address >> 15;
	ts->tx[2] = address >> 7;
	return nvt_transfer(ts, 3, false);
}

static int nvt_read(struct nvt_ts *ts, u32 address, void *data, size_t length)
{
	int error;

	if (!length || length > NVT_TRANSFER_SIZE)
		return -EINVAL;

	error = nvt_set_page(ts, address);
	if (error)
		return error;

	memset(ts->tx, 0, length + 2);
	ts->tx[0] = address & 0x7f;
	error = nvt_transfer(ts, length + 2, true);
	if (!error)
		memcpy(data, ts->rx + 2, length);

	return error;
}

static int nvt_write(struct nvt_ts *ts, u32 address, const void *data,
		     size_t length)
{
	int error;

	if (!length || length > NVT_FIRMWARE_BURST_SIZE || length + 1 > ts->tx_size)
		return -EINVAL;

	error = nvt_set_page(ts, address);
	if (error)
		return error;

	ts->tx[0] = (address & 0x7f) | 0x80;
	memcpy(ts->tx + 1, data, length);
	return nvt_transfer(ts, length + 1, false);
}

static int nvt_write_value(struct nvt_ts *ts, u32 address, u32 value,
			   size_t length)
{
	__le32 bytes = cpu_to_le32(value);

	return nvt_write(ts, address, &bytes, length);
}

static int nvt_parse_partition(struct nvt_ts *ts, struct nvt_partition *part,
			       size_t offset, bool executable)
{
	const struct firmware *fw = ts->firmware;
	const u8 *header;
	size_t length;

	if (offset > fw->size || fw->size - offset < 16)
		return -EINVAL;

	header = fw->data + offset;
	if (executable) {
		part->offset = get_unaligned_le32(header);
		part->address = get_unaligned_le32(header + 4);
		part->size = get_unaligned_le32(header + 8);
		part->crc = get_unaligned_le32(fw->data + 0x18 + (offset / 12) * 4);
	} else {
		part->address = get_unaligned_le32(header);
		part->size = get_unaligned_le32(header + 4);
		part->offset = get_unaligned_le32(header + 8);
		part->crc = get_unaligned_le32(header + 12);
	}

	if (!part->size)
		return executable ? -EINVAL : 0;

	if (part->size >= NVT_SRAM_SIZE || part->address >= NVT_SRAM_SIZE)
		return -EINVAL;
	length = part->size + 1;
	if (part->address > NVT_SRAM_SIZE - length ||
	    part->offset > fw->size || length > fw->size - part->offset)
		return -EINVAL;

	return 0;
}

static int nvt_parse_firmware(struct nvt_ts *ts)
{
	const struct firmware *fw = ts->firmware;
	unsigned int header_size, info_end, overlays, num, i;
	size_t used_size;
	int error;

	if (fw->size < NVT_FW_SECTOR_SIZE)
		return -EINVAL;

	used_size = round_down(fw->size, NVT_FW_SECTOR_SIZE);
	while (used_size) {
		const u8 *marker = fw->data + used_size - 3;

		if (!memcmp(marker, "NVT", 3) || !memcmp(marker, "MOD", 3))
			break;
		used_size -= NVT_FW_SECTOR_SIZE;
	}
	if (!used_size ||
	    fw->data[used_size - NVT_FW_SECTOR_SIZE] +
	    fw->data[used_size - NVT_FW_SECTOR_SIZE + 1] != 0xff)
		return -EINVAL;

	header_size = get_unaligned_le32(fw->data);
	ts->cascade = fw->data[0x20] & BIT(1);
	ts->dma_setup = ts->cascade && (fw->data[0x29] & BIT(0));
	info_end = ts->cascade ? header_size / 2 : header_size;
	if (header_size > used_size || info_end < 0x30 ||
	    (info_end - 0x30) % 16 || (ts->cascade && header_size % 32))
		return -EINVAL;

	overlays = fw->data[0x28] & BIT(4) ? fw->data[0x28] & 0xf : 0;
	num = 2 + (info_end - 0x30) / 16 + ts->cascade + overlays;
	if (num > NVT_MAX_PARTITIONS)
		return -EINVAL;

	ts->partitions = devm_kcalloc(&ts->spi->dev, num,
				      sizeof(*ts->partitions), GFP_KERNEL);
	if (!ts->partitions)
		return -ENOMEM;
	ts->num_partitions = num;

	for (i = 0; i < num; i++) {
		size_t offset;
		unsigned int overlay_start = num - overlays;

		if (i < 2)
			offset = i * 12;
		else if (i >= overlay_start)
			offset = ts->partitions[1].offset +
				 (size_t)(i - overlay_start) * 16;
		else if (ts->cascade && i == overlay_start - 1)
			offset = header_size - 16;
		else
			offset = 0x30 + (i - 2) * 16;

		error = nvt_parse_partition(ts, &ts->partitions[i], offset, i < 2);
		if (error)
			return error;
	}

	return 0;
}

static int nvt_bootloader_reset(struct nvt_ts *ts)
{
	int error;

	error = nvt_write_value(ts, NVT_SW_RESET, 0x69, 1);
	if (error)
		return error;
	usleep_range(5000, 6000);
	return 0;
}

static int nvt_identify(struct nvt_ts *ts)
{
	u8 trim[6];
	int error;

	error = nvt_read(ts, NVT_TRIM, trim, sizeof(trim));
	if (error)
		return error;
	if (trim[0] != 0x17 || trim[3] != 0x23 ||
	    trim[4] != 0x65 || trim[5] != 0x03) {
		dev_err(&ts->spi->dev, "Unexpected controller ID: %*ph\n",
			(int)sizeof(trim), trim);
		return -ENODEV;
	}
	return 0;
}

static int nvt_configure_pen(struct nvt_ts *ts)
{
	u8 command[] = { 0x7b, pen_enabled };
	u8 status;
	int i, error;

	for (i = 0; i < 5; i++) {
		error = nvt_write(ts, NVT_HOST_CMD, command, sizeof(command));
		if (error)
			return error;
		msleep(20);
		error = nvt_read(ts, NVT_HOST_CMD, &status, 1);
		if (error || !status)
			return error;
	}
	return -ETIMEDOUT;
}

static int nvt_download_firmware(struct nvt_ts *ts)
{
	static const u32 destinations[] = { 0x3f128, 0x3f12c };
	static const u32 lengths[] = { 0x3f118, 0x3f130 };
	static const u32 golden[] = { 0x3f100, 0x3f104 };
	static const u8 dma[] = { 0x35, 0x32, 0xaa, 0x00 };
	static const u8 zero[32];
	static const u8 crc_command[] = { 0xae, 0x00 };
	u8 state, info[38], copy;
	unsigned int i, offset, length;
	int error;

	error = nvt_bootloader_reset(ts);
	if (error)
		return error;
	error = nvt_identify(ts);
	if (error)
		return error;

	for (i = 0; i < 2; i++) {
		struct nvt_partition *part = &ts->partitions[i];

		error = nvt_write_value(ts, destinations[i], part->address, 3);
		if (error)
			return error;
		error = nvt_write_value(ts, lengths[i], part->size, 3);
		if (error)
			return error;
		error = nvt_write_value(ts, golden[i], part->crc, 4);
		if (error)
			return error;
	}

	if (ts->dma_setup) {
		error = nvt_write(ts, ts->partitions[1].address, zero, sizeof(zero));
		if (error)
			return error;
		error = nvt_write(ts, NVT_DMA_CONTROL, dma, sizeof(dma));
		if (error)
			return error;
	}
	if (ts->cascade) {
		error = nvt_write_value(ts, NVT_AUTO_COPY, 0x69, 1);
		if (error)
			return error;
	}

	for (i = 0; i < ts->num_partitions; i++) {
		struct nvt_partition *part = &ts->partitions[i];

		if (!part->size)
			continue;
		for (offset = 0; offset <= part->size; offset += length) {
			length = min_t(u32, NVT_FIRMWARE_BURST_SIZE, part->size + 1 - offset);
			error = nvt_write(ts, part->address + offset,
					  ts->firmware->data + part->offset + offset, length);
			if (error)
				return error;
		}
	}

	if (ts->cascade) {
		for (i = 0; i < 200; i++) {
			error = nvt_read(ts, NVT_DMA_STATUS, &copy, 1);
			if (error)
				return error;
			if (!copy)
				break;
			usleep_range(1000, 2000);
		}
		if (i == 200)
			return -ETIMEDOUT;
	}

	error = nvt_write(ts, NVT_RESET_COMPLETE, zero, 6);
	if (error)
		return error;
	error = nvt_write(ts, NVT_HOST_CMD, crc_command, sizeof(crc_command));
	if (error)
		return error;
	error = nvt_write_value(ts, NVT_BOOT_READY, 1, 1);
	if (error)
		return error;

	for (i = 0; i < 100; i++) {
		usleep_range(10000, 11000);
		error = nvt_read(ts, NVT_RESET_COMPLETE, &state, 1);
		if (error)
			return error;
		if (state >= 0xa1 && state <= 0xaf)
			break;
	}
	if (i == 100)
		return -ETIMEDOUT;

	error = nvt_read(ts, NVT_FW_INFO, info, sizeof(info));
	if (error)
		return error;
	if (info[0] + info[1] != 0xff)
		return -EBADMSG;

	return ts->pen ? nvt_configure_pen(ts) : 0;
}

static int nvt_load_firmware(struct nvt_ts *ts)
{
	int attempt, error;

	for (attempt = 0; attempt < 3; attempt++) {
		error = nvt_download_firmware(ts);
		if (!error || error == -ENODEV)
			break;
	}
	return error;
}

static void nvt_release_pen(struct nvt_ts *ts)
{
	if (!ts->pen)
		return;
	input_report_abs(ts->pen, ABS_PRESSURE, 0);
	input_report_key(ts->pen, BTN_TOUCH, 0);
	input_report_key(ts->pen, BTN_TOOL_PEN, 0);
	input_report_key(ts->pen, BTN_STYLUS, 0);
	input_report_key(ts->pen, BTN_STYLUS2, 0);
	input_sync(ts->pen);
}

static void nvt_release_contacts(struct nvt_ts *ts)
{
	unsigned int i;

	for (i = 0; i < NVT_MAX_CONTACTS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	nvt_release_pen(ts);
	ts->active_slots = 0;
	ts->invalid_report = false;
}

static void nvt_release_invalid_report(struct work_struct *work)
{
	struct nvt_ts *ts = container_of(to_delayed_work(work), struct nvt_ts,
				       release_work);

	mutex_lock(&ts->io_lock);
	if (ts->running && ts->invalid_report)
		nvt_release_contacts(ts);
	mutex_unlock(&ts->io_lock);
}

static void nvt_report_pen(struct nvt_ts *ts, const u8 *frame)
{
	unsigned int x, y, pressure, distance;

	if (!ts->pen)
		return;
	if (frame[65] == 0xff) {
		nvt_release_pen(ts);
		return;
	}
	if (frame[65] != 1)
		return; /* pen's serial number. */

	x = get_unaligned_be16(frame + 66);
	y = get_unaligned_be16(frame + 68);
	if (x > (ts->prop.max_x + 1) * NVT_COORD_SCALE ||
	    y > (ts->prop.max_y + 1) * NVT_COORD_SCALE)
		return;
	x = min(x / NVT_COORD_SCALE, ts->prop.max_x);
	y = min(y / NVT_COORD_SCALE, ts->prop.max_y);
	pressure = min_t(unsigned int, get_unaligned_be16(frame + 70), 4095);
	distance = get_unaligned_be16(frame + 74);
	touchscreen_report_pos(ts->pen, &ts->prop, x, y, false);
	input_report_abs(ts->pen, ABS_PRESSURE, pressure);
	input_report_abs(ts->pen, ABS_TILT_X, clamp_t(s8, frame[72], -60, 60));
	input_report_abs(ts->pen, ABS_TILT_Y, clamp_t(s8, frame[73], -60, 60));
	input_report_abs(ts->pen, ABS_DISTANCE, min(distance, 1U));
	input_report_key(ts->pen, BTN_TOUCH, !!pressure);
	input_report_key(ts->pen, BTN_TOOL_PEN, !!pressure || !!distance);
	input_report_key(ts->pen, BTN_STYLUS, frame[76] & BIT(0));
	input_report_key(ts->pen, BTN_STYLUS2, frame[76] & BIT(1));
	input_sync(ts->pen);
}

static int nvt_report_contacts(struct nvt_ts *ts, const u8 *frame)
{
	unsigned long seen = 0;
	unsigned int i, id, x, y;
	u8 checksum = 0;

	for (i = 0; i < NVT_CHECKSUM_SIZE; i++)
		checksum += frame[i];
	if (checksum)
		return -EBADMSG;

	for (i = 0; i < NVT_MAX_CONTACTS; i++) {
		const u8 *point = frame + i * 6;
		unsigned int state = point[0] & 7;

		if (state != 1 && state != 2)
			continue;
		id = point[0] >> 3;
		if (!id || id > NVT_MAX_CONTACTS || test_and_set_bit(id - 1, &seen))
			return -EINVAL;
		x = get_unaligned_be16(point + 1);
		y = get_unaligned_be16(point + 3);
		if (x > (ts->prop.max_x + 1) * NVT_COORD_SCALE ||
		    y > (ts->prop.max_y + 1) * NVT_COORD_SCALE)
			return -ERANGE;
	}

	for (i = 0; i < NVT_MAX_CONTACTS; i++) {
		const u8 *point = frame + i * 6;
		unsigned int state = point[0] & 7;

		if (state != 1 && state != 2)
			continue;
		id = (point[0] >> 3) - 1;
		x = min_t(unsigned int, get_unaligned_be16(point + 1) / NVT_COORD_SCALE,
			  ts->prop.max_x);
		y = min_t(unsigned int, get_unaligned_be16(point + 3) / NVT_COORD_SCALE,
			  ts->prop.max_y);
		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, max_t(u8, point[5], 1));
		input_report_abs(ts->input, ABS_MT_PRESSURE, max_t(u8, frame[98 + i], 1));
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	nvt_report_pen(ts, frame);
	ts->active_slots = seen;
	return 0;
}

static int nvt_recover(struct nvt_ts *ts, u8 marker)
{
	int error;

	if (marker == 0xfe) {
		error = nvt_write_value(ts, NVT_SW_RESET, 0xaa, 1);
		if (error)
			return error;
		usleep_range(15000, 16000);
		error = nvt_write_value(ts, NVT_ACI_CLEAR, 0xa5, 1);
		if (error)
			return error;
		error = nvt_write_value(ts, NVT_ACI_CLEAR, 0, 1);
		if (error)
			return error;
	}
	return nvt_download_firmware(ts);
}

static void nvt_fault(struct nvt_ts *ts, int irq)
{
	nvt_release_contacts(ts);
	ts->faulted = true;
	disable_irq_nosync(irq);
	dev_err(&ts->spi->dev, "Touch stopped; retry on next panel prepare\n");
}

static irqreturn_t nvt_irq(int irq, void *context)
{
	struct nvt_ts *ts = context;
	u8 frame[NVT_REPORT_SIZE];
	unsigned int i;
	bool watchdog = true, handshake = true;
	int error;

	mutex_lock(&ts->io_lock);
	if (!ts->running)
		goto out;

	error = nvt_read(ts, NVT_EVENT, frame, sizeof(frame));
	if (error) {
		dev_err_ratelimited(&ts->spi->dev, "Report read failed: %d\n", error);
		nvt_release_contacts(ts);
		if (++ts->io_failures > 10)
			nvt_fault(ts, irq);
		goto out;
	}
	ts->io_failures = 0;

	for (i = 0; i < 6; i++) {
		watchdog &= frame[i] == 0xfd || frame[i] == 0xfe;
		handshake &= frame[i] == 0x77;
	}
	if (watchdog) {
		if (++ts->recovery_frames <= 10)
			goto out;
		ts->recovery_frames = 0;
		nvt_release_contacts(ts);
		if (ts->recovery_attempts++ < 3) {
			dev_warn(&ts->spi->dev, "Controller watchdog; reloading firmware\n");
			error = nvt_recover(ts, frame[0]);
			if (!error)
				goto out;
			dev_err(&ts->spi->dev, "Firmware recovery failed: %d\n", error);
		}
		nvt_fault(ts, irq);
		goto out;
	}
	ts->recovery_frames = 0;
	if (handshake || ((frame[0] >> 3) == 30 && frame[1] == 5))
		goto out;

	error = nvt_report_contacts(ts, frame);
	if (!error) {
		ts->invalid_report = false;
		cancel_delayed_work(&ts->release_work);
	} else {
		if (ts->active_slots) {
			ts->invalid_report = true;
			schedule_delayed_work(&ts->release_work, msecs_to_jiffies(250));
		}
		if (error != -EBADMSG)
			dev_warn_ratelimited(&ts->spi->dev, "Invalid touch report: %d\n", error);
	}
out:
	mutex_unlock(&ts->io_lock);
	return IRQ_HANDLED;
}

static int nvt_start(struct nvt_ts *ts)
{
	int error;

	if (ts->running || ts->suspended)
		return 0;

	mutex_lock(&ts->io_lock);
	error = nvt_load_firmware(ts);
	if (!error) {
		ts->running = true;
		ts->faulted = false;
		ts->recovery_frames = 0;
		ts->recovery_attempts = 0;
		ts->io_failures = 0;
	}
	mutex_unlock(&ts->io_lock);
	if (error)
		return dev_err_probe(&ts->spi->dev, error, "Firmware startup failed\n");

	enable_irq(ts->spi->irq);
	return 0;
}

static void nvt_startup_work(struct work_struct *work)
{
	struct nvt_ts *ts = container_of(to_delayed_work(work), struct nvt_ts,
				       startup_work);

	mutex_lock(&ts->state_lock);
	if (ts->prepared && ts->enabled && !ts->suspended)
		nvt_start(ts);
	mutex_unlock(&ts->state_lock);
}

static void nvt_queue_startup(struct nvt_ts *ts)
{
	if (ts->prepared && ts->enabled && !ts->suspended && !ts->running)
		mod_delayed_work(system_wq, &ts->startup_work,
				 msecs_to_jiffies(NVT_STARTUP_DELAY_MS));
}

static void nvt_stop(struct nvt_ts *ts)
{
	int error;

	if (!ts->running)
		return;

	disable_irq(ts->spi->irq);
	cancel_delayed_work_sync(&ts->release_work);
	mutex_lock(&ts->io_lock);
	if (ts->faulted)
		enable_irq(ts->spi->irq);
	ts->running = false;
	ts->faulted = false;
	nvt_release_contacts(ts);
	error = nvt_write_value(ts, NVT_HOST_CMD, 0x11, 1);
	if (error)
		dev_warn(&ts->spi->dev, "Deep sleep command failed: %d\n", error);
	mutex_unlock(&ts->io_lock);
}

static int nvt_panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	mutex_lock(&ts->state_lock);
	ts->prepared = true;
	mutex_unlock(&ts->state_lock);
	return 0;
}

static int nvt_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	mutex_lock(&ts->state_lock);
	ts->prepared = false;
	ts->enabled = false;
	mutex_unlock(&ts->state_lock);
	cancel_delayed_work_sync(&ts->startup_work);
	mutex_lock(&ts->state_lock);
	nvt_stop(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static int nvt_panel_enabled(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	mutex_lock(&ts->state_lock);
	ts->enabled = true;
	nvt_queue_startup(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static int nvt_panel_disabling(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, follower);

	mutex_lock(&ts->state_lock);
	ts->enabled = false;
	mutex_unlock(&ts->state_lock);
	cancel_delayed_work_sync(&ts->startup_work);
	mutex_lock(&ts->state_lock);
	nvt_stop(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static const struct drm_panel_follower_funcs nvt_panel_funcs = {
	.panel_prepared = nvt_panel_prepared,
	.panel_unpreparing = nvt_panel_unpreparing,
	.panel_enabled = nvt_panel_enabled,
	.panel_disabling = nvt_panel_disabling,
};

static int nvt_suspend(struct device *dev)
{
	struct nvt_ts *ts = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&ts->state_lock);
	ts->suspended = true;
	mutex_unlock(&ts->state_lock);
	cancel_delayed_work_sync(&ts->startup_work);
	mutex_lock(&ts->state_lock);
	nvt_stop(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_ts *ts = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&ts->state_lock);
	ts->suspended = false;
	nvt_queue_startup(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_pm_ops, nvt_suspend, nvt_resume);

static void nvt_shutdown(struct spi_device *spi)
{
	nvt_suspend(&spi->dev);
}

static void nvt_cleanup(void *context)
{
	struct nvt_ts *ts = context;

	mutex_lock(&ts->state_lock);
	ts->prepared = false;
	ts->enabled = false;
	mutex_unlock(&ts->state_lock);
	cancel_delayed_work_sync(&ts->startup_work);
	mutex_lock(&ts->state_lock);
	nvt_stop(ts);
	mutex_unlock(&ts->state_lock);
}

static void nvt_release_firmware(void *context)
{
	release_firmware(context);
}

static int nvt_setup_input(struct nvt_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	int error;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	ts->input = input;
	input->name = "Novatek NT36523N Touchscreen";
	input->id.bustype = BUS_SPI;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 1599, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 2559, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);
	if (ts->prop.max_x >= 65535 / NVT_COORD_SCALE ||
	    ts->prop.max_y >= 65535 / NVT_COORD_SCALE)
		return -EINVAL;
	error = input_mt_init_slots(input, NVT_MAX_CONTACTS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;
	error = input_register_device(input);
	if (error)
		return error;

	if (!device_property_read_bool(dev, "novatek,pen-support"))
		return 0;
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	ts->pen = input;
	input->name = "Novatek NT36523N Pen";
	input->id.bustype = BUS_SPI;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(input, EV_KEY, BTN_STYLUS);
	input_set_capability(input, EV_KEY, BTN_STYLUS2);
	input_set_abs_params(input, ABS_X, 0,
			     input_abs_get_max(ts->input, ABS_MT_POSITION_X), 0, 0);
	input_set_abs_params(input, ABS_Y, 0,
			     input_abs_get_max(ts->input, ABS_MT_POSITION_Y), 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 4095, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_TILT_X, -60, 60, 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, -60, 60, 0, 0);
	return input_register_device(input);
}

static int nvt_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct nvt_ts *ts;
	const char *firmware_name;
	int error;

	if (spi->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Interrupt is required\n");
	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX)
		return dev_err_probe(dev, -EINVAL, "Full duplex SPI is required\n");
	if (!device_property_present(dev, "panel"))
		return dev_err_probe(dev, -EINVAL, "Panel reference is required\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	mutex_init(&ts->state_lock);
	mutex_init(&ts->io_lock);
	INIT_DELAYED_WORK(&ts->release_work, nvt_release_invalid_report);
	INIT_DELAYED_WORK(&ts->startup_work, nvt_startup_work);
	spi_set_drvdata(spi, ts);
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	error = spi_setup(spi);
	if (error)
		return error;
	error = nvt_setup_transfer_buffers(ts);
	if (error)
		return error;

	error = device_property_read_string(dev, "firmware-name", &firmware_name);
	if (error)
		return dev_err_probe(dev, error, "Firmware name is required\n");
	error = request_firmware(&ts->firmware, firmware_name, dev);
	if (error)
		return dev_err_probe(dev, error, "Cannot load touch firmware\n");
	error = devm_add_action_or_reset(dev, nvt_release_firmware,
					 (void *)ts->firmware);
	if (error)
		return error;
	error = nvt_parse_firmware(ts);
	if (error)
		return dev_err_probe(dev, error, "Invalid firmware layout\n");
	error = nvt_setup_input(ts);
	if (error)
		return error;
	error = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  dev_name(dev), ts);
	if (error)
		return error;
	error = devm_add_action_or_reset(dev, nvt_cleanup, ts);
	if (error)
		return error;

	ts->follower.funcs = &nvt_panel_funcs;
	return devm_drm_panel_add_follower(dev, &ts->follower);
}

static const struct of_device_id nvt_of_match[] = {
	{ .compatible = "novatek,nt36523n" },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static const struct spi_device_id nvt_spi_ids[] = {
	{ "nt36523n" },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_spi_ids);

static struct spi_driver nvt_driver = {
	.driver = {
		.name = "novatek-nt36523n",
		.of_match_table = nvt_of_match,
		.pm = pm_sleep_ptr(&nvt_pm_ops),
	},
	.probe = nvt_probe,
	.shutdown = nvt_shutdown,
	.id_table = nvt_spi_ids,
};
module_spi_driver(nvt_driver);

MODULE_DESCRIPTION("Novatek NT36523N SPI touchscreen driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("novatek/novatek_ts_fw.bin");
MODULE_FIRMWARE("novatek/novatek_ts_ultra_fw.bin");
