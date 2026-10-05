// SPDX-License-Identifier: GPL-2.0-only

#include <crypto/sha2.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include "novatek-nt36523n-protocol.h"

#define NVT_EVENT 0x2fd00
#define NVT_CHUNK 1024
#define NVT_FW_NAME "novatek/tb320fc-nt36523n.bin"

struct nvt_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct mutex lock;
	struct delayed_work release_work;
	unsigned long last_valid;
	unsigned long active;
	bool stopping;
	u8 tx[NVT_CHUNK + 2];
	u8 rx[NVT_CHUNK + 2];
};

static int nvt_transfer(struct nvt_ts *ts, size_t length, bool receive)
{
	struct spi_transfer transfer = {
		.tx_buf = ts->tx,
		.rx_buf = receive ? ts->rx : NULL,
		.len = length,
		.speed_hz = 1000000,
		.bits_per_word = 8,
	};

	return spi_sync_transfer(ts->spi, &transfer, 1);
}

static int nvt_page(struct nvt_ts *ts, u32 address)
{
	ts->tx[0] = 0xff;
	ts->tx[1] = address >> 15;
	ts->tx[2] = address >> 7;
	return nvt_transfer(ts, 3, false);
}

static int nvt_read(struct nvt_ts *ts, u32 address, void *data, size_t length)
{
	int error;

	if (!length || length > NVT_CHUNK)
		return -EINVAL;
	error = nvt_page(ts, address);
	if (error)
		return error;
	memset(ts->tx, 0, length + 2);
	ts->tx[0] = address & 0x7f;
	error = nvt_transfer(ts, length + 2, true);
	if (!error)
		memcpy(data, ts->rx + 2, length);
	return error;
}

static int nvt_write(struct nvt_ts *ts, u32 address, const void *data, size_t length)
{
	int error;

	if (!length || length > NVT_CHUNK)
		return -EINVAL;
	error = nvt_page(ts, address);
	if (error)
		return error;
	ts->tx[0] = (address & 0x7f) | 0x80;
	memcpy(ts->tx + 1, data, length);
	return nvt_transfer(ts, length + 1, false);
}

static int nvt_value(struct nvt_ts *ts, u32 address, u32 value, size_t length)
{
	__le32 bytes = cpu_to_le32(value);

	return nvt_write(ts, address, &bytes, length);
}

static int nvt_trim(struct nvt_ts *ts)
{
	u8 trim[6];
	int error = nvt_read(ts, 0x3f004, trim, sizeof(trim));

	if (error)
		return error;
	return trim[0] == 0x17 && trim[3] == 0x23 && trim[4] == 0x65 &&
	       trim[5] == 0x03 ? 0 : -ENODEV;
}

static int nvt_crc_check(struct nvt_ts *ts)
{
	const u32 addresses[] = { 0x3f133, 0x3f100, 0x3f120, 0x3f104,
				  0x3f124, 0x3f114, 0x3f138 };
	u32 values[ARRAY_SIZE(addresses)];
	u8 bytes[4];
	int i, error;

	for (i = 0; i < ARRAY_SIZE(addresses); i++) {
		memset(bytes, 0, sizeof(bytes));
		error = nvt_read(ts, addresses[i], bytes, i ? 4 : 1);
		if (error)
			return error;
		values[i] = get_unaligned_le32(bytes);
	}
	values[5] &= 0xffffff;
	values[6] &= 0xffffff;
	dev_info(&ts->spi->dev, "CRC flags=%#x ILM=%#x/%#x DLM-bank=%#x/%#x BLD=%#x/%#x\n",
		 values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
	if (!(values[0] & 4) || values[1] != nvt_sections[0].crc ||
	    values[2] != nvt_sections[0].crc)
		return -EBADMSG;
	if (values[3] == nvt_sections[1].crc && values[4] == values[3])
		return 0;
	if (values[3] == nvt_sections[3].crc && values[4] == values[3] &&
	    values[5] == nvt_sections[3].address && values[6] == 251) {
		dev_info(&ts->spi->dev, "Secondary header CRC verified; initial DLM result not observed\n");
		return 0;
	}
	return -EBADMSG;
}

static int nvt_load(struct nvt_ts *ts, const struct firmware *fw)
{
	const u32 destinations[] = { 0x3f128, 0x3f12c };
	const u32 lengths[] = { 0x3f118, 0x3f130 };
	const u32 golden[] = { 0x3f100, 0x3f104 };
	const u8 dma[] = { 0x35, 0x32, 0xaa, 0x00 };
	const u8 zero[32] = {};
	const u8 command[] = { 0xae, 0x00 };
	u8 state[5], info[38], copy;
	unsigned long until;
	unsigned int i, offset, length;
	int error;

	error = nvt_trim(ts);
	if (error)
		return error;
	error = nvt_value(ts, 0x7fff80, 0x5a, 1);
	if (error)
		return error;
	msleep(11);
	error = nvt_trim(ts);
	if (error)
		return error;
	error = nvt_value(ts, 0x3f0fe, 0x69, 1);
	if (error)
		return error;
	msleep(5);
	for (i = 0; i < 2; i++) {
		error = nvt_value(ts, destinations[i], nvt_sections[i].address, 3);
		if (error)
			return error;
		error = nvt_value(ts, lengths[i], nvt_sections[i].length - 1, 3);
		if (error)
			return error;
		error = nvt_value(ts, golden[i], nvt_sections[i].crc, 4);
		if (error)
			return error;
	}
	error = nvt_write(ts, nvt_sections[1].address, zero, sizeof(zero));
	if (error)
		return error;
	error = nvt_write(ts, 0x3f7d0, dma, sizeof(dma));
	if (error)
		return error;
	error = nvt_value(ts, 0x3f7e8, 0x69, 1);
	if (error)
		return error;
	for (i = 0; i < ARRAY_SIZE(nvt_sections); i++) {
		const struct nvt_section *part = &nvt_sections[i];

		for (offset = 0; offset < part->length; offset += length) {
			length = min_t(u32, NVT_CHUNK, part->length - offset);
			error = nvt_write(ts, part->address + offset,
					  fw->data + part->offset + offset, length);
			if (error)
				return error;
		}
	}
	until = jiffies + msecs_to_jiffies(250);
	do {
		error = nvt_read(ts, 0x3f7f1, &copy, 1);
		if (error)
			return error;
		if (!copy)
			break;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, until));
	if (copy)
		return -ETIMEDOUT;
	error = nvt_write(ts, NVT_EVENT + 0x60, zero, 6);
	if (error)
		return error;
	error = nvt_write(ts, NVT_EVENT + 0x50, command, sizeof(command));
	if (error)
		return error;
	error = nvt_value(ts, 0x3f10d, 1, 1);
	if (error)
		return error;
	msleep(5);
	until = jiffies + msecs_to_jiffies(1000);
	do {
		error = nvt_read(ts, NVT_EVENT + 0x60, state, sizeof(state));
		if (error)
			return error;
		if (state[0] >= 0xa1 && state[0] <= 0xaf)
			break;
		msleep(10);
	} while (time_before(jiffies, until));
	if (state[0] < 0xa1 || state[0] > 0xaf) {
		dev_err(&ts->spi->dev, "Baseline not ready: %*ph\n", (int)sizeof(state), state);
		nvt_crc_check(ts);
		return -ETIMEDOUT;
	}
	error = nvt_crc_check(ts);
	if (error)
		return error;
	error = nvt_read(ts, NVT_EVENT + 0x78, info, sizeof(info));
	if (error)
		return error;
	if (info[0] != 0x1d || info[1] != 0xe2 ||
	    get_unaligned_be16(info + 4) != NVT_X_MAX ||
	    get_unaligned_be16(info + 6) != NVT_Y_MAX ||
	    get_unaligned_le16(info + 34) != 0x6066)
		return -ENODEV;
	dev_info(&ts->spi->dev, "Firmware v%u project=%#x ready; single SRAM load completed\n",
		 info[0], get_unaligned_le16(info + 34));
	return nvt_page(ts, NVT_EVENT);
}

static void nvt_release(struct nvt_ts *ts)
{
	unsigned int i;

	for (i = 0; i < NVT_CONTACTS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	ts->active = 0;
}

static void nvt_release_timeout(struct work_struct *work)
{
	struct nvt_ts *ts = container_of(to_delayed_work(work), struct nvt_ts, release_work);
	unsigned long deadline;

	mutex_lock(&ts->lock);
	if (!ts->stopping && ts->active) {
		deadline = ts->last_valid + msecs_to_jiffies(250);
		if (time_after_eq(jiffies, deadline))
			nvt_release(ts);
		else
			mod_delayed_work(system_wq, &ts->release_work, deadline - jiffies);
	}
	mutex_unlock(&ts->lock);
}

static irqreturn_t nvt_irq(int irq, void *context)
{
	struct nvt_ts *ts = context;
	struct nvt_contact points[NVT_CONTACTS];
	u8 frame[NVT_REPORT_SIZE];
	unsigned int i;
	int error;

	mutex_lock(&ts->lock);
	if (ts->stopping)
		goto out;
	error = nvt_read(ts, NVT_EVENT, frame, sizeof(frame));
	if (!error)
		error = nvt_decode(frame, points);
	if (error) {
		if (error == -EHOSTDOWN) {
			nvt_release(ts);
			dev_err_ratelimited(&ts->spi->dev, "Firmware recovery marker; no automatic reload\n");
		} else if (error != -EBADMSG) {
			dev_warn_ratelimited(&ts->spi->dev, "Rejected report: %d\n", error);
		}
		if (ts->active && error != -EHOSTDOWN)
			mod_delayed_work(system_wq, &ts->release_work, msecs_to_jiffies(250));
		goto out;
	}
	cancel_delayed_work(&ts->release_work);
	ts->last_valid = jiffies;
	ts->active = 0;
	for (i = 0; i < NVT_CONTACTS; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, points[i].active);
		if (points[i].active) {
			ts->active |= BIT(i);
			touchscreen_report_pos(ts->input, &ts->prop, points[i].x, points[i].y, true);
		}
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
out:
	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}

static void nvt_stop(void *context)
{
	struct nvt_ts *ts = context;

	disable_irq(ts->spi->irq);
	mutex_lock(&ts->lock);
	ts->stopping = true;
	mutex_unlock(&ts->lock);
	cancel_delayed_work_sync(&ts->release_work);
	mutex_lock(&ts->lock);
	nvt_release(ts);
	mutex_unlock(&ts->lock);
}

static int nvt_probe(struct spi_device *spi)
{
	static const u8 expected_sha[SHA256_DIGEST_SIZE] = {
		0xd0, 0x8d, 0x48, 0x44, 0x35, 0xdd, 0x43, 0x34,
		0x55, 0xb4, 0x1b, 0x8b, 0xc6, 0x59, 0x4f, 0x43,
		0x85, 0x51, 0xe9, 0xbd, 0x3d, 0x8b, 0xf8, 0xce,
		0x03, 0x7f, 0xdd, 0xe4, 0x92, 0xd4, 0xda, 0xd3,
	};
	struct device *dev = &spi->dev;
	const struct firmware *fw;
	struct gpio_desc *irq_gpio;
	struct nvt_ts *ts;
	const char *name;
	u8 digest[SHA256_DIGEST_SIZE];
	int error;

	error = device_property_read_string(dev, "firmware-name", &name);
	if (error || strcmp(name, NVT_FW_NAME))
		return dev_err_probe(dev, -EINVAL, "Expected board-specific firmware name\n");
	error = request_firmware_direct(&fw, name, dev);
	if (error)
		return dev_err_probe(dev, error, "Firmware unavailable; bind after SD root mounts\n");
	error = nvt_fw_layout(fw->data, fw->size);
	if (error)
		goto out_firmware;
	sha256(fw->data, fw->size, digest);
	if (memcmp(digest, expected_sha, sizeof(digest))) {
		error = -EBADMSG;
		goto out_firmware;
	}
	if (spi->irq <= 0 || irq_get_trigger_type(spi->irq) != IRQ_TYPE_EDGE_RISING) {
		error = -EINVAL;
		goto out_firmware;
	}
	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		error = -ENOMEM;
		goto out_firmware;
	}
	ts->spi = spi;
	mutex_init(&ts->lock);
	INIT_DELAYED_WORK(&ts->release_work, nvt_release_timeout);
	spi_set_drvdata(spi, ts);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error)
		goto out_firmware;
	irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(irq_gpio)) {
		error = PTR_ERR(irq_gpio);
		goto out_firmware;
	}
	if (gpiod_to_irq(irq_gpio) != spi->irq) {
		error = -EINVAL;
		goto out_firmware;
	}
	error = nvt_load(ts, fw);
	if (error)
		goto out_firmware;
	ts->input = devm_input_allocate_device(dev);
	if (!ts->input) {
		error = -ENOMEM;
		goto out_firmware;
	}
	ts->input->name = "Novatek NT36523N Touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, NVT_X_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, NVT_Y_MAX, 0, 0);
	input_abs_set_res(ts->input, ABS_MT_POSITION_X, 136);
	input_abs_set_res(ts->input, ABS_MT_POSITION_Y, 135);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	error = input_mt_init_slots(ts->input, NVT_CONTACTS, INPUT_MT_DIRECT);
	if (error)
		goto out_firmware;
	error = input_register_device(ts->input);
	if (error)
		goto out_firmware;
	error = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_irq,
					 IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(dev), ts);
	if (error)
		goto out_firmware;
	error = devm_add_action_or_reset(dev, nvt_stop, ts);
	if (error)
		goto out_firmware;
	enable_irq(spi->irq);
	dev_info(dev, "Native touch ready on IRQ %d; no polling bridge\n", spi->irq);
out_firmware:
	release_firmware(fw);
	return error ? dev_err_probe(dev, error, "Native touch probe stopped; no retry\n") : 0;
}

static const struct of_device_id nvt_of_match[] = {
	{ .compatible = "novatek,nt36523n" },
	{}
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static const struct spi_device_id nvt_spi_ids[] = {
	{ "nt36523n" },
	{}
};
MODULE_DEVICE_TABLE(spi, nvt_spi_ids);

static struct spi_driver nvt_driver = {
	.driver = {
		.name = "novatek-nt36523n",
		.of_match_table = nvt_of_match,
	},
	.probe = nvt_probe,
	.id_table = nvt_spi_ids,
};
module_spi_driver(nvt_driver);

MODULE_DESCRIPTION("Experimental TB320FC NT36523N SPI touchscreen");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(NVT_FW_NAME);
