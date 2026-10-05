/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _NOVATEK_NT36523N_PROTOCOL_H
#define _NOVATEK_NT36523N_PROTOCOL_H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/unaligned.h>

#define NVT_FW_SIZE 196608
#define NVT_REPORT_SIZE 108
#define NVT_CONTACTS 10
#define NVT_X_MAX 16000
#define NVT_Y_MAX 25600

struct nvt_section {
	u32 offset;
	u32 address;
	u32 length; /* encoded size in the file is length - 1 */
	u32 crc;
};

static const struct nvt_section nvt_sections[] = {
	{ 512, 0, 98112, 0x4c0ce2ae },
	{ 136192, 128000, 52072, 0xd6850337 },
	{ 0, 180072, 256, 0x417eeaa6 },
	{ 256, 180328, 256, 0xecb27187 },
};

static inline int nvt_fw_layout(const u8 *data, size_t size)
{
	unsigned int i, j;
	u32 info_offset;
	const struct nvt_section *part;

	if (size != NVT_FW_SIZE || get_unaligned_le32(data) != 512 ||
	    !(data[0x20] & 2) || !(data[0x29] & 1) || (data[0x28] & 0x10) ||
	    data[size - 4096] != 0x1d || data[size - 4095] != 0xe2 ||
	    data[size - 3] != 'N' || data[size - 2] != 'V' || data[size - 1] != 'T')
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		part = &nvt_sections[i];
		if (get_unaligned_le32(data + i * 12) != part->offset ||
		    get_unaligned_le32(data + i * 12 + 4) != part->address ||
		    get_unaligned_le32(data + i * 12 + 8) != part->length - 1 ||
		    get_unaligned_le32(data + 24 + i * 4) != part->crc)
			return -EINVAL;
	}
	for (i = 0; i <= 12; i++) {
		info_offset = 0x30 + i * 16;
		if (i < 12) {
			if (get_unaligned_le32(data + info_offset + 4))
				return -EINVAL;
			continue;
		}
		for (j = 0; j < 2; j++) {
			part = &nvt_sections[2 + j];
			info_offset = j ? 496 : info_offset;
			if (get_unaligned_le32(data + info_offset) != part->address ||
			    get_unaligned_le32(data + info_offset + 4) != part->length - 1 ||
			    get_unaligned_le32(data + info_offset + 8) != part->offset ||
			    get_unaligned_le32(data + info_offset + 12) != part->crc)
				return -EINVAL;
		}
	}
	if (get_unaligned_le32(data + 256) != 512 ||
	    get_unaligned_le32(data + 260) != 0 ||
	    get_unaligned_le32(data + 264) != 98111 ||
	    get_unaligned_le32(data + 268) != 136192 ||
	    get_unaligned_le32(data + 272) != 131072 ||
	    get_unaligned_le32(data + 276) != 52071)
		return -EINVAL;
	return 0;
}

struct nvt_contact {
	bool active;
	u16 x;
	u16 y;
};

static inline int nvt_decode(const u8 *data, struct nvt_contact *points)
{
	unsigned int i, id, status;
	u8 checksum = 0;
	u16 x, y;
	bool recovery = data[0] == 0xfe || data[0] == 0xfd;

	for (i = 1; i < 6; i++)
		recovery &= data[i] == data[0];
	if (recovery)
		return -EHOSTDOWN;
	for (i = 0; i < 65; i++)
		checksum += data[i];
	if (checksum)
		return -EBADMSG;
	for (i = 0; i < NVT_CONTACTS; i++)
		points[i].active = false;
	for (i = 0; i < NVT_CONTACTS; i++) {
		const u8 *record = data + i * 6;

		id = record[0] >> 3;
		status = record[0] & 7;
		if (id < 1 || id > NVT_CONTACTS || (status != 1 && status != 2))
			continue;
		x = get_unaligned_be16(record + 1);
		y = get_unaligned_be16(record + 3);
		if (x > NVT_X_MAX || y > NVT_Y_MAX || points[id - 1].active)
			return -EINVAL;
		points[id - 1].active = true;
		points[id - 1].x = x;
		points[id - 1].y = y;
	}
	return 0;
}
#endif
