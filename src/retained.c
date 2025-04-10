/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "retained.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(retainedmemdevice))
#define MEMORY_REGION DT_PARENT(DT_ALIAS(retainedmemdevice))
#else
#error "retained_mem region not defined"
#endif

struct retained_data *retained = (struct retained_data *)DT_REG_ADDR(MEMORY_REGION);

#define RETAINED_CRC_OFFSET offsetof(struct retained_data, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(retained->crc))

bool retained_validate(void)
{
	/* The residue of a CRC is what you get from the CRC over the
	 * message catenated with its CRC.  This is the post-final-xor
	 * residue for CRC-32 (CRC-32/ISO-HDLC) which Zephyr calls
	 * crc32_ieee.
	 */
	const uint32_t residue = 0x2144df1c;
	uint32_t crc = crc32_ieee((const uint8_t *)retained,
				  RETAINED_CHECKED_SIZE);
	bool valid = (crc == residue);

	/* If the CRC isn't valid, reset the retained data. */
	if (!valid) {
		memset(retained, 0, sizeof(struct retained_data));
	
		retained->gyroSensScale[0] = 1.0f;
		retained->gyroSensScale[1] = 1.0f;
		retained->gyroSensScale[2] = 1.0f;
	}

	/* Reset to accrue runtime from this session. */
	retained->uptime_latest = 0;

	/* Reset to accrue runtime from this session. */
	retained->uptime_latest = 0;

	return valid;
}

void retained_update(void)
{
	uint64_t now = k_uptime_ticks();

	retained->uptime_sum += (now - retained->uptime_latest);
	retained->uptime_latest = now;

	uint32_t crc = crc32_ieee((const uint8_t *)retained,
				  RETAINED_CRC_OFFSET);

	retained->crc = sys_cpu_to_le32(crc);
}
