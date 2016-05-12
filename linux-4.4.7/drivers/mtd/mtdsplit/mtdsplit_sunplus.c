/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/byteorder/generic.h>

#include "mtdsplit.h"

#define SUNPLUS_NR_PARTS		3
#define LINUX_KERNEL_SIZE		0x300000
#define LINUX_ROOTFS_SIZE		0x300000

static int mtdsplit_parse_sunplus(struct mtd_info *master,
				struct mtd_partition **pparts,
				struct mtd_part_parser_data *data)
{
	size_t offset;
	struct mtd_partition *part, *parts;

	parts = kzalloc(SUNPLUS_NR_PARTS * sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;

	offset = 0;
	part = parts;

	part->name = KERNEL_PART_NAME;
	part->offset = 0;
	part->size = LINUX_KERNEL_SIZE;
	offset += part->size;
	part++;

	part->name = ROOTFS_PART_NAME;
	part->offset = offset;
	part->size = LINUX_ROOTFS_SIZE;
	offset += part->size;
	part++;

	part->name = ROOTFS_SPLIT_NAME;
	part->offset = offset;
	part->size = master->size - offset;

	*pparts = parts;
	return SUNPLUS_NR_PARTS;
}

static struct mtd_part_parser mtdsplit_sunplus_parser = {
	.owner = THIS_MODULE,
	.name = "sunplus-fw",
	.parse_fn = mtdsplit_parse_sunplus,
	.type = MTD_PARSER_TYPE_FIRMWARE,
};

static int __init mtdsplit_sunplus_init(void)
{
	register_mtd_parser(&mtdsplit_sunplus_parser);

	return 0;
}

subsys_initcall(mtdsplit_sunplus_init);
