// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Clockwork Tech LLC
 * Copyright (c) 2021 Max Fierke <max@maxfierke.com>
 *
 * Merged driver: supports the CM3 carrier (v1 panel only, HW-reset-only
 * pin, no panel-ID readback) and the CM4/CM5 carrier (v1-v3 panel,
 * ID-readback based auto-detection), unified under one module.
 */

#include <drm/drm_modes.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>

/*
 * Controls how cwu50_unprepare() powers the panel down on CM3.
 * 1 = HW reset only, no DCS shutdown commands sent (default, matches
 *     the original CM3 driver's tested behaviour).
 * 2 = Send DCS display-off / sleep-in first, then HW reset, falling
 *     back to case 1 if either DCS command fails.
 * Not used on the CM4/CM5 path, which always uses the DCS sequence.
 */
static int power_off_case = 1;
module_param(power_off_case, int, 0660);
MODULE_PARM_DESC(power_off_case,
	"CM3 unprepare method: 1 = HW reset only (default), 2 = DCS shutdown then HW reset");

struct cwu50 {
	struct device *dev;
	struct drm_panel panel;
	struct regulator *vci;
	struct regulator *iovcc;
	struct gpio_desc *id_gpio;		/* reset pin (CM3) / reset+ID pin (CM4/5) */
	struct backlight_device *backlight;	/* only used on the CM4/5 path */
	bool prepared;
	bool enabled;
	bool is_new_panel;			/* v2/v3 panel detected */
	bool is_reset_only;				/* true when the reset pin is a plain HW
						 * reset output with no ID-readback wiring
						 * (CM3 and CM4S carriers) */
	enum drm_panel_orientation orientation;
	int dsi_status;				/* 0: ok, 1: error - for sysfs */
	bool sysfs_node_created;
};

static ssize_t dsi_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct cwu50 *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, ctx->dsi_status ? "error\n" : "ok\n");
}

static DEVICE_ATTR(dsi_state, 0444, dsi_state_show, NULL);

static struct attribute *dsi_state_attrs[] = {
	&dev_attr_dsi_state.attr,
	NULL
};

static const struct attribute_group dsi_attr_group = {
	.attrs = dsi_state_attrs,
};

static const struct drm_display_mode default_mode = {
	.clock = 61020,
	.hdisplay = 720,
	.hsync_start = 720 + 30,
	.hsync_end = 720 + 30 + 15,
	.htotal = 720 + 30 + 15 + 15,
	.vdisplay = 1280,
	.vsync_start = 1280 + 8,
	.vsync_end = 1280 + 8 + 2,
	.vtotal = 1280 + 8 + 2 + 16,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static inline struct cwu50 *panel_to_cwu50(struct drm_panel *panel)
{
	return container_of(panel, struct cwu50, panel);
}

#define dcs_write_seq(seq...)                                   \
({                                                               \
	static const u8 d[] = { seq };                           \
	mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));        \
})

/* v1 panel init sequence (used by CM3 always, and by CM4/5 for old panels) */
static void cwu50_init_sequence(struct cwu50 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	dcs_write_seq(0xE1,0x93);
	dcs_write_seq(0xE2,0x65);
	dcs_write_seq(0xE3,0xF8);
	dcs_write_seq(0x70,0x20);
	dcs_write_seq(0x71,0x13);
	dcs_write_seq(0x72,0x06);
	dcs_write_seq(0x75,0x03);
	dcs_write_seq(0xE0,0x01);
	dcs_write_seq(0x00,0x00);
	dcs_write_seq(0x01,0x47);//VCOM0x47
	dcs_write_seq(0x03,0x00);
	dcs_write_seq(0x04,0x4D);
	dcs_write_seq(0x0C,0x64);
	dcs_write_seq(0x17,0x00);
	dcs_write_seq(0x18,0xBF);
	dcs_write_seq(0x19,0x00);
	dcs_write_seq(0x1A,0x00);
	dcs_write_seq(0x1B,0xBF);
	dcs_write_seq(0x1C,0x00);
	dcs_write_seq(0x1F,0x7E);
	dcs_write_seq(0x20,0x24);
	dcs_write_seq(0x21,0x24);
	dcs_write_seq(0x22,0x4E);
	dcs_write_seq(0x24,0xFE);
	dcs_write_seq(0x37,0x09);
	dcs_write_seq(0x38,0x04);
	dcs_write_seq(0x3C,0x76);
	dcs_write_seq(0x3D,0xFF);
	dcs_write_seq(0x3E,0xFF);
	dcs_write_seq(0x3F,0x7F);
	dcs_write_seq(0x40,0x04);//Dot inversion type
	dcs_write_seq(0x41,0xA0);
	dcs_write_seq(0x44,0x11);
	dcs_write_seq(0x55,0x02);
	dcs_write_seq(0x56,0x01);
	dcs_write_seq(0x57,0x49);
	dcs_write_seq(0x58,0x09);
	dcs_write_seq(0x59,0x2A);
	dcs_write_seq(0x5A,0x1A);
	dcs_write_seq(0x5B,0x1A);
	dcs_write_seq(0x5D,0x78);
	dcs_write_seq(0x5E,0x6E);
	dcs_write_seq(0x5F,0x66);
	dcs_write_seq(0x60,0x5E);
	dcs_write_seq(0x61,0x60);
	dcs_write_seq(0x62,0x54);
	dcs_write_seq(0x63,0x5C);
	dcs_write_seq(0x64,0x47);
	dcs_write_seq(0x65,0x5F);
	dcs_write_seq(0x66,0x5D);
	dcs_write_seq(0x67,0x5B);
	dcs_write_seq(0x68,0x76);
	dcs_write_seq(0x69,0x61);
	dcs_write_seq(0x6A,0x63);
	dcs_write_seq(0x6B,0x50);
	dcs_write_seq(0x6C,0x45);
	dcs_write_seq(0x6D,0x34);
	dcs_write_seq(0x6E,0x1C);
	dcs_write_seq(0x6F,0x07);
	dcs_write_seq(0x70,0x78);
	dcs_write_seq(0x71,0x6E);
	dcs_write_seq(0x72,0x66);
	dcs_write_seq(0x73,0x5E);
	dcs_write_seq(0x74,0x60);
	dcs_write_seq(0x75,0x54);
	dcs_write_seq(0x76,0x5C);
	dcs_write_seq(0x77,0x47);
	dcs_write_seq(0x78,0x5F);
	dcs_write_seq(0x79,0x5D);
	dcs_write_seq(0x7A,0x5B);
	dcs_write_seq(0x7B,0x76);
	dcs_write_seq(0x7C,0x61);
	dcs_write_seq(0x7D,0x63);
	dcs_write_seq(0x7E,0x50);
	dcs_write_seq(0x7F,0x45);
	dcs_write_seq(0x80,0x34);
	dcs_write_seq(0x81,0x1C);
	dcs_write_seq(0x82,0x07);
	dcs_write_seq(0xE0,0x02);
	dcs_write_seq(0x00,0x44);
	dcs_write_seq(0x01,0x46);
	dcs_write_seq(0x02,0x48);
	dcs_write_seq(0x03,0x4A);
	dcs_write_seq(0x04,0x40);
	dcs_write_seq(0x05,0x42);
	dcs_write_seq(0x06,0x1F);
	dcs_write_seq(0x07,0x1F);
	dcs_write_seq(0x08,0x1F);
	dcs_write_seq(0x09,0x1F);
	dcs_write_seq(0x0A,0x1F);
	dcs_write_seq(0x0B,0x1F);
	dcs_write_seq(0x0C,0x1F);
	dcs_write_seq(0x0D,0x1F);
	dcs_write_seq(0x0E,0x1F);
	dcs_write_seq(0x0F,0x1F);
	dcs_write_seq(0x10,0x1F);
	dcs_write_seq(0x11,0x1F);
	dcs_write_seq(0x12,0x1F);
	dcs_write_seq(0x13,0x1F);
	dcs_write_seq(0x14,0x1E);
	dcs_write_seq(0x15,0x1F);
	dcs_write_seq(0x16,0x45);
	dcs_write_seq(0x17,0x47);
	dcs_write_seq(0x18,0x49);
	dcs_write_seq(0x19,0x4B);
	dcs_write_seq(0x1A,0x41);
	dcs_write_seq(0x1B,0x43);
	dcs_write_seq(0x1C,0x1F);
	dcs_write_seq(0x1D,0x1F);
	dcs_write_seq(0x1E,0x1F);
	dcs_write_seq(0x1F,0x1F);
	dcs_write_seq(0x20,0x1F);
	dcs_write_seq(0x21,0x1F);
	dcs_write_seq(0x22,0x1F);
	dcs_write_seq(0x23,0x1F);
	dcs_write_seq(0x24,0x1F);
	dcs_write_seq(0x25,0x1F);
	dcs_write_seq(0x26,0x1F);
	dcs_write_seq(0x27,0x1F);
	dcs_write_seq(0x28,0x1F);
	dcs_write_seq(0x29,0x1F);
	dcs_write_seq(0x2A,0x1E);
	dcs_write_seq(0x2B,0x1F);
	dcs_write_seq(0x2C,0x0B);
	dcs_write_seq(0x2D,0x09);
	dcs_write_seq(0x2E,0x07);
	dcs_write_seq(0x2F,0x05);
	dcs_write_seq(0x30,0x03);
	dcs_write_seq(0x31,0x01);
	dcs_write_seq(0x32,0x1F);
	dcs_write_seq(0x33,0x1F);
	dcs_write_seq(0x34,0x1F);
	dcs_write_seq(0x35,0x1F);
	dcs_write_seq(0x36,0x1F);
	dcs_write_seq(0x37,0x1F);
	dcs_write_seq(0x38,0x1F);
	dcs_write_seq(0x39,0x1F);
	dcs_write_seq(0x3A,0x1F);
	dcs_write_seq(0x3B,0x1F);
	dcs_write_seq(0x3C,0x1F);
	dcs_write_seq(0x3D,0x1F);
	dcs_write_seq(0x3E,0x1F);
	dcs_write_seq(0x3F,0x1F);
	dcs_write_seq(0x40,0x1F);
	dcs_write_seq(0x41,0x1E);
	dcs_write_seq(0x42,0x0A);
	dcs_write_seq(0x43,0x08);
	dcs_write_seq(0x44,0x06);
	dcs_write_seq(0x45,0x04);
	dcs_write_seq(0x46,0x02);
	dcs_write_seq(0x47,0x00);
	dcs_write_seq(0x48,0x1F);
	dcs_write_seq(0x49,0x1F);
	dcs_write_seq(0x4A,0x1F);
	dcs_write_seq(0x4B,0x1F);
	dcs_write_seq(0x4C,0x1F);
	dcs_write_seq(0x4D,0x1F);
	dcs_write_seq(0x4E,0x1F);
	dcs_write_seq(0x4F,0x1F);
	dcs_write_seq(0x50,0x1F);
	dcs_write_seq(0x51,0x1F);
	dcs_write_seq(0x52,0x1F);
	dcs_write_seq(0x53,0x1F);
	dcs_write_seq(0x54,0x1F);
	dcs_write_seq(0x55,0x1F);
	dcs_write_seq(0x56,0x1F);
	dcs_write_seq(0x57,0x1E);
	dcs_write_seq(0x58,0x40);
	dcs_write_seq(0x59,0x00);
	dcs_write_seq(0x5A,0x00);
	dcs_write_seq(0x5B,0x30);
	dcs_write_seq(0x5C,0x02);
	dcs_write_seq(0x5D,0x40);
	dcs_write_seq(0x5E,0x01);
	dcs_write_seq(0x5F,0x02);
	dcs_write_seq(0x60,0x00);
	dcs_write_seq(0x61,0x01);
	dcs_write_seq(0x62,0x02);
	dcs_write_seq(0x63,0x65);
	dcs_write_seq(0x64,0x66);
	dcs_write_seq(0x65,0x00);
	dcs_write_seq(0x66,0x00);
	dcs_write_seq(0x67,0x74);
	dcs_write_seq(0x68,0x06);
	dcs_write_seq(0x69,0x65);
	dcs_write_seq(0x6A,0x66);
	dcs_write_seq(0x6B,0x10);
	dcs_write_seq(0x6C,0x00);
	dcs_write_seq(0x6D,0x04);
	dcs_write_seq(0x6E,0x04);
	dcs_write_seq(0x6F,0x88);
	dcs_write_seq(0x70,0x00);
	dcs_write_seq(0x71,0x00);
	dcs_write_seq(0x72,0x06);
	dcs_write_seq(0x73,0x7B);
	dcs_write_seq(0x74,0x00);
	dcs_write_seq(0x75,0x87);
	dcs_write_seq(0x76,0x00);
	dcs_write_seq(0x77,0x5D);
	dcs_write_seq(0x78,0x17);
	dcs_write_seq(0x79,0x1F);
	dcs_write_seq(0x7A,0x00);
	dcs_write_seq(0x7B,0x00);
	dcs_write_seq(0x7C,0x00);
	dcs_write_seq(0x7D,0x03);
	dcs_write_seq(0x7E,0x7B);
	dcs_write_seq(0xE0,0x04);
	dcs_write_seq(0x09,0x10);
	dcs_write_seq(0xE0,0x00);
	dcs_write_seq(0xE6,0x02);
	dcs_write_seq(0xE7,0x02);
}

/* v2/v3 panel init sequence (CM4/5 only) */
static int cwu50_init_sequence2(struct cwu50 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	dcs_write_seq(0xE0,0x00);

	//--- PASSWORD	----//
	dcs_write_seq(0xE1,0x93);
	dcs_write_seq(0xE2,0x65);
	dcs_write_seq(0xE3,0xF8);
	dcs_write_seq(0x80,0x03);//03:4lane 02:3lane 01:2lane

	//--- Page1  ----//
	dcs_write_seq(0xE0,0x01);

	dcs_write_seq(0x00,0x00);
	dcs_write_seq(0x01,0x62);
	dcs_write_seq(0x03,0x10);
	dcs_write_seq(0x04,0x6A);

	dcs_write_seq(0x17,0x00);
	dcs_write_seq(0x18,0xDF);
	dcs_write_seq(0x19,0x01);
	dcs_write_seq(0x1A,0x00);
	dcs_write_seq(0x1B,0xDF);
	dcs_write_seq(0x1C,0x01);

	dcs_write_seq(0x24,0xFE);

	dcs_write_seq(0x37,0x09);

	dcs_write_seq(0x38,0x04);
	dcs_write_seq(0x39,0x08);
	dcs_write_seq(0x3A,0x12);
	dcs_write_seq(0x3C,0x78);
	dcs_write_seq(0x3D,0xFF);
	dcs_write_seq(0x3E,0xFF);
	dcs_write_seq(0x3F,0xFF);

	dcs_write_seq(0x40,0x04);
	dcs_write_seq(0x41,0xA0);
	dcs_write_seq(0x42,0x7F);
	dcs_write_seq(0x43,0x10);
	dcs_write_seq(0x44,0x17);
	dcs_write_seq(0x45,0x40);

	dcs_write_seq(0x55,0x02);
	dcs_write_seq(0x57,0x69);
	dcs_write_seq(0x59,0x2A);
	dcs_write_seq(0x5A,0x1A);
	dcs_write_seq(0x5B,0x1A);

	dcs_write_seq(0x5D,0x7F);
	dcs_write_seq(0x5E,0x67);
	dcs_write_seq(0x5F,0x58);
	dcs_write_seq(0x60,0x4B);
	dcs_write_seq(0x61,0x47);
	dcs_write_seq(0x62,0x39);
	dcs_write_seq(0x63,0x3D);
	dcs_write_seq(0x64,0x25);
	dcs_write_seq(0x65,0x3D);
	dcs_write_seq(0x66,0x3C);
	dcs_write_seq(0x67,0x3C);
	dcs_write_seq(0x68,0x5B);
	dcs_write_seq(0x69,0x4A);
	dcs_write_seq(0x6A,0x50);
	dcs_write_seq(0x6B,0x42);
	dcs_write_seq(0x6C,0x3B);
	dcs_write_seq(0x6D,0x2D);
	dcs_write_seq(0x6E,0x19);
	dcs_write_seq(0x6F,0x00);
	dcs_write_seq(0x70,0x7F);
	dcs_write_seq(0x71,0x67);
	dcs_write_seq(0x72,0x58);
	dcs_write_seq(0x73,0x4B);
	dcs_write_seq(0x74,0x47);
	dcs_write_seq(0x75,0x39);
	dcs_write_seq(0x76,0x3D);
	dcs_write_seq(0x77,0x25);
	dcs_write_seq(0x78,0x3D);
	dcs_write_seq(0x79,0x3C);
	dcs_write_seq(0x7A,0x3C);
	dcs_write_seq(0x7B,0x5B);
	dcs_write_seq(0x7C,0x4A);
	dcs_write_seq(0x7D,0x50);
	dcs_write_seq(0x7E,0x42);
	dcs_write_seq(0x7F,0x3B);
	dcs_write_seq(0x80,0x2D);
	dcs_write_seq(0x81,0x19);
	dcs_write_seq(0x82,0x00);

	//Page2, for GIP
	dcs_write_seq(0xE0,0x02);

	dcs_write_seq(0x00,0x5F);
	dcs_write_seq(0x01,0x5F);
	dcs_write_seq(0x02,0x44);
	dcs_write_seq(0x03,0x46);
	dcs_write_seq(0x04,0x48);
	dcs_write_seq(0x05,0x4A);
	dcs_write_seq(0x06,0x5F);
	dcs_write_seq(0x07,0x5F);
	dcs_write_seq(0x08,0x5F);
	dcs_write_seq(0x09,0x5F);
	dcs_write_seq(0x0A,0x5F);
	dcs_write_seq(0x0B,0x5F);
	dcs_write_seq(0x0C,0x5F);
	dcs_write_seq(0x0D,0x5F);
	dcs_write_seq(0x0E,0x5F);
	dcs_write_seq(0x0F,0x5F);
	dcs_write_seq(0x10,0x5F);
	dcs_write_seq(0x11,0x5F);
	dcs_write_seq(0x12,0x5E);
	dcs_write_seq(0x13,0x5E);
	dcs_write_seq(0x14,0x40);
	dcs_write_seq(0x15,0x42);

	dcs_write_seq(0x16,0x5F);
	dcs_write_seq(0x17,0x5F);
	dcs_write_seq(0x18,0x45);
	dcs_write_seq(0x19,0x47);
	dcs_write_seq(0x1A,0x49);
	dcs_write_seq(0x1B,0x4B);
	dcs_write_seq(0x1C,0x5F);
	dcs_write_seq(0x1D,0x5F);
	dcs_write_seq(0x1E,0x5F);
	dcs_write_seq(0x1F,0x5F);
	dcs_write_seq(0x20,0x5F);
	dcs_write_seq(0x21,0x5F);
	dcs_write_seq(0x22,0x5F);
	dcs_write_seq(0x23,0x5F);
	dcs_write_seq(0x24,0x5F);
	dcs_write_seq(0x25,0x5F);
	dcs_write_seq(0x26,0x5F);
	dcs_write_seq(0x27,0x5F);
	dcs_write_seq(0x28,0x5E);
	dcs_write_seq(0x29,0x5E);
	dcs_write_seq(0x2A,0x41);
	dcs_write_seq(0x2B,0x43);

	dcs_write_seq(0x2C,0x1F);
	dcs_write_seq(0x2D,0x1E);
	dcs_write_seq(0x2E,0x0B);
	dcs_write_seq(0x2F,0x09);
	dcs_write_seq(0x30,0x07);
	dcs_write_seq(0x31,0x05);
	dcs_write_seq(0x32,0x1F);
	dcs_write_seq(0x33,0x1F);
	dcs_write_seq(0x34,0x1F);
	dcs_write_seq(0x35,0x1F);
	dcs_write_seq(0x36,0x1F);
	dcs_write_seq(0x37,0x1F);
	dcs_write_seq(0x38,0x1F);
	dcs_write_seq(0x39,0x1F);
	dcs_write_seq(0x3A,0x1F);
	dcs_write_seq(0x3B,0x1F);
	dcs_write_seq(0x3C,0x1F);
	dcs_write_seq(0x3D,0x1F);
	dcs_write_seq(0x3E,0x1E);
	dcs_write_seq(0x3F,0x1F);
	dcs_write_seq(0x40,0x03);
	dcs_write_seq(0x41,0x01);

	dcs_write_seq(0x42,0x1F);
	dcs_write_seq(0x43,0x1E);
	dcs_write_seq(0x44,0x0A);
	dcs_write_seq(0x45,0x08);
	dcs_write_seq(0x46,0x06);
	dcs_write_seq(0x47,0x04);
	dcs_write_seq(0x48,0x1F);
	dcs_write_seq(0x49,0x1F);
	dcs_write_seq(0x4A,0x1F);
	dcs_write_seq(0x4B,0x1F);
	dcs_write_seq(0x4C,0x1F);
	dcs_write_seq(0x4D,0x1F);
	dcs_write_seq(0x4E,0x1F);
	dcs_write_seq(0x4F,0x1F);
	dcs_write_seq(0x50,0x1F);
	dcs_write_seq(0x51,0x1F);
	dcs_write_seq(0x52,0x1F);
	dcs_write_seq(0x53,0x1F);
	dcs_write_seq(0x54,0x1E);
	dcs_write_seq(0x55,0x1F);
	dcs_write_seq(0x56,0x02);
	dcs_write_seq(0x57,0x00);

	dcs_write_seq(0x58,0x40);
	dcs_write_seq(0x59,0x00);
	dcs_write_seq(0x5A,0x00);
	dcs_write_seq(0x5B,0x30);
	dcs_write_seq(0x5C,0x0B);
	dcs_write_seq(0x5D,0x30);
	dcs_write_seq(0x5E,0x01);
	dcs_write_seq(0x5F,0x02);
	dcs_write_seq(0x63,0x06);
	dcs_write_seq(0x64,0x6A);
	dcs_write_seq(0x67,0x73);
	dcs_write_seq(0x68,0x0D);
	dcs_write_seq(0x69,0x06);
	dcs_write_seq(0x6A,0x6A);
	dcs_write_seq(0x6B,0x10);
	dcs_write_seq(0x6C,0x00);
	dcs_write_seq(0x6D,0x04);
	dcs_write_seq(0x6E,0x04);
	dcs_write_seq(0x6F,0x88);

	//Page4
	dcs_write_seq(0xE0,0x04);
	dcs_write_seq(0x00,0x0E);
	dcs_write_seq(0x02,0xB3);
	dcs_write_seq(0x09,0x60);
	dcs_write_seq(0x0E,0x48);

	//Page0
	dcs_write_seq(0xE0,0x00);

	dcs_write_seq(0x11);// SLPOUT
	msleep(200);

	dcs_write_seq(0x29);// DISPON
	msleep(100);

	//--- TE----//
	dcs_write_seq(0x35,0x00);

	return 0;
}

static int cwu50_disable(struct drm_panel *panel)
{
	struct cwu50 *ctx = panel_to_cwu50(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight)
		backlight_disable(ctx->backlight);

	ctx->enabled = false;

	return 0;
}

static int cwu50_unprepare(struct drm_panel *panel)
{
	struct cwu50 *ctx = panel_to_cwu50(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	if (ctx->is_reset_only) {
		/*
		 * CM3: power off using the case described in JD9365D.pdf
		 * chapter 9.5.3. Case 1 (default) is a plain HW reset with
		 * no DCS traffic, which is what's actually been validated
		 * on this carrier. Case 2 attempts the DCS shutdown first
		 * and falls back to the HW reset on any failure.
		 */
		if (power_off_case == 2) {
			ret = mipi_dsi_dcs_set_display_off(dsi);
			if (ret) {
				dev_warn(ctx->dev, "failed to send display off command (%d)\n", ret);
				goto cm3_hw_reset;
			}
			/* tDISOFF >= 50ms */
			msleep(50);

			ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
			if (ret) {
				dev_warn(ctx->dev, "failed to enter sleep mode (%d)\n", ret);
				goto cm3_hw_reset;
			}
			/* tSLPIN >= 100ms */
			msleep(100);
		}

cm3_hw_reset:
		gpiod_set_value_cansleep(ctx->id_gpio, 1); /* assert reset */
		/* tRSTOFF1 >= 120ms */
		msleep(120);

		if (ctx->vci)
			regulator_disable(ctx->vci);
		if (ctx->iovcc)
			regulator_disable(ctx->iovcc);

		ctx->prepared = false;
		return 0;
	}

	/* CM4/CM5 path (unchanged from the original multi-panel driver) */
	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to turn display off (%d)\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to enter sleep mode (%d)\n", ret);
		return ret;
	}
	msleep(120);

	if (!ctx->is_new_panel) {
		/* Assert reset on RESX */
		dev_info(ctx->dev, "asserting reset pin for old panel\n");
		gpiod_set_value_cansleep(ctx->id_gpio, 1);
		msleep(5);
	}

	ctx->prepared = false;

	return 0;
}

/*
 * CM3/CM4S power-on sequence. This mirrors the original, independently-
 * working CM3 driver step for step (order of regulator/reset/init-
 * sequence/TE all matter to the panel firmware) rather than sharing
 * control flow with the CM4/5 path, since small ordering differences
 * (e.g. requesting TE before the panel is out of reset) are enough to
 * make prepare() fail outright.
 *
 * Unlike CM4/CM5, these carriers have no external wiring on the reset
 * pin to indicate panel version, so panel version can't be read from a
 * GPIO before power-up. Instead, after reset we ask the panel directly:
 * send SLPOUT and read back register 0x04, the same check the CM4/CM5
 * path uses to confirm its own GPIO-based guess. That register read is
 * a DSI/panel-side property, not a board-wiring one, so it works the
 * same way here.
 */
static int cwu50_prepare_cm3(struct cwu50 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;
	u8 buf[4];
	u8 response;

	gpiod_set_value_cansleep(ctx->id_gpio, 1); /* ensure asserted state */

	/* IOVCC first, then VCI */
	ret = regulator_enable(ctx->iovcc);
	if (ret) {
		dev_err(ctx->dev, "failed to enable iovcc (%d)\n", ret);
		goto error_finalize;
	}

	/* tPWON >= 0ms */

	/* MIPI should change to LP-11 after turning on vci according to JD9365D.pdf */
	ret = regulator_enable(ctx->vci);
	if (ret) {
		dev_err(ctx->dev, "failed to enable vci (%d)\n", ret);
		goto disable_iovcc;
	}

	/*
	 * Wait for MIPI to initialize.
	 * tRPWIRES >= 5ms
	 * 0 <= tMIPI_ON <= tRPWIRES
	 */
	msleep(30);

	/* MIPI should be LP-11 now */

	/* tRESETL=10us */
	/* tRESETH >= 5ms */
	gpiod_set_value_cansleep(ctx->id_gpio, 0); /* deassert */
	msleep(5);

	/* Probe panel version via register read (see comment above) */
	dcs_write_seq(0x11); /* SLPOUT */
	msleep(120);
	dcs_write_seq(0xE0, 0x00);
	mipi_dsi_dcs_read(dsi, 0x04, buf, 3);
	if (buf[0] == 0x39)
		ctx->is_new_panel = true;

	if (ctx->is_new_panel)
		cwu50_init_sequence2(ctx);
	else
		cwu50_init_sequence(ctx);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to exit sleep mode (%d)\n", ret);
		goto disable_vci;
	}

	/* tSLPOUT 120ms */
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to turn display on (%d)\n", ret);
		goto disable_vci;
	}
	msleep(20);

	/*
	 * Enable tearing mode: send TE (tearing effect) at VBLANK.
	 * Sent as a raw write, and only *after* display-on, matching the
	 * proven-working CM3 sequence. JD9365D seems to need a parameter
	 * for this command.
	 */
	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){ 0x35, 0x00 }, 2);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable vblank TE (%d)\n", ret);
		goto disable_vci;
	}

	ret = mipi_dsi_dcs_get_power_mode(dsi, &response);
	if (!ret) {
		/* debug, normally the command will fail */
		dev_info(ctx->dev, "Read display power mode got: %d", response);
	}

	ctx->dsi_status = 0; /* ok */

	return 0;

disable_vci:
	regulator_disable(ctx->vci);
disable_iovcc:
	regulator_disable(ctx->iovcc);
error_finalize:
	gpiod_set_value_cansleep(ctx->id_gpio, 1);
	ctx->dsi_status = 1; /* error */
	return ret;
}

/* CM4/CM5 power-on sequence (unchanged from the original multi-panel driver) */
static int cwu50_prepare_cm45(struct cwu50 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;
	u8 buf[4];

	if (ctx->iovcc != NULL && ctx->vci != NULL) {
		dev_info(ctx->dev, "regulator iovcc and vci defined, enabling\n");

		/* IOVCC first, then VCI */
		ret = regulator_enable(ctx->iovcc);
		if (ret) {
			dev_err(ctx->dev, "failed to enable iovcc (%d)\n", ret);
			return ret;
		}

		ret = regulator_enable(ctx->vci);
		if (ret) {
			dev_err(ctx->dev, "failed to enable vci (%d)\n", ret);
			regulator_disable(ctx->iovcc);
			return ret;
		}

		/* tRPWIRES >= 5ms */
		msleep(5);
	}

	if (!ctx->is_new_panel) {
		dev_info(ctx->dev, "old panel, cycling the reset pin\n");
		/* Cycle RESX (Hardware Reset) */
		gpiod_set_value_cansleep(ctx->id_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(ctx->id_gpio, 0);
		msleep(5);
	}

	/* Enable tearing mode: send TE (tearing effect) at VBLANK */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret) {
		dev_err(ctx->dev, "failed to enable vblank TE (%d)\n", ret);
		return ret;
	}

	/* Exit sleep mode and power on */
	dcs_write_seq(0x11); // SLPOUT
	msleep(120);
	dcs_write_seq(0xE0, 0x00);
	mipi_dsi_dcs_read(dsi, 0x04, buf, 3);

	if (buf[0] == 0x39)
		ctx->is_new_panel = 1;

	if (ctx->is_new_panel)
		cwu50_init_sequence2(ctx);
	else
		cwu50_init_sequence(ctx);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to exit sleep mode (%d)\n", ret);
		return ret;
	}
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret) {
		dev_err(ctx->dev, "failed to turn display on (%d)\n", ret);
		return ret;
	}
	msleep(20);

	return 0;
}

static int cwu50_prepare(struct drm_panel *panel)
{
	struct cwu50 *ctx = panel_to_cwu50(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = ctx->is_reset_only ? cwu50_prepare_cm3(ctx) : cwu50_prepare_cm45(ctx);
	if (ret)
		return ret;

	ctx->prepared = true;

	return 0;
}

static int cwu50_enable(struct drm_panel *panel)
{
	struct cwu50 *ctx = panel_to_cwu50(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight)
		backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return 0;
}

static int cwu50_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct cwu50 *ctx = panel_to_cwu50(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "bad mode or failed to add mode\n");
		return -EINVAL;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	/* set up connector's "panel orientation" property */
	drm_connector_set_panel_orientation(connector, ctx->orientation);

	drm_mode_probed_add(connector, mode);

	return 1; /* Number of modes */
}

static const struct drm_panel_funcs cwu50_drm_funcs = {
	.disable = cwu50_disable,
	.unprepare = cwu50_unprepare,
	.prepare = cwu50_prepare,
	.enable = cwu50_enable,
	.get_modes = cwu50_get_modes,
};

static const struct of_device_id cwu50_of_match[];

static int cwu50_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct cwu50 *ctx;
	const struct of_device_id *match;
	int ret, err;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_VIDEO_BURST |
			   MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	match = of_match_device(cwu50_of_match, dev);
	ctx->is_reset_only = match && match->data;

	if (ctx->is_reset_only) {
		/*
		 * CM3: "reset-gpios" is a plain HW reset output. There is
		 * no ID-readback wiring, so we don't try to read it first.
		 */
		ctx->id_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(ctx->id_gpio)) {
			ret = PTR_ERR(ctx->id_gpio);
			return dev_err_probe(dev, ret, "Failed to request GPIO (%d)\n", ret);
		}
		ctx->is_new_panel = false;
	} else {
		/*
		 * CM4/CM5: same DT property, but wired so the carrier board
		 * pulls it to indicate the panel version, so read it first.
		 */
		ctx->id_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_IN);
		if (IS_ERR(ctx->id_gpio)) {
			ret = PTR_ERR(ctx->id_gpio);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to request GPIO (%d)\n", ret);
			return ret;
		}

		ctx->is_new_panel = gpiod_get_value_cansleep(ctx->id_gpio);
		if (ctx->is_new_panel)
			dev_info(dev, "Detected new panel type\n");
		else
			dev_info(dev, "Detected old panel type\n");

		/*
		 * Switch the ID GPIO to OUTPUT for use with resetting,
		 * only if we're using the old panel. The new panel's
		 * ID (RESX) pin is always pulled down (or: asserted)
		 * externally.
		 */
		if (!ctx->is_new_panel) {
			dev_info(dev, "Old panel type, setting ID GPIO to OUTPUT for resetting\n");
			ret = gpiod_direction_output(ctx->id_gpio, 0);
			if (ret) {
				dev_err(dev, "failed to set id_gpio to OUTPUT\n");
				return ret;
			}
		}
	}

	/*
	 * Request vci and iovcc regulators when they are defined.
	 * Even though these regulators may be always-on, we still need
	 * to ensure that the panel only becomes ready _after_ them.
	 * This is achieved by bubbling up EPROBE_DEFER from them.
	 */
	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		err = PTR_ERR(ctx->vci);
		if (err == -EPROBE_DEFER) {
			dev_info(dev, "vci regulator isn't ready, retry later\n");
			return err;
		}
		dev_err(dev, "Failed to request vci regulator: %d\n", err);
		ctx->vci = NULL;
	}

	ctx->iovcc = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(ctx->iovcc)) {
		err = PTR_ERR(ctx->iovcc);
		if (err == -EPROBE_DEFER) {
			dev_info(dev, "iovcc regulator isn't ready, retry later\n");
			return err;
		}
		dev_err(dev, "Failed to request iovcc regulator: %d\n", err);
		ctx->iovcc = NULL;
	}

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}

	ctx->panel.prepare_prev_first = true;

	drm_panel_init(&ctx->panel, dev, &cwu50_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	if (ctx->is_reset_only) {
		/*
		 * Let the DRM core drive the backlight, matching the
		 * original CM3 driver. CM3 device trees may not describe
		 * an explicit "backlight" phandle the way CM4/5 ones do.
		 */
		err = drm_panel_of_backlight(&ctx->panel);
		if (err)
			return dev_err_probe(dev, err, "Failed to get backlight\n");
	} else {
		ctx->backlight = devm_of_find_backlight(dev);
		if (IS_ERR(ctx->backlight)) {
			dev_err(ctx->dev, "devm_of_find_backlight");
			return PTR_ERR(ctx->backlight);
		}
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	err = sysfs_create_group(&dsi->dev.kobj, &dsi_attr_group);
	if (err < 0)
		dev_warn(dev, "Cannot create optional sysfs nodes: %d\n", err);
	else
		ctx->sysfs_node_created = true;

	return 0;
}

static void cwu50_remove(struct mipi_dsi_device *dsi)
{
	struct cwu50 *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->sysfs_node_created)
		sysfs_remove_group(&dsi->dev.kobj, &dsi_attr_group);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id cwu50_of_match[] = {
	{ .compatible = "cw,cwu50", .data = (void *)0 },
	{ .compatible = "clockwork,cwu50-cm3", .data = (void *)1 },
	/*
	 * CM4S shares the CM3's form factor and reset-only pin wiring
	 * (no ID readback), unlike the regular CM4/CM5 carrier, so it
	 * takes the same code path as CM3.
	 */
	{ .compatible = "clockwork,cwu50-cm4s", .data = (void *)1 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cwu50_of_match);

static struct mipi_dsi_driver cwu50_driver = {
	.probe = cwu50_probe,
	.remove = cwu50_remove,
	.driver = {
		.name = "panel-cwu50",
		.of_match_table = cwu50_of_match,
	},
};
module_mipi_dsi_driver(cwu50_driver);

MODULE_DESCRIPTION("DRM Driver for cwu50 MIPI DSI panel (CM3/CM4S/CM4/CM5, v1-v3 panels)");
MODULE_LICENSE("GPL v2");
