// SPDX-License-Identifier: GPL-2.0

#include <drm/drm_modes.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <video/mipi_display.h>

struct cwd686 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *supply;
	struct backlight_device *backlight;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 54465,
	.hdisplay = 480,
	.hsync_start = 480 + 150,
	.hsync_end = 480 + 150 + 24,
	.htotal = 480 + 150 + 24 + 40,
	.vdisplay = 1280,
	.vsync_start = 1280 + 12,
	.vsync_end = 1280 + 12+ 6,
	.vtotal = 1280 + 12 + 6 + 10,
	.flags = 0,
};

static inline struct cwd686 *panel_to_cwd686(struct drm_panel *panel)
{
	return container_of(panel, struct cwd686, panel);
}

#define dcs_write_seq(seq...)                              \
({                                                              \
	static const u8 d[] = { seq };                          \
	mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	 \
})

static void cwd686_init_sequence(struct cwd686 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	dcs_write_seq(0xF0,0x5A,0x5A);
	dcs_write_seq(0xF1,0xA5,0xA5);
	dcs_write_seq(0xB6,0x0D,0x0D);
	dcs_write_seq(0xB4,0x0A,0x08,0x12,0x10,0x0E,0x0C,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x04,0x06);
	dcs_write_seq(0xB3,0x0B,0x09,0x13,0x11,0x0F,0x0D,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x05,0x07);
	dcs_write_seq(0xB0,0x54,0x32,0x23,0x45,0x44,0x44,0x44,0x44,0x90,0x01,0x90,0x01);
	dcs_write_seq(0xB1,0x32,0x84,0x02,0x83,0x30,0x01,0x6B,0x01);
	dcs_write_seq(0xB2,0x73);
	dcs_write_seq(0xBD,0x4E,0x0E,0x50,0x50,0x26,0x1D,0x00,0x14,0x42,0x03);
	dcs_write_seq(0xB7,0x01,0x01,0x09,0x11,0x0D,0x55,0x19,0x19,0x21,0x1D,0x00,0x00,0x00,0x00,0x02,0xFF,0x3C);
	dcs_write_seq(0xB8,0x23,0x01,0x30,0x34,0x63);
	dcs_write_seq(0xB9,0xA0,0x22,0x00,0x44);
	dcs_write_seq(0xBA,0x12,0x63);
	dcs_write_seq(0xC1,0x0C,0x16,0x04,0x0C,0x10,0x04);
	dcs_write_seq(0xC2,0x11,0x41);
	dcs_write_seq(0xC3,0x22,0x31,0x04);
	dcs_write_seq(0xC7,0x05,0x23,0x6B,0x49,0x00);
	dcs_write_seq(0xC5,0x00);
	dcs_write_seq(0xD0,0x37,0xFF,0xFF);
	dcs_write_seq(0xD2,0x63,0x0B,0x08,0x88);
	dcs_write_seq(0xD3,0x01,0x00,0x00,0x01,0x01,0x37,0x25,0x38,0x31,0x06,0x07);
	dcs_write_seq(0xC8,0x7C,0x6A,0x5D,0x53,0x53,0x45,0x4B,0x35,0x4D,0x4A,0x49,0x66,0x53,0x57,0x4A,0x48,0x3B,0x2A,0x06,0x7C,0x6A,0x5D,0x53,0x53,0x45,0x4B,0x35,0x4D,0x4A,0x49,0x66,0x53,0x57,0x4A,0x48,0x3B,0x2A,0x06);//GAMMA2.2
	dcs_write_seq(0xC6,0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00);
	dcs_write_seq(0xF4,0x08,0x77);
	dcs_write_seq(0x36,0x14);
	dcs_write_seq(0x35,0x00);
	dcs_write_seq(0xF1,0x5A,0x5A);
	dcs_write_seq(0xF0,0xA5,0xA5);
}

static int cwd686_disable(struct drm_panel *panel)
{
	struct cwd686 *ctx = panel_to_cwd686(panel);

	if (!ctx->enabled)
		return 0;

	backlight_disable(ctx->backlight);

	ctx->enabled = false;

	return 0;
}

static int cwd686_unprepare(struct drm_panel *panel)
{
	struct cwd686 *ctx = panel_to_cwd686(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

#if 0
	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		DRM_WARN("failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		DRM_WARN("failed to enter sleep mode: %d\n", ret);

	msleep(120);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	regulator_disable(ctx->supply);

	ctx->prepared = false;
#endif

	return 0;
}

static int cwd686_prepare(struct drm_panel *panel)
{
	struct cwd686 *ctx = panel_to_cwd686(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		return ret;
	}

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(100);
	}

	cwd686_init_sequence(ctx);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(20);

	ctx->prepared = true;

	return 0;
}

static int cwd686_enable(struct drm_panel *panel)
{
	struct cwd686 *ctx = panel_to_cwd686(panel);

	if (ctx->enabled)
		return 0;

	backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return 0;
}

static int cwd686_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
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

	drm_mode_probed_add(connector, mode);

	return 1; /* Number of modes */
}

static const struct drm_panel_funcs cwd686_drm_funcs = {
	.disable = cwd686_disable,
	.unprepare = cwd686_unprepare,
	.prepare = cwd686_prepare,
	.enable = cwd686_enable,
	.get_modes = cwd686_get_modes,
};

static int cwd686_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct cwd686 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "display-vcc");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		dev_err(dev, "cannot get regulator: %d\n", ret);
		return ret;
	}

	ctx->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(ctx->backlight))
		return PTR_ERR(ctx->backlight);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &cwd686_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void cwd686_remove(struct mipi_dsi_device *dsi)
{
	struct cwd686 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id cwd686_of_match[] = {
	{ .compatible = "cw,cwd686-cm3" },
	{ }
};
MODULE_DEVICE_TABLE(of, cwd686_of_match);

static struct mipi_dsi_driver cwd686_driver = {
	.probe = cwd686_probe,
	.remove = cwd686_remove,
	.driver = {
		.name = "panel-cwd686-cm3",
		.of_match_table = cwd686_of_match,
	},
};
module_mipi_dsi_driver(cwd686_driver);

MODULE_DESCRIPTION("DRM Driver for cwd686-cm3 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
