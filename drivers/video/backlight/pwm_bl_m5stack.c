// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/* 自定义平台数据结构，添加 force_enable_on_init 字段 */
struct platform_pwm_backlight_data {
    unsigned int max_brightness;
    unsigned int dft_brightness;
    unsigned int lth_brightness;
    unsigned int pwm_period_ns;
    unsigned int post_pwm_on_delay;
    unsigned int pwm_off_delay;
    unsigned int *levels;
    bool force_enable_on_init;  /* 新增：强制初始化时开启背光 */
    
    int (*init)(struct device *dev);
    int (*notify)(struct device *dev, int brightness);
    void (*notify_after)(struct device *dev, int brightness);
    void (*exit)(struct device *dev);
};

struct pwm_bl_data {
    struct pwm_device *pwm;
    struct device *dev;
    unsigned int lth_brightness;
    unsigned int *levels;
    bool enabled;
    struct regulator *power_supply;
    struct gpio_desc *enable_gpio;
    unsigned int scale;
    unsigned int post_pwm_on_delay;
    unsigned int pwm_off_delay;
    bool force_enable_on_init;  /* 新增：强制初始化时开启背光 */
    int (*notify)(struct device *,
          int brightness);
    void (*notify_after)(struct device *,
        int brightness);
    void (*exit)(struct device *);
};

static void pwm_backlight_power_on(struct pwm_bl_data *pb)
{
    int err;

    if (pb->enabled)
        return;

    if (pb->power_supply) {
        err = regulator_enable(pb->power_supply);
        if (err < 0)
            dev_err(pb->dev, "failed to enable power supply\n");
    }

    if (pb->post_pwm_on_delay)
        msleep(pb->post_pwm_on_delay);

    gpiod_set_value_cansleep(pb->enable_gpio, 1);

    pb->enabled = true;
}

static void pwm_backlight_power_off(struct pwm_bl_data *pb)
{
    if (!pb->enabled)
        return;

    gpiod_set_value_cansleep(pb->enable_gpio, 0);

    if (pb->pwm_off_delay)
        msleep(pb->pwm_off_delay);

    if (pb->power_supply)
        regulator_disable(pb->power_supply);

    pb->enabled = false;
}

static int compute_duty_cycle(struct pwm_bl_data *pb, int brightness, struct pwm_state *state)
{
    unsigned int lth = pb->lth_brightness;
    u64 duty_cycle;

    if (pb->levels)
        duty_cycle = pb->levels[brightness];
    else
        duty_cycle = brightness;

    duty_cycle *= state->period - lth;
    do_div(duty_cycle, pb->scale);

    return duty_cycle + lth;
}

static int pwm_backlight_update_status(struct backlight_device *bl)
{
    struct pwm_bl_data *pb = bl_get_data(bl);
    int brightness = backlight_get_brightness(bl);
    struct pwm_state state;

    if (pb->notify)
        brightness = pb->notify(pb->dev, brightness);

    if (brightness > 0) {
        pwm_get_state(pb->pwm, &state);
        state.duty_cycle = compute_duty_cycle(pb, brightness, &state);
        state.enabled = true;
        pwm_apply_might_sleep(pb->pwm, &state);

        pwm_backlight_power_on(pb);
    } else {
        pwm_backlight_power_off(pb);

        pwm_get_state(pb->pwm, &state);
        state.duty_cycle = 0;
        state.enabled = !pb->power_supply && !pb->enable_gpio;
        pwm_apply_might_sleep(pb->pwm, &state);
    }

    if (pb->notify_after)
        pb->notify_after(pb->dev, brightness);

    return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
    .update_status = pwm_backlight_update_status,
};

#ifdef CONFIG_OF
#define PWM_LUMINANCE_SHIFT	16
#define PWM_LUMINANCE_SCALE	(1 << PWM_LUMINANCE_SHIFT)

static u64 cie1931(unsigned int lightness)
{
    u64 retval;

    lightness *= 100;
    if (lightness <= (8 * PWM_LUMINANCE_SCALE)) {
        retval = DIV_ROUND_CLOSEST(lightness * 10, 9033);
    } else {
        retval = (lightness + (16 * PWM_LUMINANCE_SCALE)) / 116;
        retval *= retval * retval;
        retval += 1ULL << (2*PWM_LUMINANCE_SHIFT - 1);
        retval >>= 2*PWM_LUMINANCE_SHIFT;
    }

    return retval;
}

static
int pwm_backlight_brightness_default(struct device *dev,
                     struct platform_pwm_backlight_data *data,
                     unsigned int period)
{
    unsigned int i;
    u64 retval;

    data->max_brightness =
        min((int)DIV_ROUND_UP(period, fls(period)), 4096);

    data->levels = devm_kcalloc(dev, data->max_brightness,
                    sizeof(*data->levels), GFP_KERNEL);
    if (!data->levels)
        return -ENOMEM;

    for (i = 0; i < data->max_brightness; i++) {
        retval = cie1931((i * PWM_LUMINANCE_SCALE) /
                data->max_brightness) * period;
        retval = DIV_ROUND_CLOSEST_ULL(retval, PWM_LUMINANCE_SCALE);
        if (retval > UINT_MAX)
            return -EINVAL;
        data->levels[i] = (unsigned int)retval;
    }

    data->dft_brightness = data->max_brightness / 2;
    data->max_brightness--;

    return 0;
}

static int pwm_backlight_parse_dt(struct device *dev,
                  struct platform_pwm_backlight_data *data)
{
    struct device_node *node = dev->of_node;
    unsigned int num_levels;
    unsigned int num_steps = 0;
    struct property *prop;
    unsigned int *table;
    int length;
    u32 value;
    int ret;

    if (!node)
        return -ENODEV;

    memset(data, 0, sizeof(*data));

    of_property_read_u32(node, "post-pwm-on-delay-ms",
                 &data->post_pwm_on_delay);
    of_property_read_u32(node, "pwm-off-delay-ms", &data->pwm_off_delay);

    /* 新增：读取强制开启背光的属性 */
    data->force_enable_on_init = 
        of_property_read_bool(node, "force-enable-on-init");
    
    if (data->force_enable_on_init)
        dev_info(dev, "Backlight will be forced on during initialization\n");

    prop = of_find_property(node, "brightness-levels", &length);
    if (!prop)
        return 0;

    num_levels = length / sizeof(u32);

    if (num_levels > 0) {
        data->levels = devm_kcalloc(dev, num_levels,
                        sizeof(*data->levels), GFP_KERNEL);
        if (!data->levels)
            return -ENOMEM;

        ret = of_property_read_u32_array(node, "brightness-levels",
                        data->levels,
                        num_levels);
        if (ret < 0)
            return ret;

        ret = of_property_read_u32(node, "default-brightness-level",
                       &value);
        if (ret < 0)
            return ret;

        data->dft_brightness = value;

        of_property_read_u32(node, "num-interpolated-steps",
                     &num_steps);

        if (num_steps) {
            unsigned int num_input_levels = num_levels;
            unsigned int i;
            u32 x1, x2, x, dx;
            u32 y1, y2;
            s64 dy;

            if (num_input_levels < 2) {
                dev_err(dev, "can't interpolate\n");
                return -EINVAL;
            }

            num_levels = (num_input_levels - 1) * num_steps + 1;
            dev_dbg(dev, "new number of brightness levels: %d\n",
                num_levels);

            table = devm_kcalloc(dev, num_levels, sizeof(*table),
                         GFP_KERNEL);
            if (!table)
                return -ENOMEM;

            dx = num_steps;
            for (i = 0; i < num_input_levels - 1; i++) {
                x1 = i * dx;
                x2 = x1 + dx;
                y1 = data->levels[i];
                y2 = data->levels[i + 1];
                dy = (s64)y2 - y1;

                for (x = x1; x < x2; x++) {
                    table[x] = y1 +
                        div_s64(dy * (x - x1), dx);
                }
            }
            table[x2] = y2;

            devm_kfree(dev, data->levels);
            data->levels = table;
        }

        data->max_brightness = num_levels - 1;
    }

    return 0;
}

static const struct of_device_id pwm_backlight_of_match[] = {
    { .compatible = "pwm-backlight" },
    { .compatible = "m5stack,pwm-backlight" },  /* 添加你的兼容字符串 */
    { }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
                  struct platform_pwm_backlight_data *data)
{
    return -ENODEV;
}

static
int pwm_backlight_brightness_default(struct device *dev,
                     struct platform_pwm_backlight_data *data,
                     unsigned int period)
{
    return -ENODEV;
}
#endif

static bool pwm_backlight_is_linear(struct platform_pwm_backlight_data *data)
{
    unsigned int nlevels = data->max_brightness + 1;
    unsigned int min_val = data->levels[0];
    unsigned int max_val = data->levels[nlevels - 1];
    unsigned int slope = (128 * (max_val - min_val)) / nlevels;
    unsigned int margin = (max_val - min_val) / 20;
    int i;

    for (i = 1; i < nlevels; i++) {
        unsigned int linear_value = min_val + ((i * slope) / 128);
        unsigned int delta = abs(linear_value - data->levels[i]);

        if (delta > margin)
            return false;
    }

    return true;
}

static int pwm_backlight_initial_power_state(const struct pwm_bl_data *pb)
{
    struct device_node *node = pb->dev->of_node;
    bool active = true;

    /* 新增：如果设置了强制开启标志，直接返回开启状态 */
    if (pb->force_enable_on_init) {
        dev_info(pb->dev, "Force enable backlight on initialization\n");
        return BACKLIGHT_POWER_ON;
    }

    if (pb->enable_gpio && gpiod_get_value_cansleep(pb->enable_gpio) == 0)
        active = false;

    if (pb->power_supply && !regulator_is_enabled(pb->power_supply))
        active = false;

    if (!pwm_is_enabled(pb->pwm))
        active = false;

    gpiod_direction_output(pb->enable_gpio, active);

    if (!node || !node->phandle)
        return BACKLIGHT_POWER_ON;

    return active ? BACKLIGHT_POWER_ON : BACKLIGHT_POWER_OFF;
}

static int pwm_backlight_probe(struct platform_device *pdev)
{
    struct platform_pwm_backlight_data *data = dev_get_platdata(&pdev->dev);
    struct platform_pwm_backlight_data defdata;
    struct backlight_properties props;
    struct backlight_device *bl;
    struct pwm_bl_data *pb;
    struct pwm_state state;
    unsigned int i;
    int ret;

    if (!data) {
        ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
        if (ret < 0)
            return dev_err_probe(&pdev->dev, ret,
                         "failed to find platform data\n");
        data = &defdata;
    }

    if (data->init) {
        ret = data->init(&pdev->dev);
        if (ret < 0)
            return ret;
    }

    pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
    if (!pb) {
        ret = -ENOMEM;
        goto err_alloc;
    }

    pb->notify = data->notify;
    pb->notify_after = data->notify_after;
    pb->exit = data->exit;
    pb->dev = &pdev->dev;
    pb->enabled = false;
    pb->post_pwm_on_delay = data->post_pwm_on_delay;
    pb->pwm_off_delay = data->pwm_off_delay;
    pb->force_enable_on_init = data->force_enable_on_init;  /* 新增 */

    pb->enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
                          GPIOD_ASIS);
    if (IS_ERR(pb->enable_gpio)) {
        ret = dev_err_probe(&pdev->dev, PTR_ERR(pb->enable_gpio),
                    "failed to acquire enable GPIO\n");
        goto err_alloc;
    }

    pb->power_supply = devm_regulator_get_optional(&pdev->dev, "power");
    if (IS_ERR(pb->power_supply)) {
        ret = PTR_ERR(pb->power_supply);
        if (ret == -ENODEV) {
            pb->power_supply = NULL;
        } else {
            dev_err_probe(&pdev->dev, ret,
                      "failed to acquire power regulator\n");
            goto err_alloc;
        }
    }

    pb->pwm = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR(pb->pwm)) {
        ret = dev_err_probe(&pdev->dev, PTR_ERR(pb->pwm),
                    "unable to request PWM\n");
        goto err_alloc;
    }

    dev_dbg(&pdev->dev, "got pwm for backlight\n");

    pwm_init_state(pb->pwm, &state);

    if (!state.period && (data->pwm_period_ns > 0))
        state.period = data->pwm_period_ns;

    ret = pwm_apply_might_sleep(pb->pwm, &state);
    if (ret) {
        dev_err_probe(&pdev->dev, ret,
                  "failed to apply initial PWM state");
        goto err_alloc;
    }

    memset(&props, 0, sizeof(struct backlight_properties));

    if (data->levels) {
        pb->levels = data->levels;

        for (i = 0; i <= data->max_brightness; i++)
            if (data->levels[i] > pb->scale)
                pb->scale = data->levels[i];

        if (pwm_backlight_is_linear(data))
            props.scale = BACKLIGHT_SCALE_LINEAR;
        else
            props.scale = BACKLIGHT_SCALE_NON_LINEAR;
    } else if (!data->max_brightness) {
        pwm_get_state(pb->pwm, &state);

        ret = pwm_backlight_brightness_default(&pdev->dev, data,
                               state.period);
        if (ret < 0) {
            dev_err_probe(&pdev->dev, ret,
                      "failed to setup default brightness table\n");
            goto err_alloc;
        }

        for (i = 0; i <= data->max_brightness; i++) {
            if (data->levels[i] > pb->scale)
                pb->scale = data->levels[i];
        }

        pb->levels = data->levels;

        props.scale = BACKLIGHT_SCALE_NON_LINEAR;
    } else {
        pb->scale = data->max_brightness;
    }

    pb->lth_brightness = data->lth_brightness * (div_u64(state.period,
                            pb->scale));

    props.type = BACKLIGHT_RAW;
    props.max_brightness = data->max_brightness;
    bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
                       &pwm_backlight_ops, &props);
    if (IS_ERR(bl)) {
        ret = dev_err_probe(&pdev->dev, PTR_ERR(bl),
                    "failed to register backlight\n");
        goto err_alloc;
    }

    if (data->dft_brightness > data->max_brightness) {
        dev_warn(&pdev->dev,
            "invalid default brightness level: %u, using %u\n",
            data->dft_brightness, data->max_brightness);
        data->dft_brightness = data->max_brightness;
    }

    bl->props.brightness = data->dft_brightness;
    bl->props.power = pwm_backlight_initial_power_state(pb);

    /* 新增：如果强制开启，确保亮度不为0 */
    if (pb->force_enable_on_init) {
        if (bl->props.brightness == 0) {
            bl->props.brightness = data->dft_brightness > 0 ? 
                        data->dft_brightness : data->max_brightness / 2;
            dev_info(&pdev->dev, "Force set brightness to %d\n", 
                 bl->props.brightness);
        }
        bl->props.power = BACKLIGHT_POWER_ON;
    }

    backlight_update_status(bl);

    platform_set_drvdata(pdev, bl);
    return 0;

err_alloc:
    if (data->exit)
        data->exit(&pdev->dev);
    return ret;
}

static void pwm_backlight_remove(struct platform_device *pdev)
{
    struct backlight_device *bl = platform_get_drvdata(pdev);
    struct pwm_bl_data *pb = bl_get_data(bl);
    struct pwm_state state;

    backlight_device_unregister(bl);

    pwm_backlight_power_off(pb);

    pwm_get_state(pb->pwm, &state);
    state.duty_cycle = 0;
    state.enabled = false;
    pwm_apply_might_sleep(pb->pwm, &state);

    if (pb->exit)
        pb->exit(&pdev->dev);
}

static void pwm_backlight_shutdown(struct platform_device *pdev)
{
    struct backlight_device *bl = platform_get_drvdata(pdev);
    struct pwm_bl_data *pb = bl_get_data(bl);
    struct pwm_state state;

    pwm_backlight_power_off(pb);

    pwm_get_state(pb->pwm, &state);
    state.duty_cycle = 0;
    state.enabled = false;
    pwm_apply_might_sleep(pb->pwm, &state);
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
    struct backlight_device *bl = dev_get_drvdata(dev);
    struct pwm_bl_data *pb = bl_get_data(bl);
    struct pwm_state state;

    if (pb->notify)
        pb->notify(pb->dev, 0);

    pwm_backlight_power_off(pb);

    pwm_get_state(pb->pwm, &state);
    state.duty_cycle = 0;
    state.enabled = false;
    pwm_apply_might_sleep(pb->pwm, &state);

    if (pb->notify_after)
        pb->notify_after(pb->dev, 0);

    return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
    struct backlight_device *bl = dev_get_drvdata(dev);

    backlight_update_status(bl);

    return 0;
}
#endif

static const struct dev_pm_ops pwm_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = pwm_backlight_suspend,
    .resume = pwm_backlight_resume,
    .poweroff = pwm_backlight_suspend,
    .restore = pwm_backlight_resume,
#endif
};

static struct platform_driver pwm_backlight_driver = {
    .driver = {
        .name = "pwm-backlight",
        .pm = &pwm_backlight_pm_ops,
        .of_match_table = of_match_ptr(pwm_backlight_of_match),
    },
    .probe = pwm_backlight_probe,
    .remove = pwm_backlight_remove,
    .shutdown = pwm_backlight_shutdown,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver with M5Stack support");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pwm-backlight");
MODULE_AUTHOR("Your Name");