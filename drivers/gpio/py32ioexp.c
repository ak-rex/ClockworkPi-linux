// SPDX-License-Identifier: GPL-2.0-only
/*
* Driver for the PY32 GPIO Expander with ADC and Pinctrl support.
*
* Copyright (C) 2020 Tesla Motors, Inc.
*/
#include <linux/acpi.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

// 寄存器定义保持不变...
/* ============================================================================ */
#define PY32IO_UID_L 0x00
#define PY32IO_UID_H 0x01
#define PY32IO_REV 0x02
#define PY32IO_GPIO_M_L 0x03
#define PY32IO_GPIO_M_H 0x04
#define PY32IO_GPIO_O_L 0x05
#define PY32IO_GPIO_O_H 0x06
#define PY32IO_GPIO_I_L 0x07
#define PY32IO_GPIO_I_H 0x08
#define PY32IO_GPIO_PU_L 0x09
#define PY32IO_GPIO_PU_H 0x0A
#define PY32IO_GPIO_PD_L 0x0B
#define PY32IO_GPIO_PD_H 0x0C
#define PY32IO_GPIO_IE_L 0x0D
#define PY32IO_GPIO_IE_H 0x0E
#define PY32IO_GPIO_IP_L 0x0F
#define PY32IO_GPIO_IP_H 0x10
#define PY32IO_GPIO_IS_L 0x11
#define PY32IO_GPIO_IS_H 0x12
#define PY32IO_GPIO_DRV_L 0x13
#define PY32IO_GPIO_DRV_H 0x14
#define PY32IO_ADC_CTRL 0x15
#define PY32IO_ADC_D_L 0x16
#define PY32IO_ADC_D_H 0x17
#define PY32IO_TEMP_CTRL 0x18
#define PY32IO_TEMP_D_L 0x19
#define PY32IO_TEMP_D_H 0x1A
#define PY32IO_PWM1_L 0x1B
#define PY32IO_PWM1_H 0x1C
#define PY32IO_PWM2_L 0x1D
#define PY32IO_PWM2_H 0x1E
#define PY32IO_PWM3_L 0x1F
#define PY32IO_PWM3_H 0x20
#define PY32IO_PWM4_L 0x21
#define PY32IO_PWM4_H 0x22
#define PY32IO_IIC_CFG 0x23
#define PY32IO_LED_CFG 0x24
#define PY32IO_PWM_FREQ_L 0x25
#define PY32IO_PWM_FREQ_H 0x26
#define PY32IO_REF_VOLT_L 0x27
#define PY32IO_REF_VOLT_H 0x28
#define PY32IO_RESET 0x29
#define PY32IO_LED_RAM_S 0x30
#define PY32IO_LED_RAM_E 0x6F
#define PY32IO_RTC_RAM_S 0x70
#define PY32IO_RTC_RAM_E 0x8F
#define PY32IO_PULSE 0x90
#define PY32IO_REG_MAX 0x90

/* ADC控制位定义 */
#define PY32IO_ADC_CTRL_BUSY               (1 << 7)
#define PY32IO_ADC_CTRL_START              (1 << 6)
#define PY32IO_ADC_CTRL_CH_MASK            0x07
#define PY32IO_ADC_CH_DISABLE              0
#define PY32IO_ADC_CH_ADC1                 1  // IO2
#define PY32IO_ADC_CH_ADC2                 2  // IO4
#define PY32IO_ADC_CH_ADC3                 3  // IO5
#define PY32IO_ADC_CH_ADC4                 4  // IO7

/* 温度传感器控制位定义 */
#define PY32IO_TEMP_CTRL_BUSY              (1 << 7)
#define PY32IO_TEMP_CTRL_START             (1 << 6)

/* I2C配置位定义 */
#define PY32IO_I2C_CFG_INTERNAL_PULL       (1 << 6)
#define PY32IO_I2C_CFG_WAKE_TYPE           (1 << 5)
#define PY32IO_I2C_CFG_SPD                 (1 << 4)
#define PY32IO_I2C_CFG_SLEEP_MASK          0x0F

/* LED配置位定义 */
#define PY32IO_LED_CFG_REFRESH             (1 << 6)
#define PY32IO_LED_CFG_LED_MASK            0x3F

#define PY32IO_PWM_EN_BIT      (1 << 7)
#define PY32IO_PWM_POL_BIT     (1 << 6)
#define PY32IO_PWM_DUTY_MASK   0x0F

/* 默认值定义 */
#define PY32IO_DEFAULT_GPIO_DRV_L          0x00
#define PY32IO_DEFAULT_GPIO_DRV_H          0x00
#define PY32IO_DEFAULT_PWM_FREQ_L          0xF4
#define PY32IO_DEFAULT_PWM_FREQ_H          0x01
#define PY32IO_DEFAULT_PWM_FREQ            500

#define I2C_READ_RETRIES 5
#define PY32IO_RESET_DELAY_MS 1
#define PY32IO_N_GPIO 14
#define PY32IO_ADC_RESOLUTION 12
#define PY32IO_ADC_MAX_VAL ((1 << PY32IO_ADC_RESOLUTION) - 1)

/* ADC超时时间 */
#define PY32IO_ADC_TIMEOUT_MS 100

/* GPIO方向定义 */
#define PY32IO_DIRECTION_TO_GPIOD(x) ((x) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN)
#define GPIOD_DIRECTION_TO_PY32IO(x) ((x) == GPIO_LINE_DIRECTION_OUT ? 1 : 0)

/* ADC通道枚举 */
enum py32io_adc_channel {
    PY32IO_ADC_CHANNEL_1 = 0,  // IO2
    PY32IO_ADC_CHANNEL_2,      // IO4
    PY32IO_ADC_CHANNEL_3,      // IO5
    PY32IO_ADC_CHANNEL_4,      // IO7
    PY32IO_ADC_CHANNEL_TEMP,   // 温度传感器
    PY32IO_ADC_CHANNEL_VREF,   // 参考电压
    PY32IO_ADC_NUM_CHANNELS,
};

/* Pinctrl功能定义 */
enum py32io_pin_function {
    PY32IO_FUNC_GPIO = 0,
    PY32IO_FUNC_ADC,
    PY32IO_FUNC_PWM,
    PY32IO_FUNC_LED,
};

struct py32io_priv {
    struct i2c_client *i2c;
    struct regmap *regmap;
    struct gpio_chip gpio;
    struct pwm_chip *pwm_chip;
    struct pinctrl_dev *pctldev;
    struct gpio_desc *reset_gpio;
    struct mutex lock;  // 保护ADC访问
    u16 vref_mv;  // 参考电压(mV)
#ifdef CONFIG_GPIO_PI4IOE5V64XX_IRQ
    struct irq_chip irq_chip;
    struct mutex irq_lock;
    uint16_t irq_mask;
#endif
};

/* ============================================================================
 * Regmap 配置
 * ============================================================================ */

static bool py32io_readable_reg(struct device *dev, unsigned int reg)
{
    return reg <= PY32IO_REG_MAX;
}

static bool py32io_writeable_reg(struct device *dev, unsigned int reg)
{
    return reg <= PY32IO_REG_MAX;
}

static bool py32io_volatile_reg(struct device *dev, unsigned int reg)
{
    switch (reg) {
    case PY32IO_GPIO_I_L:
    case PY32IO_GPIO_I_H:
    case PY32IO_ADC_CTRL:
    case PY32IO_ADC_D_L:
    case PY32IO_ADC_D_H:
    case PY32IO_TEMP_CTRL:
    case PY32IO_TEMP_D_L:
    case PY32IO_TEMP_D_H:
    case PY32IO_REF_VOLT_L:
    case PY32IO_REF_VOLT_H:
    case PY32IO_RTC_RAM_S ... PY32IO_RTC_RAM_E:
    case PY32IO_GPIO_IE_L:
    case PY32IO_GPIO_IE_H:
        return true;
    default:
        return false;
    }
}

static const struct regmap_config py32io16_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = PY32IO_REG_MAX,
    .writeable_reg = py32io_writeable_reg,
    .readable_reg = py32io_readable_reg,
    .volatile_reg = py32io_volatile_reg,
};

static int py32io_byte_reg_read(void *context, unsigned int reg,
                      unsigned int *val)
{
    struct device *dev = context;
    struct i2c_client *i2c = to_i2c_client(dev);
    int ret;
    int retries = I2C_READ_RETRIES;

    if (reg > 0xff)
        return -EINVAL;

    do {
        ret = i2c_smbus_read_byte_data(i2c, reg);
    } while (ret < 0 && retries-- > 0);

    if (ret < 0)
        return ret;

    *val = ret;
    return 0;
}

static int py32io_byte_reg_write(void *context, unsigned int reg,
                unsigned int val)
{
    struct device *dev = context;
    struct i2c_client *i2c = to_i2c_client(dev);
    int ret;
    int retries = I2C_READ_RETRIES;

    if (val > 0xff || reg > 0xff)
        return -EINVAL;

    do {
        ret = i2c_smbus_write_byte_data(i2c, reg, val);
    } while (ret != 0 && retries-- > 0);

    return ret;
}

static struct regmap_bus py32io_regmap_bus = {
    .reg_write = py32io_byte_reg_write,
    .reg_read = py32io_byte_reg_read,
};

static struct regmap* py32io_setup_regmap(struct i2c_client *i2c, int py32io_dev_id)
{
    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        return ERR_PTR(-ENOTSUPP);
    }
    if(py32io_dev_id == 0){
        return devm_regmap_init(&i2c->dev, &py32io_regmap_bus, &i2c->dev,
                    &py32io16_regmap_config);
    }
    return ERR_PTR(-EINVAL);
}

/* ============================================================================
 * GPIO 功能实现
 * ============================================================================ */

static int py32io_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
    int ret, io_dir, direction;
    struct py32io_priv *py32io = gpiochip_get_data(chip);
    struct device *dev = &py32io->i2c->dev;

    if(py32io->gpio.ngpio == PY32IO_N_GPIO) {
        if (offset < 8) {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_M_L, &io_dir);
        } else {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_M_H, &io_dir);
            offset = offset % 8;
        }
        if (ret) {
            dev_err(dev, "Failed to read I/O direction: %d", ret);
            return ret;
        }
        direction = PY32IO_DIRECTION_TO_GPIOD((io_dir >> offset) & 1);
        dev_dbg(dev, "get_direction : offset=%u, direction=%s, reg=0x%X",
            offset, (direction == GPIO_LINE_DIRECTION_IN) ? "input" : "output",
            io_dir);
        return direction;
    }
    return -1;
}

static int py32io_gpio_set_direction(struct gpio_chip *chip, unsigned offset,
                      int direction)
{
    int ret, reg;
    struct py32io_priv *py32io = gpiochip_get_data(chip);
    struct device *dev = &py32io->i2c->dev;

    dev_dbg(dev, "set_direction : offset=%u, direction=%s", offset,
        (direction == GPIO_LINE_DIRECTION_IN) ? "input" : "output");

    if(py32io->gpio.ngpio == PY32IO_N_GPIO){
        if (offset < 8) {
            reg = PY32IO_GPIO_M_L;
        } else {
            reg = PY32IO_GPIO_M_H;
            offset = offset % 8;
        }
        ret = regmap_update_bits(py32io->regmap, reg, 1 << offset,
                    GPIOD_DIRECTION_TO_PY32IO(direction) <<
                    offset);
        if (ret) {
            dev_err(dev, "Failed to set direction: %d", ret);
            return ret;
        }
        return ret;
    }
    return -1;
}

static int py32io_gpio_get(struct gpio_chip *chip, unsigned offset)
{
    int ret, out, reg;
    struct py32io_priv *py32io = gpiochip_get_data(chip);
    struct device *dev = &py32io->i2c->dev;

    if(py32io->gpio.ngpio == PY32IO_N_GPIO){
        if (offset < 8) {
            reg = PY32IO_GPIO_I_L;
        } else {
            reg = PY32IO_GPIO_I_H;
            offset = offset % 8;
        }
    }

    ret = regmap_read(py32io->regmap, reg, &out);
    if (ret) {
        dev_err(dev, "Failed to read output: %d", ret);
        return ret;
    }

    dev_dbg(dev, "gpio_get : offset=%u, val=%s reg=0x%X", offset,
        out & (1 << (offset % 8)) ? "1" : "0", out);

    if (out & (1 << (offset % 8))) {
        return 1;
    }
    return 0;
}

static int py32io_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
    int ret, reg;
    struct py32io_priv *py32io = gpiochip_get_data(chip);
    struct device *dev = &py32io->i2c->dev;

    dev_dbg(dev, "gpio_set : offset=%u, val=%s", offset, value ? "1" : "0");

    if(py32io->gpio.ngpio == PY32IO_N_GPIO){
        if (offset < 8) {
            reg = PY32IO_GPIO_O_L;
        } else {
            reg = PY32IO_GPIO_O_H;
            offset = offset % 8;
        }
    }

    ret = regmap_update_bits(py32io->regmap, reg, 1 << offset,
                value << offset);
    if (ret) {
        dev_err(dev, "Failed to write output: %d", ret);
    }

    return ret;
}

static int py32io_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
    return py32io_gpio_set_direction(chip, offset, GPIO_LINE_DIRECTION_IN);
}

static int py32io_gpio_direction_output(struct gpio_chip *chip,
                     unsigned offset, int value)
{
    int ret;
    struct py32io_priv *py32io = gpiochip_get_data(chip);
    struct device *dev = &py32io->i2c->dev;

    ret = py32io_gpio_set_direction(chip, offset, GPIO_LINE_DIRECTION_OUT);
    if (ret) {
        dev_err(dev, "Failed to set direction: %d", ret);
        return ret;
    }

    return py32io_gpio_set(chip, offset, value);
}

static int py32io_gpio_setup(struct py32io_priv *py32io, int py32io_dev_id)
{
    int ret;
    struct device *dev = &py32io->i2c->dev;
    struct gpio_chip *gc = &py32io->gpio;

    ret = regmap_write(py32io->regmap, PY32IO_GPIO_DRV_L,
                       PY32IO_DEFAULT_GPIO_DRV_L);
    if (ret) {
        dev_err(dev, "Failed to init GPIO_DRV_L: %d\n", ret);
        return ret;
    }

    ret = regmap_write(py32io->regmap, PY32IO_GPIO_DRV_H,
                       PY32IO_DEFAULT_GPIO_DRV_H);
    if (ret) {
        dev_err(dev, "Failed to init GPIO_DRV_H: %d\n", ret);
        return ret;
    }

    gc->ngpio = PY32IO_N_GPIO;
    gc->label = py32io->i2c->name;
    gc->parent = &py32io->i2c->dev;
    gc->owner = THIS_MODULE;
    gc->base = -1;
    gc->can_sleep = true;
    gc->get_direction = py32io_gpio_get_direction;
    gc->direction_input = py32io_gpio_direction_input;
    gc->direction_output = py32io_gpio_direction_output;
    gc->get = py32io_gpio_get;
    gc->set = py32io_gpio_set;

    ret = devm_gpiochip_add_data(dev, gc, py32io);
    if (ret) {
        dev_err(dev, "devm_gpiochip_add_data failed: %d", ret);
        return ret;
    }

    return 0;
}

/* ============================================================================
 * PWM 功能实现
 * ============================================================================ */

static int py32io_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                            const struct pwm_state *state)
{
    struct py32io_priv *pc = pwmchip_get_drvdata(chip);
    struct device *dev = &pc->i2c->dev;
    unsigned int freq, duty;
    unsigned int pwm_l_reg, pwm_h_reg;
    u8 pwm_h_val = 0;
    int ret;

    pwm_l_reg = PY32IO_PWM1_L + (pwm->hwpwm * 2);
    pwm_h_reg = PY32IO_PWM1_H + (pwm->hwpwm * 2);

    if (!state->enabled) {
        ret = regmap_update_bits(pc->regmap, pwm_h_reg,
                                PY32IO_PWM_EN_BIT, 0);
        if (ret) {
            dev_err(dev, "Failed to disable PWM%d: %d\n",
                    pwm->hwpwm + 1, ret);
            return ret;
        }
        dev_dbg(dev, "PWM%d disabled\n", pwm->hwpwm + 1);
        return 0;
    }

    if (state->period == 0)
        return -EINVAL;

    freq = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, state->period);
    duty = DIV_ROUND_CLOSEST_ULL((u64)state->duty_cycle * 4095, state->period);
    if (duty > 4095)
        duty = 4095;

    dev_dbg(dev, "PWM%d: period=%lluns, duty=%lluns, freq=%uHz, duty_val=%u, pol=%d\n",
            pwm->hwpwm + 1, state->period, state->duty_cycle,
            freq, duty, state->polarity);

    ret = regmap_write(pc->regmap, PY32IO_PWM_FREQ_L, freq & 0xFF);
    if (ret) {
        dev_err(dev, "Failed to write PWM_FREQ_L: %d\n", ret);
        return ret;
    }

    ret = regmap_write(pc->regmap, PY32IO_PWM_FREQ_H, (freq >> 8) & 0xFF);
    if (ret) {
        dev_err(dev, "Failed to write PWM_FREQ_H: %d\n", ret);
        return ret;
    }

    ret = regmap_write(pc->regmap, pwm_l_reg, duty & 0xFF);
    if (ret) {
        dev_err(dev, "Failed to write PWM%d_L: %d\n",
                pwm->hwpwm + 1, ret);
        return ret;
    }

    pwm_h_val = (duty >> 8) & PY32IO_PWM_DUTY_MASK;
    pwm_h_val |= PY32IO_PWM_EN_BIT;
    if (state->polarity == PWM_POLARITY_INVERSED)
        pwm_h_val |= PY32IO_PWM_POL_BIT;

    ret = regmap_write(pc->regmap, pwm_h_reg, pwm_h_val);
    if (ret) {
        dev_err(dev, "Failed to write PWM%d_H: %d\n",
                pwm->hwpwm + 1, ret);
        return ret;
    }

    dev_dbg(dev, "PWM%d enabled\n", pwm->hwpwm + 1);
    return 0;
}

static const struct pwm_ops py32io_pwm_ops = {
    .apply = py32io_pwm_apply,
};

static int py32io_pwm_setup(struct py32io_priv *py32io, int py32io_dev_id)
{
    int ret, i;
    struct device *dev = &py32io->i2c->dev;
    struct pwm_chip *chip;

    chip = devm_pwmchip_alloc(dev, 4, sizeof(*py32io));
    if (IS_ERR(chip)) {
        ret = PTR_ERR(chip);
        dev_err(dev, "Failed to allocate PWM chip: %d\n", ret);
        return ret;
    }

    py32io->pwm_chip = chip;
    pwmchip_set_drvdata(chip, py32io);
    chip->ops = &py32io_pwm_ops;

    ret = regmap_write(py32io->regmap, PY32IO_PWM_FREQ_L,
                       PY32IO_DEFAULT_PWM_FREQ_L);
    if (ret) {
        dev_err(dev, "Failed to init PWM_FREQ_L: %d\n", ret);
        return ret;
    }

    ret = regmap_write(py32io->regmap, PY32IO_PWM_FREQ_H,
                       PY32IO_DEFAULT_PWM_FREQ_H);
    if (ret) {
        dev_err(dev, "Failed to init PWM_FREQ_H: %d\n", ret);
        return ret;
    }

    for (i = 0; i < 4; i++) {
        unsigned int pwm_l_reg = PY32IO_PWM1_L + (i * 2);
        unsigned int pwm_h_reg = PY32IO_PWM1_H + (i * 2);

        ret = regmap_write(py32io->regmap, pwm_l_reg, 0x00);
        if (ret) {
            dev_err(dev, "Failed to init PWM%d_L: %d\n", i + 1, ret);
            return ret;
        }

        ret = regmap_write(py32io->regmap, pwm_h_reg, 0x00);
        if (ret) {
            dev_err(dev, "Failed to init PWM%d_H: %d\n", i + 1, ret);
            return ret;
        }
    }

    ret = devm_pwmchip_add(dev, chip);
    if (ret < 0) {
        dev_err(dev, "pwmchip_add() failed: %d\n", ret);
        return ret;
    }

    dev_info(dev, "PWM device registered with %d channels\n", chip->npwm);
    return 0;
}

/* ============================================================================
 * ADC/IIO 功能实现
 * ============================================================================ */

static int py32io_adc_read_raw(struct py32io_priv *py32io, int channel, int *val)
{
    struct device *dev = &py32io->i2c->dev;
    unsigned int ctrl_val, busy;
    unsigned int val_l, val_h;
    int ret;
    unsigned long timeout;

    mutex_lock(&py32io->lock);

    if (channel == PY32IO_ADC_CHANNEL_TEMP) {
        /* 读取温度传感器 */
        ret = regmap_write(py32io->regmap, PY32IO_TEMP_CTRL, 
                          PY32IO_TEMP_CTRL_START);
        if (ret) {
            dev_err(dev, "Failed to start temperature conversion: %d\n", ret);
            goto out_unlock;
        }

        /* 等待转换完成 */
        timeout = jiffies + msecs_to_jiffies(PY32IO_ADC_TIMEOUT_MS);
        do {
            ret = regmap_read(py32io->regmap, PY32IO_TEMP_CTRL, &busy);
            if (ret) {
                dev_err(dev, "Failed to read TEMP_CTRL: %d\n", ret);
                goto out_unlock;
            }
            if (!(busy & PY32IO_TEMP_CTRL_BUSY))
                break;
            usleep_range(100, 200);
        } while (time_before(jiffies, timeout));

        if (busy & PY32IO_TEMP_CTRL_BUSY) {
            dev_err(dev, "Temperature conversion timeout\n");
            ret = -ETIMEDOUT;
            goto out_unlock;
        }

        /* 读取温度数据 */
        ret = regmap_read(py32io->regmap, PY32IO_TEMP_D_L, &val_l);
        if (ret) {
            dev_err(dev, "Failed to read TEMP_D_L: %d\n", ret);
            goto out_unlock;
        }

        ret = regmap_read(py32io->regmap, PY32IO_TEMP_D_H, &val_h);
        if (ret) {
            dev_err(dev, "Failed to read TEMP_D_H: %d\n", ret);
            goto out_unlock;
        }

        *val = (val_l | ((val_h & 0x0F) << 8));

    } else if (channel == PY32IO_ADC_CHANNEL_VREF) {
        /* 读取参考电压 */
        ret = regmap_read(py32io->regmap, PY32IO_REF_VOLT_L, &val_l);
        if (ret) {
            dev_err(dev, "Failed to read REF_VOLT_L: %d\n", ret);
            goto out_unlock;
        }

        ret = regmap_read(py32io->regmap, PY32IO_REF_VOLT_H, &val_h);
        if (ret) {
            dev_err(dev, "Failed to read REF_VOLT_H: %d\n", ret);
            goto out_unlock;
        }

        *val = (val_l | (val_h << 8));
        py32io->vref_mv = *val;  // 更新缓存的参考电压

    } else {
        /* 读取ADC通道 */
        if (channel < PY32IO_ADC_CHANNEL_1 || channel > PY32IO_ADC_CHANNEL_4) {
            ret = -EINVAL;
            goto out_unlock;
        }

        /* 配置并启动ADC转换 */
        ctrl_val = (channel + 1) | PY32IO_ADC_CTRL_START;
        ret = regmap_write(py32io->regmap, PY32IO_ADC_CTRL, ctrl_val);
        if (ret) {
            dev_err(dev, "Failed to start ADC conversion: %d\n", ret);
            goto out_unlock;
        }

        /* 等待转换完成 */
        timeout = jiffies + msecs_to_jiffies(PY32IO_ADC_TIMEOUT_MS);
        do {
            ret = regmap_read(py32io->regmap, PY32IO_ADC_CTRL, &busy);
            if (ret) {
                dev_err(dev, "Failed to read ADC_CTRL: %d\n", ret);
                goto out_unlock;
            }
            if (!(busy & PY32IO_ADC_CTRL_BUSY))
                break;
            usleep_range(100, 200);
        } while (time_before(jiffies, timeout));

        if (busy & PY32IO_ADC_CTRL_BUSY) {
            dev_err(dev, "ADC conversion timeout\n");
            ret = -ETIMEDOUT;
            goto out_unlock;
        }

        /* 读取ADC数据 */
        ret = regmap_read(py32io->regmap, PY32IO_ADC_D_L, &val_l);
        if (ret) {
            dev_err(dev, "Failed to read ADC_D_L: %d\n", ret);
            goto out_unlock;
        }

        ret = regmap_read(py32io->regmap, PY32IO_ADC_D_H, &val_h);
        if (ret) {
            dev_err(dev, "Failed to read ADC_D_H: %d\n", ret);
            goto out_unlock;
        }

        *val = (val_l | ((val_h & 0x0F) << 8));
    }

    ret = 0;
    dev_dbg(dev, "ADC read channel %d: raw=%d\n", channel, *val);

out_unlock:
    mutex_unlock(&py32io->lock);
    return ret;
}

static int py32io_iio_read_raw(struct iio_dev *indio_dev,
                              struct iio_chan_spec const *chan,
                              int *val, int *val2, long mask)
{
    struct py32io_priv *py32io = iio_priv(indio_dev);
    int ret;

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        ret = py32io_adc_read_raw(py32io, chan->channel, val);
        if (ret < 0)
            return ret;
        return IIO_VAL_INT;

    case IIO_CHAN_INFO_SCALE:
        if (chan->type == IIO_VOLTAGE) {
            /* 电压 = (ADC_raw * vref_mv) / 4096 */
            *val = py32io->vref_mv;
            *val2 = PY32IO_ADC_RESOLUTION;
            return IIO_VAL_FRACTIONAL_LOG2;
        } else if (chan->type == IIO_TEMP) {
            /* 温度直接以摄氏度输出 */
            *val = 1;
            return IIO_VAL_INT;
        }
        return -EINVAL;

    default:
        return -EINVAL;
    }
}

static const struct iio_info py32io_iio_info = {
    .read_raw = py32io_iio_read_raw,
};

#define PY32IO_ADC_CHANNEL(_idx, _type, _name) { \
    .type = _type, \
    .indexed = 1, \
    .channel = _idx, \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
                         BIT(IIO_CHAN_INFO_SCALE), \
    .datasheet_name = _name, \
}

static const struct iio_chan_spec py32io_iio_channels[] = {
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_1, IIO_VOLTAGE, "ADC1_IO2"),
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_2, IIO_VOLTAGE, "ADC2_IO4"),
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_3, IIO_VOLTAGE, "ADC3_IO5"),
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_4, IIO_VOLTAGE, "ADC4_IO7"),
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_TEMP, IIO_TEMP, "TEMP"),
    PY32IO_ADC_CHANNEL(PY32IO_ADC_CHANNEL_VREF, IIO_VOLTAGE, "VREF"),
};

static int py32io_adc_setup(struct py32io_priv *py32io)
{
    struct device *dev = &py32io->i2c->dev;
    struct iio_dev *indio_dev;
    struct py32io_priv *priv;
    int ret;
    unsigned int vref_l, vref_h;

    /* 分配IIO设备，并在其后附加py32io_priv空间 */
    indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
    if (!indio_dev)
        return -ENOMEM;

    /* 获取IIO私有数据指针并复制py32io指针 */
    priv = iio_priv(indio_dev);
    *priv = *py32io;

    indio_dev->name = py32io->i2c->name;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->info = &py32io_iio_info;
    indio_dev->channels = py32io_iio_channels;
    indio_dev->num_channels = ARRAY_SIZE(py32io_iio_channels);

    /* 读取参考电压 */
    ret = regmap_read(py32io->regmap, PY32IO_REF_VOLT_L, &vref_l);
    if (ret) {
        dev_err(dev, "Failed to read REF_VOLT_L: %d\n", ret);
        return ret;
    }

    ret = regmap_read(py32io->regmap, PY32IO_REF_VOLT_H, &vref_h);
    if (ret) {
        dev_err(dev, "Failed to read REF_VOLT_H: %d\n", ret);
        return ret;
    }

    py32io->vref_mv = (vref_l | (vref_h << 8));
    if (py32io->vref_mv == 0)
        py32io->vref_mv = 3300;  // 默认3.3V

    dev_info(dev, "Reference voltage: %u mV\n", py32io->vref_mv);

    ret = devm_iio_device_register(dev, indio_dev);
    if (ret) {
        dev_err(dev, "Failed to register IIO device: %d\n", ret);
        return ret;
    }

    dev_info(dev, "ADC device registered with %d channels\n",
             indio_dev->num_channels);

    return 0;
}

/* ============================================================================
 * Pinctrl 功能实现
 * ============================================================================ */

static const struct pinctrl_pin_desc py32io_pins[] = {
    PINCTRL_PIN(0, "IO1"),
    PINCTRL_PIN(1, "IO2"),
    PINCTRL_PIN(2, "IO3"),
    PINCTRL_PIN(3, "IO4"),
    PINCTRL_PIN(4, "IO5"),
    PINCTRL_PIN(5, "IO6"),
    PINCTRL_PIN(6, "IO7"),
    PINCTRL_PIN(7, "IO8"),
    PINCTRL_PIN(8, "IO9"),
    PINCTRL_PIN(9, "IO10"),
    PINCTRL_PIN(10, "IO11"),
    PINCTRL_PIN(11, "IO12"),
    PINCTRL_PIN(12, "IO13"),
    PINCTRL_PIN(13, "IO14"),
};

static int py32io_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
    return 0;  // 不使用组
}

static const char *py32io_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
                                                 unsigned selector)
{
    return NULL;
}

static int py32io_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
                                        unsigned selector,
                                        const unsigned **pins,
                                        unsigned *num_pins)
{
    return -EINVAL;
}

static const struct pinctrl_ops py32io_pinctrl_ops = {
    .get_groups_count = py32io_pinctrl_get_groups_count,
    .get_group_name = py32io_pinctrl_get_group_name,
    .get_group_pins = py32io_pinctrl_get_group_pins,
    .dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
    .dt_free_map = pinconf_generic_dt_free_map,
};

static int py32io_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
    return 0;  // 简化实现，主要使用pinconf
}

static const char *py32io_pinmux_get_function_name(struct pinctrl_dev *pctldev,
                                                   unsigned selector)
{
    return NULL;
}

static int py32io_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
                                            unsigned selector,
                                            const char * const **groups,
                                            unsigned * const num_groups)
{
    return -EINVAL;
}

static int py32io_pinmux_set_mux(struct pinctrl_dev *pctldev,
                                unsigned func_selector,
                                unsigned group_selector)
{
    return 0;
}

static const struct pinmux_ops py32io_pinmux_ops = {
    .get_functions_count = py32io_pinmux_get_functions_count,
    .get_function_name = py32io_pinmux_get_function_name,
    .get_function_groups = py32io_pinmux_get_function_groups,
    .set_mux = py32io_pinmux_set_mux,
};

static int py32io_pinconf_get(struct pinctrl_dev *pctldev, unsigned pin,
                             unsigned long *config)
{
    struct py32io_priv *py32io = pinctrl_dev_get_drvdata(pctldev);
    struct device *dev = &py32io->i2c->dev;
    enum pin_config_param param = pinconf_to_config_param(*config);
    unsigned int val, reg;
    int ret;
    u16 arg = 0;

    if (pin >= PY32IO_N_GPIO)
        return -EINVAL;

    if (pin < 8) {
        reg = pin;
    } else {
        reg = pin % 8;
    }

    switch (param) {
    case PIN_CONFIG_BIAS_PULL_UP:
        if (pin < 8) {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_PU_L, &val);
        } else {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_PU_H, &val);
        }
        if (ret)
            return ret;
        arg = !!(val & (1 << reg));
        break;

    case PIN_CONFIG_BIAS_PULL_DOWN:
        if (pin < 8) {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_PD_L, &val);
        } else {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_PD_H, &val);
        }
        if (ret)
            return ret;
        arg = !!(val & (1 << reg));
        break;

    case PIN_CONFIG_DRIVE_OPEN_DRAIN:
        if (pin < 8) {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_DRV_L, &val);
        } else {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_DRV_H, &val);
        }
        if (ret)
            return ret;
        arg = !!(val & (1 << reg));
        break;

    case PIN_CONFIG_DRIVE_PUSH_PULL:
        if (pin < 8) {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_DRV_L, &val);
        } else {
            ret = regmap_read(py32io->regmap, PY32IO_GPIO_DRV_H, &val);
        }
        if (ret)
            return ret;
        arg = !(val & (1 << reg));
        break;

    default:
        dev_dbg(dev, "Unsupported pinconf param %d\n", param);
        return -ENOTSUPP;
    }

    *config = pinconf_to_config_packed(param, arg);
    return 0;
}

static int py32io_pinconf_set(struct pinctrl_dev *pctldev, unsigned pin,
                             unsigned long *configs, unsigned num_configs)
{
    struct py32io_priv *py32io = pinctrl_dev_get_drvdata(pctldev);
    struct device *dev = &py32io->i2c->dev;
    enum pin_config_param param;
    u32 arg;
    unsigned int reg_pu, reg_pd, reg_drv;
    int i, ret;
    unsigned int bit;

    if (pin >= PY32IO_N_GPIO)
        return -EINVAL;

    if (pin < 8) {
        reg_pu = PY32IO_GPIO_PU_L;
        reg_pd = PY32IO_GPIO_PD_L;
        reg_drv = PY32IO_GPIO_DRV_L;
        bit = pin;
    } else {
        reg_pu = PY32IO_GPIO_PU_H;
        reg_pd = PY32IO_GPIO_PD_H;
        reg_drv = PY32IO_GPIO_DRV_H;
        bit = pin % 8;
    }

    for (i = 0; i < num_configs; i++) {
        param = pinconf_to_config_param(configs[i]);
        arg = pinconf_to_config_argument(configs[i]);

        switch (param) {
        case PIN_CONFIG_BIAS_DISABLE:
            ret = regmap_update_bits(py32io->regmap, reg_pu,
                                    1 << bit, 0);
            if (ret)
                return ret;
            ret = regmap_update_bits(py32io->regmap, reg_pd,
                                    1 << bit, 0);
            if (ret)
                return ret;
            dev_dbg(dev, "Pin %u: bias disabled\n", pin);
            break;

        case PIN_CONFIG_BIAS_PULL_UP:
            ret = regmap_update_bits(py32io->regmap, reg_pu,
                                    1 << bit, 1 << bit);
            if (ret)
                return ret;
            ret = regmap_update_bits(py32io->regmap, reg_pd,
                                    1 << bit, 0);
            if (ret)
                return ret;
            dev_dbg(dev, "Pin %u: pull-up enabled\n", pin);
            break;

        case PIN_CONFIG_BIAS_PULL_DOWN:
            ret = regmap_update_bits(py32io->regmap, reg_pu,
                                    1 << bit, 0);
            if (ret)
                return ret;
            ret = regmap_update_bits(py32io->regmap, reg_pd,
                                    1 << bit, 1 << bit);
            if (ret)
                return ret;
            dev_dbg(dev, "Pin %u: pull-down enabled\n", pin);
            break;

        case PIN_CONFIG_DRIVE_OPEN_DRAIN:
            ret = regmap_update_bits(py32io->regmap, reg_drv,
                                    1 << bit, 1 << bit);
            if (ret)
                return ret;
            dev_dbg(dev, "Pin %u: open-drain mode\n", pin);
            break;

        case PIN_CONFIG_DRIVE_PUSH_PULL:
            ret = regmap_update_bits(py32io->regmap, reg_drv,
                                    1 << bit, 0);
            if (ret)
                return ret;
            dev_dbg(dev, "Pin %u: push-pull mode\n", pin);
            break;

        default:
            dev_err(dev, "Unsupported pinconf param %d\n", param);
            return -ENOTSUPP;
        }
    }

    return 0;
}

static const struct pinconf_ops py32io_pinconf_ops = {
    .pin_config_get = py32io_pinconf_get,
    .pin_config_set = py32io_pinconf_set,
    .is_generic = true,
};

static struct pinctrl_desc py32io_pinctrl_desc = {
    .name = "py32io-pinctrl",
    .pins = py32io_pins,
    .npins = ARRAY_SIZE(py32io_pins),
    .pctlops = &py32io_pinctrl_ops,
    .pmxops = &py32io_pinmux_ops,
    .confops = &py32io_pinconf_ops,
    .owner = THIS_MODULE,
};

static int py32io_pinctrl_setup(struct py32io_priv *py32io)
{
    struct device *dev = &py32io->i2c->dev;
    int ret;

    py32io->pctldev = devm_pinctrl_register(dev, &py32io_pinctrl_desc, py32io);
    if (IS_ERR(py32io->pctldev)) {
        ret = PTR_ERR(py32io->pctldev);
        dev_err(dev, "Failed to register pinctrl device: %d\n", ret);
        return ret;
    }

    dev_info(dev, "Pinctrl device registered with %d pins\n",
             py32io_pinctrl_desc.npins);

    return 0;
}

/* ============================================================================
 * Reset 和 Probe 实现
 * ============================================================================ */

static int py32io_reset_setup(struct py32io_priv *py32io)
{
    // struct i2c_client *client = py32io->i2c;
    // struct device *dev = &client->dev;
    // struct gpio_desc *gpio;

    // gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    // if (IS_ERR(gpio))
    //     return PTR_ERR(gpio);

    // if (gpio) {
    //     dev_info(dev, "Reset pin=%d\n", desc_to_gpio(gpio));
    //     py32io->reset_gpio = gpio;
    // }

    return 0;
}

static void py32io_reset(struct py32io_priv *py32io)
{
    // struct i2c_client *client = py32io->i2c;
    // struct device *dev = &client->dev;

    // if (py32io->reset_gpio) {
    //     dev_info(dev, "Resetting\n");
    //     gpiod_set_value(py32io->reset_gpio, 0);
    //     msleep(PY32IO_RESET_DELAY_MS);
    //     gpiod_set_value(py32io->reset_gpio, 1);
    //     msleep(PY32IO_RESET_DELAY_MS);
    //     gpiod_set_value(py32io->reset_gpio, 0);
    //     msleep(PY32IO_RESET_DELAY_MS);
    // }
}

static int py32io_probe(struct i2c_client *client)
{
    int ret;
    struct device *dev = &client->dev;
    struct py32io_priv *py32io;
    const struct i2c_device_id *id = i2c_client_get_device_id(client);
    int py32io_dev_id = (int)id->driver_data;

    if(py32io_dev_id)
        dev_info(dev, "py32io08 probe()\n");
    else
        dev_info(dev, "py32io16 probe()\n");

    py32io = devm_kzalloc(dev, sizeof(struct py32io_priv), GFP_KERNEL);
    if (!py32io) {
        return -ENOMEM;
    }

    i2c_set_clientdata(client, py32io);
    py32io->i2c = client;
    mutex_init(&py32io->lock);

    py32io->regmap = py32io_setup_regmap(client, py32io_dev_id);
    if (IS_ERR(py32io->regmap)) {
        ret = PTR_ERR(py32io->regmap);
        dev_err(&client->dev, "Failed to init register map: %d\n", ret);
        goto err_mutex;
    }

    ret = py32io_reset_setup(py32io);
    if (ret < 0) {
        dev_err(dev, "failed to configure reset-gpio: %d", ret);
        goto err_mutex;
    }

    py32io_reset(py32io);

    /* 初始化Pinctrl */
    ret = py32io_pinctrl_setup(py32io);
    if (ret < 0) {
        dev_err(dev, "Failed to setup Pinctrl: %d", ret);
        goto err_mutex;
    }

    /* 初始化GPIO */
    ret = py32io_gpio_setup(py32io, py32io_dev_id);
    if (ret < 0) {
        dev_err(dev, "Failed to setup GPIOs: %d", ret);
        goto err_mutex;
    }

    /* 初始化PWM */
    ret = py32io_pwm_setup(py32io, py32io_dev_id);
    if (ret < 0) {
        dev_err(dev, "Failed to setup PWM: %d", ret);
        goto err_mutex;
    }

    /* 初始化ADC */
    ret = py32io_adc_setup(py32io);
    if (ret < 0) {
        dev_err(dev, "Failed to setup ADC: %d", ret);
        goto err_mutex;
    }

    dev_info(dev, "probe finished successfully");
    return 0;

err_mutex:
    mutex_destroy(&py32io->lock);
    return ret;
}

static void py32io_remove(struct i2c_client *client)
{
    struct py32io_priv *py32io = i2c_get_clientdata(client);
    
    mutex_destroy(&py32io->lock);
}

static const struct i2c_device_id py32io_id_table[] = {
    {"py32io16", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, py32io_id_table);

#ifdef CONFIG_OF
static const struct of_device_id py32io_of_match[] = {
    {.compatible = "m5stack,py32io16", .data = (void *)0},
    {},
};
MODULE_DEVICE_TABLE(of, py32io_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id py32io16_acpi_match_table[] = {
    {"PY32IO16", 0},
    {},
};
MODULE_DEVICE_TABLE(acpi, py32io16_acpi_match_table);
#endif

static struct i2c_driver py32io_driver = {
    .driver = {
           .name = "py32io-gpio",
           .of_match_table = of_match_ptr(py32io_of_match),
#ifdef CONFIG_ACPI
           .acpi_match_table = ACPI_PTR(py32io16_acpi_match_table),
#endif
    },
    .probe = py32io_probe,
    .remove = py32io_remove,
    .id_table = py32io_id_table,
};

static int __init py32io_init(void)
{
    return i2c_add_driver(&py32io_driver);
}
subsys_initcall(py32io_init);

static void __exit py32io_exit(void)
{
    i2c_del_driver(&py32io_driver);
}
module_exit(py32io_exit);

MODULE_AUTHOR("dianjixz <dianjixz@m5stack.com>");
MODULE_DESCRIPTION("py32 I2C GPIO expander driver with ADC and Pinctrl");
MODULE_LICENSE("GPL v2");