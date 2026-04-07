/*
 * Copyright (c) 2026 Valerio Setti
 * SPDX-License-Identifier: MIT
 *
 * Analog joystick input driver for the Moustick board.
 *
 * Reads two ADC channels (X and Y axes), applies a deadzone around the center
 * (2048 for a 12-bit ADC), and emits INPUT_EV_REL events scaled to [-127, 127].
 * This gives velocity-based cursor movement: deflect further = faster cursor.
 *
 * DT binding: compatible = "valerio,moustick-joystick"
 */

#define DT_DRV_COMPAT valerio_moustick_joystick

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/io-channels.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_REGISTER(moustick_joystick, CONFIG_INPUT_LOG_LEVEL);

struct joystick_config {
    const struct device *adc_dev;
    uint8_t channel_x;
    uint8_t channel_y;
    uint16_t deadzone;
    uint16_t max_speed;
    uint32_t poll_period_ms;
    uint8_t invert_x;
    uint8_t invert_y;
};

struct joystick_data {
    const struct device *dev;
    struct k_work_delayable work;
    uint16_t center_x;
    uint16_t center_y;
    int16_t scale_x;
    int16_t scale_y;
};

/*
 * Map a raw 12-bit ADC value to a signed velocity in [-127, 127].
 * Values within `deadzone` of center (2048) report 0.
 */
static int16_t adc_to_velocity(uint16_t value, uint16_t center, const struct joystick_config *cfg) {
    int16_t deadzone = (int16_t)cfg->deadzone;
    int16_t max_speed = (int16_t)cfg->max_speed;
    int16_t deflection = (int16_t)(value - center);

    if (deflection > -((int16_t)deadzone) && deflection < ((int16_t)deadzone)) {
        return 0;
    }

    int16_t magnitude = (deflection > 0) ? deflection - cfg->deadzone : deflection + deadzone;
    int16_t range = center - deadzone;
    int32_t velocity = (magnitude * max_speed) / range;

    return (int16_t)CLAMP(velocity, -127, 127);
}

static inline int joystick_read_axis_single(const struct device *adc_dev, uint8_t adc_channel,
                                            uint16_t *val) {
    struct adc_sequence seq = {
        .channels = BIT(adc_channel),
        .buffer = val,
        .buffer_size = sizeof(*val),
        .resolution = 12,
    };
    return adc_read(adc_dev, &seq);
}

#define AVG_READ_COUNT 16
#define MS_WAIT_BETWEEN_READS 5
static int joystick_read_axis_average(const struct device *adc_dev, uint8_t adc_channel,
                                      uint16_t *val) {
    uint16_t tmp;
    uint32_t total = 0;
    int ret;

    for (int i = 0; i < AVG_READ_COUNT; i++) {
        ret = joystick_read_axis_single(adc_dev, adc_channel, &tmp);
        if (ret < 0) {
            return ret;
        }
        k_busy_wait(1000 * MS_WAIT_BETWEEN_READS);
        total += (uint32_t)tmp;
    }

    *val = (uint16_t)(total / AVG_READ_COUNT);

    return 0;
}

static void joystick_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct joystick_data *data = CONTAINER_OF(dwork, struct joystick_data, work);
    const struct device *dev = data->dev;
    const struct joystick_config *cfg = dev->config;
    uint16_t x_val, y_val;
    int ret;

    ret = joystick_read_axis_single(cfg->adc_dev, cfg->channel_x, &x_val);
    if (ret != 0) {
        LOG_ERR("ADC X read error: %u", ret);
        goto exit;
    }

    ret = joystick_read_axis_single(cfg->adc_dev, cfg->channel_y, &y_val);
    if (ret != 0) {
        LOG_ERR("ADC Y read error: %u", ret);
        goto exit;
    }

    int16_t vx = adc_to_velocity(x_val, data->center_x, cfg) * data->scale_x;
    int16_t vy = adc_to_velocity(y_val, data->center_y, cfg) * data->scale_y;

    if (vx != 0) {
        input_report_rel(dev, INPUT_REL_X, vx, (vy == 0), K_FOREVER);
    }
    if (vy != 0) {
        input_report_rel(dev, INPUT_REL_Y, vy, true, K_FOREVER);
    }

exit:
    k_work_reschedule(&data->work, K_MSEC(cfg->poll_period_ms));
}

static int joystick_init(const struct device *dev) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;
    uint16_t val;
    int ret;

    data->dev = dev;

    if (!device_is_ready(cfg->adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    struct adc_channel_cfg ch_x_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = cfg->channel_x,
        .differential = 0,
    };

    ret = adc_channel_setup(cfg->adc_dev, &ch_x_cfg);
    if (ret < 0) {
        LOG_ERR("ADC X channel setup failed: %d", ret);
        return ret;
    }

    struct adc_channel_cfg ch_y_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = cfg->channel_y,
        .differential = 0,
    };
    ret = adc_channel_setup(cfg->adc_dev, &ch_y_cfg);
    if (ret < 0) {
        LOG_ERR("ADC Y channel setup failed: %d", ret);
        return ret;
    }

    ret = joystick_read_axis_average(cfg->adc_dev, cfg->channel_x, &val);
    if (ret < 0) {
        LOG_ERR("Error reading center X value: %d", ret);
        return ret;
    }
    data->center_x = val;

    ret = joystick_read_axis_average(cfg->adc_dev, cfg->channel_y, &val);
    if (ret < 0) {
        LOG_ERR("Error reading center Y value: %d", ret);
        return ret;
    }
    data->center_y = val;

    data->scale_x = (cfg->invert_x) ? -1 : 1;
    data->scale_y = (cfg->invert_y) ? -1 : 1;

    k_work_init_delayable(&data->work, joystick_work_handler);
    k_work_reschedule(&data->work, K_MSEC(cfg->poll_period_ms));

    return 0;
}

#define MOUSTICK_JOYSTICK_INST(n)                                                                  \
    static struct joystick_data joystick_data_##n;                                                 \
    static const struct joystick_config joystick_cfg_##n = {                                       \
        .adc_dev = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_NAME(n, x)),                          \
        .channel_x = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, x),                                      \
        .channel_y = DT_INST_IO_CHANNELS_INPUT_BY_NAME(n, y),                                      \
        .deadzone = DT_INST_PROP(n, deadzone),                                                     \
        .max_speed = DT_INST_PROP(n, max_speed),                                                   \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
        .invert_x = DT_INST_NODE_HAS_PROP(n, invert_x),                                            \
        .invert_y = DT_INST_NODE_HAS_PROP(n, invert_y),                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, joystick_init, NULL, &joystick_data_##n, &joystick_cfg_##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MOUSTICK_JOYSTICK_INST)
