/*
 * Custom nRF VDDH battery driver that maps voltage to state-of-charge
 * using a LiPo discharge-curve LUT instead of the linear approximation
 * used by ZMK's upstream zmk,battery-nrf-vddh driver.
 *
 * Devicetree compatible: customts,battery-nrf-vddh-lut
 */

#define DT_DRV_COMPAT customts_battery_nrf_vddh_lut

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_nrf_vddh_lut, CONFIG_SENSOR_LOG_LEVEL);

#define VDDHDIV (5)

struct lut_entry {
    uint16_t mv;
    uint8_t pct;
};

/* Typical single-cell LiPo curve at low load (≈±5%). Sorted high → low. */
static const struct lut_entry lipo_lut[] = {
    {4200, 100}, {4060, 90}, {3980, 80}, {3920, 70}, {3870, 60}, {3820, 50},
    {3790, 40},  {3770, 30}, {3730, 20}, {3680, 10}, {3450, 0},
};

static uint8_t lipo_mv_to_pct(uint16_t mv) {
    if (mv >= lipo_lut[0].mv) {
        return 100;
    }
    const size_t n = ARRAY_SIZE(lipo_lut);
    if (mv <= lipo_lut[n - 1].mv) {
        return 0;
    }
    for (size_t i = 1; i < n; i++) {
        if (mv >= lipo_lut[i].mv) {
            const struct lut_entry *hi = &lipo_lut[i - 1];
            const struct lut_entry *lo = &lipo_lut[i];
            uint32_t span_mv = hi->mv - lo->mv;
            uint32_t span_pct = hi->pct - lo->pct;
            return lo->pct + (uint32_t)(mv - lo->mv) * span_pct / span_mv;
        }
    }
    return 0;
}

struct vddh_lut_data {
    struct adc_channel_cfg acc;
    struct adc_sequence as;
    uint16_t adc_raw;
    uint16_t millivolts;
    uint8_t state_of_charge;
};

static const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc));

static int vddh_lut_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    if (chan != SENSOR_CHAN_GAUGE_VOLTAGE && chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE &&
        chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }

    struct vddh_lut_data *drv_data = dev->data;
    struct adc_sequence *as = &drv_data->as;

    int rc = adc_read(adc, as);
    as->calibrate = false;

    if (rc != 0) {
        LOG_ERR("Failed to read ADC: %d", rc);
        return rc;
    }

    int32_t val = drv_data->adc_raw;
    rc = adc_raw_to_millivolts(adc_ref_internal(adc), drv_data->acc.gain, as->resolution, &val);
    if (rc != 0) {
        LOG_ERR("Failed to convert raw ADC to mV: %d", rc);
        return rc;
    }

    drv_data->millivolts = (uint16_t)val * VDDHDIV;
    drv_data->state_of_charge = lipo_mv_to_pct(drv_data->millivolts);

    LOG_DBG("ADC raw %d ~ %d mV => %d%% (LUT)", drv_data->adc_raw, drv_data->millivolts,
            drv_data->state_of_charge);

    return 0;
}

static int vddh_lut_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val) {
    const struct vddh_lut_data *drv_data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = drv_data->millivolts / 1000;
        val->val2 = (drv_data->millivolts % 1000) * 1000U;
        return 0;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = drv_data->state_of_charge;
        val->val2 = 0;
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api vddh_lut_api = {
    .sample_fetch = vddh_lut_sample_fetch,
    .channel_get = vddh_lut_channel_get,
};

static int vddh_lut_init(const struct device *dev) {
    struct vddh_lut_data *drv_data = dev->data;

    if (!device_is_ready(adc)) {
        LOG_ERR("ADC device is not ready: %s", adc->name);
        return -ENODEV;
    }

    drv_data->as = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &drv_data->adc_raw,
        .buffer_size = sizeof(drv_data->adc_raw),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    drv_data->acc = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_2,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .input_positive = SAADC_CH_PSELN_PSELN_VDDHDIV5,
    };
    drv_data->as.resolution = 12;
#else
#error "battery_nrf_vddh_lut: only nRF SAADC is supported"
#endif

    const int rc = adc_channel_setup(adc, &drv_data->acc);
    LOG_DBG("VDDHDIV5 setup returned %d", rc);
    return rc;
}

static struct vddh_lut_data vddh_lut_data_inst;

DEVICE_DT_INST_DEFINE(0, &vddh_lut_init, NULL, &vddh_lut_data_inst, NULL, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &vddh_lut_api);
