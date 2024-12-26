/**
 * @file BNO08xRptCircleDetector.cpp
 * @author Myles Parfeniuk
 */

#define SCALE_Q(n) (1.0f / (1 << n))

#include "BNO08xRptCircleDetector.hpp"
#include "esp_log.h"
#include "math.h"

/**
 * @brief Enables circle detector reports such that the BNO08x begins sending them (only sends reports
 * when a circle gesture is detected).
 *
 * @param time_between_reports The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptCircleDetector::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    static bool configured = false;

    sensor_cfg.changeSensitivity = 0U;
    sensor_cfg.changeSensitivityRelative = true;
    sensor_cfg.changeSensitivityEnabled = true;
    int op_success = SH2_ERR;

    uint32_t settings[3] = {static_cast<uint32_t>(4.0f / SCALE_Q(24)), static_cast<uint32_t>(0.1f / SCALE_Q(30)),
            static_cast<uint32_t>((3.0f * M_PI) / SCALE_Q(25))};

    if (!configured)
    {
        lock_sh2_HAL();
        op_success = sh2_setFrs(CIRCLE_DETECTOR_CONFIG, settings, 0U);
        unlock_sh2_HAL();

        if (op_success != SH2_OK)
        {
            ESP_LOGE(TAG, "%d", op_success);
            return false;
        }
        else
        {
            ESP_LOGE(TAG, "erased");
        }

        lock_sh2_HAL();
        op_success = sh2_setFrs(CIRCLE_DETECTOR_CONFIG, settings, 3U);
        unlock_sh2_HAL();

        if (op_success != SH2_OK)
        {
            ESP_LOGE(TAG, "%d", op_success);
            return false;
        }

        configured = true;
    }

    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Updates circle detector data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptCircleDetector::update_data(sh2_SensorValue_t* sensor_val)
{
    lock_user_data();
    data += sensor_val->un.circleDetector.circle;
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Grabs most recent circle detector detector data.
 *
 * @return Struct containing requested data;
 */
uint16_t BNO08xRptCircleDetector::get()
{
    lock_user_data();
    uint16_t rqdata = data;
    unlock_user_data();
    return rqdata;
}