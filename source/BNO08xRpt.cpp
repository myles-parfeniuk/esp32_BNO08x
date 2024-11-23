#include "BNO08xRpt.hpp"
#include "BNO08x.hpp"

/**
 * @brief Enables a sensor report such that the BNO08x begins sending it.
 *
 * @param sensor_ID The ID of the sensor for the respective report to be enabled.
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration.
 *
 * @return ESP_OK if report was successfully enabled.
 */
bool BNO08xRpt::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    EventBits_t evt_grp_report_en_bits = xEventGroupGetBits(imu->evt_grp_report_en);

    // already enabled
    if (!(evt_grp_report_en_bits & rpt_bit))
    {
        if (imu->enable_report(ID, time_between_reports, sensor_cfg) != ESP_OK)
        {
            return false; // Return false if enable_report fails
        }
        else
        {
            period_us = time_between_reports;                    // Update the period
            xEventGroupSetBits(imu->evt_grp_report_en, rpt_bit); // Set the event group bit
        }
    }

    return true;
}

/**
 * @brief Disables a sensor report by setting its period to 0us such that the BNO08x stops sending it.
 *
 * @param sensor_ID The ID of the sensor for the respective report to be disabled.
 * @param sensor_cfg Sensor special configuration.
 *
 * @return ESP_OK if report was successfully disabled.
 */
bool BNO08xRpt::disable(sh2_SensorConfig_t sensor_cfg)
{
    EventBits_t evt_grp_report_en_bits = xEventGroupGetBits(imu->evt_grp_report_en);

    if (evt_grp_report_en_bits & rpt_bit)
    {
        if (imu->enable_report(ID, 0UL, sensor_cfg) != ESP_OK)
            return false;
        else
            xEventGroupClearBits(imu->evt_grp_report_en, rpt_bit); // Set the event group bit
    }

    return true;
}