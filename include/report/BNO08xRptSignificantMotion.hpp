/**
 * @file BNO08xRptSignificantMotion.hpp
 * @author Extended from Myles Parfeniuk's work
 * 
 * @brief Significant Motion Detector report class.
 * 
 * The Significant Motion Detector is a "one-shot" sensor that triggers when significant 
 * motion is detected. Once triggered, it automatically disables itself. This makes it
 * ideal for wake-on-motion scenarios where the host can sleep and only wake when the 
 * device is moved.
 * 
 * See BNO08x Datasheet Section 2.4.6 for more details.
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptSignificantMotion
 *
 * @brief Class to represent significant motion detector reports. (See Ref. Manual 6.5.30)
 *
 * This is a one-shot sensor that triggers when significant motion is detected.
 * After triggering, it automatically disables itself and must be re-enabled.
 *
 * Significant motion detection sends a single report when motion is detected,
 * then the sensor disables itself. Call enable() again to re-arm the detector.
 */
class BNO08xRptSignificantMotion : public BNO08xRpt
{
    public:
        BNO08xRptSignificantMotion(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        /**
         * @brief Enables significant motion detector reports.
         *
         * This arms the one-shot significant motion detector. Once significant motion
         * is detected, the sensor will send one report and automatically disable itself.
         *
         * @param time_between_reports The period/interval of the report in microseconds.
         *        For significant motion, this typically doesn't affect detection behavior
         *        since it's a one-shot event-based sensor.
         * @param sensor_cfg Sensor special configuration (optional).
         *
         * @return True if report was successfully enabled.
         */
        bool enable(uint32_t time_between_reports,
                    sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg) override;
        /**
         * @brief Get the motion detection value from the most recent report.
         *
         * @return Motion detection value (typically 1 when motion detected, 0 otherwise).
         */
         bno08x_significant_motion_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_significant_motion_t data;
        static const constexpr char* TAG = "BNO08xRptSignificantMotion";
};

