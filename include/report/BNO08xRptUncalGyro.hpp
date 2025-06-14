/**
 * @file BNO08xRptUncalGyro.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptUncalGyro
 *
 * @brief Class to represent uncalibrated gyro reports. (See Ref. Manual 6.5.14)
 */
class BNO08xRptUncalGyro : public BNO08xRpt
{
    public:
        BNO08xRptUncalGyro(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bool enable(
                uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg) override;
        void get(bno08x_gyro_t& vel, bno08x_gyro_bias_t& bias);
        bno08x_gyro_t get_vel();
        bno08x_gyro_bias_t get_bias();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_gyro_t data;
        bno08x_gyro_bias_t bias_data;
        static const constexpr char* TAG = "BNO08xRptUncalGyro";
};
