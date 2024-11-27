/**
 * @file BNO08xRptIGyroRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @brief Class to represent integrated gyro rotation vector reports. (See Ref. Manual 6.5.44)
 */
class BNO08xRptIGyroRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptIGyroRV(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRptRVGeneric(imu, report_ID, period_us, rpt_bit)
        {
        }

        void get(bno08x_quat_t& quat, bno08x_ang_vel_t& vel);
        bno08x_ang_vel_t get_vel();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_ang_vel_t data_vel;
        static const constexpr char* TAG = "BNO08xRptIGyroRV";
};
