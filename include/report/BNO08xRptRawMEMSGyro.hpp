#pragma once

#include "BNO08xRpt.hpp"

/**
 * @brief Class to represent raw gyro reports. (See Ref. Manual 6.5.12)
 */
class BNO08xRptRawMEMSGyro : public BNO08xRpt
{
    public:
        BNO08xRptRawMEMSGyro(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : BNO08xRpt(imu, report_ID, period_us, rpt_bit)
        {
        }

        bno08x_raw_gyro_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_raw_gyro_t data;
        static const constexpr char* TAG = "BNO08xRptRawMEMSGyro";
};
