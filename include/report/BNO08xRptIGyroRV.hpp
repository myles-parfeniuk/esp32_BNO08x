/**
 * @file BNO08xRptIGyroRV.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRptRVGeneric.hpp"

/**
 * @class BNO08xRptIGyroRV
 *
 * @brief Class to represent integrated gyro rotation vector reports. (See Ref. Manual 6.5.44)
 */
class BNO08xRptIGyroRV : public BNO08xRptRVGeneric
{
    public:
        BNO08xRptIGyroRV(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx)
            : BNO08xRptRVGeneric(ID, rpt_bit, sync_ctx)
        {
        }

        void get(bno08x_quat_t& quat, bno08x_ang_vel_t& vel);
        bno08x_ang_vel_t get_vel();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_ang_vel_t data_vel;
        static const constexpr char* TAG = "BNO08xRptIGyroRV";
};
