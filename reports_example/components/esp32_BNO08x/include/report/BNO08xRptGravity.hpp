/**
 * @file BNO08xRptGravity.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xRptGravity
 *
 * @brief Class to represent gravity reports. (See Ref. Manual 6.5.11)
 */
class BNO08xRptGravity : public BNO08xRpt
{
    public:
        BNO08xRptGravity(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t* sync_ctx)
            : BNO08xRpt(ID, rpt_bit, sync_ctx)
        {
        }

        bno08x_accel_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_accel_t data;
        static const constexpr char* TAG = "BNO08xRptGravity";
};
