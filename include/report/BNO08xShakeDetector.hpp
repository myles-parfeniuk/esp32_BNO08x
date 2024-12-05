/**
 * @file BNO08xShakeDetector.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xRpt.hpp"

/**
 * @class BNO08xShakeDetector
 *
 * @brief Class to represent shake detector reports. (See Ref. Manual 6.5.32)
 */
class BNO08xShakeDetector : public BNO08xRpt
{
    public:
        BNO08xShakeDetector(BNO08xPrivateTypes::bno08x_report_info_t info)
            : BNO08xRpt(info)
        {
        }

        bno08x_shake_detector_t get();

    private:
        void update_data(sh2_SensorValue_t* sensor_val) override;
        bno08x_shake_detector_t data;
        static const constexpr char* TAG = "BNO08xShakeDetector";
};
