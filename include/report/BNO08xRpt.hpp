/**
 * @file BNO08xRpt.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// standard library includes
#include <functional>
// esp-idf includes
#include "esp_log.h"
// in-house includes
#include "BNO08xGlobalTypes.hpp"
#include "BNO08xPrivateTypes.hpp"
#include "BNO08xGuard.hpp"
// hill-crest labs includes (apache 2.0 license, compatible with MIT)
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/**
 * @class BNO08xRpt
 *
 * @brief Class to represent and manage reports returned from BNO08x.
 */
class BNO08xRpt
{
    public:
        bool disable(sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        bool register_cb(std::function<void(void)> cb_fxn);
        bool has_new_data();
        bool flush();
        bool get_sample_counts(bno08x_sample_counts_t& sample_counts);
        bool clear_sample_counts();
        bool get_meta_data(bno08x_meta_data_t& meta_data);
        virtual bool enable(
                uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg) = 0;

    protected:
        uint8_t ID;          ///< Report ID, ex. SH2_ACCELERATION.
        EventBits_t rpt_bit; ///< Respective enable and data bit for report in evt_grp_rpt_en and evt_grp_rpt_data
        uint32_t period_us;  ///< The period/interval of the report in microseconds.
        BNO08xPrivateTypes::bno08x_sync_ctx_t &sync_ctx; ///< Holds context used to synchronize tasks and callback execution.

        bool rpt_enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        virtual void update_data(sh2_SensorValue_t* sensor_val) = 0;

        /**
         * @brief BNO08xRpt report constructor.
         *
         * Construct a BNO08xRpt object for managing a BNO08x sensor report.
         *
         * @param imu Pointer to BNO08x imu object.
         * @param report_ID Report ID, ex. SH2_ACCELERATION.
         *  @param rpt_bit Respective enable bit for report in BNO08x::evt_grp_report_en.
         *  @param period_us The period/interval of the report in microseconds.
         *
         * @return void, nothing to return
         */
        BNO08xRpt(uint8_t ID, EventBits_t rpt_bit, BNO08xPrivateTypes::bno08x_sync_ctx_t& sync_ctx)
            : ID(ID)
            , rpt_bit(rpt_bit)
            , period_us(0UL)
            , sync_ctx(sync_ctx)

        {
        }

        void signal_data_available();

        static const constexpr float RAD_2_DEG =
                (180.0f / M_PI); ///< Constant for radian to degree conversions, sed in quaternion to euler function conversions.

        static const constexpr char* TAG = "BNO08xRpt";

        friend class BNO08x;
};