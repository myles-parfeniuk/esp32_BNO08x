/**
 * @file BNO08xRpt.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// standard library includes
#include <functional>
// in-house includes
#include "BNO08xGlobalTypes.hpp"
#include "BNO08xPrivateTypes.hpp"
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
        bool enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        bool disable(sh2_SensorConfig_t sensor_cfg = BNO08xPrivateTypes::default_sensor_cfg);
        bool register_cb(std::function<void(void)> cb_fxn);
        bool has_new_data();
        bool flush();
        bool get_sample_counts(bno08x_sample_counts_t& sample_counts);
        bool clear_sample_counts();
        bool get_meta_data(bno08x_meta_data_t& meta_data);

        uint8_t ID;          ///< Report ID, ex. SH2_ACCELERATION.
        EventBits_t rpt_bit; ///< Respective enable and data bit for report in BNO08x::evt_grp_report_en and BNO08x::evt_grp_report_data
        uint32_t period_us;  ///< The period/interval of the report in microseconds.

    protected:
        SemaphoreHandle_t* _sh2_HAL_lock;    ///<Mutex to prevent sh2 HAL lib functions from being accessed at same time.
        SemaphoreHandle_t* _data_lock;       ///<Mutex to prevent user from reading data while data_proc_task() updates it, and vice versa.
        EventGroupHandle_t* _evt_grp_rpt_en; ///<Event group for indicating which reports are currently enabled.
        EventGroupHandle_t*
                _evt_grp_rpt_data_available; ///< Event group for indicating to BNO08xRpt::has_new_data() that a module received a new report since the last time it was called (note this group is unaffected by data read through callbacks).
        EventGroupHandle_t* _evt_grp_bno08x_task;              ///<Event group for indicating various BNO08x related events between tasks.
        etl::vector<uint8_t, TOTAL_RPT_COUNT>* _en_report_ids; ///< Vector to contain IDs of currently enabled reports
        BNO08xPrivateTypes::bno08x_cb_list_t* _cb_list;        ///< Vector to contain registered callbacks.

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
        BNO08xRpt(BNO08xPrivateTypes::bno08x_report_info_t info)
            : ID(info.ID)
            , rpt_bit(info.rpt_bit)
            , period_us(0UL)
            , _sh2_HAL_lock(info._sh2_HAL_lock)
            , _data_lock(info._data_lock)
            , _evt_grp_rpt_en(info._evt_grp_rpt_en)
            , _evt_grp_rpt_data_available(info._evt_grp_rpt_data_available)
            , _evt_grp_bno08x_task(info._evt_grp_bno08x_task)
            , _en_report_ids(info._en_report_ids)
            , _cb_list(info._cb_list)

        {
        }

        void unlock_sh2_HAL();
        void lock_sh2_HAL();
        void unlock_user_data();
        void lock_user_data();
        void signal_data_available();

        static const constexpr float RAD_2_DEG =
                (180.0f / M_PI); ///< Constant for radian to degree conversions, sed in quaternion to euler function conversions.

        static const constexpr char* TAG = "BNO08xRpt";

        friend class BNO08x;
};