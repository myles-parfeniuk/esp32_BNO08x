#pragma once

#include <stdint.h>
#include <functional>
#include "BNO08x_global_types.hpp"

// forward dec to prevent compile errors
class BNO08x;

/**
 * @brief Class to represent and manage reports returned from BNO08x.
 *
 * @return ESP_OK if report was successfully enabled.
 */
class BNO08xRpt
{
    public:
        inline static sh2_SensorConfig default_sensor_cfg = {.changeSensitivityEnabled = false, ///<TODO
                .changeSensitivityRelative = false,
                .wakeupEnabled = false,
                .alwaysOnEnabled = false,
                .changeSensitivity = 0,
                .reportInterval_us = 0,
                .batchInterval_us = 0,
                .sensorSpecific = 0};

        bool enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        bool disable(sh2_SensorConfig_t sensor_cfg = default_sensor_cfg);
        void register_cb(std::function<void(void)> cb_fxn);
        bool has_new_data();
        bool flush();
        bool get_sample_counts(bno08x_sample_counts_t& sample_counts);
        bool clear_sample_counts();
        bool get_meta_data(bno08x_meta_data_t& meta_data);

    protected:
        BNO08x* imu;        ///< Pointer to BNO08x object for enable ,disable, and update operations.
        uint8_t ID;         ///< Report ID, ex. SH2_ACCELERATION.
        uint32_t rpt_bit;   ///< Respective enable and data bit for report in BNO08x::evt_grp_report_en and BNO08x::evt_grp_report_data
        uint32_t period_us; ///< The period/interval of the report in microseconds.

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
        BNO08xRpt(BNO08x* imu, uint8_t report_ID, uint32_t period_us, uint32_t rpt_bit)
            : imu(imu)
            , ID(report_ID)
            , rpt_bit(rpt_bit)
            , period_us(period_us)

        {
        }

        void signal_data_available();

        static const constexpr float RAD_2_DEG =
                (180.0f / M_PI); ///< Constant for radian to degree conversions, sed in quaternion to euler function conversions.

        static const constexpr char* TAG = "BNO08xRpt";

        friend class BNO08x;
};