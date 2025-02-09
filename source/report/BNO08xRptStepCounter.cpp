/**
 * @file BNO08xRptStepCounter.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xRptStepCounter.hpp"

/**
 * @brief Updates step counter data from decoded sensor event.
 *
 * @param sensor_val The sh2_SensorValue_t struct used in sh2_decodeSensorEvent() call.
 *
 * @return void, nothing to return
 */
void BNO08xRptStepCounter::update_data(sh2_SensorValue_t* sensor_val)
{
    static uint16_t prev_steps = 0;

    lock_user_data();
    data = sensor_val->un.stepCounter;

    if (prev_steps > data.steps)
    {
        // rollover detected
        step_accumulator += prev_steps;
        prev_steps = 0UL;
    }

    prev_steps = data.steps;
    data.accuracy = static_cast<BNO08xAccuracy>(sensor_val->status);
    update_timestamp(sensor_val);
    unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(sync_ctx->evt_grp_rpt_en))
        signal_data_available();
}

/**
 * @brief Enables step counter reports such that the BNO08x begins sending them.
 *
 * @param report_period_us The period/interval of the report in microseconds.
 * @param sensor_cfg Sensor special configuration (optional, see
 * BNO08xPrivateTypes::default_sensor_cfg for defaults).
 *
 * @return True if report was successfully enabled.
 */
bool BNO08xRptStepCounter::enable(uint32_t time_between_reports, sh2_SensorConfig_t sensor_cfg)
{
    return BNO08xRpt::rpt_enable(time_between_reports, sensor_cfg);
}

/**
 * @brief Grabs most recent step counter data (rollover not accounted for in step count, just most
 * recent report data).
 *
 * @return Struct containing requested data.
 */
bno08x_step_counter_t BNO08xRptStepCounter::get()
{
    lock_user_data();
    bno08x_step_counter_t rqdata = data;
    unlock_user_data();
    return rqdata;
}

/**
 * @brief Grabs the total step count since boot, accounts for rollover in report data.
 *
 * @return Total steps since boot.
 */
uint32_t BNO08xRptStepCounter::get_total_steps()
{
    lock_user_data();
    uint32_t total_steps = step_accumulator + data.steps;
    unlock_user_data();
    return total_steps;
}