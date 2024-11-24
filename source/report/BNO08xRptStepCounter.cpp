#include "BNO08xRptStepCounter.hpp"
#include "BNO08x.hpp"

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

    imu->lock_user_data();
    data = sensor_val->un.stepCounter;

    if (prev_steps > data.steps)
    {
        // rollover detected
        step_accumulator += prev_steps;
        prev_steps = 0UL;
    }

    prev_steps = data.steps;
    imu->unlock_user_data();

    if (rpt_bit & xEventGroupGetBits(imu->evt_grp_report_en))
        signal_data_available();
}

/**
 * @brief Grabs the total step count since boot, accounts for rollover in report data.
 *
 * @return Total steps since boot.
 */
uint32_t BNO08xRptStepCounter::get_total_steps()
{
    imu->lock_user_data();
    uint32_t total_steps = step_accumulator + data.steps;
    imu->unlock_user_data();
    return total_steps;
}

/**
 * @brief Grabs most recent step counter data (rollover not accounted for in step count, just most recent report data).
 *
 * @return Struct containing requested data.
 */
bno08x_step_counter_t BNO08xRptStepCounter::get()
{
    imu->lock_user_data();
    bno08x_step_counter_t rqdata = data;
    imu->unlock_user_data();
    return rqdata;
}