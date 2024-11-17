#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("BNO08x Driver Creation for [CallbackTests] Tests", "[CallbackTests]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Creation for [CallbackTests] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
}

TEST_CASE("Callback Receives New Data (Single Report)", "[CallbackTests]")
{
    const constexpr char* TEST_TAG = "Callback Receives New Data (Single Report)";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms
    bool new_data = false;
    char msg_buff[200] = {};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    // register a callback and check for new data within it
    imu->register_cb(
            [&imu, &new_data, &report_data, &prev_report_data, &msg_buff]()
            {
                static int cb_execution_cnt = 0;

                /* callback code */

                cb_execution_cnt++;
                // check if new data was received from enabled report(s)
                BNO08xTestHelper::update_report_data(&report_data);

                if (BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data))
                {
                    new_data = true;

                    sprintf(msg_buff,
                            "Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                            "%.2lf Accuracy %s",
                            cb_execution_cnt, report_data.accel_x, report_data.accel_y, report_data.accel_z,
                            BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

                    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                }
            });

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Enabling report(s) and checking for new data through subscribed callback.");
    imu->enable_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        if (imu->data_available())
        {
            // callbacks should ALWAYS execute before data_available returns true, therefore something has gone wrong if new_data is not set to true
            TEST_ASSERT_EQUAL(true, new_data);

            // reset all data used in report test
            new_data = false;
            BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
            BNO08xTestHelper::update_report_data(&report_data);
        }
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test complete, disabling report(s).");
    imu->disable_accelerometer();
}

TEST_CASE("Callback Receives New Data (Dual Report)", "[CallbackTests]")
{
    const constexpr char* TEST_TAG = "Callback Receives New Data (Dual Report)";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    const constexpr uint8_t ENABLED_REPORT_CNT = 2;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 2 * ENABLED_REPORT_CNT;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms
    bool new_data[2] = {false, false};
    char msg_buff[200] = {};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    // register a callback and check for new data within it
    imu->register_cb(
            [&imu, &new_data, &report_data, &prev_report_data, &msg_buff]()
            {
                static int cb_execution_cnt = 0;

                /* callback code */

                cb_execution_cnt++;
                // check if new data was received from enabled report(s)
                BNO08xTestHelper::update_report_data(&report_data);

                if (BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data))
                {
                    new_data[0] = true;

                    sprintf(msg_buff,
                            "Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                            "%.2lf Accuracy %s",
                            cb_execution_cnt, report_data.accel_x, report_data.accel_y, report_data.accel_z,
                            BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

                    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                }

                if (BNO08xTestHelper::linear_accelerometer_data_is_new(&report_data, &prev_report_data))
                {
                    new_data[1] = true;

                    sprintf(msg_buff,
                            "Rx Data Trial %d Success: LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                            "%.2lf Accuracy: %s",
                            (cb_execution_cnt + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                            BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

                    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                }
            });

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Enabling report(s) and checking for new data through subscribed callback.");
    imu->enable_accelerometer(REPORT_PERIOD);
    imu->enable_linear_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        if (imu->data_available())
        {
            // callbacks should ALWAYS execute before data_available returns true
            BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
            BNO08xTestHelper::update_report_data(&report_data);
        }
    }

    // check that the callback received new data for all reports after all trials have completed
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test complete, disabling report(s).");
    imu->disable_accelerometer();
    imu->disable_linear_accelerometer();
}

TEST_CASE("BNO08x Driver Cleanup for [CallbackTests] Tests", "[CallbackTests]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [CallbackTests] Tests";
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
}
