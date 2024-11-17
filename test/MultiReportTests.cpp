#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("BNO08x Driver Creation for [MultiReportEnableDisable] Tests", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Creation for [MultiReportEnableDisable] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
}

TEST_CASE("Dual Report Enable/Disable", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Dual Report Enable/Disable";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    char msg_buff[200] = {};
    const constexpr uint8_t ENABLED_REPORT_CNT = 2;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_CNT + 1;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    bool new_data[ENABLED_REPORT_CNT] = {false, false};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Enabling Accelerometer and Linear Accelerometer reports, checking for new data on both.");

    imu->enable_accelerometer(REPORT_PERIOD);
    imu->enable_linear_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            if (BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[0] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                        "%.2lf Accuracy %s",
                        (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::linear_accelerometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[1] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for each report
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling only Linear Accelerometer report and checking for new data on both.");
    imu->disable_linear_accelerometer();

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            if (BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[0] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                        "%.2lf Accuracy %s",
                        (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::linear_accelerometer_data_is_new(&report_data, &prev_report_data))
            {
                // no new data should be detected here, report is disabled.
                new_data[1] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Failure (REPORT DISABLED): LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received only for accelerometer report
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[1]);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling Accelerometer and Linear Accelerometer report");
    imu->disable_accelerometer();
}

TEST_CASE("BNO08x Driver Cleanup for [MultiReportEnableDisable] Tests", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [MultiReportEnableDisable] Tests";
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
}
