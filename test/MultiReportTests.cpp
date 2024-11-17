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
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 2 * ENABLED_REPORT_CNT;
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

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling only Linear Accelerometer report and checking for new data on both.");
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

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling Accelerometer report");
    imu->disable_accelerometer();
}

TEST_CASE("Tri Report Enable/Disable", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Tri Report Enable/Disable";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    char msg_buff[200] = {};
    const constexpr uint8_t ENABLED_REPORT_CNT = 3;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 2 * ENABLED_REPORT_CNT;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    bool new_data[ENABLED_REPORT_CNT] = {false, false, false};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(
            TEST_TAG, "Enabling Accelerometer, Linear Accelerometer, and Magnetometer  reports, checking for new data on all.");

    imu->enable_accelerometer(REPORT_PERIOD);
    imu->enable_linear_accelerometer(REPORT_PERIOD);
    imu->enable_magnetometer(REPORT_PERIOD);

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for each report
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_EQUAL(true, new_data[2]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    new_data[2] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling Magnetometer report and checking for new data on remaining reports.");
    imu->disable_magnetometer();

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Failure (REPORT DISABLED): Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for only enabled reports
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[2]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    new_data[2] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling Linear Accelerometer report and checking for new data on remaining reports.");
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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Failure (REPORT DISABLED): Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for only enabled reports
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[1]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[2]);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling Accelerometer report");
    imu->disable_accelerometer();
}

TEST_CASE("Quad Report Enable/Disable", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Quad Report Enable/Disable";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    char msg_buff[200] = {};
    const constexpr uint8_t ENABLED_REPORT_CNT = 4;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 2 * ENABLED_REPORT_CNT;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    bool new_data[ENABLED_REPORT_CNT] = {false, false, false, false};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(
            TEST_TAG, "Enabling Accelerometer, Linear Accelerometer, Magnetometer, and Rotation Vector  reports, checking for new data on all.");

    imu->enable_accelerometer(REPORT_PERIOD);
    imu->enable_linear_accelerometer(REPORT_PERIOD);
    imu->enable_magnetometer(REPORT_PERIOD);
    imu->enable_rotation_vector(REPORT_PERIOD);

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data))
            {
                new_data[3] = true;

                sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                        report_data.quat_J, report_data.quat_K, report_data.quat_real,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for each report
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_EQUAL(true, new_data[2]);
    TEST_ASSERT_EQUAL(true, new_data[3]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    new_data[2] = false;
    new_data[3] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling Rotation Vector report and checking for new data on remaining reports.");
    imu->disable_rotation_vector();

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data))
            {
                new_data[3] = true;

                sprintf(msg_buff, "Rx Data Trial %d Failure (REPORT DISABLED): Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                        report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for only enabled reports
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_EQUAL(true, new_data[2]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[3]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    new_data[2] = false;
    new_data[3] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling Magnetometer report and checking for new data on remaining reports.");
    imu->disable_magnetometer();

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Failure (REPORT DISABLED): Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data))
            {
                new_data[3] = true;

                sprintf(msg_buff, "Rx Data Trial %d Failure (REPORT DISABLED): Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                        report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Disabling Linear Accelerometer report and checking for new data on remaining reports.");
    imu->disable_linear_accelerometer();

    // check that new data was received for only enabled reports
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[2]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[3]);

    // reset all data used in report test
    new_data[0] = false;
    new_data[1] = false;
    new_data[2] = false;
    new_data[3] = false;
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

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
                        "Rx Data Trial %d Failure (REPORT DISABLED): LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Failure (REPORT DISABLED): Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data))
            {
                new_data[3] = true;

                sprintf(msg_buff, "Rx Data Trial %d Failure (REPORT DISABLED): Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                        report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for only enabled reports
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[1]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[2]);
    TEST_ASSERT_NOT_EQUAL(true, new_data[3]);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling Accelerometer report");
    imu->disable_accelerometer();
}

TEST_CASE("Hex Report Enable", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Hex Report Enable";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    char msg_buff[200] = {};
    const constexpr uint8_t ENABLED_REPORT_CNT = 8;
    const constexpr uint8_t RX_REPORT_TRIAL_CNT = 2 * ENABLED_REPORT_CNT;
    const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    bool new_data[ENABLED_REPORT_CNT] = {false, false, false, false, false, false, false, false};

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Enabling Accelerometer, Linear Accelerometer, Magnetometer, Rotation Vector, Uncalibrated Gyro, "
                                               "Calibrated Gyro, Gravity, and Step Counter   "
                                               "reports, checking for new data on all.");

    imu->enable_accelerometer(REPORT_PERIOD);
    imu->enable_linear_accelerometer(REPORT_PERIOD);
    imu->enable_magnetometer(REPORT_PERIOD);
    imu->enable_rotation_vector(REPORT_PERIOD);
    imu->enable_uncalibrated_gyro(REPORT_PERIOD);
    imu->enable_calibrated_gyro(REPORT_PERIOD);
    imu->enable_gravity(REPORT_PERIOD);
    imu->enable_step_counter(REPORT_PERIOD);

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

            if (BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data))
            {
                new_data[2] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data))
            {
                new_data[3] = true;

                sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                        report_data.quat_J, report_data.quat_K, report_data.quat_real,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::uncalibrated_gyro_data_is_new(&report_data, &prev_report_data))
            {
                new_data[4] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: UncalibratedGyro: vX: %.2lf vY: %.2lf vZ: %.2lf driftX: %.2lf driftY: %.2lf driftZ: "
                        "%.2lf",
                        (i + 1), report_data.uncalib_gyro_vel_x, report_data.uncalib_gyro_vel_y, report_data.uncalib_gyro_vel_z,
                        report_data.uncalib_gyro_drift_x, report_data.uncalib_gyro_drift_y, report_data.uncalib_gyro_drift_z);

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::calibrated_gyro_data_is_new(&report_data, &prev_report_data))
            {
                new_data[5] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: CalibratedGyro: vX: %.2lf vY: %.2lf vZ: "
                        "%.2lf",
                        (i + 1), report_data.calib_gyro_vel_x, report_data.calib_gyro_vel_y, report_data.calib_gyro_vel_z);

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::gravity_data_is_new(&report_data, &prev_report_data))
            {
                new_data[6] = true;

                sprintf(msg_buff,
                        "Rx Data Trial %d Success: Gravity: gX: %.2lf gY: %.2lf gZ: "
                        "%.2lf Accuracy: %s",
                        (i + 1), report_data.grav_x, report_data.grav_y, report_data.grav_z,
                        BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.grav_accuracy));

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }

            if (BNO08xTestHelper::step_counter_data_is_new(&report_data, &prev_report_data))
            {
                new_data[7] = true;

                sprintf(msg_buff, "Rx Data Trial %d Success: StepCounter: %d steps", (i + 1), report_data.step_count);

                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
            }
        }
    }

    // check that new data was received for each report
    TEST_ASSERT_EQUAL(true, new_data[0]);
    TEST_ASSERT_EQUAL(true, new_data[1]);
    TEST_ASSERT_EQUAL(true, new_data[2]);
    TEST_ASSERT_EQUAL(true, new_data[3]);
    TEST_ASSERT_EQUAL(true, new_data[4]);
    TEST_ASSERT_EQUAL(true, new_data[5]);
    TEST_ASSERT_EQUAL(true, new_data[6]);
    TEST_ASSERT_EQUAL(true, new_data[7]);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Test completed, disabling all reports.");

    imu->disable_accelerometer();
    imu->disable_linear_accelerometer();
    imu->disable_magnetometer();
    imu->disable_rotation_vector();
    imu->disable_uncalibrated_gyro();
    imu->disable_calibrated_gyro();
    imu->disable_gravity();
    imu->disable_step_counter();
}

TEST_CASE("BNO08x Driver Cleanup for [MultiReportEnableDisable] Tests", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [MultiReportEnableDisable] Tests";
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
}
