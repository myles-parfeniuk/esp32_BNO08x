#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

TEST_CASE("BNO08x Driver Creation for [SingleReportEnableDisable] Tests", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Creation for [SingleReportEnableDisable] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
}

TEST_CASE("Enable Incorrect Report", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable Incorrect Report";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable linear accelerometer report but check for angular accelerometer data (should remain as default test values) */
    imu->enable_linear_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from accelerometer has not been rx'd, only linear accelerometer data should have been rx'd
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: AngularAccelDefaults: aX: %.2lf accel aY: %.2lf accel aZ: "
                "%.2lf Accuracy %s",
                (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    imu->disable_linear_accelerometer();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_rotation_vector(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real, BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: QuatDefaults: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Game Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Game Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_game_rotation_vector(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real, BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_game_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: QuatDefaults: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_ARVR_stabilized_rotation_vector(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real, BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_ARVR_stabilized_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: QuatDefaults: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized Game Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized Game Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_ARVR_stabilized_game_rotation_vector(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real, BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_ARVR_stabilized_game_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: QuatDefaults: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gyro Integrated Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gyro Integrated Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_gyro_integrated_rotation_vector(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gyro_integrated_rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: GyroIntegratedRotVector: I: %.2lf J: %.2lf K: %.2lf real: %.2lf gyro vel X: %.2lf gyro vel Y: %.2lf gyro "
                "vel "
                "Z: "
                "%.2lf ",
                (i + 1), report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real, report_data.integrated_gyro_vel_x,
                report_data.integrated_gyro_vel_y, report_data.integrated_gyro_vel_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_gyro_integrated_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gyro_integrated_rotation_vector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: GyroIntegratedRotVectorDefaults: I: %.2lf J: %.2lf K: %.2lf real: %.2lf gyro vel X: %.2lf gyro vel Y: "
                "%.2lf gyro vel "
                "Z: "
                "%.2lf ",
                (i + 1), report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real, report_data.integrated_gyro_vel_x,
                report_data.integrated_gyro_vel_y, report_data.integrated_gyro_vel_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Uncalibrated Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Uncalibrated Gyro";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_uncalibrated_gyro(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::uncalibrated_gyro_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: UncalibratedGyro: vX: %.2lf vY: %.2lf vZ: %.2lf driftX: %.2lf driftY: %.2lf driftZ: "
                "%.2lf",
                (i + 1), report_data.uncalib_gyro_vel_x, report_data.uncalib_gyro_vel_y, report_data.uncalib_gyro_vel_z,
                report_data.uncalib_gyro_drift_x, report_data.uncalib_gyro_drift_y, report_data.uncalib_gyro_drift_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_uncalibrated_gyro();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::uncalibrated_gyro_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: UncalibratedGyroDefaults: vX: %.2lf vY: %.2lf vZ: %.2lf driftX: %.2lf driftY: %.2lf driftZ: "
                "%.2lf",
                (i + 1), report_data.uncalib_gyro_vel_x, report_data.uncalib_gyro_vel_y, report_data.uncalib_gyro_vel_z,
                report_data.uncalib_gyro_drift_x, report_data.uncalib_gyro_drift_y, report_data.uncalib_gyro_drift_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Calibrated Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Calibrated Gyro";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_calibrated_gyro(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::calibrated_gyro_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: CalibratedGyro: vX: %.2lf vY: %.2lf vZ: "
                "%.2lf",
                (i + 1), report_data.calib_gyro_vel_x, report_data.calib_gyro_vel_y, report_data.calib_gyro_vel_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_calibrated_gyro();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::calibrated_gyro_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: CalibratedGyroDefaults: vX: %.2lf vY: %.2lf vZ: "
                "%.2lf",
                (i + 1), report_data.calib_gyro_vel_x, report_data.calib_gyro_vel_y, report_data.calib_gyro_vel_z);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Accelerometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                "%.2lf Accuracy %s",
                (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_accelerometer();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::accelerometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: AngularAccelDefaults: aX: %.2lf accel aY: %.2lf accel aZ: "
                "%.2lf Accuracy %s",
                (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Linear Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Linear Accelerometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_linear_accelerometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::linear_accelerometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_linear_accelerometer();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::linear_accelerometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: LinearAccelDefaults: laX: %.2lf laY: %.2lf laZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gravity", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gravity";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_gravity(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gravity_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: Gravity: gX: %.2lf gY: %.2lf gZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.grav_x, report_data.grav_y, report_data.grav_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.grav_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_gravity();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gravity_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: GravityDefaults: gX: %.2lf gY: %.2lf gZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.grav_x, report_data.grav_y, report_data.grav_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.grav_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Magnetometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Magnetometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_magnetometer(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_magnetometer();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::magnetometer_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: MagfDefaults: mX: %.2lf mY: %.2lf mZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Step Counter", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Step Counter";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_step_counter(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::step_detector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: StepCounter: %d steps", (i + 1), report_data.step_count);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_step_counter();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::step_detector_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: StepCounterDefault: %d steps", (i + 1), report_data.step_count);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Stability Classifier", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Stability Classifier";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_stability_classifier(REPORT_PERIOD);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::stability_classifier_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: StabilityClassifier: %d", (i + 1), report_data.stability_classifier);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_stability_classifier();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::stability_classifier_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: StabilityClassifierDefault: %d", (i + 1), report_data.stability_classifier);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Activity Classifier", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Activity Classifier";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    BNO08xTestHelper::imu_report_data_t prev_report_data;
    bool new_data = false;
    char msg_buff[200] = {};
    uint8_t activity_confidence_vals[9] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
    BNO08xTestHelper::update_report_data(&report_data);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_activity_classifier(5*REPORT_PERIOD, BNO08x::ACTIVITY_CLASSIFIER_ALL_EN, activity_confidence_vals);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::activity_classifier_data_is_new(&report_data, &prev_report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Rx Data Trial %d Success: ActivityClassifier: %d", (i + 1), report_data.activity_classifier);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_activity_classifier();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            prev_report_data = report_data;
            BNO08xTestHelper::update_report_data(&report_data);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::activity_classifier_data_is_new(&report_data, &prev_report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "No Rx Data Trial %d Success: ActivityClassifierDefault: %d", (i + 1), report_data.activity_classifier);

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        BNO08xTestHelper::reset_all_imu_data_to_test_defaults();
        BNO08xTestHelper::update_report_data(&report_data);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("BNO08x Driver Cleanup for [SingleReportEnableDisable] Tests", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [SingleReportEnableDisable] Tests";
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
}
