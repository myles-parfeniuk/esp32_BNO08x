#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;

TEST_CASE("Enable/Disable Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing IMU for Enable/Disable report tests.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Game Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Game Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_game_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_ARVR_stabilized_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized Game Rotation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized Game Rotation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_ARVR_stabilized_game_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf Accuracy: %s", (i + 1),
                report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.quat_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::rotation_vector_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gyro Integrated Roation Vector", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gyro Integrated Roation Vector";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_gyro_integrated_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gyro_integrated_rotation_vector_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: Quat: I: %.2lf J: %.2lf K: %.2lf real: %.2lf gyro vel X: %.2lf gyro vel Y: %.2lf gyro vel "
                "Z: "
                "%.2lf ",
                (i + 1), report_data.quat_I, report_data.quat_J, report_data.quat_K, report_data.quat_real, report_data.integrated_gyro_vel_x,
                report_data.integrated_gyro_vel_y, report_data.integrated_gyro_vel_z);
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gyro_integrated_rotation_vector_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Uncalibrated Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Uncalibrated Gyro";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_uncalibrated_gyro(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::uncalibrated_gyro_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: Uncalib Gyro: vX: %.2lf vY: %.2lf vZ: %.2lf driftX: %.2lf driftY: %.2lf driftZ: "
                "%.2lf  Accuracy: %s",
                (i + 1), report_data.uncalib_gyro_vel_x, report_data.uncalib_gyro_vel_y, report_data.uncalib_gyro_vel_z,
                report_data.uncalib_gyro_drift_x, report_data.uncalib_gyro_drift_y, report_data.uncalib_gyro_drift_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.uncalib_gyro_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::uncalibrated_gyro_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Calibrated Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Calibrated Gyro";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_calibrated_gyro(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::calibrated_gyro_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: Calibrated Gyro: vX: %.2lf vY: %.2lf vZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.calib_gyro_vel_x, report_data.calib_gyro_vel_y, report_data.calib_gyro_vel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.calib_gyro_accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::calibrated_gyro_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Accelerometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_accelerometer(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::accelerometer_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: AngularAccel: aX: %.2lf accel aY: %.2lf accel aZ: "
                "%.2lf Accuracy %s",
                (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::accelerometer_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Linear Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Linear Accelerometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_linear_accelerometer(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::linear_accelerometer_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: LinearAccel: laX: %.2lf laY: %.2lf laZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.lin_accel_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::linear_accelerometer_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gravity", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gravity";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_gravity(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gravity_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: Gravity: gX: %.2lf gY: %.2lf gZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.grav_x, report_data.grav_y, report_data.grav_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.grav_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::gravity_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Magnetometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Magnetometer";
    BNO08x* imu = nullptr;
    BNO08xTestHelper::imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    BNO08xTestHelper::update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_magnetometer(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::magnetometer_data_is_default(&report_data);
        }

        // assert that new data from respective report has been received
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff,
                "Enabled Report Rx Data Trial %d Success: Magf: mX: %.2lf mY: %.2lf mZ: "
                "%.2lf Accuracy: %s",
                (i + 1), report_data.magf_x, report_data.magf_y, report_data.magf_z,
                BNO08xTestHelper::BNO08xAccuracy_to_str(report_data.magf_accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
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
            BNO08xTestHelper::update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            new_data = BNO08xTestHelper::magnetometer_data_is_default(&report_data);
        }

        // assert that no new data from respective report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        BNO08xTestHelper::update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}