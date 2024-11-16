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

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
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

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
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

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
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

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
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
                "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf gyro vel X: %.2lf gyro vel Y: %.2lf gyro vel Z: "
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
                "Enabled Report Rx Data Trial %d Success: accel X: %.2lf accel Y: %.2lf accel Z: "
                "%.2lf ",
                (i + 1), report_data.accel_x, report_data.accel_y, report_data.accel_z);
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
                "Enabled Report Rx Data Trial %d Success: lin accel X: %.2lf lin accel Y: %.2lf lin accel Z: "
                "%.2lf ",
                (i + 1), report_data.lin_accel_x, report_data.lin_accel_y, report_data.lin_accel_z);
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
                "Enabled Report Rx Data Trial %d Success: grav X: %.2lf grav Y: %.2lf grav Z: "
                "%.2lf ",
                (i + 1), report_data.grav_x, report_data.grav_y, report_data.grav_z);
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


