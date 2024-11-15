#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;

typedef struct imu_report_data_t
{
        uint32_t time_stamp;

        float quat_I;
        float quat_J;
        float quat_K;
        float quat_real;
        IMUAccuracy raw_quat_radian_accuracy;
        IMUAccuracy quat_accuracy;

} imu_report_data_t;

void update_report_data(imu_report_data_t* report_data, BNO08x* imu)
{
    report_data->quat_I = imu->get_quat_I();
    report_data->quat_J = imu->get_quat_J();
    report_data->quat_K = imu->get_quat_K();
    report_data->quat_real = imu->get_quat_real();
    report_data->raw_quat_radian_accuracy = static_cast<IMUAccuracy>(imu->get_raw_quat_radian_accuracy());
    report_data->quat_accuracy = static_cast<IMUAccuracy>(imu->get_quat_accuracy());
}

TEST_CASE("Enable/Disable Rotation Vector", "[ReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Rotation Vector";
    BNO08x* imu = nullptr;
    imu_report_data_t report_data;
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
    update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            if (report_data.quat_I != 0.0f)
                new_data = true;

            if (report_data.quat_J != 0.0f)
                new_data = true;

            if (report_data.quat_K != 0.0f)
                new_data = true;

            if (report_data.quat_real != 1.0f)
                new_data = true;

            // if the accuracy still contains its default value, something has gone wrong, a defined accuracy should be received with every report
            if (report_data.quat_accuracy == IMUAccuracy::UNDEFINED)
                new_data = false;

            if (report_data.raw_quat_radian_accuracy == IMUAccuracy::UNDEFINED)
                new_data = false;
        }

        // assert whether new data was received or not
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        // use "true" argument to force wait for data even if no reports are enabled
        if (imu->data_available(true))
        {
            update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            if (report_data.quat_I != 0.0f)
                new_data = true;

            if (report_data.quat_J != 0.0f)
                new_data = true;

            if (report_data.quat_K != 0.0f)
                new_data = true;

            if (report_data.quat_real != 1.0f)
                new_data = true;

            // if the accuracy does not contain its default value, something has gone wrong, respective report should be disabled
            if (report_data.quat_accuracy != IMUAccuracy::UNDEFINED)
                new_data = true;

            if (report_data.raw_quat_radian_accuracy != IMUAccuracy::UNDEFINED)
                new_data = true;
        }

        // assert that no new data from this report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Game Rotation Vector", "[ReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Game Rotation Vector";
    BNO08x* imu = nullptr;
    imu_report_data_t report_data;
    bool new_data = false;
    char msg_buff[200] = {};

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset all data used in report test
    imu->reset_all_data();
    update_report_data(&report_data, imu);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase started.");
    /*enable respective report to test and ensure it reports new data */
    imu->enable_game_rotation_vector(100000UL);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        if (imu->data_available())
        {
            update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            if (report_data.quat_I != 0.0f)
                new_data = true;

            if (report_data.quat_J != 0.0f)
                new_data = true;

            if (report_data.quat_K != 0.0f)
                new_data = true;

            if (report_data.quat_real != 1.0f)
                new_data = true;

            // if the accuracy still contains its default value, something has gone wrong, a defined accuracy should be received with every report
            if (report_data.quat_accuracy == IMUAccuracy::UNDEFINED)
                new_data = false;

            if (report_data.raw_quat_radian_accuracy == IMUAccuracy::UNDEFINED)
                new_data = false;
        }

        // assert whether new data was received or not
        TEST_ASSERT_EQUAL(true, new_data);

        sprintf(msg_buff, "Enabled Report Rx Data Trial %d Success: I: %.2lf J: %.2lf K: %.2lf real: %.2lf", (i + 1), report_data.quat_I,
                report_data.quat_J, report_data.quat_K, report_data.quat_real);
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report enabled testing phase completed.");

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase started.");
    /*disable respective report to test and ensure it stops reporting new data */
    imu->disable_game_rotation_vector();
    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        new_data = false;

        // use "true" argument to force wait for data even if no reports are enabled
        if (imu->data_available(true))
        {
            update_report_data(&report_data, imu);

            // check if any default values have been overwritten, implying new data from respective report
            if (report_data.quat_I != 0.0f)
                new_data = true;

            if (report_data.quat_J != 0.0f)
                new_data = true;

            if (report_data.quat_K != 0.0f)
                new_data = true;

            if (report_data.quat_real != 1.0f)
                new_data = true;

            // if the accuracy does not contain its default value, something has gone wrong, respective report should be disabled
            if (report_data.quat_accuracy != IMUAccuracy::UNDEFINED)
                new_data = true;

            if (report_data.raw_quat_radian_accuracy != IMUAccuracy::UNDEFINED)
                new_data = true;
        }

        // assert that no new data from this report has been received
        TEST_ASSERT_NOT_EQUAL(true, new_data);

        sprintf(msg_buff, "Disabled Report No Rx Data Trial %d Success", (i + 1));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

        // reset all data used in report test
        imu->reset_all_data();
        update_report_data(&report_data, imu);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Report disabled testing phase completed.");

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}