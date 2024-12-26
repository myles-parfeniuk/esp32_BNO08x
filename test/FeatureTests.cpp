/**
 * @file FeatureTets.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("BNO08x Driver Creation for [FeatureTests] Tests", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Creation for [FeatureTests] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Hard Reset", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Hard Reset";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Hard reseting...");
    TEST_ASSERT_EQUAL(true, imu->hard_reset());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Soft Reset", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Soft Reset";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Soft reseting...");
    TEST_ASSERT_EQUAL(true, imu->soft_reset());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Sleep", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Sleep";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->sleep());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        sprintf(msg_buff, "Sleeping... %ds", RX_REPORT_TRIAL_CNT - i);
        vTaskDelay(1000UL / portTICK_PERIOD_MS);
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->on());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Woke.");

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Get Metadata", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Get Metadata";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_meta_data_t meta_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Getting meta data...");
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.get_meta_data(meta_data));

    sprintf(msg_buff,
            "Re-enabling report with fastest possible period of %ldus from accelerometer meta "
            "data.",
            meta_data.min_period_us);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(meta_data.min_period_us));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Get Sample Counts", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Get Sample Counts";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Getting sample counts...");
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.get_sample_counts(sample_counts));
    sprintf(msg_buff, "offered: %ld on: %ld accepted: %ld attempted: %ld", sample_counts.offered, sample_counts.on,
            sample_counts.accepted, sample_counts.attempted);

    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Enable Dynamic Calibration";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_enable(BNO08xCalSel::all));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }
}

TEST_CASE("Save Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Save Dynamic Calibration";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->save_dynamic_calibration());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }
}

TEST_CASE("Autosave Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Autosave Dynamic Calibration";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 200;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_autosave_enable());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_autosave_disable());
}

TEST_CASE("Disable Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Disable Dynamic Calibration";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_disable(BNO08xCalSel::all));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }
}

TEST_CASE("Clear Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Clear Dynamic Calibration";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->clear_dynamic_calibration());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08x::accuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
}

TEST_CASE("BNO08x Driver Cleanup for [FeatureTests] Tests", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [FeatureTests] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}