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

TEST_CASE("Driver Creation for [FeatureTests] Tests", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [FeatureTests] Tests";
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
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
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
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Soft Reset", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Soft Reset";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
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
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Sleep", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Sleep";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
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
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Get Metadata", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Get Metadata";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_meta_data_t meta_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
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
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Set and Get System Orientation", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Set System Orientation";
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms
    BNO08x* imu = nullptr;
    imu = BNO08xTestHelper::get_test_imu();
    char msg_buff[200] = {};
    float Qw = 1.0f;
    float Qx = 0.0f;
    float Qy = 0.0f;
    float Qz = 0.0f;
    float epsilon = 0.000001f;

    // 1. enable report as per FlyteSailing's instructions
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 2. set the system orientation to a value defined in figure 4-3 of the data sheet (X == West Aligned, Y == West Aligned, Z
    // == Up)
    sprintf(msg_buff, "Setting orientation to: Qw: %.6f Qx: %.6f Qy: %.6f Qz: %.6f", Qw, Qx, Qy, Qz);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    TEST_ASSERT_EQUAL(true, imu->set_system_orientation(Qw, Qx, Qy, Qz));

    // 3. hard reset to apply changes and verify they are persistent within flash
    TEST_ASSERT_EQUAL(true, imu->hard_reset());

    // 4. read back the system orientation frs and verify it contains the orientation we wrote
    TEST_ASSERT_EQUAL(true, imu->get_system_orientation(Qw, Qx, Qy, Qz));
    sprintf(msg_buff, "Read back orientation: Qw: %.6f Qx: %.6f Qy: %.6f Qz: %.6f", Qw, Qx, Qy, Qz);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    // check that values match expected
    TEST_ASSERT_EQUAL(true, fabs(1.0 - Qw) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qx) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qy) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qz) < epsilon);

    // 5. reset the system orientation to default (all 0.0f)
    Qw = 0.0f;
    sprintf(msg_buff, "Re-setting orientation to: Qw: %.6f Qx: %.6f Qy: %.6f Qz: %.6f", Qw, Qx, Qy, Qz);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    TEST_ASSERT_EQUAL(true, imu->set_system_orientation(Qw, Qx, Qy, Qz));

    // 6. hard reset to apply changes and verify they are persistent within flash
    TEST_ASSERT_EQUAL(true, imu->hard_reset());

    // 7. read back the system orientation frs and verify it has been returned to default
    TEST_ASSERT_EQUAL(true, imu->get_system_orientation(Qw, Qx, Qy, Qz));
    sprintf(msg_buff, "Read back orientation after reset: Qw: %.6f Qx: %.6f Qy: %.6f Qz: %.6f", Qw, Qx, Qy, Qz);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    // check that values match expected
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qw) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qx) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qy) < epsilon);
    TEST_ASSERT_EQUAL(true, fabs(0.0 - Qz) < epsilon);

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Get Sample Counts", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Get Sample Counts";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_accel_t data_accel;
    bno08x_sample_counts_t sample_counts;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.has_new_data());
        data_accel = imu->rpt.accelerometer.get();
        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data_accel.x,
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
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
                data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Enable Dynamic Calibration";
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_quat_t quat_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD_US));
    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_enable(BNO08xCalSel::all));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.has_new_data());
        quat_data = imu->rpt.rv_game.get_quat();
        sprintf(msg_buff, "Rx Data Trial %d Success: quat_data: i: %.3f j: %.3f k: %.3f real: %.3f Accuracy: %s", (i + 1),
                quat_data.i, quat_data.j, quat_data.k, quat_data.real, BNO08xAccuracy_to_str(quat_data.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Save Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Save Dynamic Calibration";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_quat_t quat_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_save());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.has_new_data());
        quat_data = imu->rpt.rv_game.get_quat();
        sprintf(msg_buff, "Rx Data Trial %d Success: quat_data: i: %.3f j: %.3f k: %.3f real: %.3f Accuracy: %s", (i + 1),
                quat_data.i, quat_data.j, quat_data.k, quat_data.real, BNO08xAccuracy_to_str(quat_data.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Autosave Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Autosave Dynamic Calibration";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 200;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_quat_t quat_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_autosave_enable());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.has_new_data());
        quat_data = imu->rpt.rv_game.get_quat();
        sprintf(msg_buff, "Rx Data Trial %d Success: quat_data: i: %.3f j: %.3f k: %.3f real: %.3f Accuracy: %s", (i + 1),
                quat_data.i, quat_data.j, quat_data.k, quat_data.real, BNO08xAccuracy_to_str(quat_data.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_autosave_disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Disable Dynamic Calibration", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Disable Dynamic Calibration";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_quat_t quat_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_disable(BNO08xCalSel::all));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.has_new_data());
        quat_data = imu->rpt.rv_game.get_quat();
        sprintf(msg_buff, "Rx Data Trial %d Success: quat_data: i: %.3f j: %.3f k: %.3f real: %.3f Accuracy: %s", (i + 1),
                quat_data.i, quat_data.j, quat_data.k, quat_data.real, BNO08xAccuracy_to_str(quat_data.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Clear Dynamic Calibration Data Ram", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Clear Dynamic Calibration Data Ram";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bno08x_quat_t quat_data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_data_clear_ram());

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        TEST_ASSERT_EQUAL(true, imu->data_available());
        TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.has_new_data());
        quat_data = imu->rpt.rv_game.get_quat();
        sprintf(msg_buff, "Rx Data Trial %d Success: quat_data: i: %.3f j: %.3f k: %.3f real: %.3f Accuracy: %s", (i + 1),
                quat_data.i, quat_data.j, quat_data.k, quat_data.real, BNO08xAccuracy_to_str(quat_data.accuracy));
        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Delete Dynamic Calibration Data", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Delete Dynamic Calibration Data";

    BNO08x* imu = nullptr;
    static char msg_buff[200] = {0};
    static uint32_t frs_data[16] = {0};
    uint16_t words_rxd = 0U;
    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1. read the DCD data from FRS before we delete it and ensure it is non-0 (should be set by previous test)
    TEST_ASSERT_EQUAL(true, imu->get_frs(BNO08xFrsID::DYNAMIC_CALIBRATION, frs_data, words_rxd));
    sprintf(msg_buff, "DCD before delete: %ld", frs_data[0]);
    TEST_ASSERT_EQUAL(true, (frs_data[0] != 0));
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

    // 2. run FlyteSailing's delete DCD fxn and ensure DCD is now 0
    TEST_ASSERT_EQUAL(true, imu->dynamic_calibration_data_delete());
    TEST_ASSERT_EQUAL(true, imu->get_frs(BNO08xFrsID::DYNAMIC_CALIBRATION, frs_data, words_rxd));
    sprintf(msg_buff, "DCD after delete: %ld", frs_data[0]);
    TEST_ASSERT_EQUAL(true, (frs_data[0] == 0));
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Cleanup for [FeatureTests] Tests", "[FeatureTests]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [FeatureTests] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}