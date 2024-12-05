/**
 * @file SingleReportTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("BNO08x Driver Creation for [SingleReportEnableDisable] Tests",
        "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Creation for [SingleReportEnableDisable] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable Incorrect Report", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable Incorrect Report";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool wrong_report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        wrong_report_data_available = imu->rpt_linear_accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(false, wrong_report_data_available);

        data = imu->rpt_linear_accelerometer.get();

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: LinAccelDefaults: [m/s^2] x: %.2f y: %.2f z: %.2f "
                "accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_accelerometer.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Accelerometer";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_accelerometer.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_accelerometer.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Linear Accelerometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Linear Accelerometer";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_linear_accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_linear_accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_linear_accelerometer.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: LinearAccel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                "%s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_linear_accelerometer.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gravity", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gravity";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_gravity.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_gravity.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_gravity.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_gravity.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Cal Magnetometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Cal Magnetometer";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_magf_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_cal_magnetometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_cal_magnetometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_cal_magnetometer.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f y: %.2f z: %.2f "
                "accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_cal_magnetometer.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Uncal Magnetometer", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Uncal Magnetometer";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_magf_t data_magf;
    bno08x_magf_bias_t data_bias;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_uncal_magnetometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_uncal_magnetometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        imu->rpt_uncal_magnetometer.get(data_magf, data_bias);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: UncalMagnetometer: [uTesla] x: %.2f y: %.2f z: %.2f "
                "x_bias: %.2f y_bias: %.2f z_bias: %.2f accuracy: %s ",
                (i + 1), data_magf.x, data_magf.y, data_magf.z, data_bias.x, data_bias.y,
                data_bias.z, BNO08x::accuracy_to_str(data_magf.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_uncal_magnetometer.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Cal Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Cal Gyro";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_gyro_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_cal_gyro.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_cal_gyro.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_cal_gyro.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: %.2f accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_cal_gyro.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Uncal Gyro", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Uncal Gyro";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_gyro_t data_vel;
    bno08x_gyro_bias_t data_bias;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_uncal_gyro.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_uncal_gyro.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        imu->rpt_uncal_gyro.get(data_vel, data_bias);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: UncalGyro: [rad/s] x: %.2f y: %.2f z: %.2f x_bias: %.2f "
                "y_bias: %.2f z_bias: %.2f accuracy: %s ",
                (i + 1), data_vel.x, data_vel.y, data_vel.z, data_bias.x, data_bias.y, data_bias.z,
                BNO08x::accuracy_to_str(data_vel.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_uncal_gyro.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f accuracy: "
                "%s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Game RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Game RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_game.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv_game.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv_game.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f "
                "accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_game.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_ARVR_stabilized.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv_ARVR_stabilized.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv_ARVR_stabilized.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV ARVR Stabilized: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_ARVR_stabilized.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable ARVR Stabilized Game RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable ARVR Stabilized Game RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_ARVR_stabilized_game.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv_ARVR_stabilized_game.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv_ARVR_stabilized_game.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV ARVR Stabilized Game: [n/a] real: %.2f i: %.2f j: "
                "%.2f k: %.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_ARVR_stabilized_game.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Gyro Integrated RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Gyro Integrated RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_gyro_integrated.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv_gyro_integrated.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv_gyro_integrated.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Gyro Integrated: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_gyro_integrated.disable());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Geomagnetic RV", "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Geomagnetic RV";
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = 5;
    static const constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_geomagnetic.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        report_data_available = imu->rpt_rv_geomagnetic.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt_rv_geomagnetic.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08x::accuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    TEST_ASSERT_EQUAL(true, imu->rpt_rv_geomagnetic.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("BNO08x Driver Cleanup for [SingleReportEnableDisable] Tests",
        "[SingleReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "BNO08x Driver Cleanup for [SingleReportEnableDisable] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}