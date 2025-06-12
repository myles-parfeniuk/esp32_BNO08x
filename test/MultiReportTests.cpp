/**
 * @file MultiReportTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("Driver Creation for [MultiReportEnableDisable] Tests", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [MultiReportEnableDisable] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Dual Report", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Dual Report";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 2;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool data_available_accel = false;
    bool data_available_lin_accel = false;

    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        if (imu->rpt.accelerometer.has_new_data())
        {
            data_available_accel = true;
            data = imu->rpt.accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data.x, data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.linear_accelerometer.has_new_data())
        {
            data_available_lin_accel = true;
            data = imu->rpt.linear_accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data.x, data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Quad Report", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Quad Report";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 4;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_gravity = false;
    bool data_available_cal_gyro = false;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        if (imu->rpt.accelerometer.has_new_data())
        {
            data_available_accel = true;
            data_accel = imu->rpt.accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.linear_accelerometer.has_new_data())
        {
            data_available_lin_accel = true;
            data_accel = imu->rpt.linear_accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.gravity.has_new_data())
        {
            data_available_gravity = true;
            data_accel = imu->rpt.gravity.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.cal_gyro.has_new_data())
        {
            data_available_cal_gyro = true;
            data_vel = imu->rpt.cal_gyro.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_gravity);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Enable/Disable Octo Report", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Enable/Disable Octo Report";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_gravity = false;
    bool data_available_cal_gyro = false;
    bool data_available_cal_magnetometer = false;
    bool data_available_rv = false;
    bool data_available_rv_game = false;
    bool data_available_rv_geomagnetic = false;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;
    bno08x_magf_t data_magf;
    bno08x_quat_t data_quat;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        if (imu->rpt.accelerometer.has_new_data())
        {
            data_available_accel = true;
            data_accel = imu->rpt.accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.linear_accelerometer.has_new_data())
        {
            data_available_lin_accel = true;
            data_accel = imu->rpt.linear_accelerometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.gravity.has_new_data())
        {
            data_available_gravity = true;
            data_accel = imu->rpt.gravity.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.cal_gyro.has_new_data())
        {
            data_available_cal_gyro = true;
            data_vel = imu->rpt.cal_gyro.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: %.2f accuracy: "
                    "%s ",
                    (i + 1), data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.cal_magnetometer.has_new_data())
        {
            data_available_cal_magnetometer = true;
            data_magf = imu->rpt.cal_magnetometer.get();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f y: %.2f z: %.2f "
                    "accuracy: %s ",
                    (i + 1), data_magf.x, data_magf.y, data_magf.z, BNO08xAccuracy_to_str(data_magf.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.rv.has_new_data())
        {
            data_available_rv = true;
            data_quat = imu->rpt.rv.get_quat();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f "
                    "accuracy: %s ",
                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k, BNO08xAccuracy_to_str(data_quat.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.rv_game.has_new_data())
        {
            data_available_rv_game = true;
            data_quat = imu->rpt.rv_game.get_quat();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f "
                    "accuracy: %s ",
                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k, BNO08xAccuracy_to_str(data_quat.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }

        if (imu->rpt.rv_geomagnetic.has_new_data())
        {
            data_available_rv_geomagnetic = true;
            data_quat = imu->rpt.rv_geomagnetic.get_quat();
            sprintf(msg_buff,
                    "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: %.2f j: %.2f k: "
                    "%.2f accuracy: %s ",
                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k, BNO08xAccuracy_to_str(data_quat.accuracy));
            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
        }
    }

    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_gravity);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);
    TEST_ASSERT_EQUAL(true, data_available_cal_magnetometer);
    TEST_ASSERT_EQUAL(true, data_available_rv);
    TEST_ASSERT_EQUAL(true, data_available_rv_game);
    TEST_ASSERT_EQUAL(true, data_available_rv_geomagnetic);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Cleanup for [MultiReportEnableDisable] Tests", "[MultiReportEnableDisable]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [MultiReportEnableDisable] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}