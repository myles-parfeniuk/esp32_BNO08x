/**
 * @file CallbackTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("Driver Creation for [CallbackAllReportVoidInputParam] Tests", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackAllReportVoidInputParam] Tests";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Void Input Param Flavor Cb", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Void Input Param Flavor Cb";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_grav = false;
    bool data_available_cal_gyro = false;
    bool data_available_cal_magnetometer = false;
    bool data_available_rv = false;
    bool data_available_rv_game = false;
    bool data_available_rv_geomagnetic = false;

    bool test_running = true;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;
    bno08x_magf_t data_magf;
    bno08x_quat_t data_quat;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    imu->register_cb(
            [&imu, &data_available_accel, &data_available_lin_accel, &data_available_grav, &data_available_cal_gyro,
                    &data_available_cal_magnetometer, &data_accel, &data_available_rv, &data_available_rv_game,
                    &data_available_rv_geomagnetic, &data_quat, &data_vel, &data_magf, &msg_buff, &test_running]()
            {
                static int i = 0;

                if (i < RX_REPORT_TRIAL_CNT)
                {
                    if (imu->rpt.accelerometer.has_new_data())
                    {
                        data_available_accel = true;
                        data_accel = imu->rpt.accelerometer.get();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                "accuracy: %s ",
                                (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.linear_accelerometer.has_new_data())
                    {
                        data_available_lin_accel = true;
                        data_accel = imu->rpt.linear_accelerometer.get();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: "
                                "%.2f accuracy: %s ",
                                (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.gravity.has_new_data())
                    {
                        data_available_grav = true;
                        data_accel = imu->rpt.gravity.get();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: "
                                "%.2f accuracy: %s ",
                                (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.cal_gyro.has_new_data())
                    {
                        data_available_cal_gyro = true;
                        data_vel = imu->rpt.cal_gyro.get();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: "
                                "%.2f accuracy: %s ",
                                (i + 1), data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.cal_magnetometer.has_new_data())
                    {
                        data_available_cal_magnetometer = true;
                        data_magf = imu->rpt.cal_magnetometer.get();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f y: "
                                "%.2f z: %.2f accuracy: %s ",
                                (i + 1), data_magf.x, data_magf.y, data_magf.z, BNO08xAccuracy_to_str(data_magf.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.rv.has_new_data())
                    {
                        data_available_rv = true;
                        data_quat = imu->rpt.rv.get_quat();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: %.2f k: "
                                "%.2f accuracy: %s ",
                                (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                BNO08xAccuracy_to_str(data_quat.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.rv_game.has_new_data())
                    {
                        data_available_rv_game = true;
                        data_quat = imu->rpt.rv_game.get_quat();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f j: "
                                "%.2f k: %.2f accuracy: %s ",
                                (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                BNO08xAccuracy_to_str(data_quat.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }
                    else if (imu->rpt.rv_geomagnetic.has_new_data())
                    {
                        data_available_rv_geomagnetic = true;
                        data_quat = imu->rpt.rv_geomagnetic.get_quat();
                        sprintf(msg_buff,
                                "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: "
                                "%.2f j: %.2f k: %.2f accuracy: %s ",
                                (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                BNO08xAccuracy_to_str(data_quat.accuracy));
                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                    }

                    i++;
                }
                else if (test_running)
                {
                    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                    test_running = false;
                }
            });

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD));

    while (test_running)
    {
    }

    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_grav);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);
    TEST_ASSERT_EQUAL(true, data_available_cal_magnetometer);
    TEST_ASSERT_EQUAL(true, data_available_rv);
    TEST_ASSERT_EQUAL(true, data_available_rv_game);
    TEST_ASSERT_EQUAL(true, data_available_rv_geomagnetic);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Cleanup for [CallbackAllReportVoidInputParam] Tests", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackAllReportVoidInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Creation for [CallbackAllReportIDInputParam] Tests", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackAllReportIDInputParam] Tests";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Report ID Input Param Flavor Cb", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Report ID Input Param Flavor Cb";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_grav = false;
    bool data_available_cal_gyro = false;
    bool data_available_cal_magnetometer = false;
    bool data_available_rv = false;
    bool data_available_rv_game = false;
    bool data_available_rv_geomagnetic = false;
    bool test_running = true;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;
    bno08x_magf_t data_magf;
    bno08x_quat_t data_quat;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    imu->register_cb(
            [&imu, &data_available_accel, &data_available_lin_accel, &data_available_grav, &data_available_cal_gyro,
                    &data_available_cal_magnetometer, &data_accel, &data_available_rv, &data_available_rv_game,
                    &data_available_rv_geomagnetic, &data_quat, &data_vel, &data_magf, &msg_buff,
                    &test_running](uint8_t report_ID)
            {
                static int i = 0;
                if (i < RX_REPORT_TRIAL_CNT)
                {
                    switch (report_ID)
                    {
                        case SH2_ACCELEROMETER:

                            data_available_accel = true;
                            data_accel = imu->rpt.accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    (i + 1), data_accel.x, data_accel.y, data_accel.z,
                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_LINEAR_ACCELERATION:
                            data_available_lin_accel = true;
                            data_accel = imu->rpt.linear_accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f "
                                    "z: %.2f accuracy: %s ",
                                    (i + 1), data_accel.x, data_accel.y, data_accel.z,
                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_GRAVITY:
                            data_available_grav = true;
                            data_accel = imu->rpt.gravity.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    (i + 1), data_accel.x, data_accel.y, data_accel.z,
                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_GYROSCOPE_CALIBRATED:
                            data_available_cal_gyro = true;
                            data_vel = imu->rpt.cal_gyro.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    (i + 1), data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_MAGNETIC_FIELD_CALIBRATED:
                            data_available_cal_magnetometer = true;
                            data_magf = imu->rpt.cal_magnetometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f "
                                    "y: %.2f z: %.2f accuracy: %s ",
                                    (i + 1), data_magf.x, data_magf.y, data_magf.z, BNO08xAccuracy_to_str(data_magf.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_ROTATION_VECTOR:
                            data_available_rv = true;
                            data_quat = imu->rpt.rv.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: "
                                    "%.2f k: %.2f accuracy: %s ",
                                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_GAME_ROTATION_VECTOR:
                            data_available_rv_game = true;
                            data_quat = imu->rpt.rv_game.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f "
                                    "j: %.2f k: %.2f accuracy: %s ",
                                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                            data_available_rv_geomagnetic = true;
                            data_quat = imu->rpt.rv_geomagnetic.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: "
                                    "%.2f j: %.2f k: %.2f accuracy: %s ",
                                    (i + 1), data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            break;

                        default:

                            break;
                    }

                    i++;
                }
                else if (test_running)
                {
                    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                    test_running = false;
                }
            });

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD));
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD));

    while (test_running)
    {
    }

    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_grav);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);
    TEST_ASSERT_EQUAL(true, data_available_cal_magnetometer);
    TEST_ASSERT_EQUAL(true, data_available_rv);
    TEST_ASSERT_EQUAL(true, data_available_rv_game);
    TEST_ASSERT_EQUAL(true, data_available_rv_geomagnetic);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Cleanup for [CallbackAllReportIDInputParam] Tests", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackAllReportIDInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Creation for [CallbackSingleReportVoidInputParam] Tests", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackSingleReportVoidInputParam] Tests";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Single Report Void Input Param Flavor Cb", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Single Report Void Input Param Flavor Cb";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool test_running = true;

    bno08x_accel_t data_accel;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    imu->rpt.accelerometer.register_cb(
            [&imu, &data_available_accel, &data_accel, &msg_buff, &test_running]()
            {
                static int i = 0;

                if (i < RX_REPORT_TRIAL_CNT)
                {
                    data_available_accel = true;
                    data_accel = imu->rpt.accelerometer.get();
                    sprintf(msg_buff,
                            "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                            "accuracy: %s ",
                            (i + 1), data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

                    i++;
                }
                else if (test_running)
                {
                    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                    test_running = false;
                }
            });

    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    while (test_running)
    {
    }

    TEST_ASSERT_EQUAL(true, data_available_accel);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Driver Cleanup for [CallbackSingleReportVoidInputParam] Tests", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackSingleReportVoidInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");

    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}
