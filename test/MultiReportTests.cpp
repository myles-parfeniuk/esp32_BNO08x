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

/**
 * @test Driver Creation for [MultiReportRead] Tests
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [MultiReportRead] test group.
 *
 * 1. Create a test IMU instance for use in [MultiReportRead] test group.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for [MultiReportRead] Tests", "[MultiReportRead]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [MultiReportRead] Tests";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Dual Report
 *
 * This test validates that two reports can simultaneously be enabled and accessed
 * through the data_available polling methods without issue.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 3. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 4. Attempt to disable all reports and assert that it happens successfully.
 *
 * 5. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Dual Report", "[MultiReportRead]")
{
    const constexpr char* TEST_TAG = "Read Dual Report";
    constexpr uint8_t ENABLED_REPORT_COUNT = 2;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool data_available_accel = false;
    bool data_available_lin_accel = false;

    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 3.
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

    // 4.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 5.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Quad Report
 *
 * This test validates that four reports can simultaneously be enabled and accessed
 * through the data_available polling methods without issue.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to enable gravity reports and assert that it happens successfully.
 *
 * 4. Attempt to enable calibrated gyroscope reports and assert that it happens successfully.
 *
 * 5. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 6. Attempt to disable all reports and assert that it happens successfully.
 *
 * 7. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Quad Report", "[MultiReportRead]")
{
    const constexpr char* TEST_TAG = "Read Quad Report";
    constexpr uint8_t ENABLED_REPORT_COUNT = 4;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

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

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD_US));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 5.
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

    // 6.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 7.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_gravity);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Octo Report
 *
 * This test validates that eight reports can simultaneously be enabled and accessed
 * through the data_available polling methods without issue.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to enable gravity reports and assert that it happens successfully.
 *
 * 4. Attempt to enable calibrated gyroscope reports and assert that it happens successfully.
 *
 * 5. Attempt to enable calibrated magnetometer reports and assert that it happens successfully.
 *
 * 6. Attempt to enable rotation vector reports and assert that it happens successfully.
 *
 * 7. Attempt to enable game rotation vector reports and assert that it happens successfully.
 *
 * 8. Attempt to enable geomagnetic rotation vector reports and assert that it happens successfully.
 *
 * 9. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 10. Attempt to disable all reports and assert that it happens successfully.
 *
 * 11. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Octo Report", "[MultiReportRead]")
{
    const constexpr char* TEST_TAG = "Read Octo Report";
    constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

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

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD_US));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD_US));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD_US));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    // 7.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD_US));

    // 8.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 9.
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

    // 10.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 11.
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

/**
 * @test Driver Cleanup for [MultiReportRead] Tests"
 *
 * This test destroys the BNO08x IMU driver object for use in the [MultiReportRead] test group.
 *
 * 1. Destroys the test IMU instance that was used with the MultiReportRead test group.
 *
 */
TEST_CASE("Driver Cleanup for [MultiReportRead] Tests", "[MultiReportRead]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [MultiReportRead] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}
