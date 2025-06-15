/**
 * @file ReportTests.cpp
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
 * @test Driver Creation for [SingleReportRead] Tests
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [SingleReportRead] test group.
 *
 * 1. Create a test IMU instance for use in [SingleReportRead] test group.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for [SingleReportRead] Tests", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [SingleReportRead] Tests";
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
 * @test Enable Incorrect Report
 *
 * This test validates that a report isn't set to data available if a unrelated report
 * rightfully is.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is NOT a linear accelerometer report.
 *
 * 3. Attempt to disable accelerometer reports and assert that it happens successfully.
 *
 */
TEST_CASE("Enable Incorrect Report", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Enable Incorrect Report";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool wrong_report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        wrong_report_data_available = imu->rpt.linear_accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(false, wrong_report_data_available);

        data = imu->rpt.linear_accelerometer.get();

        sprintf(msg_buff,
                "No Rx Data Trial %d Success: LinAccelDefaults: [m/s^2] x: %.2f y: %.2f z: %.2f "
                "accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Accelerometer
 *
 * This test validates the basic functionality of the Accelerometer report.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is an accelerometer report.
 *
 * 3. Attempt to disable accelerometer reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Accelerometer", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Accelerometer";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.accelerometer.get();

        sprintf(msg_buff, "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data.x,
                data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Linear Accelerometer
 *
 * This test validates the basic functionality of the Linear Accelerometer report.
 *
 * 1. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a linear accelerometer report.
 *
 * 3. Attempt to disable linear accelerometer reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Linear Accelerometer", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Linear Accelerometer";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5UL;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.linear_accelerometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.linear_accelerometer.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: LinearAccel: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: "
                "%s ",
                (i + 1), data.x, data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Gravity
 *
 * This test validates the basic functionality of the Gravity report.
 *
 * 1. Attempt to enable gravity reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a gravity report.
 *
 * 3. Attempt to disable gravity reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Gravity", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Gravity";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5UL;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_accel_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.gravity.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.gravity.get();

        sprintf(msg_buff, "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data.x,
                data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Cal Magnetometer
 *
 * This test validates the basic functionality of the Calibrated Magnetometer report.
 *
 * 1. Attempt to enable calibrated magnetometer reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a calibrated magnetometer report.
 *
 * 3. Attempt to disable calibrated magnetometer reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Cal Magnetometer", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Cal Magnetometer";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_magf_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.cal_magnetometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.cal_magnetometer.get();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f y: %.2f z: %.2f "
                "accuracy: %s ",
                (i + 1), data.x, data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Uncal Magnetometer
 *
 * This test validates the basic functionality of the Uncalibrated Magnetometer report.
 *
 * 1. Attempt to enable uncalibrated magnetometer reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is an uncalibrated magnetometer report.
 *
 * 3. Attempt to disable uncalibrated magnetometer reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Uncal Magnetometer", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Uncal Magnetometer";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_magf_t data_magf;
    bno08x_magf_bias_t data_bias;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.uncal_magnetometer.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.uncal_magnetometer.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        imu->rpt.uncal_magnetometer.get(data_magf, data_bias);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: UncalMagnetometer: [uTesla] x: %.2f y: %.2f z: %.2f "
                "x_bias: %.2f y_bias: %.2f z_bias: %.2f accuracy: %s ",
                (i + 1), data_magf.x, data_magf.y, data_magf.z, data_bias.x, data_bias.y, data_bias.z,
                BNO08xAccuracy_to_str(data_magf.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.uncal_magnetometer.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Cal Gyro
 *
 * This test validates the basic functionality of the Calibrated Gyroscope report.
 *
 * 1. Attempt to enable calibrated gyroscope reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a calibrated gyroscope report.
 *
 * 3. Attempt to disable calibrated gyroscope reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Cal Gyro", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Cal Gyro";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_gyro_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.cal_gyro.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.cal_gyro.get();

        sprintf(msg_buff, "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: %.2f accuracy: %s ", (i + 1), data.x,
                data.y, data.z, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Uncal Gyro
 *
 * This test validates the basic functionality of the Uncalibrated Gyroscope report.
 *
 * 1. Attempt to enable uncalibrated gyroscope reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is an uncalibrated gyroscope report.
 *
 * 3. Attempt to disable uncalibrated gyroscope reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Uncal Gyro", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Uncal Gyro";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_gyro_t data_vel;
    bno08x_gyro_bias_t data_bias;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.uncal_gyro.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.uncal_gyro.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        imu->rpt.uncal_gyro.get(data_vel, data_bias);

        sprintf(msg_buff,
                "Rx Data Trial %d Success: UncalGyro: [rad/s] x: %.2f y: %.2f z: %.2f x_bias: %.2f "
                "y_bias: %.2f z_bias: %.2f accuracy: %s ",
                (i + 1), data_vel.x, data_vel.y, data_vel.z, data_bias.x, data_bias.y, data_bias.z,
                BNO08xAccuracy_to_str(data_vel.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.uncal_gyro.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read RV
 *
 * This test validates the basic functionality of the Rotation Vector report.
 *
 * 1. Attempt to enable rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a rotation vector report.
 *
 * 3. Attempt to disable rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f accuracy: "
                "%s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Game RV
 *
 * This test validates the basic functionality of the Game Rotation Vector report.
 *
 * 1. Attempt to enable game rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a game rotation vector report.
 *
 * 3. Attempt to disable game rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Game RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Game RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv_game.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv_game.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f j: %.2f k: %.2f "
                "accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read ARVR Stabilized Rotation Vector
 *
 * This test validates the basic functionality of the ARVR Stabilized Rotation Vector report.
 *
 * 1. Attempt to enable ARVR stabilized rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is an ARVR stabilized rotation vector report.
 *
 * 3. Attempt to disable ARVR stabilized rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read ARVR Stabilized RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read ARVR Stabilized RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_ARVR_stabilized.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv_ARVR_stabilized.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv_ARVR_stabilized.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV ARVR Stabilized: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_ARVR_stabilized.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read ARVR Stabilized Game RV
 *
 * This test validates the basic functionality of the ARVR Stabilized Game Rotation Vector report.
 *
 * 1. Attempt to enable ARVR stabilized game rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is an ARVR stabilized game rotation vector report.
 *
 * 3. Attempt to disable ARVR stabilized game rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read ARVR Stabilized Game RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read ARVR Stabilized Game RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_ARVR_stabilized_game.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv_ARVR_stabilized_game.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv_ARVR_stabilized_game.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV ARVR Stabilized Game: [n/a] real: %.2f i: %.2f j: "
                "%.2f k: %.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_ARVR_stabilized_game.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Gyro Integrated RV
 *
 * This test validates the basic functionality of the Gyroscope Integrated Rotation Vector report.
 *
 * 1. Attempt to enable gyroscope integrated rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a gyroscope integrated rotation vector report.
 *
 * 3. Attempt to disable gyroscope integrated rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Gyro Integrated RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Gyro Integrated RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_gyro_integrated.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv_gyro_integrated.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv_gyro_integrated.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Gyro Integrated: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_gyro_integrated.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Geomagnetic RV
 *
 * This test validates the basic functionality of the Geomagnetic Rotation Vector report.
 *
 * 1. Attempt to enable geomagnetic rotation vector reports and assert that it happens successfully.
 *
 * 2. Assert that new report data is available RX_REPORT_TRIAL_COUNT times.
 *
 * 2.1. Assert that the report data is a geomagnetic rotation vector report.
 *
 * 3. Attempt to disable geomagnetic rotation vector reports and assert that it happens successfully.
 *
 */
TEST_CASE("Read Geomagnetic RV", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Read Geomagnetic RV";
    constexpr uint8_t RX_REPORT_TRIAL_CNT = 5U;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available = false;
    bool report_data_available = true;
    bno08x_quat_t data;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD_US));

    for (int i = 0; i < RX_REPORT_TRIAL_CNT; i++)
    {
        // 2.
        data_available = imu->data_available();
        TEST_ASSERT_EQUAL(true, data_available);

        // 2.1
        report_data_available = imu->rpt.rv_geomagnetic.has_new_data();
        TEST_ASSERT_EQUAL(true, report_data_available);

        data = imu->rpt.rv_geomagnetic.get_quat();

        sprintf(msg_buff,
                "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: %.2f j: %.2f k: "
                "%.2f accuracy: %s ",
                (i + 1), data.real, data.i, data.j, data.k, BNO08xAccuracy_to_str(data.accuracy));

        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    }

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.disable());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for [SingleReportRead] Tests"
 *
 * This test destroys the BNO08x IMU driver object for use in the [SingleReportRead] test group.
 *
 * 1. Destroys the test IMU instance that was used with the SingleReportRead test group.
 *
 */
TEST_CASE("Driver Cleanup for [SingleReportRead] Tests", "[SingleReportRead]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [SingleReportRead] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}
