#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("Init Config Args", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Init Config Args";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating test IMU.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_config_args());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Init GPIO", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Init GPIO";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_gpio());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Init HINT ISR", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Init HINT_ISR";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_hint_isr());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Init SPI", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Init SPI";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_spi());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive Tasks", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Init Tasks";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_launch_tasks());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Finish Init", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "Finish Init";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // reset imu
    TEST_ASSERT_EQUAL(true, imu->hard_reset());

    // check if reason is valid
    TEST_ASSERT_NOT_EQUAL(IMUResetReason::UNDEFINED, imu->get_reset_reason());

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying test IMU.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Init & Deinit", "[InitDenit]")
{
    const constexpr char* TEST_TAG = "Init & Deinit";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing IMU attempt 1.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing IMU.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing IMU attempt 2.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing IMU.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}