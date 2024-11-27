/**
 * @file InitDenitTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS TEST SUITE TO BE BUILT WITH PROJECT:
 *          set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("InitComprehensive Config Args", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive Config Args";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating test IMU for [InitComprehensive] and [DeinitComprehensive].");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_config_args());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive GPIO", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive GPIO";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_gpio());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive HINT ISR", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive HINT ISR";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_hint_isr());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive SPI", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive SPI";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_spi());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive sh2 HAL", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive sh2 HAL";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_sh2_HAL());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("InitComprehensive Tasks", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive Tasks";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_tasks());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("DeinitComprehensive Tasks", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive Tasks";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_tasks());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("DeinitComprehensive sh2 HAL", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive sh2 HAL";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_sh2_HAL());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("DeinitComprehensive HINT ISR", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive HINT ISR";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_hint_isr());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("DeinitComprehensive SPI", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive SPI";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_spi());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("DeinitComprehensive GPIO", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive GPIO";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_gpio());
    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

TEST_CASE("Init and Deinit", "[InitDenit]")
{
    const constexpr char* TEST_TAG = "Init and Deinit";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing BNO08x Driver Object attempt 1.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing BNO08x Driver Object.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing BNO08x Driver Object attempt 2.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing BNO08x Driver Object.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}