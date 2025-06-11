/**
 * @file InitDenitTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

void setUp()
{
    // do nothing
}

void tearDown()
{
    // do nothing
}

/**
 * @test InitComprehensive Config Args
 *
 * This test verifies the internal logic for error checking the default bno08x_config_t struct. It performs the following steps:
 *
 * 1. Creates a test IMU instance for use in [InitComprehensive] and [DeinitComprehensive] test groups.
 *
 * 2. Call the internal BNO08x::init_config_args() function to error check the default imu_config struct and assert it returns
 *  without fail.
 *
 */
TEST_CASE("InitComprehensive Config Args", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive Config Args";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating test IMU for [InitComprehensive] and [DeinitComprehensive].");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_config_args());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test InitComprehensive GPIO
 *
 * This test verifies the internal gpio initialization function can be executed successfully.
 * It is responsible for initializing GPIO not controlled by SPI peripheral (RST & CS).
 *
 * 1. Call the internal BNO08x::init_gpio() function and assert it returns without fail.
 *
 */
TEST_CASE("InitComprehensive GPIO", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive GPIO";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_gpio());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test InitComprehensive HINT ISR
 *
 * This test verifies the internal HINT ISR initialization function can be executed successfully.
 * It is responsible for registering the HINT ISR with esp-idf.
 *
 * 1. Call the internal BNO08x::init_hint_isr() function and assert it returns without fail.
 *
 */
TEST_CASE("InitComprehensive HINT ISR", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive HINT ISR";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_hint_isr());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test InitComprehensive SPI
 *
 * This test verifies the internal SPI initialization function can be executed successfully.
 * It is responsible for initializing the selected SPI host/peripheral.
 *
 * 1. Call the internal BNO08x::init_spi() function and assert it returns without fail.
 *
 */
TEST_CASE("InitComprehensive SPI", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive SPI";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    imu = BNO08xTestHelper::get_test_imu();

    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_spi());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test InitComprehensive sh2 HAL
 *
 * This test verifies the internal sh2 HAL initialization function can be executed successfully.
 * It is responsible for registering the BNO08xSH2HALCallbacks with the sh2 lib and establishing
 * initial communication with IMU through a product ID request.
 *
 * 1. Call the internal BNO08x::init_sh2_HAL() function and assert it returns without fail.
 *
 */
TEST_CASE("InitComprehensive sh2 HAL", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive sh2 HAL";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_sh2_HAL());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test InitComprehensive Tasks
 *
 * This test verifies the internal task initialization function can be executed successfully.
 * It is responsible launching the 3 tasks used by this library (data processing task, callback task,
 * and sh2 HAL service task).
 *
 * 1. Call the internal BNO08x::init_tasks() function and assert it returns without fail.
 *
 */
TEST_CASE("InitComprehensive Tasks", "[InitComprehensive]")
{
    const constexpr char* TEST_TAG = "InitComprehensive Tasks";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_init_tasks());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test DeinitComprehensive Tasks
 *
 * This test verifies the internal task deinitialization function can be executed successfully. 
 * It is responsible for safely deleting the 3 tasks used by this library (data processing task, callback task,
 * and sh2 HAL service task).
 *
 * 1. Call the internal BNO08x::deinit_tasks() function and assert it returns without fail.
 *
 */
TEST_CASE("DeinitComprehensive Tasks", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive Tasks";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1. 
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_tasks());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test DeinitComprehensive sh2 HAL
 *
 * This test verifies the internal sh2 HAL deinitialization function can be executed successfully.  
 * It is responsible for closing the sh2 HAL lib instance.
 *
 * 1. Call the internal BNO08x::deinit_sh2_HAL() function and assert it returns without fail.
 *
 */
TEST_CASE("DeinitComprehensive sh2 HAL", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive sh2 HAL";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_sh2_HAL());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test DeinitComprehensive HINT ISR
 *
 * This test verifies the internal HINT ISR deinitialization function can be executed successfully.  
 * It is responsible unregistering the HINT ISR handler with esp-idf. 
 *
 * 1. Call the internal BNO08x::deinit_hint_isr() function and assert it returns without fail.
 *
 */
TEST_CASE("DeinitComprehensive HINT ISR", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive HINT ISR";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1. 
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_hint_isr());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test DeinitComprehensive SPI
 *
 * This test verifies the internal SPI deinitialization function can be executed successfully.   
 * It is responsible for releasing the selected SPI host/peripheral.
 *
 * 1. Call the internal BNO08x::deinit_spi() function and assert it returns without fail.
 *
 */
TEST_CASE("DeinitComprehensive SPI", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive SPI";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1. 
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_spi());

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test DeinitComprehensive GPIO
 *
 * This test verifies the internal GPIO deinitialization function can be executed successfully.  
 * It is responsible for returning the GPIO not controlled by the SPI peripheral to their default state.
 *
 * 1. Call the internal BNO08x::deinit_gpio() function and assert it returns without fail.
 * 
 * 2. Destroys the test IMU instance that was used with the [InitComprehensive] and [DeinitComprehensive] test groups 
 * this will also release any freeRTOS resources (queues, semaphores, event groups, etc...).
 *
 */
TEST_CASE("DeinitComprehensive GPIO", "[DeinitComprehensive]")
{
    const constexpr char* TEST_TAG = "DeinitComprehensive GPIO";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(ESP_OK, BNO08xTestHelper::call_deinit_gpio());

    // 2.
    BNO08xTestHelper::destroy_test_imu();
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Init and Deinit
 *
 * This test verifies the full initialization and deinitialization without calling the internal functions
 * individually.
 *
 * 1. Creates a test IMU instance 
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail. 
 * 
 * 3. Destroy the test IMU instance deinitialize flow is performed in deconstructor. 
 * 
 * 4. Create another test IMU instance
 * 
 * 5. Call the public BNO08x::initialize() function and assert it returns without fail. 
 * 
 * 6. Destroy the test IMU instance deinitialize flow is performed in deconstructor.
 *
 */
TEST_CASE("Init and Deinit", "[InitDenit]")
{
    const constexpr char* TEST_TAG = "Init and Deinit";
    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing BNO08x Driver Object attempt 1.");
    // 1.
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing BNO08x Driver Object.");

    // 3.
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing BNO08x Driver Object attempt 2.");
    // 4.
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 5.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Success, deinitializing BNO08x Driver Object.");

    // 6.
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}