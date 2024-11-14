#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

TEST_CASE("Full Init & Deinit", "[InitDenit]")
{
    const constexpr char *TEST_TAG ="Full Init & Deinit";
    BNO08x* imu;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing IMU attempt 1.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG,  "Success, deinitializing IMU.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Initializing IMU attempt 2.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_msg(TEST_TAG,  "Success, deinitializing IMU.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}