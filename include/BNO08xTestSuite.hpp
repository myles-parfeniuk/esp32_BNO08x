#pragma once

/*
YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS TEST SUITE TO BE FUNCTIONAL:
set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components to test.")
*/

#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "BNO08xTestHelper.hpp"

class BNO08xTestSuite
{
    public:
        static void run_all_tests();
        static void run_init_deinit_tests();

};