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
    private:
        static void print_begin_tests_banner(const char* test_set_name)
        {
            printf("####################### BEGIN TESTS: %s #######################\n\r", test_set_name);
        }

        static void print_end_tests_banner(const char* test_set_name)
        {
            printf("####################### END TESTS: %s #######################\n\r", test_set_name);
        }

    public:
        static void run_all_tests()
        {
            run_init_deinit_tests();
            run_report_tests();
        }

        static void run_init_deinit_tests()
        {
            print_begin_tests_banner("init_denit_tests");

            UNITY_BEGIN();
            unity_run_tests_by_tag("[Init]", false);
            unity_run_tests_by_tag("[InitDenit]", false);
            UNITY_END();

            print_end_tests_banner("init_denit_tests");
        }

        static void run_report_tests()
        {
            print_begin_tests_banner("report_tests");

            UNITY_BEGIN();
            unity_run_tests_by_tag("[ReportEnableDisable]", false);
            UNITY_END();

            print_end_tests_banner("report_tests");
        }
};