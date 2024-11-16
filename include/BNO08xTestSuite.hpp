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
            UNITY_BEGIN();
            run_init_deinit_tests(false);
            run_report_tests(false);
            UNITY_END();
        }

        static void run_init_deinit_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("init_denit_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[InitComprehensive]", false);
            unity_run_tests_by_tag("[InitDenit]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("init_denit_tests");
        }

        static void run_report_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("report_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[SingleReportEnableDisable]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("report_tests");
        }
};