/**
 * @file BNO08xTestSuite.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// esp-idf includes
#include "unity.h"
// in-house includes
#include "BNO08xTestHelper.hpp"

/**
 * @class BNO08xTestSuite
 *
 * @brief BNO08x unit test launch point class.
 * */
class BNO08xTestSuite
{

    private:
        static void print_begin_tests_banner(const char* test_set_name)
        {
            printf("####################### BEGIN TESTS: %s #######################\n\r",
                    test_set_name);
        }

        static void print_end_tests_banner(const char* test_set_name)
        {
            printf("####################### END TESTS: %s #######################\n\r",
                    test_set_name);
        }

    public:
        static void run_all_tests()
        {
            UNITY_BEGIN();
            run_init_deinit_tests(false);
            run_single_report_tests(false);
            run_multi_report_tests(false);
            run_callback_tests(false);
            run_feature_tests(false);
            UNITY_END();
        }

        static void run_init_deinit_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("init_denit_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[InitComprehensive]", false);
            unity_run_tests_by_tag("[DeinitComprehensive]", false);
            unity_run_tests_by_tag("[InitDenit]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("init_denit_tests");
        }

        static void run_single_report_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("single_report_enable_disable_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[SingleReportEnableDisable]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("single_report_enable_disable_tests");
        }

        static void run_multi_report_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("multi_report_enable_disable_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[MultiReportEnableDisable]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("multi_report_enable_disable_tests");
        }

        static void run_callback_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("callback_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[CallbackAllReportVoidInputParam]", false);
            unity_run_tests_by_tag("[CallbackAllReportIDInputParam]", false);
            unity_run_tests_by_tag("[CallbackSingleReportVoidInputParam]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("callback_tests");
        }

        static void run_feature_tests(bool call_unity_end_begin = true)
        {
            print_begin_tests_banner("feature_tests");

            if (call_unity_end_begin)
                UNITY_BEGIN();

            unity_run_tests_by_tag("[FeatureTests]", false);

            if (call_unity_end_begin)
                UNITY_END();

            print_end_tests_banner("feature_tests");
        }
};
