#include "BNO08xTestSuite.hpp"

void BNO08xTestSuite::run_all_tests()
{
    run_init_deinit_tests();
}

void BNO08xTestSuite::run_init_deinit_tests()
{
    UNITY_BEGIN();
    unity_run_tests_by_tag("[Init]", false);
    UNITY_END();

    UNITY_BEGIN();
    unity_run_test_by_name("Full Init & Deinit");
    UNITY_END();
}