#include "BNO08xTestSuite.hpp"

void BNO08xTestSuite::run_tests()
{
    BNO08x imu;

    BNO08xTestHelper::set_test_imu(&imu);

}