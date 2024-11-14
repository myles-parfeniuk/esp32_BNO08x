#include "BNO08xTestHelper.hpp"

BNO08x* BNO08xTestHelper::test_imu = nullptr;

void BNO08xTestHelper::set_test_imu(BNO08x *imu)
{
    test_imu = imu;
}

BNO08x* BNO08xTestHelper::get_test_imu()
{
    return test_imu;
}