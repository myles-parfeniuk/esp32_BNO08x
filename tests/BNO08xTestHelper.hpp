#pragma once

#include "stdio.h"
#include "BNO08x.hpp"

class BNO08xTestHelper
{
    public: 
        static void set_test_imu(BNO08x *imu);
        static BNO08x* get_test_imu();
    private:
        static BNO08x *test_imu;
};