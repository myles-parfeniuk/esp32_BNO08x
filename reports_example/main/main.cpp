#include <stdio.h>
#include "BNO08x.hpp"

static const constexpr char* TAG = "Main";

extern "C" void app_main(void)
{
    static BNO08x imu;

    // initialize imu
    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }

    // enable game rotation vector and calibrated gyro reports
    imu.rpt.rv_game.enable(100000UL); // 100,000us == 100ms report interval
    imu.rpt.cal_gyro.enable(100000UL);
    imu.rpt.gravity.enable(100000UL);
    imu.rpt.accelerometer.enable(100000UL);
    imu.rpt.linear_accelerometer.enable(100000UL);
    // see BNO08x::bno08x_reports_t for all possible reports to enable

    while (1)
    {
        // block until new report is detected
        if (imu.data_available())
        {

            if (imu.rpt.rv_game.has_new_data())
            {
                bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
                ESP_LOGW(TAG, "Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler.x, euler.y, euler.z);
            }

            if (imu.rpt.cal_gyro.has_new_data())
            {
                bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
                ESP_LOGW(TAG, "Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s]", velocity.x, velocity.y, velocity.z);
            }

            if (imu.rpt.gravity.has_new_data())
            {
                bno08x_accel_t grav = imu.rpt.gravity.get();
                ESP_LOGW(TAG, "Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2]", grav.x, grav.y, grav.z);
            }

            if (imu.rpt.accelerometer.has_new_data())
            {
                bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
                ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
            }

            if (imu.rpt.linear_accelerometer.has_new_data())
            {
                bno08x_accel_t lin_accel = imu.rpt.accelerometer.get();
                ESP_LOGW(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel.x, lin_accel.y, lin_accel.z);
            }
        }
    }
}