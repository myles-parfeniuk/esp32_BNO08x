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

    // enable report(s)
    imu.rpt.accelerometer.enable(100000UL);

    while (1)
    {
        // read some reports
        for (int i = 0; i < 50; i++)
        {
            if (imu.data_available())
            {
                if (imu.rpt.accelerometer.has_new_data())
                {
                    bno08x_accel_t ang_accel = imu.rpt.accelerometer.get();
                    ESP_LOGW(TAG, "Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", ang_accel.x, ang_accel.y, ang_accel.z);
                }
            }
        }

        // go to bed
        imu.sleep();
        ESP_LOGE(TAG, "Sleeping...");

        for (int i = 0; i < 5; i++)
        {
            ESP_LOGE(TAG, "Zzzz %ds...", 5-i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // wake up
        imu.on();
        ESP_LOGE(TAG, "Awake...");
    }
}