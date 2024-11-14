#pragma once

#include "stdio.h"
#include "BNO08x.hpp"

class BNO08xTestHelper
{
    private:
        inline static BNO08x* test_imu = nullptr;
        inline static bno08x_config_t imu_cfg;

        static const constexpr char* TAG = "BNO08xTestHelper";

    public:
        static void print_test_start_banner(const char* TEST_TAG)
        {
            printf("------------------------BEGIN TEST: %s------------------------\n\r", TEST_TAG);
        }

        static void print_test_end_banner(const char* TEST_TAG)
        {
            printf("------------------------END TEST: %s------------------------\n\r", TEST_TAG);
        }

        static void print_test_msg(const char* TEST_TAG, const char* msg)
        {
            printf("%s: %s: %s\n\r", TAG, TEST_TAG, msg);
        }

        static void set_test_imu_cfg(bno08x_config_t cfg)
        {
            imu_cfg = cfg;
        }

        static void create_test_imu()
        {
            if (test_imu != nullptr)
                destroy_test_imu();

            test_imu = new BNO08x();
        }

        static void destroy_test_imu()
        {
            if (test_imu != nullptr)
            {
                delete test_imu;
                test_imu = nullptr;
            }
        }

        static BNO08x* get_test_imu()
        {
            return test_imu;
        }

        static esp_err_t call_init_config_args()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_config_args();
        }

        static esp_err_t call_init_gpio()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_gpio();
        }

        static esp_err_t call_init_hint_isr()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_hint_isr();
        }

        static esp_err_t call_init_spi()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_spi();
        }

        static esp_err_t call_launch_tasks()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->launch_tasks();
        }
};