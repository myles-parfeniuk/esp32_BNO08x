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
        typedef struct imu_report_data_t
        {
                uint32_t time_stamp;

                float quat_I;
                float quat_J;
                float quat_K;
                float quat_real;
                float quat_radian_accuracy;
                BNO08xAccuracy quat_accuracy;

                float integrated_gyro_vel_x;
                float integrated_gyro_vel_y;
                float integrated_gyro_vel_z;

                float accel_x;
                float accel_y;
                float accel_z;
                BNO08xAccuracy accel_accuracy;

                float lin_accel_x;
                float lin_accel_y;
                float lin_accel_z;
                BNO08xAccuracy lin_accel_accuracy;

                float grav_x;
                float grav_y;
                float grav_z;
                BNO08xAccuracy grav_accuracy;

                float calib_gyro_vel_x;
                float calib_gyro_vel_y;
                float calib_gyro_vel_z;
                BNO08xAccuracy calib_gyro_accuracy;


        } imu_report_data_t;

        static void print_test_start_banner(const char* TEST_TAG)
        {
            printf("------------------------ BEGIN TEST: %s ------------------------\n\r", TEST_TAG);
        }

        static void print_test_end_banner(const char* TEST_TAG)
        {
            printf("------------------------ END TEST: %s ------------------------\n\r", TEST_TAG);
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

        static bool rotation_vector_data_is_default(imu_report_data_t* report_data)
        {
            bool new_data = false;

            if (report_data->quat_I != 0.0f)
                new_data = true;

            if (report_data->quat_J != 0.0f)
                new_data = true;

            if (report_data->quat_K != 0.0f)
                new_data = true;

            if (report_data->quat_real != 1.0f)
                new_data = true;

            if (report_data->quat_accuracy != BNO08xAccuracy::UNDEFINED)
                new_data = true;

            if (report_data->quat_radian_accuracy != 0.0f)
                new_data = true;

            return new_data;
        }

        static bool gyro_integrated_rotation_vector_data_is_default(imu_report_data_t* report_data)
        {
            bool new_data = false;

            if (report_data->quat_I != 0.0f)
                new_data = true;

            if (report_data->quat_J != 0.0f)
                new_data = true;

            if (report_data->quat_K != 0.0f)
                new_data = true;

            if (report_data->quat_real != 1.0f)
                new_data = true;

            if (report_data->integrated_gyro_vel_x != 0.0f)
                new_data = true;

            if (report_data->integrated_gyro_vel_y != 0.0f)
                new_data = true;

            if (report_data->integrated_gyro_vel_z != 0.0f)
                new_data = true;

            return new_data;
        }

        static bool accelerometer_data_is_default(imu_report_data_t* report_data)
        {
            bool new_data = false;

            if (report_data->accel_x != 0.0f)
                new_data = true;

            if (report_data->accel_y != 0.0f)
                new_data = true;

            if (report_data->accel_z != 0.0f)
                new_data = true;

            if (report_data->accel_accuracy != BNO08xAccuracy::UNDEFINED)
                new_data = true;

            return new_data;
        }

        static bool linear_accelerometer_data_is_default(imu_report_data_t* report_data)
        {
            bool new_data = false;

            if (report_data->lin_accel_x != 0.0f)
                new_data = true;

            if (report_data->lin_accel_y != 0.0f)
                new_data = true;

            if (report_data->lin_accel_z != 0.0f)
                new_data = true;

            if (report_data->lin_accel_accuracy != BNO08xAccuracy::UNDEFINED)
                new_data = true;

            return new_data;
        }

        static bool gravity_data_is_default(imu_report_data_t* report_data)
        {
            bool new_data = false;

            if (report_data->grav_x != 0.0f)
                new_data = true;

            if (report_data->grav_y != 0.0f)
                new_data = true;

            if (report_data->grav_z != 0.0f)
                new_data = true;

            if (report_data->grav_accuracy != BNO08xAccuracy::UNDEFINED)
                new_data = true;

            return new_data;
        }

        static void update_report_data(imu_report_data_t* report_data, BNO08x* imu)
        {

            imu->get_quat(report_data->quat_I, report_data->quat_J, report_data->quat_K, report_data->quat_real, report_data->quat_radian_accuracy,
                    report_data->quat_accuracy);
            imu->get_integrated_gyro_velocity(report_data->integrated_gyro_vel_x, report_data->integrated_gyro_vel_y, report_data->integrated_gyro_vel_z);
            imu->get_accel(report_data->accel_x, report_data->accel_y, report_data->accel_z, report_data->accel_accuracy);
            imu->get_linear_accel(report_data->lin_accel_x, report_data->lin_accel_y, report_data->lin_accel_z, report_data->lin_accel_accuracy);
            imu->get_gravity(report_data->grav_x, report_data->grav_y, report_data->grav_z, report_data->grav_accuracy);
            imu->get_calibrated_gyro_velocity(report_data->calib_gyro_vel_x, report_data->calib_gyro_vel_y, report_data->calib_gyro_vel_z, report_data->calib_gyro_accuracy);
        }
};