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

                float uncalib_gyro_vel_x;
                float uncalib_gyro_vel_y;
                float uncalib_gyro_vel_z;
                float uncalib_gyro_drift_x;
                float uncalib_gyro_drift_y;
                float uncalib_gyro_drift_z;

                float magf_x;
                float magf_y;
                float magf_z;
                BNO08xAccuracy magf_accuracy;

                uint16_t raw_mems_gyro_x;
                uint16_t raw_mems_gyro_y;
                uint16_t raw_mems_gyro_z;

                uint16_t step_count;
                uint8_t stability_classifier;
                uint8_t activity_classifier;

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

        static bool rotation_vector_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            // prev report should always contain the default test values as per test structure
            if (report_data->quat_I != prev_report_data->quat_I)
                new_data = true;

            if (report_data->quat_J != prev_report_data->quat_J)
                new_data = true;

            if (report_data->quat_K != prev_report_data->quat_K)
                new_data = true;

            if (report_data->quat_real != prev_report_data->quat_real)
                new_data = true;

            if (report_data->quat_accuracy != prev_report_data->quat_accuracy)
                new_data = true;

            if (report_data->quat_radian_accuracy != prev_report_data->quat_radian_accuracy)
                new_data = true;

            return new_data;
        }

        static bool gyro_integrated_rotation_vector_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->quat_I != prev_report_data->quat_I)
                new_data = true;

            if (report_data->quat_J != prev_report_data->quat_J)
                new_data = true;

            if (report_data->quat_K != prev_report_data->quat_K)
                new_data = true;

            if (report_data->quat_real != prev_report_data->quat_real)
                new_data = true;

            if (report_data->integrated_gyro_vel_x != prev_report_data->integrated_gyro_vel_x)
                new_data = true;

            if (report_data->integrated_gyro_vel_y != prev_report_data->integrated_gyro_vel_y)
                new_data = true;

            if (report_data->integrated_gyro_vel_z != prev_report_data->integrated_gyro_vel_z)
                new_data = true;

            return new_data;
        }

        static bool uncalibrated_gyro_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->uncalib_gyro_vel_x != prev_report_data->uncalib_gyro_vel_x)
                new_data = true;

            if (report_data->uncalib_gyro_vel_y != prev_report_data->uncalib_gyro_vel_y)
                new_data = true;

            if (report_data->uncalib_gyro_vel_z != prev_report_data->uncalib_gyro_vel_z)
                new_data = true;

            if (report_data->uncalib_gyro_drift_x != prev_report_data->uncalib_gyro_drift_x)
                new_data = true;

            if (report_data->uncalib_gyro_drift_y != prev_report_data->uncalib_gyro_drift_y)
                new_data = true;

            if (report_data->uncalib_gyro_drift_z != prev_report_data->uncalib_gyro_drift_z)
                new_data = true;

            return new_data;
        }

        static bool calibrated_gyro_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->calib_gyro_vel_x != prev_report_data->calib_gyro_vel_x)
                new_data = true;

            if (report_data->calib_gyro_vel_y != prev_report_data->calib_gyro_vel_y)
                new_data = true;

            if (report_data->calib_gyro_vel_z != prev_report_data->calib_gyro_vel_z)
                new_data = true;

            return new_data;
        }

        static bool accelerometer_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->accel_x != prev_report_data->accel_x)
                new_data = true;

            if (report_data->accel_y != prev_report_data->accel_y)
                new_data = true;

            if (report_data->accel_z != prev_report_data->accel_z)
                new_data = true;

            if (report_data->accel_accuracy != prev_report_data->accel_accuracy)
                new_data = true;

            return new_data;
        }

        static bool linear_accelerometer_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->lin_accel_x != prev_report_data->lin_accel_x)
                new_data = true;

            if (report_data->lin_accel_y != prev_report_data->lin_accel_y)
                new_data = true;

            if (report_data->lin_accel_z != prev_report_data->lin_accel_z)
                new_data = true;

            if (report_data->lin_accel_accuracy != prev_report_data->lin_accel_accuracy)
                new_data = true;

            return new_data;
        }

        static bool gravity_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->grav_x != prev_report_data->grav_x)
                new_data = true;

            if (report_data->grav_y != prev_report_data->grav_y)
                new_data = true;

            if (report_data->grav_z != prev_report_data->grav_z)
                new_data = true;

            if (report_data->grav_accuracy != prev_report_data->grav_accuracy)
                new_data = true;

            return new_data;
        }

        static bool magnetometer_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->magf_x != prev_report_data->magf_x)
                new_data = true;

            if (report_data->magf_y != prev_report_data->magf_y)
                new_data = true;

            if (report_data->magf_z != prev_report_data->magf_z)
                new_data = true;

            if (report_data->magf_accuracy != prev_report_data->magf_accuracy)
                new_data = true;

            return new_data;
        }

        static bool step_detector_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->step_count != prev_report_data->step_count)
                new_data = true;

            return new_data;
        }

        static bool stability_classifier_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->stability_classifier != prev_report_data->stability_classifier)
                new_data = true;

            return new_data;
        }

        static bool activity_classifier_data_is_default(imu_report_data_t* report_data, imu_report_data_t* prev_report_data)
        {
            bool new_data = false;

            if (report_data->activity_classifier != prev_report_data->activity_classifier)
                new_data = true;

            return new_data;
        }

        static void update_report_data(imu_report_data_t* report_data, BNO08x* imu)
        {

            imu->get_quat(report_data->quat_I, report_data->quat_J, report_data->quat_K, report_data->quat_real, report_data->quat_radian_accuracy,
                    report_data->quat_accuracy);
            imu->get_integrated_gyro_velocity(
                    report_data->integrated_gyro_vel_x, report_data->integrated_gyro_vel_y, report_data->integrated_gyro_vel_z);
            imu->get_accel(report_data->accel_x, report_data->accel_y, report_data->accel_z, report_data->accel_accuracy);
            imu->get_linear_accel(report_data->lin_accel_x, report_data->lin_accel_y, report_data->lin_accel_z, report_data->lin_accel_accuracy);
            imu->get_gravity(report_data->grav_x, report_data->grav_y, report_data->grav_z, report_data->grav_accuracy);
            imu->get_calibrated_gyro_velocity(report_data->calib_gyro_vel_x, report_data->calib_gyro_vel_y, report_data->calib_gyro_vel_z);
            imu->get_uncalibrated_gyro_velocity(report_data->uncalib_gyro_vel_x, report_data->uncalib_gyro_vel_y, report_data->uncalib_gyro_vel_z,
                    report_data->uncalib_gyro_drift_x, report_data->uncalib_gyro_drift_y, report_data->uncalib_gyro_drift_z);
            imu->get_magf(report_data->magf_x, report_data->magf_y, report_data->magf_z, report_data->magf_accuracy);
            imu->get_raw_mems_gyro(report_data->raw_mems_gyro_x, report_data->raw_mems_gyro_y, report_data->raw_mems_gyro_z);
            report_data->step_count = imu->get_step_count();
            report_data->stability_classifier = imu->get_stability_classifier();
            report_data->activity_classifier = imu->get_activity_classifier(); 
        }

        static void reset_all_imu_data_to_test_defaults(BNO08x* imu)
        {
            static const constexpr uint16_t TEST_VAL_UINT16 = 65535U;
            static const constexpr uint16_t TEST_VAL_UINT8 = 255;
            imu->time_stamp = 0UL;

            imu->raw_accel_X = TEST_VAL_UINT16;
            imu->raw_accel_Y = TEST_VAL_UINT16;
            imu->raw_accel_Z = TEST_VAL_UINT16;
            imu->accel_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            imu->raw_lin_accel_X = TEST_VAL_UINT16;
            imu->raw_lin_accel_Y = TEST_VAL_UINT16;
            imu->raw_lin_accel_Z = TEST_VAL_UINT16;
            imu->accel_lin_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            imu->raw_calib_gyro_X = TEST_VAL_UINT16;
            imu->raw_calib_gyro_Y = TEST_VAL_UINT16;
            imu->raw_calib_gyro_Z = TEST_VAL_UINT16;

            // reset quaternion to nan
            imu->raw_quat_I = TEST_VAL_UINT16;
            imu->raw_quat_J = TEST_VAL_UINT16;
            imu->raw_quat_K = TEST_VAL_UINT16;
            imu->raw_quat_real = TEST_VAL_UINT16;
            imu->raw_quat_radian_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);
            imu->quat_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            imu->integrated_gyro_velocity_X = TEST_VAL_UINT16;
            imu->integrated_gyro_velocity_Y = TEST_VAL_UINT16;
            imu->integrated_gyro_velocity_Z = TEST_VAL_UINT16;

            imu->gravity_X = TEST_VAL_UINT16;
            imu->gravity_Y = TEST_VAL_UINT16;
            imu->gravity_Z = TEST_VAL_UINT16;
            imu->gravity_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            imu->raw_uncalib_gyro_X = TEST_VAL_UINT16;
            imu->raw_uncalib_gyro_Y = TEST_VAL_UINT16;
            imu->raw_uncalib_gyro_Z = TEST_VAL_UINT16;
            imu->raw_bias_X = TEST_VAL_UINT16;
            imu->raw_bias_Y = TEST_VAL_UINT16;
            imu->raw_bias_Z = TEST_VAL_UINT16;

            imu->raw_magf_X = TEST_VAL_UINT16;
            imu->raw_magf_Y = TEST_VAL_UINT16;
            imu->raw_magf_Z = TEST_VAL_UINT16;
            imu->magf_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            imu->tap_detector = TEST_VAL_UINT8;
            imu->step_count = TEST_VAL_UINT16;
            imu->stability_classifier = TEST_VAL_UINT8;
            imu->activity_classifier = TEST_VAL_UINT8;

            imu->mems_raw_accel_X = TEST_VAL_UINT16;
            imu->mems_raw_accel_Y = TEST_VAL_UINT16;
            imu->mems_raw_accel_Z = TEST_VAL_UINT16;

            imu->mems_raw_gyro_X = TEST_VAL_UINT16;
            imu->mems_raw_gyro_Y = TEST_VAL_UINT16;
            imu->mems_raw_gyro_Z = TEST_VAL_UINT16;

            imu->mems_raw_magf_X = TEST_VAL_UINT16;
            imu->mems_raw_magf_Y = TEST_VAL_UINT16;
            imu->mems_raw_magf_Z = TEST_VAL_UINT16;
        }

        static const char* BNO08xAccuracy_to_str(BNO08xAccuracy accuracy)
        {
            switch (accuracy)
            {
                case BNO08xAccuracy::LOW:
                    return "LOW";
                case BNO08xAccuracy::MED:
                    return "MED";
                case BNO08xAccuracy::HIGH:
                    return "HIGH";
                case BNO08xAccuracy::UNDEFINED:
                    return "UNDEFINED";
                default:
                    return "UNKNOWN"; // For undefined cases or future-proofing
            }
        };
};