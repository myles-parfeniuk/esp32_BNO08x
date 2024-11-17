/**
 * @file BNO08xTestHelper.hpp
 * @author Myles Parfeniuk
 */
#pragma once

#include "stdio.h"
#include "BNO08x.hpp"


/**
 * @class BNO08xTestHelper
 * @brief BNO08x unit test helper class.
 * */
class BNO08xTestHelper
{
    private:
        inline static BNO08x* test_imu = nullptr;
        inline static bno08x_config_t imu_cfg;

        static const constexpr char* TAG = "BNO08xTestHelper";

    public:
        /// @brief IMU configuration settings passed into constructor
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

        /**
         * @brief Prints test begin banner.
         *
         * @param TEST_TAG String containing test name.
         *
         * @return void, nothing to return
         */
        static void print_test_start_banner(const char* TEST_TAG)
        {
            printf("------------------------ BEGIN TEST: %s ------------------------\n\r", TEST_TAG);
        }

        /**
         * @brief Prints end begin banner.
         *
         * @param TEST_TAG String containing test name.
         *
         * @return void, nothing to return
         */
        static void print_test_end_banner(const char* TEST_TAG)
        {
            printf("------------------------ END TEST: %s ------------------------\n\r", TEST_TAG);
        }

        /**
         * @brief Prints a message during a test.
         *
         * @param TEST_TAG String containing test name.
         * @param msg String containing message to print.
         *
         * @return void, nothing to return
         */
        static void print_test_msg(const char* TEST_TAG, const char* msg)
        {
            printf("%s: %s: %s\n\r", TAG, TEST_TAG, msg);
        }

        /**
         * @brief Set test imu configuration used with create_test_imu()
         *
         * @param cfg String containing test name.
         *
         * @return void, nothing to return
         */
        static void set_test_imu_cfg(bno08x_config_t cfg)
        {
            imu_cfg = cfg;
        }

        /**
         * @brief Calls BNO08x constructor and creates new test IMU on heap.
         *
         * @return void, nothing to return
         */
        static void create_test_imu()
        {
            if (test_imu != nullptr)
                destroy_test_imu();

            test_imu = new BNO08x();
        }

        /**
         * @brief Deletes test IMU calling deconstructor and releases heap allocated memory.
         *
         * @return void, nothing to return
         */
        static void destroy_test_imu()
        {
            if (test_imu != nullptr)
            {
                delete test_imu;
                test_imu = nullptr;
            }
        }

        /**
         * @brief Deletes test IMU calling deconstructor and releases heap allocated memory.
         *
         * @return Pointer to BNO08x IMU object to test.
         */
        static BNO08x* get_test_imu()
        {
            return test_imu;
        }

        /**
         * @brief Used to call private BNO08x::init_config_args() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_config_args()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_config_args();
        }

        /**
         * @brief Used to call private BNO08x::init_gpio() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_gpio()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_gpio();
        }

        /**
         * @brief Used to call private BNO08x::init_hint_isr() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_hint_isr()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_hint_isr();
        }

        /**
         * @brief Used to call private BNO08x::init_spi() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_spi()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_spi();
        }

        /**
         * @brief Used to call private BNO08x::launch_tasks() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_launch_tasks()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->launch_tasks();
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool rotation_vector_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            // prev report should always contain the default test values as per test structure
            if (report_data->quat_I != default_report_data->quat_I)
                new_data = true;

            if (report_data->quat_J != default_report_data->quat_J)
                new_data = true;

            if (report_data->quat_K != default_report_data->quat_K)
                new_data = true;

            if (report_data->quat_real != default_report_data->quat_real)
                new_data = true;

            if (report_data->quat_accuracy != default_report_data->quat_accuracy)
                new_data = true;

            if (report_data->quat_radian_accuracy != default_report_data->quat_radian_accuracy)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool gyro_integrated_rotation_vector_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->quat_I != default_report_data->quat_I)
                new_data = true;

            if (report_data->quat_J != default_report_data->quat_J)
                new_data = true;

            if (report_data->quat_K != default_report_data->quat_K)
                new_data = true;

            if (report_data->quat_real != default_report_data->quat_real)
                new_data = true;

            if (report_data->integrated_gyro_vel_x != default_report_data->integrated_gyro_vel_x)
                new_data = true;

            if (report_data->integrated_gyro_vel_y != default_report_data->integrated_gyro_vel_y)
                new_data = true;

            if (report_data->integrated_gyro_vel_z != default_report_data->integrated_gyro_vel_z)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool uncalibrated_gyro_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->uncalib_gyro_vel_x != default_report_data->uncalib_gyro_vel_x)
                new_data = true;

            if (report_data->uncalib_gyro_vel_y != default_report_data->uncalib_gyro_vel_y)
                new_data = true;

            if (report_data->uncalib_gyro_vel_z != default_report_data->uncalib_gyro_vel_z)
                new_data = true;

            if (report_data->uncalib_gyro_drift_x != default_report_data->uncalib_gyro_drift_x)
                new_data = true;

            if (report_data->uncalib_gyro_drift_y != default_report_data->uncalib_gyro_drift_y)
                new_data = true;

            if (report_data->uncalib_gyro_drift_z != default_report_data->uncalib_gyro_drift_z)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool calibrated_gyro_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->calib_gyro_vel_x != default_report_data->calib_gyro_vel_x)
                new_data = true;

            if (report_data->calib_gyro_vel_y != default_report_data->calib_gyro_vel_y)
                new_data = true;

            if (report_data->calib_gyro_vel_z != default_report_data->calib_gyro_vel_z)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool accelerometer_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->accel_x != default_report_data->accel_x)
                new_data = true;

            if (report_data->accel_y != default_report_data->accel_y)
                new_data = true;

            if (report_data->accel_z != default_report_data->accel_z)
                new_data = true;

            if (report_data->accel_accuracy != default_report_data->accel_accuracy)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool linear_accelerometer_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->lin_accel_x != default_report_data->lin_accel_x)
                new_data = true;

            if (report_data->lin_accel_y != default_report_data->lin_accel_y)
                new_data = true;

            if (report_data->lin_accel_z != default_report_data->lin_accel_z)
                new_data = true;

            if (report_data->lin_accel_accuracy != default_report_data->lin_accel_accuracy)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool gravity_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->grav_x != default_report_data->grav_x)
                new_data = true;

            if (report_data->grav_y != default_report_data->grav_y)
                new_data = true;

            if (report_data->grav_z != default_report_data->grav_z)
                new_data = true;

            if (report_data->grav_accuracy != default_report_data->grav_accuracy)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool magnetometer_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->magf_x != default_report_data->magf_x)
                new_data = true;

            if (report_data->magf_y != default_report_data->magf_y)
                new_data = true;

            if (report_data->magf_z != default_report_data->magf_z)
                new_data = true;

            if (report_data->magf_accuracy != default_report_data->magf_accuracy)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool step_counter_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->step_count != default_report_data->step_count)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool stability_classifier_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->stability_classifier != default_report_data->stability_classifier)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Checks if report_data matches the default states stored within prev_report_data data for respective report.
         *
         * @param report_data Current report data.
         * @param default_report_data Default report data to compare (should always contain default values)
         *
         * @return True if new data was received for respective report.
         */
        static bool activity_classifier_data_is_new(imu_report_data_t* report_data, imu_report_data_t* default_report_data)
        {
            bool new_data = false;

            if (report_data->activity_classifier != default_report_data->activity_classifier)
                new_data = true;

            return new_data;
        }

        /**
         * @brief Updates report data with calls relevant test_imu methods.
         *
         * @param report_data Pointer to imu_report_data_t struct to save report data.
         *
         * @return void, noting to return.
         */
        static void update_report_data(imu_report_data_t* report_data)
        {

            test_imu->get_quat(report_data->quat_I, report_data->quat_J, report_data->quat_K, report_data->quat_real,
                    report_data->quat_radian_accuracy, report_data->quat_accuracy);
            test_imu->get_integrated_gyro_velocity(
                    report_data->integrated_gyro_vel_x, report_data->integrated_gyro_vel_y, report_data->integrated_gyro_vel_z);
            test_imu->get_accel(report_data->accel_x, report_data->accel_y, report_data->accel_z, report_data->accel_accuracy);
            test_imu->get_linear_accel(report_data->lin_accel_x, report_data->lin_accel_y, report_data->lin_accel_z, report_data->lin_accel_accuracy);
            test_imu->get_gravity(report_data->grav_x, report_data->grav_y, report_data->grav_z, report_data->grav_accuracy);
            test_imu->get_calibrated_gyro_velocity(report_data->calib_gyro_vel_x, report_data->calib_gyro_vel_y, report_data->calib_gyro_vel_z);
            test_imu->get_uncalibrated_gyro_velocity(report_data->uncalib_gyro_vel_x, report_data->uncalib_gyro_vel_y,
                    report_data->uncalib_gyro_vel_z, report_data->uncalib_gyro_drift_x, report_data->uncalib_gyro_drift_y,
                    report_data->uncalib_gyro_drift_z);
            test_imu->get_magf(report_data->magf_x, report_data->magf_y, report_data->magf_z, report_data->magf_accuracy);
            test_imu->get_raw_mems_gyro(report_data->raw_mems_gyro_x, report_data->raw_mems_gyro_y, report_data->raw_mems_gyro_z);
            report_data->step_count = test_imu->get_step_count();
            report_data->stability_classifier = test_imu->get_stability_classifier();
            report_data->activity_classifier = test_imu->get_activity_classifier();
        }

        /**
         * @brief Resets internal test imu data with test defaults.
         *
         * @return void, nothing to return.
         */
        static void reset_all_imu_data_to_test_defaults()
        {
            static const constexpr uint16_t TEST_VAL_UINT16 = 65535U;
            static const constexpr uint16_t TEST_VAL_UINT8 = 255;
            test_imu->time_stamp = 0UL;

            test_imu->raw_accel_X = TEST_VAL_UINT16;
            test_imu->raw_accel_Y = TEST_VAL_UINT16;
            test_imu->raw_accel_Z = TEST_VAL_UINT16;
            test_imu->accel_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            test_imu->raw_lin_accel_X = TEST_VAL_UINT16;
            test_imu->raw_lin_accel_Y = TEST_VAL_UINT16;
            test_imu->raw_lin_accel_Z = TEST_VAL_UINT16;
            test_imu->accel_lin_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            test_imu->raw_calib_gyro_X = TEST_VAL_UINT16;
            test_imu->raw_calib_gyro_Y = TEST_VAL_UINT16;
            test_imu->raw_calib_gyro_Z = TEST_VAL_UINT16;

            // reset quaternion to nan
            test_imu->raw_quat_I = TEST_VAL_UINT16;
            test_imu->raw_quat_J = TEST_VAL_UINT16;
            test_imu->raw_quat_K = TEST_VAL_UINT16;
            test_imu->raw_quat_real = TEST_VAL_UINT16;
            test_imu->raw_quat_radian_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);
            test_imu->quat_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            test_imu->integrated_gyro_velocity_X = TEST_VAL_UINT16;
            test_imu->integrated_gyro_velocity_Y = TEST_VAL_UINT16;
            test_imu->integrated_gyro_velocity_Z = TEST_VAL_UINT16;

            test_imu->gravity_X = TEST_VAL_UINT16;
            test_imu->gravity_Y = TEST_VAL_UINT16;
            test_imu->gravity_Z = TEST_VAL_UINT16;
            test_imu->gravity_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            test_imu->raw_uncalib_gyro_X = TEST_VAL_UINT16;
            test_imu->raw_uncalib_gyro_Y = TEST_VAL_UINT16;
            test_imu->raw_uncalib_gyro_Z = TEST_VAL_UINT16;
            test_imu->raw_bias_X = TEST_VAL_UINT16;
            test_imu->raw_bias_Y = TEST_VAL_UINT16;
            test_imu->raw_bias_Z = TEST_VAL_UINT16;

            test_imu->raw_magf_X = TEST_VAL_UINT16;
            test_imu->raw_magf_Y = TEST_VAL_UINT16;
            test_imu->raw_magf_Z = TEST_VAL_UINT16;
            test_imu->magf_accuracy = static_cast<uint16_t>(BNO08xAccuracy::UNDEFINED);

            test_imu->tap_detector = TEST_VAL_UINT8;
            test_imu->step_count = TEST_VAL_UINT16;
            test_imu->stability_classifier = TEST_VAL_UINT8;
            test_imu->activity_classifier = TEST_VAL_UINT8;

            test_imu->mems_raw_accel_X = TEST_VAL_UINT16;
            test_imu->mems_raw_accel_Y = TEST_VAL_UINT16;
            test_imu->mems_raw_accel_Z = TEST_VAL_UINT16;

            test_imu->mems_raw_gyro_X = TEST_VAL_UINT16;
            test_imu->mems_raw_gyro_Y = TEST_VAL_UINT16;
            test_imu->mems_raw_gyro_Z = TEST_VAL_UINT16;

            test_imu->mems_raw_magf_X = TEST_VAL_UINT16;
            test_imu->mems_raw_magf_Y = TEST_VAL_UINT16;
            test_imu->mems_raw_magf_Z = TEST_VAL_UINT16;
        }

        /**
         * @brief Converts BNO08xAccuracy enum class object to string.
         *
         * @param report_data BNO08xAccuracy object to convert to string.
         *
         * @return The resulting string conversion.
         */
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