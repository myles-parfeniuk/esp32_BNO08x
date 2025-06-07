/**
 * @file BNO08xTestHelper.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// in-house includes
#include "BNO08x.hpp"

/**
 * @class BNO08xTestHelper
 *
 * @brief BNO08x unit test helper class.
 * */
class BNO08xTestHelper
{
    private:
        static inline uint8_t test_imu_buffer[sizeof(BNO08x)];
        inline static BNO08x* test_imu = nullptr;
        inline static bno08x_config_t imu_cfg;

        static const constexpr char* TAG = "BNO08xTestHelper";

    public:
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

            test_imu = new (&test_imu_buffer) BNO08x();
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
                test_imu->~BNO08x();  
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
         * @brief Used to call private BNO08x::init_tasks() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_tasks()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_tasks();
        }

        /**
         * @brief Used to call private BNO08x::init_tasks() member for tests.
         *
         * @return ESP_OK if init succeeded.
         */
        static esp_err_t call_init_sh2_HAL()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->init_sh2_HAL();
        }

        /**
         * @brief Used to call private BNO08x::deinit_gpio() member for tests.
         *
         * @return ESP_OK if deinit succeeded.
         */
        static esp_err_t call_deinit_gpio()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->deinit_gpio();
        }

        /**
         * @brief Used to call private BNO08x::deinit_hint_isr() member for tests.
         *
         * @return ESP_OK if deinit succeeded.
         */
        static esp_err_t call_deinit_hint_isr()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->deinit_hint_isr();
        }

        /**
         * @brief Used to call private BNO08x::deinit_spi() member for tests.
         *
         * @return ESP_OK if deinit succeeded.
         */
        static esp_err_t call_deinit_spi()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->deinit_spi();
        }

        /**
         * @brief Used to call private BNO08x::deinit_tasks() member for tests.
         *
         * @return ESP_OK if deinit succeeded.
         */
        static esp_err_t call_deinit_tasks()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->deinit_tasks();
        }

        /**
         * @brief Used to call private BNO08x::deinit_tasks() member for tests.
         *
         * @return ESP_OK if deinit succeeded.
         */
        static esp_err_t call_deinit_sh2_HAL()
        {
            if (test_imu == nullptr)
                return ESP_FAIL;

            return test_imu->deinit_sh2_HAL();
        }
};