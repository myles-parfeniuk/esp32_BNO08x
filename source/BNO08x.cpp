#include "BNO08x.hpp"

/**
 * @brief BNO08x imu constructor.
 *
 * Construct a BNO08x object for managing a BNO08x sensor.
 *
 * @param imu_config Configuration settings (optional), default settings can be seen in bno08x_config_t
 * @return void, nothing to return
 */
BNO08x::BNO08x(bno08x_config_t imu_config)
    : accelerometer(this, SH2_ACCELEROMETER, 0UL, EVT_GRP_RPT_ACCELEROMETER_BIT)
    , linear_accelerometer(this, SH2_LINEAR_ACCELERATION, 0UL, EVT_GRP_RPT_LINEAR_ACCELEROMETER_BIT)
    , gravity(this, SH2_GRAVITY, 0UL, EVT_GRP_RPT_GRAVITY_BIT)
    , cal_magnetometer(this, SH2_MAGNETIC_FIELD_CALIBRATED, 0UL, EVT_GRP_RPT_CAL_MAGNETOMETER_BIT)
    , uncal_magnetometer(this, SH2_MAGNETIC_FIELD_UNCALIBRATED, 0UL, EVT_GRP_RPT_UNCAL_MAGNETOMETER_BIT)
    , cal_gyro(this, SH2_GYROSCOPE_CALIBRATED, 0UL, EVT_GRP_RPT_CAL_GYRO_BIT)
    , uncal_gyro(this, SH2_GYROSCOPE_UNCALIBRATED, 0UL, EVT_GRP_RPT_UNCAL_GYRO_BIT)
    , rv(this, SH2_ROTATION_VECTOR, 0UL, EVT_GRP_RPT_RV_BIT)
    , rv_game(this, SH2_GAME_ROTATION_VECTOR, 0UL, EVT_GRP_RPT_RV_GAME_BIT)
    , rv_ARVR_stabilized(this, SH2_ARVR_STABILIZED_RV, 0UL, EVT_GRP_RPT_RV_ARVR_S_BIT)
    , rv_ARVR_stabilized_game(this, SH2_ARVR_STABILIZED_GRV, 0UL, EVT_GRP_RPT_RV_ARVR_S_GAME_BIT)
    , rv_gyro_integrated(this, SH2_GYRO_INTEGRATED_RV, 0UL, EVT_GRP_RPT_GYRO_INTEGRATED_RV_BIT)
    , rv_geomagnetic(this, SH2_GEOMAGNETIC_ROTATION_VECTOR, 0UL, EVT_GRP_RPT_GEOMAG_RV_BIT)
    , raw_gyro(this, SH2_RAW_GYROSCOPE, 0UL, EVT_GRP_RPT_RAW_GYRO_BIT)
    , raw_accelerometer(this, SH2_RAW_ACCELEROMETER, 0UL, EVT_GRP_RPT_RAW_ACCELEROMETER_BIT)
    , raw_magnetometer(this, SH2_RAW_MAGNETOMETER, 0UL, EVT_GRP_RPT_RAW_MAGNETOMETER_BIT)
    , step_counter(this, SH2_STEP_COUNTER, 0UL, EVT_GRP_RPT_STEP_COUNTER_BIT)
    , activity_classifier(this, SH2_PERSONAL_ACTIVITY_CLASSIFIER, 0UL, EVT_GRP_RPT_ACTIVITY_CLASSIFIER_BIT)
    , stability_classifier(this, SH2_STABILITY_CLASSIFIER, 0UL, EVT_GRP_RPT_STABILITY_CLASSIFIER_BIT)
    , shake_detector(this, SH2_SHAKE_DETECTOR, 0UL, EVT_GRP_RPT_SHAKE_DETECTOR_BIT)
    , tap_detector(this, SH2_TAP_DETECTOR, 0UL, EVT_GRP_RPT_TAP_DETECTOR_BIT)
    , data_proc_task_hdl(NULL)
    , sh2_HAL_service_task_hdl(NULL)
    , cb_task_hdl(NULL)
    , sh2_HAL_lock(xSemaphoreCreateMutex())
    , data_lock(xSemaphoreCreateMutex())
    , sem_kill_tasks(NULL)
    , evt_grp_bno08x_task(xEventGroupCreate())
    , evt_grp_report_en(xEventGroupCreate())
    , queue_rx_sensor_event(xQueueCreate(5, sizeof(sh2_SensorEvent_t)))
    , queue_cb_report_id(xQueueCreate(5, sizeof(uint8_t)))
    , imu_config(imu_config)
{
}

/**
 * @brief BNO08x imu deconstructor.
 *
 * Deconstructs a BNO08x object and releases any utilized resources.
 *
 * @return void, nothing to return.
 */
BNO08x::~BNO08x()
{
    // disable interrupts before beginning so we can ensure SPI transaction doesn't attempt to run
    gpio_intr_disable(imu_config.io_int);

    // deinitialize sh2 HAL if it has been initialized
    ESP_ERROR_CHECK(deinit_sh2_HAL());

    // deinitialize spi if has been initialized
    ESP_ERROR_CHECK(deinit_spi());

    // deinitialize hint ISR if it has been initialized
    ESP_ERROR_CHECK(deinit_hint_isr());

    // deinitialize GPIO if they have been initialized
    ESP_ERROR_CHECK(deinit_gpio());

    // deinitialize tasks if they have been initialized
    ESP_ERROR_CHECK(deinit_tasks());

    // delete all semaphores
    vSemaphoreDelete(sh2_HAL_lock);
    vSemaphoreDelete(data_lock);
    if (sem_kill_tasks != NULL)
        vSemaphoreDelete(sem_kill_tasks);

    // delete event groups
    vEventGroupDelete(evt_grp_bno08x_task);
    vEventGroupDelete(evt_grp_report_en);

    // delete all queues
    vQueueDelete(queue_rx_sensor_event);
    vQueueDelete(queue_cb_report_id);

    // clear callback lists
    cb_list.clear();
}

/**
 * @brief Initializes BNO08x sensor
 *
 * Resets sensor and goes through initialization process.
 * Configures GPIO, required ISRs, and launches two tasks, one to manage SPI transactions, another to process any received data.
 *
 * @return True if initialization was success, false if otherwise.
 */
bool BNO08x::initialize()
{

    // initialize configuration arguments
    if (init_config_args() != ESP_OK)
        return false;

    // initialize GPIO
    if (init_gpio() != ESP_OK)
        return false;

    // initialize HINT ISR
    if (init_hint_isr() != ESP_OK)
        return false;

    // initialize SPI
    if (init_spi() != ESP_OK)
        return false;

    // initialize SH2 HAL
    if (init_sh2_HAL() != ESP_OK)
        return false;

    // initialize tasks
    if (init_tasks() != ESP_OK)
        return false;

        // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    ESP_LOGI(TAG, "Successfully initialized....");
    #endif
    // clang-format on

    return true;
}

/**
 * @brief Static function used to launch data processing task.
 *
 * Used such that data_proc_task() can be non-static class member.
 *
 * @param arg void pointer to BNO08x imu object
 * @return void, nothing to return
 */
void BNO08x::data_proc_task_trampoline(void* arg)
{
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    imu->data_proc_task();       // launch data processing task task from object
}

/**
 * @brief Task responsible for parsing/handling sensor events sent by SH2 HAL and updating data that is returned to user.
 *
 * @return void, nothing to return
 */
void BNO08x::data_proc_task()
{
    EventBits_t evt_grp_bno08x_task_bits = 0U;
    sh2_SensorEvent_t sensor_evt;
    sh2_SensorValue_t sensor_val;

    while (1)
    {
        if (xQueueReceive(queue_rx_sensor_event, &sensor_evt, portMAX_DELAY) == pdTRUE)
        {
            evt_grp_bno08x_task_bits = xEventGroupGetBits(evt_grp_bno08x_task);

            if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING)
            {

                if (sh2_decodeSensorEvent(&sensor_val, &sensor_evt) != SH2_ERR)
                    handle_sensor_report(&sensor_val);
            }
            else
            {
                break; // exit loop, deconstructor requested task to commit self-deletion
            }
        }
    }

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    vTaskDelete(NULL);
}

/**
 * @brief Static function used to launch sh2 HAL service task.
 *
 * Used such that sh2_HAL_service_task() can be non-static class member.
 *
 * @param arg void pointer to BNO08x imu object
 * @return void, nothing to return
 */
void BNO08x::sh2_HAL_service_task_trampoline(void* arg)
{
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    imu->sh2_HAL_service_task(); // launch data processing task task from object
}

/**
 * @brief Task responsible for calling shtp_service() when HINT is asserted to dispatch any sh2 HAL lib callbacks.
 *
 * @return void, nothing to return
 */
void BNO08x::sh2_HAL_service_task()
{
    EventBits_t evt_grp_bno08x_task_bits = 0U;

    while (1)
    {
        xEventGroupWaitBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        evt_grp_bno08x_task_bits = xEventGroupGetBits(evt_grp_bno08x_task);

        if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING)
        {
            if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASK_RESET_OCCURRED)
                if (!re_enable_reports())
                {
                    // clang-format off
                    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
                    ESP_LOGE(TAG, "Failed to re-enable enabled reports after IMU reset.");
                    #endif
                    // clang-format on
                }

            lock_sh2_HAL();
            sh2_service();
            unlock_sh2_HAL();
        }
        else
        {
            break; // exit loop, deconstructor requested task to commit self-deletion
        }
    }

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    vTaskDelete(NULL);
}

/**
 * @brief Static function used to launch cb_task task.
 *
 * Used such that cb_task() can be non-static class member.
 *
 * @param arg void pointer to BNO08x imu object
 * @return void, nothing to return
 */
void BNO08x::cb_task_trampoline(void* arg)
{
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    imu->cb_task();              // launch data processing task task from object
}

/**
 * @brief Task responsible for executing callbacks registered with register_cb().
 *
 * @return void, nothing to return
 */
void BNO08x::cb_task()
{
    EventBits_t evt_grp_bno08x_task_bits = 0U;
    uint8_t rpt_ID;

    while (1)
    {
        xQueueReceive(queue_cb_report_id, &rpt_ID, portMAX_DELAY);

        evt_grp_bno08x_task_bits = xEventGroupGetBits(evt_grp_bno08x_task);

        if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING)
        {
            // execute callbacks
            for (auto& cb_entry : cb_list)
            {
                // only execute callback if it is registered to this report or all reports
                if ((cb_entry.report_ID == 0) || (cb_entry.report_ID == rpt_ID))
                {
                    std::visit(
                            [rpt_ID](auto&& cb_fxn)
                            {
                                if constexpr (std::is_invocable_v<decltype(cb_fxn)>)
                                {
                                    // Handles std::function<void()> (i.e., no parameters)
                                    cb_fxn();
                                }
                                else
                                {
                                    // Handles std::function<void(uint8_t)> (i.e., with a report ID)
                                    cb_fxn(rpt_ID);
                                }
                            },
                            cb_entry.cb);
                }
            }
        }
        else
        {
            break; // exit loop, deconstructor requested task to commit self-deletion
        }
    }

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    vTaskDelete(NULL);
}

/**
 * @brief Locks sh2 HAL lib to only allow the calling task to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08x::lock_sh2_HAL()
{
    xSemaphoreTake(sh2_HAL_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks sh2 HAL lib to allow other tasks to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08x::unlock_sh2_HAL()
{
    xSemaphoreGive(sh2_HAL_lock);
}

/**
 * @brief Locks locks user data to only allow the calling task to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08x::lock_user_data()
{
    xSemaphoreTake(data_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks user data to allow other tasks to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08x::unlock_user_data()
{
    xSemaphoreGive(data_lock);
}

/**
 * @brief Parses receieved report and updates uer data with it.
 *
 * @return void, nothing to return
 */
void BNO08x::handle_sensor_report(sh2_SensorValue_t* sensor_val)
{
    uint8_t rpt_ID = sensor_val->sensorId;

    // update respective report with new data
    usr_reports.at(rpt_ID)->update_data(sensor_val);

    // send report ids to cb_task for callback execution (only if this report is enabled)
    if (usr_reports.at(rpt_ID)->rpt_bit & xEventGroupGetBits(evt_grp_report_en))
    {
        if (cb_list.size() != 0)
            if (xQueueSend(queue_cb_report_id, &rpt_ID, 0) != pdTRUE)
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGE(TAG, "Callback queue full, callback execution for report missed.");
                #endif
                // clang-format on
            }
    }
}

/**
 * @brief Initializes required esp-idf SPI data structures with values from user passed bno08x_config_t struct.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_config_args()
{
    if ((imu_config.io_cs == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, CS GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_miso == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, MISO GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_mosi == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, MOSI GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_sclk == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, SCLK GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    if ((imu_config.io_rst == GPIO_NUM_NC))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "RST GPIO cannot be unassigned.");
        #endif
        // clang-format on

        return ESP_ERR_INVALID_ARG;
    }

    // SPI bus config
    bus_config.mosi_io_num = imu_config.io_mosi; // assign mosi gpio pin
    bus_config.miso_io_num = imu_config.io_miso; // assign miso gpio pin
    bus_config.sclk_io_num = imu_config.io_sclk; // assign sclk gpio pin
    bus_config.quadhd_io_num = -1;               // hold signal gpio (not used)
    bus_config.quadwp_io_num = -1;               // write protect signal gpio (not used)

    // SPI slave device specific config
    imu_spi_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus high when idle)

    if (imu_config.sclk_speed > SCLK_MAX_SPEED) // max sclk speed of 3MHz for BNO08x
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Max SPI clock speed exceeded, %ld overwritten with 3MHz", imu_config.sclk_speed);
        #endif
        // clang-format on

        imu_config.sclk_speed = SCLK_MAX_SPEED;
    }

    imu_spi_config.clock_source = SPI_CLK_SRC_DEFAULT;
    imu_spi_config.clock_speed_hz = imu_config.sclk_speed; // assign SCLK speed
    imu_spi_config.address_bits = 0;                       // 0 address bits, not using this system
    imu_spi_config.command_bits = 0;                       // 0 command bits, not using this system
    imu_spi_config.spics_io_num = -1;                      // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
                                                           // driver, it must be handled via calls to gpio pins
    imu_spi_config.queue_size = static_cast<int>(CONFIG_ESP32_BNO08X_SPI_QUEUE_SZ); // set max allowable queued SPI transactions

    return ESP_OK;
}

/**
 * @brief Initializes required gpio inputs.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio_inputs()
{
    esp_err_t ret = ESP_OK;

    // configure input(s) (HINT)
    gpio_config_t inputs_config;
    inputs_config.pin_bit_mask = (1ULL << imu_config.io_int);
    inputs_config.mode = GPIO_MODE_INPUT;
    inputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    inputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputs_config.intr_type = GPIO_INTR_NEGEDGE;

    ret = gpio_config(&inputs_config);

    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to configure HINT gpio.");
        #endif
        // clang-format on
    }
    else
    {
        init_status.gpio_inputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up
    }

    return ret;
}

/**
 * @brief Initializes required gpio outputs.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio_outputs()
{
    esp_err_t ret = ESP_OK;

    // configure output(s) (CS, RST, and WAKE)
    gpio_config_t outputs_config;

    outputs_config.pin_bit_mask = (imu_config.io_wake != GPIO_NUM_NC)
                                          ? ((1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst) | (1ULL << imu_config.io_wake))
                                          : ((1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst));

    outputs_config.mode = GPIO_MODE_OUTPUT;
    outputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs_config.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&outputs_config);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to configure CS, RST, and WAKE (if used) gpio.");
        #endif
        // clang-format on
    }
    else
    {
        init_status.gpio_outputs = true; // set gpio_inputs to initialized such that deconstructor knows to clean them up
    }

    return ret;
}

/**
 * @brief Initializes required gpio.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_gpio()
{
    esp_err_t ret = ESP_OK;

    /*GPIO config for pins not controlled by SPI peripheral*/

    ret = init_gpio_outputs();
    if (ret != ESP_OK)
        return ret;

    ret = init_gpio_inputs();
    if (ret != ESP_OK)
        return ret;

    gpio_set_level(imu_config.io_cs, 1);
    gpio_set_level(imu_config.io_rst, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    return ret;
}

/**
 * @brief Initializes host interrupt ISR.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_hint_isr()
{
    esp_err_t ret = ESP_OK;

    // check if installation of ISR service has been requested by user (default is true)
    if (imu_config.install_isr_service)
        ret = gpio_install_isr_service(0); // install isr service

    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to install global ISR service.");
        #endif
        // clang-format on

        return ret;
    }
    else
    {
        init_status.isr_service = true; // set isr service to initialized such that deconstructor knows to clean it up (this will be ignored if
                                        // imu_config.install_isr_service == false)
    }

    ret = gpio_isr_handler_add(imu_config.io_int, hint_handler, (void*) this);
    if (ret != ESP_OK)
    {

        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add hint_handler ISR.");
        #endif
        // clang-format on

        return ret;
    }
    else
    {
        init_status.isr_handler = true; // set isr handler to initialized such that deconstructor knows to clean it up
    }

    return ret;
}

/**
 * @brief Initializes data_proc_task.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_tasks()
{
    BaseType_t task_created = pdFALSE;

    xEventGroupSetBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASKS_RUNNING);

    // launch data processing task
    task_created = xTaskCreate(&data_proc_task_trampoline, "bno08x_data_processing_task", DATA_PROC_TASK_SZ, this, 6, &data_proc_task_hdl);

    if (task_created != pdTRUE)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, data_proc_task failed to launch.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }
    else
    {
        init_status.task_init_cnt++;
        init_status.data_proc_task = true;
    }

    // launch data processing task
    task_created =
            xTaskCreate(&sh2_HAL_service_task_trampoline, "bno08x_sh2_HAL_service_task", SH2_HAL_SERVICE_TASK_SZ, this, 7, &sh2_HAL_service_task_hdl);

    if (task_created != pdTRUE)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, sh2_HAL_service_task failed to launch.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }
    else
    {
        init_status.task_init_cnt++;
        init_status.sh2_HAL_service_task = true;
    }

    // launch cb task
    task_created = xTaskCreate(&cb_task_trampoline, "bno08x_cb_task", CB_TASK_SZ, this, 5, &cb_task_hdl);

    if (task_created != pdTRUE)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, cb_task failed to launch.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }
    else
    {
        init_status.task_init_cnt++;
        init_status.cb_task = true;
    }

    return ESP_OK;
}

/**
 * @brief Initializes SPI.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_spi()
{
    esp_err_t ret = ESP_OK;

    // initialize the spi peripheral
    ret = spi_bus_initialize(imu_config.spi_peripheral, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, SPI bus failed to initialize.");
        #endif
        // clang-format on

        return ret;
    }
    else
    {
        init_status.spi_bus = true;
    }

    // add the imu device to the bus
    ret = spi_bus_add_device(imu_config.spi_peripheral, &imu_spi_config, &spi_hdl);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add device to SPI bus.");
        #endif
        // clang-format on

        return ret;
    }
    else
    {
        init_status.spi_device = true;
    }
    return ret;
}

/**
 * @brief Initializes sh2 HAL.
 *
 * @return ESP_OK if initialization was success.
 */
esp_err_t BNO08x::init_sh2_HAL()
{
    // use this IMU in sh2 HAL callbacks
    BNO08xSH2HAL::set_hal_imu(this);

    // register sh2 HAL callbacks
    sh2_HAL.open = BNO08xSH2HAL::spi_open;
    sh2_HAL.close = BNO08xSH2HAL::spi_close;
    sh2_HAL.read = BNO08xSH2HAL::spi_read;
    sh2_HAL.write = BNO08xSH2HAL::spi_write;
    sh2_HAL.getTimeUs = BNO08xSH2HAL::get_time_us;

    // reset BNO08x
    hard_reset();

    if (sh2_open(&sh2_HAL, BNO08xSH2HAL::hal_cb, NULL) != SH2_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, sh2_open() call failed.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }

    init_status.sh2_HAL = true;

    memset(&product_IDs, 0, sizeof(product_IDs));

    if (sh2_getProdIds(&product_IDs) != SH2_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, sh2_getProdIds() call failed.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    print_product_ids(); 
    #endif
    // clang-format on

    if (sh2_setSensorCallback(BNO08xSH2HAL::sensor_event_cb, NULL) != SH2_OK)
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Deinitializes GPIO, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio()
{
    esp_err_t ret = ESP_OK;

    if (init_status.gpio_inputs)
    {
        ret = deinit_gpio_inputs();
        if (ret != ESP_OK)
            return ret;
    }

    if (init_status.gpio_outputs)
    {
        ret = deinit_gpio_outputs();
        if (ret != ESP_OK)
            return ret;
    }

    return ret;
}

/**
 * @brief Deinitializes GPIO inputs, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio_inputs()
{
    esp_err_t ret = ESP_OK;

    ret = gpio_reset_pin(imu_config.io_int);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to add device to SPI bus.");
        #endif
        // clang-format on
    }

    return ret;
}

/**
 * @brief Deinitializes GPIO outputs, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_gpio_outputs()
{
    esp_err_t ret = ESP_OK;

    if (imu_config.io_wake != GPIO_NUM_NC)
    {
        ret = gpio_reset_pin(imu_config.io_wake);
        if (ret != ESP_OK)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could reset gpio WAKE pin to default state.");
            #endif
            // clang-format on

            return ret;
        }
    }

    ret = gpio_reset_pin(imu_config.io_cs);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio CS pin to default state.");
        #endif
        // clang-format on

        return ret;
    }

    ret = gpio_reset_pin(imu_config.io_rst);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Deconstruction failed, could reset gpio RST pin to default state.");
        #endif
        // clang-format on

        return ret;
    }

    return ret;
}

/**
 * @brief Deinitializes host interrupt ISR, called from deconstructor.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_hint_isr()
{
    esp_err_t ret = ESP_OK;

    if (init_status.isr_handler)
    {
        ret = gpio_isr_handler_remove(imu_config.io_int);
        if (ret != ESP_OK)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could not remove hint ISR handler.");
            #endif
            // clang-format on

            return ret;
        }
    }

    if (init_status.isr_service)
    {
        // only remove the ISR service if it was requested to be installed by user
        if (imu_config.install_isr_service)
        {
            gpio_uninstall_isr_service();
        }
    }

    return ret;
}

/**
 * @brief Deinitializes SPI.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_spi()
{
    esp_err_t ret = ESP_OK;

    if (init_status.spi_device)
    {
        ret = spi_bus_remove_device(spi_hdl);
        if (ret != ESP_OK)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could not remove spi device.");
            #endif
            // clang-format on

            return ret;
        }
    }

    if (init_status.spi_bus)
    {
        ret = spi_bus_free(imu_config.spi_peripheral);
        if (ret != ESP_OK)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Deconstruction failed, could free SPI peripheral.");
            #endif
            // clang-format on

            return ret;
        }
    }

    return ret;
}

/**
 * @brief Deinitializes tasks used by BNO08x driver.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_tasks()
{
    static const constexpr uint8_t TASK_DELETE_TIMEOUT_MS = 100UL;
    uint8_t kill_count = 0;
    sh2_SensorEvent_t empty_event;
    uint8_t empty_ID = 0;

    sem_kill_tasks = xSemaphoreCreateCounting(init_status.task_init_cnt, 0);

    // signal tasks to commit self-deletion
    xEventGroupClearBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASKS_RUNNING);

    if (init_status.data_proc_task)
        xQueueSend(queue_rx_sensor_event, &empty_event, 0);

    if (init_status.sh2_HAL_service_task)
        xEventGroupSetBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT);

    if (init_status.cb_task)
        xQueueSend(queue_cb_report_id, &empty_ID, 0);

    for (uint8_t i = 0; i < init_status.task_init_cnt; i++)
        if (xSemaphoreTake(sem_kill_tasks, TASK_DELETE_TIMEOUT_MS / portTICK_PERIOD_MS) == pdTRUE)
            kill_count++;

    if (kill_count != init_status.task_init_cnt)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Task deletion timed out in deconstructor call.");
        #endif
        // clang-format on

        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Deinitializes sh2 HAL.
 *
 * @return ESP_OK if deinitialization was success.
 */
esp_err_t BNO08x::deinit_sh2_HAL()
{
    if (init_status.sh2_HAL)
        sh2_close();

    return ESP_OK;
}

/**
 * @brief Hard resets BNO08x sensor.
 *
 * @return void, nothing to return
 */
void BNO08x::hard_reset()
{

    gpio_intr_disable(imu_config.io_int); // disable interrupts before reset

    gpio_set_level(imu_config.io_cs, 1);

    if (imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config.io_wake, 1);

    gpio_set_level(imu_config.io_rst, 0); // set reset pin low
    gpio_intr_enable(imu_config.io_int);  // enable interrupts before bringing out of reset
    vTaskDelay(HARD_RESET_DELAY_MS);      // 10ns min, set to larger delay to let things stabilize(Anton)
    gpio_set_level(imu_config.io_rst, 1); // bring out of reset
}

/**
 * @brief Waits for HINT pin assertion or HOST_INT_TIMEOUT_DEFAULT_MS to elapse.
 *
 *
 * @return ESP_OK if HINT was asserted.
 */
esp_err_t BNO08x::wait_for_hint()
{
    EventBits_t spi_evt_bits;

    spi_evt_bits = xEventGroupWaitBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT, pdTRUE, pdFALSE, HOST_INT_TIMEOUT_DEFAULT_MS);

    if (spi_evt_bits & EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT)
        return ESP_OK;
    else
        return ESP_ERR_TIMEOUT;
}

/**
 * @brief Re-enables all reports enabled by user (called when BNO08x reset is detected by sh2 HAL lib).
 *
 * @return ESP_OK if enabled reports were successfuly re-enabled.
 */
esp_err_t BNO08x::re_enable_reports()
{
    EventBits_t report_en_bits = xEventGroupGetBits(evt_grp_report_en);

    // loop through all entries of map and check if they are enabled
    for (auto entry = usr_reports.begin(); entry != usr_reports.end(); ++entry)
    {
        BNO08xRpt* rpt = entry->second;
        if (rpt->rpt_bit & report_en_bits)
        {
            if (!rpt->enable(rpt->period_us))
                return ESP_FAIL;
        }
    }

    xEventGroupClearBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_RESET_OCCURRED);

    return ESP_OK;
}

/**
 * @brief Polls for new data/report to become available.
 *
 * @return True if new data/report became available before DATA_AVAILABLE_TIMEOUT_MS.
 */
bool BNO08x::data_available()
{

    if (xEventGroupWaitBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_DATA_AVAILABLE, pdTRUE, pdFALSE, DATA_AVAILABLE_TIMEOUT_MS) &
            EVT_GRP_BNO08x_TASK_DATA_AVAILABLE)
        return true;

    return false;
}

/**
 * @brief Polls for new data/report to become available, overloaded with param for report identification.
 *
 * @param report_ID Reference to save most recent report ID.
 *
 * @return True if new data/report became available before DATA_AVAILABLE_TIMEOUT_MS.
 */
bool BNO08x::data_available(uint8_t& report_ID)
{

    if (xEventGroupWaitBits(evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_DATA_AVAILABLE, pdTRUE, pdFALSE, DATA_AVAILABLE_TIMEOUT_MS) &
            EVT_GRP_BNO08x_TASK_DATA_AVAILABLE)
    {
        report_ID = most_recent_rpt;
        return true;
    }

    report_ID = 0U;
    return false;
}

/**
 * @brief Registers a callback to execute when new data from a report is received.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type void input param.
 *
 * @return void, nothing to return
 */
void BNO08x::register_cb(std::function<void(void)> cb_fxn)
{
    cb_list.push_back({0U, cb_fxn});
}

/**
 * @brief Registers a callback to execute when new data from a report is received, overloaded with callback param for most recent report ID.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type with single input param of uint8_t for most recent report ID.
 *
 * @return void, nothing to return
 */
void BNO08x::register_cb(std::function<void(uint8_t report_ID)> cb_fxn)
{
    cb_list.push_back({0U, cb_fxn});
}

/**
 * @brief Prints product IDs received at initialization.
 *
 * @return void, nothing to return
 */
void BNO08x::print_product_ids()
{
    for (int i = 0; i < product_IDs.numEntries; i++)
    {
        ESP_LOGI(TAG,
                "Product ID %d Info:                           \n\r"
                "                ---------------------------\n\r"
                "                Product ID: 0x%" PRIx32 "\n\r"
                "                SW Version Major: 0x%" PRIx8 "\n\r"
                "                SW Version Minor: 0x%" PRIx8 "\n\r"
                "                SW Build Number:  0x%" PRIx32 "\n\r"
                "                SW Version Patch: 0x%" PRIx16 "\n\r"
                "                ---------------------------\n\r",
                i, product_IDs.entry->swPartNumber, product_IDs.entry->swVersionMajor, product_IDs.entry->swVersionMinor,
                product_IDs.entry->swBuildNumber, product_IDs.entry->swVersionPatch);
    }
}

/**
 * @brief Converts a BNO08xActivity enum to string.
 *
 * @return The resulting string conversion of the enum.
 */
const char* BNO08x::activity_to_str(BNO08xActivity activity)
{
    switch (activity)
    {
        case BNO08xActivity::UNKNOWN:
            return "UNKNOWN";
        case BNO08xActivity::IN_VEHICLE:
            return "IN_VEHICLE";
        case BNO08xActivity::ON_BICYCLE:
            return "ON_BICYCLE";
        case BNO08xActivity::ON_FOOT:
            return "ON_FOOT";
        case BNO08xActivity::STILL:
            return "STILL";
        case BNO08xActivity::TILTING:
            return "TILTING";
        case BNO08xActivity::WALKING:
            return "WALKING";
        case BNO08xActivity::RUNNING:
            return "RUNNING";
        case BNO08xActivity::ON_STAIRS:
            return "ON_STAIRS";
        case BNO08xActivity::UNDEFINED:
            return "UNDEFINED";
        default:
            return "UNDEFINED";
    }
}

/**
 * @brief Converts a BNO08xStability enum to string.
 *
 * @return The resulting string conversion of the enum.
 */
const char* BNO08x::stability_to_str(BNO08xStability stability)
{
    switch (stability)
    {
        case BNO08xStability::UNKNOWN:
            return "UNKNOWN";
        case BNO08xStability::ON_TABLE:
            return "ON_TABLE";
        case BNO08xStability::STATIONARY:
            return "STATIONARY";
        case BNO08xStability::STABLE:
            return "STABLE";
        case BNO08xStability::MOTION:
            return "MOTION";
        case BNO08xStability::RESERVED:
            return "RESERVED";
        case BNO08xStability::UNDEFINED:
            return "UNDEFINED";
        default:
            return "UNDEFINED";
    }
}

/**
 * @brief HINT interrupt service routine, handles falling edge of BNO08x HINT pin.
 *
 * ISR that launches SPI task to perform transaction upon assertion of BNO08x interrupt pin.
 *
 * @return void, nothing to return
 */
void IRAM_ATTR BNO08x::hint_handler(void* arg)
{
    BaseType_t xHighPriorityTaskWoken = pdFALSE;
    BNO08x* imu = (BNO08x*) arg; // cast argument received by gpio_isr_handler_add ("this" pointer to imu object
                                 // created by constructor call)

    // notify any tasks/function calls waiting for HINT assertion
    xEventGroupSetBitsFromISR(imu->evt_grp_bno08x_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT, &xHighPriorityTaskWoken);
    portYIELD_FROM_ISR(xHighPriorityTaskWoken); // perform context switch if necessary
}
