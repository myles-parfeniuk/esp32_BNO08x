/**
 * @file BNO08x.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08x.hpp"
#include "BNO08xPrivateTypes.hpp"

using namespace BNO08xPrivateTypes;

/**
 * @brief BNO08x imu constructor.
 *
 * Construct a BNO08x object for managing a BNO08x sensor.
 *
 * @param imu_config Configuration settings (optional), default settings can be seen in
 * bno08x_config_t
 * @return void, nothing to return
 */
BNO08x::BNO08x(bno08x_config_t imu_config)
    : rpt(bno08x_reports_t(&sync_ctx))
    , data_proc_task_hdl(NULL)
    , sh2_HAL_service_task_hdl(NULL)
    , cb_task_hdl(NULL)
    , sem_kill_tasks(NULL)
    , queue_rx_sensor_event(xQueueCreate(10, sizeof(sh2_SensorEvent_t)))
    , queue_cb_report_id(xQueueCreate(CONFIG_ESP32_BNO08X_CB_QUEUE_SZ, sizeof(uint8_t)))
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
    // deinitialize tasks if they have been initialized
    ESP_ERROR_CHECK(deinit_tasks());

    // deinitialize sh2 HAL if it has been initialized
    ESP_ERROR_CHECK(deinit_sh2_HAL());

    // deinitialize hint ISR if it has been initialized
    ESP_ERROR_CHECK(deinit_hint_isr());

    // deinitialize spi if has been initialized
    ESP_ERROR_CHECK(deinit_spi());

    // deinitialize GPIO if they have been initialized
    ESP_ERROR_CHECK(deinit_gpio());

    // delete all semaphores
    vSemaphoreDelete(sync_ctx.sh2_HAL_lock);
    vSemaphoreDelete(sync_ctx.data_lock);
    if (sem_kill_tasks != NULL)
        vSemaphoreDelete(sem_kill_tasks);

    // delete event groups
    vEventGroupDelete(sync_ctx.evt_grp_task);
    vEventGroupDelete(sync_ctx.evt_grp_rpt_en);
    vEventGroupDelete(sync_ctx.evt_grp_rpt_data_available);

    // delete all queues
    vQueueDelete(queue_rx_sensor_event);
    vQueueDelete(queue_cb_report_id);
}

/**
 * @brief Initializes BNO08x sensor
 *
 * Resets sensor and goes through initialization process.
 * Configures GPIO, required ISRs, and launches two tasks, one to manage SPI transactions, another
 * to process any received data.
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
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu
                                 // object created by constructor call)
    imu->data_proc_task();       // launch data processing task task from object
}

/**
 * @brief Task responsible for parsing/handling sensor events sent by SH2 HAL and updating data that
 * is returned to user.
 *
 * @return void, nothing to return
 */
void BNO08x::data_proc_task()
{
    EventBits_t evt_grp_bno08x_task_bits = 0U;
    BaseType_t queue_rx_success = pdFALSE;
    sh2_SensorEvent_t sensor_evt;
    sh2_SensorValue_t sensor_val;

    do
    {

        if (queue_rx_success == pdTRUE)
        {
            if (sh2_decodeSensorEvent(&sensor_val, &sensor_evt) != SH2_ERR)
                handle_sensor_report(&sensor_val);
        }

        queue_rx_success = xQueueReceive(queue_rx_sensor_event, &sensor_evt, portMAX_DELAY);
        evt_grp_bno08x_task_bits = xEventGroupGetBits(sync_ctx.evt_grp_task);

    } while (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING);

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    init_status.data_proc_task = false;
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
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu
                                 // object created by constructor call)
    imu->sh2_HAL_service_task(); // launch data processing task task from object
}

/**
 * @brief Task responsible for calling shtp_service() when HINT is asserted to dispatch any sh2 HAL
 * lib callbacks.
 *
 * @return void, nothing to return
 */
void BNO08x::sh2_HAL_service_task()
{
    EventBits_t evt_grp_bno08x_task_bits = 0U;

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
    int64_t last_hint_time = esp_timer_get_time();
    int64_t current_hint_time; 
    #endif
    // clang-format on

    do
    {

        if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASK_RESET_OCCURRED)
        {
            if (re_enable_reports() != ESP_OK)
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGE(TAG, "Failed to re-enable enabled reports after IMU reset.");
                #endif
                // clang-format on
            }
        }

        if (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT)
        {
            lock_sh2_HAL();
            sh2_service();
            unlock_sh2_HAL();
        }

        evt_grp_bno08x_task_bits = xEventGroupWaitBits(sync_ctx.evt_grp_task,
                EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT | EVT_GRP_BNO08x_TASK_RESET_OCCURRED, pdFALSE, pdFALSE, portMAX_DELAY);

        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
        current_hint_time = esp_timer_get_time();
        ESP_LOGW(TAG, "HINT Asserted, time since last assertion: %lldus", current_hint_time - last_hint_time);
        last_hint_time = current_hint_time;
        #endif
        // clang-format on

    } while (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING);

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    init_status.sh2_HAL_service_task = false;
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
    BNO08x* imu = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu
                                 // object created by constructor call)
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
    uint8_t rpt_ID = 0;

    do
    {
        // execute callbacks
        for (auto& cb_entry : sync_ctx.cb_list)
        {
            BNO08xCbGeneric* cb_ptr = nullptr;

            if (auto* ptr = etl::get_if<BNO08xCbParamVoid>(&cb_entry))
                cb_ptr = ptr;

            else if (auto* ptr = etl::get_if<BNO08xCbParamRptID>(&cb_entry))
                cb_ptr = ptr;

            if (cb_ptr != nullptr)
                handle_cb(rpt_ID, cb_ptr);
        }

        xQueueReceive(queue_cb_report_id, &rpt_ID, portMAX_DELAY);

        evt_grp_bno08x_task_bits = xEventGroupGetBits(sync_ctx.evt_grp_task);

    } while (evt_grp_bno08x_task_bits & EVT_GRP_BNO08x_TASKS_RUNNING);

    xSemaphoreGive(sem_kill_tasks); // signal to deconstructor deletion is completed
    init_status.cb_task = false;
    vTaskDelete(NULL);
}

/**
 * @brief Locks sh2 HAL lib to only allow the calling task to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08x::lock_sh2_HAL()
{
    xSemaphoreTake(sync_ctx.sh2_HAL_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks sh2 HAL lib to allow other tasks to call its APIs.
 *
 * @return void, nothing to return
 */
void BNO08x::unlock_sh2_HAL()
{
    xSemaphoreGive(sync_ctx.sh2_HAL_lock);
}

/**
 * @brief Locks locks user data to only allow the calling task to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08x::lock_user_data()
{
    xSemaphoreTake(sync_ctx.data_lock, portMAX_DELAY);
}

/**
 * @brief Unlocks user data to allow other tasks to read/modify it.
 *
 * @return void, nothing to return
 */
void BNO08x::unlock_user_data()
{
    xSemaphoreGive(sync_ctx.data_lock);
}

/**
 * @brief Parses receieved report and updates uer data with it.
 *
 * @return void, nothing to return
 */
void BNO08x::handle_sensor_report(sh2_SensorValue_t* sensor_val)
{
    uint8_t rpt_ID = sensor_val->sensorId;

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_DEBUG_STATEMENTS
    ESP_LOGE(TAG, "Report RX'd, ID: %d", sensor_val->sensorId);
    #endif
    // clang-format on

    // check if report implementation exists within map
    if (rpt_ID == SH2_RESERVED)
        return;

    auto& rpt = usr_reports.at(rpt_ID);

    if (rpt == nullptr)
        return;

    // send report ids to cb_task for callback execution (only if this report is enabled)
    if (rpt->rpt_bit & xEventGroupGetBits(sync_ctx.evt_grp_rpt_en))
    {
        // update respective report with new data
        rpt->update_data(sensor_val);

        if (sync_ctx.cb_list.size() != 0)
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
 * @brief Determines the flavor of a passed callback and executes it appropriately.
 *
 * @return void, nothing to return
 */
void BNO08x::handle_cb(uint8_t rpt_ID, BNO08xCbGeneric* cb_entry)
{
    // only execute callback if it is registered to this report or all reports
    if ((cb_entry->rpt_ID == 0) || (cb_entry->rpt_ID == rpt_ID))
    {
        cb_entry->invoke(rpt_ID);
    }
}

/**
 * @brief Initializes required esp-idf SPI data structures with values from user passed
 * bno08x_config_t struct.
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
    imu_spi_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus
                               // high when idle)

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
    imu_spi_config.spics_io_num = -1; // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
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
        init_status.gpio_inputs = true; // set gpio_inputs to initialized such that deconstructor
                                        // knows to clean them up
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

    // configure output(s) (CS, RST)
    gpio_config_t outputs_config;

    outputs_config.pin_bit_mask = ((1ULL << imu_config.io_cs) | (1ULL << imu_config.io_rst));

    outputs_config.mode = GPIO_MODE_OUTPUT;
    outputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs_config.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&outputs_config);
    if (ret != ESP_OK)
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Initialization failed, failed to configure CS, and RST gpio.");
        #endif
        // clang-format on
    }
    else
    {
        init_status.gpio_outputs = true; // set gpio_inputs to initialized such that deconstructor
                                         // knows to clean them up
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
        init_status.isr_service = true; // set isr service to initialized such that deconstructor knows to clean it up
                                        // (this will be ignored if imu_config.install_isr_service == false)
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

    xEventGroupSetBits(sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASKS_RUNNING);

    // launch data processing task 6
    task_created = xTaskCreatePinnedToCore(
            &data_proc_task_trampoline, "bno08x_data_processing_task", 
            DATA_PROC_TASK_SZ, 
            this, 
            DATA_PROC_TASK_PRIORITY, 
            &data_proc_task_hdl, 
            DATA_PROC_TASK_AFFINITY);

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
        init_status.data_proc_task = true;
    }

    // launch cb task 5
    task_created = xTaskCreatePinnedToCore(&cb_task_trampoline, "bno08x_cb_task", 
        CB_TASK_SZ, 
        this, 
        CB_TASK_PRIORITY, 
        &cb_task_hdl, 
        CB_TASK_AFFINITY);

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
        init_status.cb_task = true;
    }

    // launch sh2 hal service task 7
    task_created = xTaskCreatePinnedToCore(&sh2_HAL_service_task_trampoline, "bno08x_sh2_HAL_service_task", 
        SH2_HAL_SERVICE_TASK_SZ, 
        this, 
        SH2_HAL_SERVICE_TASK_PRIORITY,
        &sh2_HAL_service_task_hdl,
        SH2_HAL_SERVICE_TASK_AFFINITY);

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
        init_status.sh2_HAL_service_task = true;
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
    toggle_reset();

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

    memset(&product_IDs, 0, sizeof(sh2_ProductIds_t));

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

        init_status.gpio_inputs = false;
    }

    if (init_status.gpio_outputs)
    {
        ret = deinit_gpio_outputs();
        if (ret != ESP_OK)
            return ret;

        init_status.gpio_outputs = false;
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

        init_status.isr_handler = false;
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

        init_status.spi_device = false;
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

        init_status.spi_bus = false;
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
    static const constexpr uint8_t TASK_DELETE_TIMEOUT_MS = HOST_INT_TIMEOUT_DEFAULT_MS;
    uint8_t kill_count = 0;
    uint8_t init_count = 0;
    sh2_SensorEvent_t empty_event;
    uint8_t empty_ID = 0;

    // disable interrupts before beginning so we can ensure SPI transaction doesn't attempt to run
    gpio_intr_disable(imu_config.io_int);

    init_count += (static_cast<uint8_t>(init_status.cb_task) + static_cast<uint8_t>(init_status.data_proc_task) +
                   static_cast<uint8_t>(init_status.sh2_HAL_service_task));

    if (init_count != 0)
    {
        sem_kill_tasks = xSemaphoreCreateCounting(init_count, 0);
        xEventGroupClearBits(sync_ctx.evt_grp_task,
                EVT_GRP_BNO08x_TASKS_RUNNING); // clear task running bit request deletion of tasks

        if (init_status.cb_task)
            xQueueSend(queue_cb_report_id, &empty_ID, 0);

        if (init_status.data_proc_task)
            xQueueSend(queue_rx_sensor_event, &empty_event, 0);

        if (init_status.sh2_HAL_service_task)
            xEventGroupSetBits(sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT);

        for (uint8_t i = 0; i < init_count; i++)
            if (xSemaphoreTake(sem_kill_tasks, TASK_DELETE_TIMEOUT_MS) == pdTRUE)
                kill_count++;

        if (kill_count != init_count)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Task deletion timed out in deconstructor call.");
            #endif
            // clang-format on

            return ESP_FAIL;
        }
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
    {
        init_status.sh2_HAL = false;
        sh2_close();
    }

    return ESP_OK;
}

/**
 * @brief Hard resets BNO08x device.
 *
 * @return True if reset was success.
 */
bool BNO08x::hard_reset()
{
    // toggle reset gpio
    toggle_reset();

    // wait for reset to be detected by SH2 HAL lib
    if (wait_for_reset() == ESP_OK)
    {
        // run service to dispatch callbacks
        lock_sh2_HAL();
        sh2_service();
        unlock_sh2_HAL();

        // get product ids and check reset reason
        if (get_reset_reason() == BNO08xResetReason::EXT_RST)
        {
            return true;
        }
        else
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Hard reset failure, incorrect reset reason returned: %d.", product_IDs.entry[0].resetCause);
            #endif
            // clang-format on
        }
    }
    else
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Hard reset failure, reset never detected after toggling reset pin");
        #endif
        // clang-format on
    }

    return false;
}

/**
 * @brief Soft resets BNO08x device by sending RESET (1) command on "device" channel.
 *
 * @return True if soft reset operation succeeded.
 */
bool BNO08x::soft_reset()
{
    int op_success = SH2_ERR;

    // send reset command
    lock_sh2_HAL();
    op_success = sh2_devReset();
    unlock_sh2_HAL();

    if (op_success == SH2_OK)
    {
        // wait for reset to be detected by SH2 HAL lib
        if (wait_for_reset() == ESP_OK)
        {
            // run service to dispatch callbacks
            lock_sh2_HAL();
            sh2_service();
            unlock_sh2_HAL();

            if (get_reset_reason() == BNO08xResetReason::EXT_RST)
            {
                return true;
            }
            else
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGE(TAG, "Soft reset failure, incorrect reset reason returned.");
                #endif
                // clang-format on
            }
        }
        else
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Soft reset failure, reset never detected after sending command.");
            #endif
            // clang-format on
        }
    }
    else
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Soft reset failure, failed to send reset command");
        #endif
        // clang-format on
    }

    return false;
}

/**
 * @brief Disables all currently enabled reports.
 *
 * @return True if all currently enabled reports were disabled successfully.
 */
bool BNO08x::disable_all_reports()
{
    int attempts = 0;

    xEventGroupClearBits(sync_ctx.evt_grp_rpt_en, EVT_GRP_RPT_ALL);

    while (sync_ctx.en_report_ids.size() != 0 && (attempts < TOTAL_RPT_COUNT))
    {
        uint8_t rpt_ID = sync_ctx.en_report_ids.back();
        BNO08xRpt* rpt = usr_reports.at(rpt_ID);
        if (rpt == nullptr)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "NULL pointer detected in usr_reports map for enabled report.");
            #endif
            // clang-format on
            return false;
        }

        if (!rpt->disable())
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "Failed to disable: %d", rpt->ID);
            #endif
            // clang-format on
            return false;
        }

        attempts++;
    }

    if (attempts < TOTAL_RPT_COUNT)
        return true;
    else
        return false;
}

/**
 * @brief Returns reason for previous reset via product ID report.
 *
 * @return Enum object containing reset reason, BNO08xResetReason::UNDEFINED if failure.
 */
BNO08xResetReason BNO08x::get_reset_reason()
{
    int op_success = SH2_ERR;
    BNO08xResetReason rr = BNO08xResetReason::UNDEFINED;

    memset(&product_IDs, 0, sizeof(sh2_ProductIds_t));
    lock_sh2_HAL();
    op_success = sh2_getProdIds(&product_IDs);
    unlock_sh2_HAL();

    if (op_success == SH2_OK)
    {
        rr = static_cast<BNO08xResetReason>(product_IDs.entry[0].resetCause);
    }
    else
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Get reset reason failure, failed to get prodIDs.");
        #endif
        // clang-format on
    }

    return rr;
}

/**
 * @brief Places BNO08x device in on state by sending ON (2) command on "device" channel.
 *
 * @return True if on operation succeeded.
 */
bool BNO08x::on()
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_devOn();
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Places BNO08x device in sleep state by sending SLEEP (3) command on "device" channel.
 *
 * @return True if sleep operation succeeded.
 */
bool BNO08x::sleep()
{

    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_devSleep();
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Starts simple calibration, see ref. manual 6.4.10.1
 *
 * @param period_us This interval should be set to whatever rate the sensor hub is expected to run
 * at after calibration.
 *
 * After the calibration is started, the IMU should be rotated 180 degrees.
 * After the IMU has been rotated call calibration_end().
 * See ref. manual 6.4.10 for more detailed instructions.
 *
 * @return True if start simple calibration operation succeeded.
 */
/*
bool BNO08x::calibration_turntable_start(uint32_t period_us)
{
    // currently broken, correct packet is sent over SPI but IMU responds
    // with unsolicited initialize response instead of the expected Turntable Cal response (0x0C)
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_startCal(period_us);
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}
*/

/**
 * @brief Ends turn-table calibration, see ref. manual 6.4.10.2
 *
 * @param status Returned status bits indicating result of turntable calibration.
 *
 * @return True if enable start turn-table calibration operation succeeded.
 */
/*bool BNO08x::calibration_turntable_end(sh2_CalStatus_t& status)
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_finishCal(&status);
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}
*/

/**
 * @brief Enables dynamic/motion engine calibration for specified sensor(s), see ref. manual 6.4.6.1
 *
 * @param sensor The sensor(s) to enable dynamic/ME calibration for.
 *
 * @return True if enable dynamic/ME calibration succeeded.
 */
bool BNO08x::dynamic_calibration_enable(BNO08xCalSel sensor)
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_setCalConfig(static_cast<uint8_t>(sensor));
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Disables dynamic/motion engine calibration for specified sensor(s), see ref.
 * manual 6.4.6.1
 *
 * @param sensor The sensor(s) to disable dynamic/ME calibration for.
 *
 * @return True if disable dynamic/ME calibration succeeded.
 */
bool BNO08x::dynamic_calibration_disable(BNO08xCalSel sensor)
{
    int op_success = SH2_ERR;
    uint8_t active_sensors = 0U;

    lock_sh2_HAL();
    op_success = sh2_getCalConfig(&active_sensors);
    unlock_sh2_HAL();

    if (op_success == SH2_OK)
    {
        active_sensors &= ~static_cast<uint8_t>(sensor);

        lock_sh2_HAL();
        op_success = sh2_setCalConfig(active_sensors);
        unlock_sh2_HAL();
    }

    return (op_success == SH2_OK);
}

/**
 * @brief Enables the automatic saving of dynamic/ME calibration data to BNO08x internal flash See
 * ref manual 6.4.7.1.
 *
 * @return True if dynamic/ME calibration autosave data enable succeeded.
 */
bool BNO08x::dynamic_calibration_autosave_enable()
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_setDcdAutoSave(true);
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Disables the automatic saving of dynamic/ME calibration data to BNO08x internal flash See
 * ref manual 6.4.7.1.
 *
 * @return True if dynamic/ME calibration autosave data enable succeeded.
 */
bool BNO08x::dynamic_calibration_autosave_disable()
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_setDcdAutoSave(false);
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Saves dynamic/motion engine calibration data to BNO08x internal flash immediately. See ref
 * manual 6.4.5.1
 *
 * @return True if save dynamic/ME calibration data succeeded.
 */
bool BNO08x::dynamic_calibration_save()
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_saveDcdNow();
    unlock_sh2_HAL();

    return (op_success == SH2_OK);
}

/**
 * @brief Clears dynamic/motion engine calibration data from ram and resets BNO08x device. See ref
 * manual 6.4.9.1
 *
 * @return True if save dynamic/ME calibration data succeeded.
 */
bool BNO08x::dynamic_calibration_clear_data_ram()
{
    int op_success = SH2_ERR;

    // send clear DCD and reset command
    lock_sh2_HAL();
    op_success = sh2_clearDcdAndReset();
    unlock_sh2_HAL();

    if (op_success == SH2_OK)
    {
        // wait for reset to be detected by SH2 HAL lib
        if (wait_for_reset() == ESP_OK)
        {
            // run service to dispatch callbacks
            lock_sh2_HAL();
            sh2_service();
            unlock_sh2_HAL();

            if (get_reset_reason() == BNO08xResetReason::EXT_RST)
            {
                return true;
            }
            else
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGE(TAG, "dynamic_calibration_clear_data_ram(): Clear dynamic calibration failure, incorrect reset reason returned.");
                #endif
                // clang-format on
            }
        }
        else
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "dynamic_calibration_clear_data_ram(): Clear dynamic calibration failure, reset never detected after sending command.");
            #endif
            // clang-format on
        }
    }
    else
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_clear_data_ram(): Clear dynamic calibration failure, failed to clearDcdAndReset command with %li", static_cast<int32_t>(op_success));
        #endif
        // clang-format on
    }

    return false;
}

/**
 * @brief Example calibration routine using  dynamic/ME calibration commands.
 * 
 * Routine does the following:alignas
 * 
 * 1) disables all enabled reports
 * 2) sends a command to enable dynamic/motion engine calibration for all possible options (SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG | SH2_CAL_PLANAR)
 * 3) enables game rotation vector reports and calibrated magnetic field reports
 * 4) moving window average for accuracy received through reports
 * 5) deems calibration accuracy threshold met when magf accuracy avg is >=2 (MED) and quat accuracy avg >=3 (HIGH) for longer than 5 seconds
 * 6) sends command to save dynamic calibration data
 * 7) disables all enabled reports
 * 
 * Note the DCD commands don't have to be used this way, this is just an example,
 * but the dynamic_calibration_autosave_enable() allows calibration to be run and
 * saved constantly even while data is used for other operations. 
 *
 * @return True if calibration routine succeeded. 
 */
bool BNO08x::dynamic_calibration_run_routine()
{
    constexpr size_t WINDOW_SZ = 50;
    constexpr int64_t STABLE_TIME_CRITERIA_US = 5000000LL; // meet accuracy criteria for 5 seconds

    bno08x_quat_t quat_data;
    bno08x_magf_t magf_data;
    uint8_t quat_accuracy_window[WINDOW_SZ] = {0U};
    uint8_t magf_accuracy_window[WINDOW_SZ] = {0U};
    uint8_t quat_window_idx = 0U;
    uint8_t magf_window_idx = 0U;
    uint16_t quat_window_sum = 0U;
    uint16_t magf_window_sum = 0U;
    BNO08xAccuracy quat_accuracy_avg = BNO08xAccuracy::UNRELIABLE;
    BNO08xAccuracy magf_accuracy_avg = BNO08xAccuracy::UNRELIABLE;
    int64_t stable_time = 0LL;
    int64_t stable_start_time = 0ULL;

    // clang-format off
     #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
     ESP_LOGI(TAG, "dynamic_calibration_run_routine(): disabling all currently enabled reports and starting calibration...");
     #endif
    // clang-format on

    if (!disable_all_reports())
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_run_routine(): failed to disable all reports before calibration");
        #endif
        // clang-format on

        return false;
    }

    if (!dynamic_calibration_enable(BNO08xCalSel::all))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_run_routine(): failed to enable dynamic calibration");
        #endif
        // clang-format on

        return false;
    }

    if (!rpt.rv_game.enable(100000UL))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_run_routine(): failed to enable game rotation vector report");
        #endif
        // clang-format on

        return false;
    }

    if (!rpt.cal_magnetometer.enable(100000UL))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_run_routine(): failed to enable calibrated magnetometer report");
        #endif
        // clang-format on

        return false;
    }

    while (1)
    {
        if (data_available())
        {
            if (rpt.rv_game.has_new_data())
            {
                quat_data = rpt.rv_game.get_quat();

                quat_accuracy_window[quat_window_idx] = static_cast<uint8_t>(quat_data.accuracy);
                quat_window_idx++;

                if (quat_window_idx >= WINDOW_SZ)
                    quat_window_idx = 0U;

                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGI(TAG, "dynamic_calibration_run_routine(): quat_data: accuracy: %d i: %.3f j: %.3f k: %.3f real: %.3f",
                        static_cast<uint8_t>(quat_data.accuracy), quat_data.i, quat_data.j, quat_data.k, quat_data.real);
                #endif
                // clang-format on
            }

            if (rpt.cal_magnetometer.has_new_data())
            {
                magf_data = rpt.cal_magnetometer.get();

                magf_accuracy_window[magf_window_idx] = static_cast<uint8_t>(magf_data.accuracy);
                magf_window_idx++;

                if (magf_window_idx >= WINDOW_SZ)
                    magf_window_idx = 0U;

                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGI(TAG, "dynamic_calibration_run_routine(): magf_data: accuracy: %d x: %.3f y: %.3f z: %.3f ",
                static_cast<uint8_t>(magf_data.accuracy), magf_data.x, magf_data.y, magf_data.z);
                #endif
                // clang-format on
            }
        }

        quat_window_sum = 0U;
        magf_window_sum = 0U;

        // sum windows and take average
        for (int i = 0U; i < WINDOW_SZ; i++)
        {
            quat_window_sum += quat_accuracy_window[i];
            magf_window_sum += magf_accuracy_window[i];
        }

        quat_accuracy_avg = static_cast<BNO08xAccuracy>(quat_window_sum / WINDOW_SZ);
        magf_accuracy_avg = static_cast<BNO08xAccuracy>(magf_window_sum / WINDOW_SZ);

        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGI(TAG, "dynamic_calibration_run_routine(): magf_avg_acc: %s quat_avg_acc: %s", BNO08xAccuracy_to_str(magf_accuracy_avg), BNO08xAccuracy_to_str(quat_accuracy_avg));
        #endif
        // clang-format on

        if ((quat_accuracy_avg >= BNO08xAccuracy::HIGH) && (magf_accuracy_avg >= BNO08xAccuracy::MED))
        {
            // start timer if accuracy criteria is met
            if (stable_start_time == 0LL)
                stable_start_time = esp_timer_get_time();

            // calculate time for which accuracy criteria has been met
            stable_time = esp_timer_get_time() - stable_start_time;
        }
        else
        {
            // reset timer if accuracy criteria is not met
            stable_time = 0LL;
        }

        // check if average accuracy has been stable for required time
        if (stable_time >= STABLE_TIME_CRITERIA_US)
            break;
    }

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    ESP_LOGI(TAG, "dynamic_calibration_run_routine(): calibration accuracy threshold reached, sending command to save calibration data...");
    #endif
    // clang-format on

    if (!dynamic_calibration_save())
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGI(TAG, "dynamic_calibration_run_routine(): failed to save calibration data");
        #endif
        // clang-format on

        return false;
    }

    if (!dynamic_calibration_disable(BNO08xCalSel::all))
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGI(TAG, "dynamic_calibration_run_routine(): failed to disable calibration");
        #endif
        // clang-format on

        return false;
    }

    if (!disable_all_reports())
    {
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "dynamic_calibration_run_routine(): failed to disable all reports after calibration");
        #endif
        // clang-format on

        return false;
    }

    // clang-format off
    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    ESP_LOGI(TAG, "dynamic_calibration_run_routine(): calibration success");
    #endif
    // clang-format on

    return true;
}

/**
 * @brief Deletes dynamic calibration data from BNO08x internal flash and resets the device.
 * Follows the steps outlined in ref. manual 6.4.9
 * @return True if delete dynamic calibration data operation succeeded.
 */
bool BNO08x::dynamic_calibration_delete_data()
{
    // 1. Reset hub (using hard_reset)
    if (!hard_reset()) {
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "delete_calibration_data(): failed to hard reset hub");
        #endif
        return false;
    }

    // 2. Delete flash copy of DCD via FRS 
    // Deleting FRS record: use sh2_setFrs with nullptr and 0 words
    if(!write_frs(BNO08xFrsID::DYNAMIC_CALIBRATION, nullptr, 0U))
        return false; 

    // 3. Issue Clear DCD and Reset Command (atomic clear DCD from RAM and reset)
    if(!dynamic_calibration_clear_data_ram())
        return false; 

    #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
    ESP_LOGI(TAG, "delete_calibration_data(): calibration data cleared successfully");
    #endif

    return true;
}

/**
 * @brief Retrieves a record from flash record system (if your goal is to retrieve sensor specific meta data use the
 * BNO08xRpt:get_meta_data() method instead)
 *
 * For more details on returned and data and frs_IDs see ref. manual 6.3.7 & 4.3
 *
 * @param frs_ID The ID of the desired record to retrieve from flash.
 * @param data Buffer of 16 uint32_t to store retrieved data.
 * @param rx_data_sz Reference to store number of 32 bit words retrieved from flash.
 *
 * @return True if get flash record system operation succeeded.
 */
bool BNO08x::get_frs(BNO08xFrsID frs_ID, uint32_t (&data)[16], uint16_t& rx_data_sz)
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_getFrs(static_cast<uint16_t>(frs_ID), data, &rx_data_sz);
    unlock_sh2_HAL();

    if (op_success != SH2_OK)
    {   
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "get_frs(): Failed to retrieve FRS record for ID: %s, op_success: %li", BNO08xFrsID_to_str(frs_ID), static_cast<int32_t>(op_success));
        #endif
        // clang-format off
        return false;
    }

    return (op_success == SH2_OK);
}


/**
 * @brief Writes a record to flash record system.
 *
 * For more details on flash records and frs_IDs see ref. manual 6.3.6 & 4.3
 *
 * @param frs_ID The ID of the desired to write to flash.
 * @param data Buffer of 16 uint32_t to store data to send.
 * @param tx_data_sz Length of data, amount of 32 bit words to write to flash.
 *
 * @return True if get flash record system operation succeeded.
 */
bool BNO08x::write_frs(BNO08xFrsID frs_ID, uint32_t *data, const uint16_t tx_data_sz)
{
    int op_success = SH2_ERR;

    lock_sh2_HAL();
    op_success = sh2_setFrs(static_cast<uint16_t>(frs_ID), data, tx_data_sz);
    unlock_sh2_HAL();

    if (op_success != SH2_OK)
    {   
        // clang-format off
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "write_frs(): Failed to write FRS record for ID: %s, op_success: %li", BNO08xFrsID_to_str(frs_ID), static_cast<int32_t>(op_success));
        #endif
        // clang-format off
        return false;
    }

    return (op_success == SH2_OK);
}


/**
 * @brief Returns product ID info sent by IMU at initialization.
 *
 * @return The product ID info returned at initialization.
 */
sh2_ProductIds_t BNO08x::get_product_IDs()
{
    return product_IDs;
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

    spi_evt_bits = xEventGroupWaitBits(
            sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT, pdTRUE, pdFALSE, HOST_INT_TIMEOUT_DEFAULT_MS);

    if (spi_evt_bits & EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT)
        return ESP_OK;
    else
        return ESP_ERR_TIMEOUT;
}

/**
 * @brief Waits for SH2 HAL lib to detect reset or HOST_INT_TIMEOUT_DEFAULT_MS to elapse.
 *
 *
 * @return ESP_OK if reset was detected by SH2 HAL lib.
 */
esp_err_t BNO08x::wait_for_reset()
{
    if (xEventGroupWaitBits(
                sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_RESET_OCCURRED, pdFALSE, pdFALSE, HOST_INT_TIMEOUT_DEFAULT_MS) &
            EVT_GRP_BNO08x_TASK_RESET_OCCURRED)
        return ESP_OK;
    else
        return ESP_ERR_TIMEOUT;
}

/**
 * @brief Toggles reset gpio pin for hard reset of BNO08x device.
 *
 *
 * @return void, nothing to return
 */
void BNO08x::toggle_reset()
{
    gpio_intr_disable(imu_config.io_int); // disable interrupts before reset

    gpio_set_level(imu_config.io_cs, 1);

    gpio_set_level(imu_config.io_rst, 0); // set reset pin low
    vTaskDelay(HARD_RESET_DELAY_MS);      // 10ns min, set to larger delay to let things stabilize(Anton)
    gpio_intr_enable(imu_config.io_int);  // enable interrupts before bringing out of reset
    gpio_set_level(imu_config.io_rst, 1); // bring out of reset
}

/**
 * @brief Re-enables all reports enabled by user (called when BNO08x reset is detected by sh2 HAL
 * lib).
 *
 * @return ESP_OK if enabled reports were successfuly re-enabled.
 */
esp_err_t BNO08x::re_enable_reports()
{
    EventBits_t report_en_bits = xEventGroupGetBits(sync_ctx.evt_grp_rpt_en);

    for (const auto& rpt_ID : sync_ctx.en_report_ids)
    {
        BNO08xRpt* rpt = usr_reports.at(rpt_ID);
        if (rpt == nullptr)
        {
            // clang-format off
            #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
            ESP_LOGE(TAG, "NULL pointer detected in usr_reports map for enabled report.");
            #endif
            // clang-format on
            continue;
        }

        if (rpt->rpt_bit & report_en_bits)
        {
            if (!rpt->enable(rpt->period_us))
            {
                // clang-format off
                #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
                ESP_LOGE(TAG, "Failed to re-enable: %d", rpt->ID);
                #endif
                // clang-format on
                return ESP_FAIL;
            }
        }
    }

    xEventGroupClearBits(sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_RESET_OCCURRED);

    return ESP_OK;
}

/**
 * @brief Polls for new data/report to become available.
 *
 * @return True if new data/report became available before DATA_AVAILABLE_TIMEOUT_MS.
 */
bool BNO08x::data_available()
{

    if (xEventGroupWaitBits(
                sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_DATA_AVAILABLE, pdTRUE, pdFALSE, DATA_AVAILABLE_TIMEOUT_MS) &
            EVT_GRP_BNO08x_TASK_DATA_AVAILABLE)
        return true;

    return false;
}

/**
 * @brief Registers a callback to execute when new data from a report is received.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type void input param.
 *
 * @return void, nothing to return
 */
bool BNO08x::register_cb(std::function<void(void)> cb_fxn)
{

    if (sync_ctx.cb_list.size() < CONFIG_ESP32_BNO08X_CB_MAX)
    {
        sync_ctx.cb_list.push_back(BNO08xCbParamVoid(cb_fxn, 0U));
        return true;
    }
    return false;
}

/**
 * @brief Registers a callback to execute when new data from a report is received, overloaded with
 * callback param for most recent report ID.
 *
 * @param cb_fxn Pointer to the call-back function should be of void return type with single input
 * param of uint8_t for most recent report ID.
 *
 * @return void, nothing to return
 */
bool BNO08x::register_cb(std::function<void(uint8_t report_ID)> cb_fxn)
{
    if (sync_ctx.cb_list.size() < CONFIG_ESP32_BNO08X_CB_MAX)
    {
        sync_ctx.cb_list.push_back(BNO08xCbParamRptID(cb_fxn, 0U));
        return true;
    }
    return false;
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


// Converts a 32-bit signed Q30 fixed-point value to float
static inline float q30_to_float(int32_t q)
{
    return ((float)q) / (float)(1UL << 30);
}

// Converts a float to 32-bit signed Q30 fixed-point value
static inline int32_t float_to_q30(float f)
{
    if (f > 1.0f) f = 1.0f;
    if (f < -1.0f) f = -1.0f;
    return (int32_t)(f * (float)(1UL << 30));
}


void BNO08x::print_system_orientation()
{
    float w, x, y, z;
    if (get_system_orientation(w, x, y, z)) {
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGI(TAG, "Mounting orientation (float): W: %.6f X: %.6f Y: %.6f Z: %.6f", w, x, y, z);
        #endif
    } else {
        #ifdef CONFIG_ESP32_BNO08x_LOG_STATEMENTS
        ESP_LOGE(TAG, "Failed to get mounting orientation");
        #endif
    }
}

/**
 * @brief Sets the system orientation of the BNO08x device and persist it in flash (FRS).
 * use SQRT2OVER2 as a constant for sqrt(2)/2
 * see Datasheet Figure 4.3 for reference
 * Note that a reset is required to apply changes.
 * Note also that this configuration seems only to work if reports are already enabled. 
 * e.g. set .rpt.rv.enable(true) prior this call
 */
bool BNO08x::set_system_orientation(float w, float x, float y, float z)
{
    uint32_t orientation_raw[4] = {
        static_cast<uint32_t>(float_to_q30(x)), // X component
        static_cast<uint32_t>(float_to_q30(y)), // Y component
        static_cast<uint32_t>(float_to_q30(z)), // Z component
        static_cast<uint32_t>(float_to_q30(w))  // W component
    };

    if(!write_frs(BNO08xFrsID::SYSTEM_ORIENTATION, orientation_raw, sizeof(orientation_raw)))
        return false; 

    return true;
}

bool BNO08x::get_system_orientation(float& real, float& i, float& j, float& k)
{
    uint16_t words_rxd = 0U;
    uint32_t raw[16] = {0};

    if(!get_frs(BNO08xFrsID::SYSTEM_ORIENTATION, raw, words_rxd))
        return false;
    
    if(words_rxd >= 4U)
        return false; 

    i = q30_to_float((int32_t)raw[0]);
    j = q30_to_float((int32_t)raw[1]);
    k = q30_to_float((int32_t)raw[2]);
    real = q30_to_float((int32_t)raw[3]);

    return true;
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
    BNO08x* imu = (BNO08x*) arg; // cast argument received by gpio_isr_handler_add ("this" pointer
                                 // to imu object created by constructor call)

    // notify any tasks/function calls waiting for HINT assertion
    xEventGroupSetBitsFromISR(imu->sync_ctx.evt_grp_task, EVT_GRP_BNO08x_TASK_HINT_ASSRT_BIT, &xHighPriorityTaskWoken);
    portYIELD_FROM_ISR(xHighPriorityTaskWoken); // perform context switch if necessary
}
