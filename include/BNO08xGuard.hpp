/**
 * @file BNO08xRpt.cpp
 * @author Myles Parfeniuk
 */

#pragma once

// in-house includes
#include "BNO08xPrivateTypes.hpp"

/**
 * @class BNO08xGuard
 *
 * @brief Class with static APIs for accessing semaphores to prevent code duplication across classes.
 * */
class BNO08xGuard
{
    public:
        /**
         * @brief Locks sh2 HAL lib to only allow the calling task to call its APIs.
         *
         * @param sync_ctx Reference to task synchronization context.
         *
         * @return void, nothing to return
         */
        static void lock_sh2_HAL(BNO08xPrivateTypes::bno08x_sync_ctx_t& sync_ctx)
        {
            xSemaphoreTake(sync_ctx.sh2_HAL_lock, portMAX_DELAY);
        }

        /**
         * @brief Unlocks sh2 HAL lib to allow other tasks to call its APIs.
         *
         * @param sync_ctx Reference to task synchronization context.
         * 
         * @return void, nothing to return
         */
        static void unlock_sh2_HAL(BNO08xPrivateTypes::bno08x_sync_ctx_t& sync_ctx)
        {
            xSemaphoreGive(sync_ctx.sh2_HAL_lock);
        }

        /**
         * @brief Locks locks user data to only allow the calling task to read/modify it.
         *
         * @param sync_ctx Reference to task synchronization context.
         * 
         * @return void, nothing to return
         */
        static void lock_user_data(BNO08xPrivateTypes::bno08x_sync_ctx_t& sync_ctx)
        {
            xSemaphoreTake(sync_ctx.data_lock, portMAX_DELAY);
        }

        /**
         * @brief Unlocks user data to allow other tasks to read/modify it.
         *
         * @param sync_ctx Reference to task synchronization context.
         * 
         * @return void, nothing to return
         */
        static void unlock_user_data(BNO08xPrivateTypes::bno08x_sync_ctx_t& sync_ctx)
        {
            xSemaphoreGive(sync_ctx.data_lock);
        }
};