/**
 * @file BNO08xCbParamVoid.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xCbGeneric.hpp"

/**
 * @class BNO08xCbParamRptID
 *
 * @brief Class to hold callback functions with void input parameters.
 */
class BNO08xCbParamVoid : public BNO08xCbGeneric
{
    public:
        static constexpr uint8_t ID_INVOKE_FOR_ANY_RPT = 0U; ///< Indicates to BNO08x::handle_cb that cb is registered to ALL enabled reports

        BNO08xCbParamVoid(std::function<void(void)> cb_fxn, uint8_t rpt_ID)
            : BNO08xCbGeneric(rpt_ID)
            , cb_fxn(cb_fxn)
        {
        }

        /**
         * @brief Invokes contained callback function.
         *
         * @param rpt_ID n/a, not used, kept to maintain same prototype as
         * BNO08xCbParamRptID::invoke()
         *
         * @return void, nothing to return
         */
        void invoke(uint8_t rpt_ID) override
        {
            cb_fxn();
        }

    private:
        std::function<void(void)> cb_fxn; ///< Wrapped callback function passed at register_cb().
};
