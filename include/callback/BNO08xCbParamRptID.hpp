/**
 * @file BNO08xCbParamRptID.hpp
 * @author Myles Parfeniuk
 */

#pragma once

#include "BNO08xCbGeneric.hpp"

/**
 * @class BNO08xCbParamRptID
 *
 * @brief Class to hold callback functions which are passed report ID as input parameter.
 */
class BNO08xCbParamRptID : public BNO08xCbGeneric
{
    public:
        BNO08xCbParamRptID(std::function<void(uint8_t)> cb_fxn, uint8_t rpt_ID)
            : BNO08xCbGeneric(rpt_ID)
            , cb_fxn(cb_fxn)

        {
        }

        /**
         * @brief Invokes contained callback function.
         *
         * @param rpt_ID Report ID to be passed to contained callback.
         *
         * @return void, nothing to return
         */
        void invoke(uint8_t rpt_ID) override
        {
            cb_fxn(rpt_ID);
        }

    private:
        std::function<void(uint8_t)> cb_fxn; ///< Wrapped callback function passed at register_cb(). 
};
