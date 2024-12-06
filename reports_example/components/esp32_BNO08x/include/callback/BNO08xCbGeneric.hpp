/**
 * @file BNO08xCbGeneric.hpp
 * @author Myles Parfeniuk
 */

#pragma once

// standard library includes
#include <stdint.h>
#include <functional>

/**
 * @class BNO08xCbGeneric
 *
 * @brief Parent class to represent callback functions as generic type such that all flavors can be
 * invoked by single type.
 */
class BNO08xCbGeneric
{
    public:
        virtual void invoke(uint8_t rpt_ID) = 0;
        virtual ~BNO08xCbGeneric() = default;
        uint8_t rpt_ID;

    protected:
        BNO08xCbGeneric(uint8_t rpt_ID)
            : rpt_ID(rpt_ID)
        {
        }
};