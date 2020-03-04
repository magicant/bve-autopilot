// 区間.cpp : 二つの点で挟まれた区間を表します
//
// Copyright © 2020 Watanabe, Yuki
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA  02110 - 1301  USA

#include "stdafx.h"
#include "区間.h"
#include <cmath>
#include <limits>
#include <utility>

namespace autopilot
{

    namespace
    {

        m increment(m v)
        {
            // BVE の計算する距離は float の精度しかないので
            // float 値の最後の桁に誤差があるとみて値をずらす
            float n = std::nextafterf(
                static_cast<float>(v.value),
                std::numeric_limits<float>::max());
            return static_cast<m>(n);
        }

        m decrement(m v)
        {
            float n = std::nextafterf(
                static_cast<float>(v.value),
                std::numeric_limits<float>::lowest());
            return static_cast<m>(n);
        }

    }

    区間 安全マージン付き区間(m 始点, m 終点, m 変位)
    {
        if (始点 > 終点) {
            std::swap(始点, 終点);
        }
        return {decrement(始点) + decrement(変位),
            increment(終点) + increment(変位)};
    }

}
