// 区間.h : 二つの点で挟まれた区間を表します
//
// Copyright © 2019 Watanabe, Yuki
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

#pragma once
#include <algorithm>
#include "物理量.h"

namespace autopilot
{

    struct 区間
    {
        m 始点, 終点;

        constexpr m 長さ() const {
            return 終点 - 始点;
        }

        constexpr bool 含む(m 点) const {
            return 始点 <= 点 && 点 <= 終点;
        }
        constexpr bool 含む(区間 i) const {
            return 始点 <= i.始点 && i.終点 <= 終点;
        }
        constexpr bool 空である() const {
            return 始点 > 終点;
        }

        constexpr bool 通過済(m 列車最後尾位置) const {
            return 終点 < 列車最後尾位置;
        }

    };

    constexpr bool 重なる(const 区間 & 区間1, const 区間 & 区間2) {
        return 区間1.始点 <= 区間2.終点 && 区間2.始点 <= 区間1.終点;
    }
    constexpr 区間 重なり(const 区間 &区間1, const 区間 &区間2) {
        return { std::max(区間1.始点, 区間2.始点),
            std::min(区間1.終点, 区間2.終点) };
    }

}
