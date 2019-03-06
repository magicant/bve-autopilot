// 単位.h : 物理量の単位を扱います
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

namespace autopilot
{

    // 秒
    using 時間型 = double;
    // メートル
    using 距離型 = double;
    // メートル毎秒
    using 速度型 = double;
    // メートル毎秒毎秒
    using 加速度型 = double;
    // メートル毎秒毎秒毎秒
    using 加加速度型 = double;

    constexpr 時間型 s_from_ms(double ms) {
        return ms / 1000.0;
    }

    constexpr 速度型 mps_from_kmph(double kmph) {
        return kmph / 3.6;
    }

}
