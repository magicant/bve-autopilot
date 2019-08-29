// 制限区間.cpp : 列車の速度が制限されている区間を表します
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

#include "stdafx.h"
#include "制限区間.h"
#include <algorithm>
#include <cmath>

namespace autopilot
{

    減速パターン 制限区間::対応パターン(
        加速度型 初期減速度, 加速度型 最終減速度,
        時間型 時間マージン, 速度型 速度マージン) const
    {
        速度型 目標速度 = std::max(速度 - 速度マージン, 0.0);
        距離型 距離マージン = 目標速度 * 時間マージン;
        距離型 目標位置 = 始点 - (std::isnan(距離マージン) ? 0 : 距離マージン);
        return 減速パターン{ 目標位置, 目標速度, 初期減速度, 最終減速度 };
    }

}
