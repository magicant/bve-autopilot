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
#include "単位.h"
#include "減速パターン.h"

namespace autopilot
{

    減速パターン 制限区間::目標パターン(加速度型 初期減速度) const
    {
        constexpr 速度型 速度マージン = mps_from_kmph(0.5);
        速度型 目標速度 = std::max(速度 - 速度マージン, 0.0);
        加速度型 最終減速度 = 目標速度 == 0.0 ?
            減速パターン::停止最終減速度 : 減速パターン::標準最終減速度;
        return 減速パターン{減速目標地点, 目標速度, 初期減速度, 最終減速度};
    }

    減速パターン 制限区間::限界パターン(加速度型 減速度) const
    {
        return 減速パターン{始点, 速度, 減速度};
    }

}
