// 力行特性.cpp : 力行ノッチと加速度の関係を計算します
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
#include "力行特性.h"

namespace autopilot
{

    力行特性::力行特性() = default;
    力行特性::~力行特性() = default;

    void 力行特性::性能設定(
        const std::vector<mps2> &加速度一覧, 力行ノッチ 最大ノッチ)
    {
        _加速度一覧[0.0_mps] = 加速度一覧;
        _最大ノッチ = 最大ノッチ;
    }

    mps2 力行特性::加速度(力行ノッチ ノッチ, mps 速度) const
    {
        auto n = static_cast<std::size_t>(ノッチ.value);
        if (n == 0u) {
            return 0.0_mps2;
        }
        --n;

        auto i = _加速度一覧.upper_bound(速度);
        if (i != _加速度一覧.begin()) {
            --i;
            if (n < i->second.size()) {
                return i->second[n];
            }
        }

        // デフォルト値
        if (n == 0u && _最大ノッチ >= 力行ノッチ{2}) {
            // 抵抗器保護のため第一ノッチは低速域でしか使わないようにする
            constexpr mps 上限速度 = 10.0_kmph;
            return 速度 > 上限速度 ? 0.0_kmphps : 2.5_kmphps;
        }
        return 5.0_kmphps;
    }

}
