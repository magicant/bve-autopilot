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
        if (!加速度一覧.empty()) {
            _加速度一覧 = 加速度一覧;
            return;
        }

        // デフォルトの加速度一覧を生成する
        auto n = static_cast<std::size_t>(最大ノッチ.value);
        _加速度一覧.reserve(n);
        if (n > 1u) {
            _加速度一覧.push_back(2.5_kmphps);
        }
        while (_加速度一覧.size() < n) {
            _加速度一覧.push_back(5.0_kmphps);
        }
    }

    mps2 力行特性::加速度(力行ノッチ ノッチ) const
    {
        if (ノッチ.value == 0u) {
            return 0.0_mps2;
        }
        if (ノッチ.value <= _加速度一覧.size()) {
            return _加速度一覧[static_cast<std::size_t>(ノッチ.value - 1)];
        }

        constexpr mps2 最大加速度 = 5.0_kmphps;
        if (_加速度一覧.empty()) {
            return 最大加速度;
        }
        return std::max(_加速度一覧.back(), 最大加速度);
    }

}
