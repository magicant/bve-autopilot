// 勾配グラフ.cpp : 勾配による列車の挙動への影響を計算します
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
#include "勾配グラフ.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include "区間.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr mps2 重力加速度 = 9.80665_mps2;

    }

    struct 勾配グラフ::勾配区間
    {
        double 勾配;
        mps2 影響加速度;

        勾配区間(double 勾配) :
            勾配{勾配}, 影響加速度{-0.75 * 重力加速度 * 勾配} { }
        // 本当は tan を sin に変換すべきだがほとんど違わないので無視する

    };

    勾配グラフ::勾配グラフ() = default;
    勾配グラフ::~勾配グラフ() = default;

    void 勾配グラフ::消去()
    {
        _区間リスト.clear();
    }

    void 勾配グラフ::勾配区間追加(m 始点, double 勾配)
    {
        _区間リスト.insert_or_assign(始点, 勾配区間{勾配});
    }

    void 勾配グラフ::通過(m 位置)
    {
        auto i = _区間リスト.upper_bound(位置);
        if (i == _区間リスト.begin()) {
            return;
        }
        --i;
        _区間リスト.erase(_区間リスト.begin(), i);
    }

    mps2 勾配グラフ::勾配加速度(区間 対象範囲) const
    {
        m 全体長さ = 対象範囲.長さ();
        if (!(全体長さ > 0.0_m)) {
            return 0.0_mps2;
        }

        mps2 加速度 = 0.0_mps2;
        auto 終点 = m::無限大();
        for (auto i = _区間リスト.rbegin();
            i != _区間リスト.rend();
            終点 = i++->first)
        {
            auto 影響区間 = 重なり({i->first, 終点}, 対象範囲);
            m 影響長さ = 影響区間.長さ();
            if (!(影響長さ > 0.0_m)) {
                continue;
            }

            double 影響割合 = 影響長さ / 全体長さ;
            if (std::isnan(影響割合)) {
                影響割合 = 1;
            }
            加速度 += i->second.影響加速度 * 影響割合;
        }
        return 加速度;
    }

}
