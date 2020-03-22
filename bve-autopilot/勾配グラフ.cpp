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
        m 始点;
        double 勾配;
        mps2 影響加速度;

        勾配区間(m 始点, double 勾配) :
            始点{始点},
            勾配{勾配},
            影響加速度{-0.75 * 重力加速度 * 勾配} { }
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
        // 新しい制限区間に上書きされる区間を消す
        _区間リスト.remove_if([始点](const 勾配区間 & 区間) {
            return 区間.始点 >= 始点;
            });

        _区間リスト.emplace_front(始点, 勾配);
    }

    void 勾配グラフ::通過(m 位置)
    {
        // 通過済みの区間を消す
        auto i = _区間リスト.begin();
        while (true) {
            if (i == _区間リスト.end())
                return;
            if (i->始点 < 位置)
                break;
            ++i;
        }
        _区間リスト.erase_after(i, _区間リスト.end());
    }

    mps2 勾配グラフ::勾配加速度(区間 対象範囲) const
    {
        m 全体長さ = 対象範囲.長さ();
        if (!(全体長さ > 0.0_m)) {
            return 0.0_mps2;
        }

        mps2 加速度 = 0.0_mps2;
        auto 終点 = m::無限大();
        for (const 勾配区間 &区間 : _区間リスト) {
            auto 影響区間 = 重なり({区間.始点, 終点}, 対象範囲);
            m 影響長さ = 影響区間.長さ();
            終点 = 区間.始点;
            if (!(影響長さ > 0.0_m)) {
                continue;
            }

            double 影響割合 = 影響長さ / 全体長さ;
            if (std::isnan(影響割合)) {
                影響割合 = 1;
            }
            加速度 += 区間.影響加速度 * 影響割合;
        }
        return 加速度;
    }

}
