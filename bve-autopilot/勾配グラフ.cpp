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
#include <cassert>
#include <cmath>
#include <iterator>
#include <utility>
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

        constexpr explicit 勾配区間(double 勾配) noexcept :
            勾配{勾配}, 影響加速度{-0.75 * 重力加速度 * 勾配} { }
        // 本当は tan を sin に変換すべきだがほとんど違わないので無視する

    };

    勾配グラフ::勾配グラフ() = default;
    勾配グラフ::~勾配グラフ() = default;

    void 勾配グラフ::消去() noexcept
    {
        _区間リスト.clear();
    }

    void 勾配グラフ::勾配区間追加(m 始点, double 勾配)
    {
        _区間リスト.insert_or_assign(始点, 勾配区間{勾配});
    }

    void 勾配グラフ::通過(m 位置)
    {
        if (_区間リスト.empty()) {
            return;
        }

        // 通過済みの区間を消す
        auto i = _区間リスト.begin();
        while (true) {
            auto j = std::next(i);
            if (j == _区間リスト.end() || j->first > 位置) {
                break;
            }
            i = j;
        }
        i = _区間リスト.erase(_区間リスト.begin(), i);
        assert(!_区間リスト.empty());
        assert(i == _区間リスト.begin());
    }

    mps2 勾配グラフ::列車勾配加速度(m 列車先頭位置) const
    {
        return mps2(); // TODO not implemented yet
    }

    m2ps2 勾配グラフ::下り勾配比エネルギー(区間 変位) const
    {
        return m2ps2(); // TODO not implemented yet
    }

}
