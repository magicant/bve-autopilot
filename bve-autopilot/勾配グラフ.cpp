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
#include <stdexcept>
#include <utility>
#include "区間.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr mps2 重力加速度 = 9.80665_mps2;

        constexpr mps2 勾配加速度(勾配グラフ::勾配 勾配) noexcept
        {
            return -0.75 * 重力加速度 * 勾配;
        }

    }

    mps2 勾配加速度グラフ::勾配加速度(m 位置) const
    {
        return 勾配加速度(_変化点リスト.upper_bound(位置), 位置);
    }

    void 勾配加速度グラフ::勾配変化追加(区間 変化区間, 勾配 勾配変化量)
    {
        if (変化区間.始点 >= 変化区間.終点) {
            throw std::invalid_argument("non-empty interval required");
        }

        // まず始点の値を設定する
        const_iterator h = _変化点リスト.upper_bound(変化区間.始点);
        iterator i = _変化点リスト.try_emplace(
            h, 変化区間.始点,
            変化点{勾配加速度(h, 変化区間.始点), m2ps2::quiet_NaN()});

        // 終点の値を求める
        mps2 加速度変化量 = autopilot::勾配加速度(勾配変化量);
        mps2 新しい終点加速度 = 勾配加速度(変化区間.終点) + 加速度変化量;

        // 始点より後にある点を更新する
        m 区間長さ = 変化区間.長さ();
        while (++i != _変化点リスト.end()) {
            double 比 = std::min((i->first - 変化区間.始点) / 区間長さ, 1.0);
            i->second.勾配加速度 += 比 * 加速度変化量;
            i->second.累積比エネルギー = m2ps2::quiet_NaN();
        }

        // 終点の値を設定する
        _変化点リスト.insert_or_assign(
            _変化点リスト.end(), 変化区間.終点,
            変化点{新しい終点加速度, m2ps2::quiet_NaN()});
    }

    mps2 勾配加速度グラフ::勾配加速度(const_iterator i, m 位置) const
    {
        if (i == _変化点リスト.begin()) {
            return 0.0_mps2;
        }
        const_iterator h = std::prev(i);
        if (i == _変化点リスト.end()) {
            return h->second.勾配加速度;
        }
        m h位置 = h->first, i位置 = i->first;
        assert(h位置 <= 位置);
        assert(位置 < i位置);
        double 比 = (位置 - h位置) / (i位置 - h位置);
        return h->second.勾配加速度 +
            比 * (i->second.勾配加速度 - h->second.勾配加速度);
    }

    勾配グラフ::勾配グラフ() = default;
    勾配グラフ::~勾配グラフ() = default;

    void 勾配グラフ::消去() noexcept
    {
        _区間リスト.clear();
    }

    void 勾配グラフ::列車長を設定(m 列車長) noexcept
    {
        // 勾配加速度グラフは長さ 0 の列車を扱えないので 0 にはしない
        _列車長 = std::max(列車長, 0.5_m);

        キャッシュ消去();
    }

    void 勾配グラフ::勾配区間追加(m 始点, 勾配 勾配)
    {
        _区間リスト.insert_or_assign(始点, 勾配);
    }

    void 勾配グラフ::通過(m 列車先頭位置)
    {
        if (_区間リスト.empty()) {
            return;
        }

        // 通過済みの区間を消す
        m 列車最後尾位置 = 列車先頭位置 - _列車長;
        auto i = _区間リスト.begin();
        while (true) {
            auto j = std::next(i);
            if (j == _区間リスト.end() || j->first > 列車最後尾位置) {
                break;
            }
            i = j;
        }
        if (i == _区間リスト.begin()) {
            return;
        }
        i = _区間リスト.erase(_区間リスト.begin(), i);
        assert(!_区間リスト.empty());
        assert(i == _区間リスト.begin());
        キャッシュ消去();
    }

    mps2 勾配グラフ::列車勾配加速度(m 列車先頭位置) const
    {
        return mps2(); // TODO not implemented yet
    }

    m2ps2 勾配グラフ::下り勾配比エネルギー(区間 変位) const
    {
        return m2ps2(); // TODO not implemented yet
    }

    void 勾配グラフ::キャッシュ消去() noexcept
    {
        // TODO not implemented yet
    }

}
