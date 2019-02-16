// 制動出力.cpp : 期待する減速度を得るために制動ノッチを加減します
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
#include "制動出力.h"
#include "共通状態.h"
#include <cmath>

namespace autopilot
{

    void 制動出力::リセット()
    {
        _前回出力ノッチ = 0;
        _前回出力ノッチ変更時刻 = 0;
    }

    void 制動出力::性能設定(
        int 常用ノッチ数, int 無効ノッチ数, 加速度型 常用最大減速度)
    {
        _常用ノッチ数 = 常用ノッチ数;
        _無効ノッチ数 = 無効ノッチ数;
        _ノッチ当たり減速度 = 常用最大減速度 / 実効ノッチ数();
    }

    void 制動出力::経過(const 加速度計 & 加速状態, 時刻型 時刻, int 前回出力ノッチ)
    {
        if (_前回出力ノッチ != 前回出力ノッチ) {
            _前回出力ノッチ = 前回出力ノッチ;
            _前回出力ノッチ変更時刻 = 時刻;
            return;
        }

        // 常用制動使用中でなければノッチ当たり減速度を再計算しない
        if (前回出力ノッチ <= _無効ノッチ数 || _常用ノッチ数 < 前回出力ノッチ) {
            return;
        }

        // ノッチ変更直後はノッチ当たり減速度を再計算しない
        if (std::abs(時刻 - _前回出力ノッチ変更時刻) < 0.5) {
            return;
        }

        // 加速度が大きく変化している間はノッチ当たり減速度を再計算しない
        if (std::abs(加速状態.加加速度()) > mps_from_kmph(1)) {
            return;
        }

        // 減速していなければノッチ当たり減速度を再計算しない
        加速度型 減速度 = -加速状態.加速度();
        if (減速度 <= 0) {
            return;
        }

        // ノッチ当たり減速度を再計算する
        int 実効ノッチ = 前回出力ノッチ - _無効ノッチ数;
        _ノッチ当たり減速度 = 減速度 / 実効ノッチ;
    }

    double 制動出力::ノッチ(加速度型 減速度) const
    {
        return 減速度 / _ノッチ当たり減速度 + _無効ノッチ数;
    }

}
