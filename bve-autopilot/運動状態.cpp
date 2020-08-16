// 運動状態.cpp : 走行列車の位置・速度・時刻をシミュレートします。
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
#include "運動状態.h"
#include <algorithm>
#include <cmath>
#include "共通状態.h"

namespace autopilot {

    mps2 運動状態::指定時間走行(s 時間, mps2 初加速度, mps3 加加速度)
        noexcept
    {
        _位置 +=
            時間 * (_速度 + 時間 * (初加速度 + 時間 * 加加速度 / 3.0) / 2.0);
        _速度 += 時間 * (初加速度 + 時間 * 加加速度 / 2.0);
        _時刻 += 時間;
        return 初加速度 + 時間 * 加加速度;
    }

    mps2 運動状態::指定時刻まで走行(
        時刻型 時刻, mps2 初加速度, mps3 加加速度) noexcept
    {
        mps2 終加速度 = 指定時間走行(時刻 - _時刻, 初加速度, 加加速度);
        _時刻 = 時刻; // 誤差をなくすため直接再代入する
        return 終加速度;
    }

    void 運動状態::指定距離走行(m 距離, mps2 加速度, bool 後退)
        // sqrt を使うので noexcept ではない
    {
        if (距離 == 0.0_m) {
            return;
        }
        if (加速度 == 0.0_mps2) {
            _時刻 += 距離 / _速度;
            _位置 += 距離;
            return;
        }

        mps 新速度 = sqrt(_速度 * _速度 + 2.0 * 距離 * 加速度);
        if (!(新速度 >= 0.0_mps)) {
            新速度 = 0.0_mps;
        }
        if (後退) {
            新速度 = -新速度;
        }
        _時刻 += (新速度 - _速度) / 加速度;
        _速度 = 新速度;
        _位置 += 距離;
    }

    void 運動状態::指定位置まで走行(m 位置, mps2 加速度, bool 後退)
    {
        指定距離走行(位置 - _位置, 加速度, 後退);
        _位置 = 位置; // 誤差をなくすため直接再代入する
    }

    void 運動状態::指定速度まで走行(mps 速度, mps2 加速度) noexcept
    {
        if (速度 == _速度) {
            return;
        }
        指定時間走行((速度 - _速度) / 加速度, 加速度);
        _速度 = 速度; // 誤差をなくすため直接再代入する
    }

#if 0
    mps2 運動状態::指定位置まで走行(m 位置, mps2 初加速度, mps3 加加速度)
    {
        // 三次方程式を代数的に解くのは面倒なのでニュートン法を使う。
        // 実数解が複数あるときどの解に行きつくかは分からない。
        運動状態 tmp = *this;
        mps2 加速度 = 初加速度;
        for (size_t i = 0; i < 10; i++) {
            tmp.指定位置まで走行(位置, 加速度);
            auto 時刻 = tmp.時刻();
            tmp = *this;
            加速度 = tmp.指定時刻まで走行(時刻, 初加速度, 加加速度);
        }

        *this = tmp;
        _位置 = 位置; // 誤差をなくすため直接再代入する
        return 加速度;
    }

    void 運動状態::等加加速度で指定加速度まで走行(
        mps2 初加速度, mps2 終加速度, mps3 加加速度) noexcept
    {
        s 時間 = (終加速度 - 初加速度) / 加加速度;
        指定時間走行(時間, 初加速度, 加加速度);
    }
#endif

    mps2 運動状態::指定速度まで走行(
        mps 速度, mps2 初加速度, mps3 加加速度, bool 減速)
    {
        // 速度を位置、加速度を速度だと思って
        // 新しい速度と加速度をシミュレートする
        運動状態 速度モデル{
            static_cast<m>(_速度.value),
            static_cast<mps>(初加速度.value),
            _時刻};
        速度モデル.指定位置まで走行(
            static_cast<m>(速度.value),
            static_cast<mps2>(加加速度.value),
            減速);
        指定時刻まで走行(速度モデル.時刻(), 初加速度, 加加速度);
        _速度 = 速度; // 誤差をなくすため直接再代入する
        return static_cast<mps2>(速度モデル.速度().value);
    }

    bool 短く力行(
        運動状態 &運動状態, 力行ノッチ 力行ノッチ, const 共通状態 &状態)
    {
        if (状態.前回力行ノッチ() >= static_cast<int>(力行ノッチ.value)) {
            // 現在行っている力行を直ちにやめる動き
            運動状態.指定時間走行(状態.設定().加速終了遅延(), 状態.加速度());
            return true;
        }

        mps2 出力加速度 = 状態.力行().加速度(力行ノッチ, 運動状態.速度());
        if (出力加速度 <= 0.0_mps2) {
            return false;
        }

        // 力行をこれから短い時間行う動き
        s 最低加速時間 = std::max(
            std::min(出力加速度 / 1.0_kmphps2, 7.5_kmph / 出力加速度),
            状態.設定().加速終了遅延());
        mps2 実効加速度 = 出力加速度 + 状態.車両勾配加速度();
        運動状態.指定時間走行(最低加速時間, 実効加速度);
        return true;
    }

}
