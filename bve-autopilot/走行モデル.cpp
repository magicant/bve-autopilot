// 走行モデル.cpp : 走行列車の位置・速度・時刻をシミュレートします。
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
#include "走行モデル.h"
#include <cmath>

namespace autopilot {

    mps2 走行モデル::指定時間走行(s 時間, mps2 初加速度, mps3 加加速度)
    {
        _位置 +=
            時間 * (_速度 + 時間 * (初加速度 + 時間 * 加加速度 / 3.0) / 2.0);
        _速度 += 時間 * (初加速度 + 時間 * 加加速度 / 2.0);
        _時刻 += 時間;
        return 初加速度 + 時間 * 加加速度;
    }

    mps2 走行モデル::指定時刻まで走行(s 時刻, mps2 初加速度, mps3 加加速度)
    {
        mps2 終加速度 = 指定時間走行(時刻 - _時刻, 初加速度, 加加速度);
        _時刻 = 時刻; // 誤差をなくすため直接再代入する
        return 終加速度;
    }

    void 走行モデル::指定距離走行(m 距離, mps2 加速度)
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
        _時刻 += (新速度 - _速度) / 加速度;
        _速度 = 新速度;
        _位置 += 距離;
    }

    void 走行モデル::指定位置まで走行(m 位置, mps2 加速度)
    {
        指定距離走行(位置 - _位置, 加速度);
        _位置 = 位置; // 誤差をなくすため直接再代入する
    }

    void 走行モデル::指定速度まで走行(mps 速度, mps2 加速度)
    {
        if (速度 == _速度) {
            return;
        }
        指定時間走行((速度 - _速度) / 加速度, 加速度);
        _速度 = 速度; // 誤差をなくすため直接再代入する
    }

    mps2 走行モデル::指定位置まで走行(m 位置, mps2 初加速度, mps3 加加速度)
    {
        // 三次方程式を代数的に解くのは面倒なのでニュートン法を使う。
        // 実数解が複数あるときどの解に行きつくかは分からない。
        走行モデル tmp = *this;
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

    void 走行モデル::等加加速度で指定加速度まで走行(
        mps2 初加速度, mps2 終加速度, mps3 加加速度)
    {
        s 時間 = (終加速度 - 初加速度) / 加加速度;
        指定時間走行(時間, 初加速度, 加加速度);
    }

    mps2 走行モデル::指定速度まで走行(mps 速度, mps2 初加速度, mps3 加加速度)
    {
        // 速度を位置、加速度を速度だと思って
        // 新しい速度とか速度をシミュレートする
        走行モデル 速度モデル{
            static_cast<m>(_速度.value),
            static_cast<mps>(初加速度.value),
            _時刻};
        速度モデル.指定位置まで走行(
            static_cast<m>(速度.value),
            static_cast<mps2>(加加速度.value));
        指定時刻まで走行(速度モデル.時刻(), 初加速度, 加加速度);
        _速度 = 速度; // 誤差をなくすため直接再代入する
        return static_cast<mps2>(速度モデル.速度().value);
    }

    mps2 走行モデル::距離と速度による加速度(m 距離, mps 初速度, mps 終速度)
    {
        return (終速度 * 終速度 - 初速度 * 初速度) / 距離 / 2.0;
    }

}
