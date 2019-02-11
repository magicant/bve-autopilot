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

    void 走行モデル::指定時間走行(時間型 時間, 加速度型 加速度)
    {
        _位置 += _速度 * 時間 + 加速度 * 時間 * 時間 / 2;
        _速度 += 加速度 * 時間;
        _時刻 += 時間;
    }

    void 走行モデル::指定時刻まで走行(時間型 時刻, 加速度型 加速度)
    {
        指定時間走行(時刻 - _時刻);
        _時刻 = 時刻; // 誤差をなくすため直接再代入する
    }

    void 走行モデル::指定距離走行(距離型 距離, 加速度型 加速度)
    {
        if (距離 == 0) {
            return;
        }
        速度型 新速度 = std::sqrt(_速度 * _速度 + 2 * 距離 * 加速度);
        if (!(新速度 >= 0)) {
            新速度 = 0;
        }
        _時刻 += (新速度 - _速度) / 加速度;
        _速度 = 新速度;
        _位置 += 距離;
    }

    void 走行モデル::指定位置まで走行(距離型 位置, 加速度型 加速度)
    {
        指定距離走行(位置 - _位置, 加速度);
    }

    void 走行モデル::指定速度まで走行(速度型 速度, 加速度型 加速度)
    {
        if (速度 == _速度) {
            return;
        }
        指定時間走行((速度 - _速度) / 加速度, 加速度);
        _速度 = 速度; // 誤差をなくすため直接再代入する
    }

    走行モデル::加速度型 走行モデル::距離と速度による加速度(
        距離型 距離, 速度型 初速度, 速度型 終速度)
    {
        return (終速度 * 終速度 - 初速度 * 初速度) / 距離 / 2.0;
    }

}
