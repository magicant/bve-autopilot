// 加速度計.cpp : 速度変化から加速度を推定します
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
#include "加速度計.h"
#include <algorithm>

namespace autopilot
{

    加速度計::加速度計()
    {
        リセット();
    }


    加速度計::~加速度計()
    {
    }

    void 加速度計::リセット()
    {
        for (観測& 記録 : _記録) {
            記録 = {};
        }
        _加速度 = {};
    }

    void 加速度計::経過(観測 今回)
    {
        // 加加速度計算用に取っておく
        加速度型 旧加速度 = _加速度;
        時刻型 旧時刻 = _記録[記録数 - 1]._時刻;

        // 過去"記録数"回分の記録との速度差から算出した加速度の平均を求める
        加速度型 加速度和 = 0;

        for (unsigned i = 0; i < 記録数; i++) {
            時刻型 時刻差 = 今回._時刻 - _記録[i]._時刻;
            速度型 速度差 = 今回._速度 - _記録[i]._速度;
            if (時刻差 > 0) {
                加速度和 += 速度差 / 時刻差;
            }

            _記録[i] = i + 1 < 記録数 ? _記録[i + 1] : 今回;
        }

        _加速度 = 加速度和 / 記録数;
        _加加速度 = (_加速度 - 旧加速度) / std::max(今回._時刻 - 旧時刻, 0.0);
    }

}
