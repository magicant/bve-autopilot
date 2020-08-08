// 減速パターン.cpp : 等加速度で減速するパターンを表します
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
#include "減速パターン.h"
#include <algorithm>
#include "勾配グラフ.h"

namespace autopilot
{

    減速パターン 減速パターン::二点間パターン(
        m 通過地点, mps 通過速度, m 現在位置, mps 現在速度,
        const 勾配グラフ &勾配)
    {
        m 距離 = 通過地点 - 現在位置;
        m2ps2 復帰地点比エネ2 = 通過速度 * 通過速度;
        m2ps2 現在位置比エネ2 = 現在速度 * 現在速度;
        m2ps2 速度比エネ差 = 0.5 * (現在位置比エネ2 - 復帰地点比エネ2);
        m2ps2 勾配比エネ差 =
            勾配.下り勾配比エネルギー差({現在位置, 通過地点});
        mps2 基準減速度 = (速度比エネ差 + 勾配比エネ差) / 距離;
        return 減速パターン{通過地点, 通過速度, 基準減速度};
    }

    mps 減速パターン::期待速度(m 位置, const 勾配グラフ &勾配) const
    {
        m2ps2 通過位置比エネ = 0.5 * _通過速度 * _通過速度;
        m2ps2 平坦行路比エネ = (_通過地点 - 位置) * _基準減速度;
        if (isnan(平坦行路比エネ)) {
            return 0.0_mps;
        }
        if (平坦行路比エネ == m2ps2::無限大()) {
            return mps::無限大();
        }
        m2ps2 勾配比エネ差 = 勾配.下り勾配比エネルギー差({位置, _通過地点});
        m2ps2 補正行路比エネ = 平坦行路比エネ - 勾配比エネ差;
        m2ps2 現在位置比エネ = 通過位置比エネ + 補正行路比エネ;
        mps 期待速度 = sqrt(std::max(2.0 * 現在位置比エネ, 0.0_m2ps2));
        return 期待速度;
    }

    mps2 減速パターン::期待減速度(m 位置, const 勾配グラフ &勾配) const
    {
        mps2 勾配加速度 = 勾配.列車勾配加速度(位置);
        mps2 補正加速度 = std::max(勾配加速度, 0.0_mps2);
        mps2 期待減速度 = _基準減速度 - 補正加速度;
        return 期待減速度;
    }

}
