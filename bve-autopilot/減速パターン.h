// 減速パターン.h : 等加速度で減速するパターンを表します
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

#pragma once
#include "物理量.h"

namespace autopilot
{

    class 勾配グラフ;

    class 減速パターン
    {
    public:
        constexpr 減速パターン(
            m 通過地点, mps 通過速度, mps2 基準減速度) noexcept :
            _通過地点{通過地点}, _通過速度{通過速度}, _基準減速度{基準減速度}
        {}

        constexpr m 通過地点() const noexcept { return _通過地点; }
        constexpr mps 通過速度() const noexcept { return _通過速度; }
        constexpr mps2 基準減速度() const noexcept { return _基準減速度; }

        static 減速パターン 二点間パターン(
            m 通過地点, mps 通過速度, m 現在位置, mps 現在速度,
            const 勾配グラフ &勾配);

        mps 期待速度(m 位置, const 勾配グラフ &勾配) const;
        mps2 期待減速度(m 位置, const 勾配グラフ &勾配) const;

    private:
        m _通過地点;
        mps _通過速度;
        /// 出力ノッチ計算の基準となる減速度。
        /// 上り勾配では実際の減速度が基準減速度に一致するように調整される。
        /// 下り勾配では乗客が感じる減速度が基準減速度に一致するようにする。
        mps2 _基準減速度;
    };

}
