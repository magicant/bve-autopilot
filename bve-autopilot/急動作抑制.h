// 急動作抑制.h : 急制動・急加速を避けるために出力ノッチを調整します
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
#include <limits>
#include "制御指令.h"
#include "物理量.h"

namespace autopilot
{

    class 共通状態;

    class 急動作抑制
    {
    public:
        constexpr void リセット() noexcept { *this = 急動作抑制{}; }
        void 経過(自動制御指令 入力ノッチ, const 共通状態 &状態, bool is_atc);

        constexpr 自動制御指令 出力ノッチ() const noexcept {
            return _出力ノッチ;
        }

    private:
        mps2 _最小出力減速度 = 0.0_mps2, _最大出力減速度 = 0.0_mps2;
        自動制御指令 _出力ノッチ;

        s _最終出力減速度計算時刻 = -s::無限大();
        s _最終力行時刻 = -s::無限大();
        s _最終制動時刻 = -s::無限大();

        std::pair<mps2, mps2> 新出力減速度(
            自動制御指令 入力ノッチ, const 共通状態 &状態, bool is_atc) const;
        自動制御指令 新出力ノッチ(
            自動制御指令 入力ノッチ, const 共通状態 &状態) const;
    };

}
