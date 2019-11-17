// 急動作抑制.h : 急制動・急加速を避けるために出力指令を調整します
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
        void リセット();
        void 経過(自動制御指令 入力指令, const 共通状態 &状態, bool is_atc);

        自動制御指令 出力指令() const { return _出力指令; }

    private:
        mps2 _出力減速度 = 0.0_mps2;
        自動制御指令 _出力指令;

        s _最終出力減速度計算時刻 = -s::無限大();
        s _最終力行時刻 = -s::無限大();
        s _最終制動時刻 = -s::無限大();

        mps2 新出力減速度(
            自動制御指令 入力指令, const 共通状態 &状態, bool is_atc) const;
        自動制御指令 新出力指令(
            自動制御指令 入力指令, const 共通状態 &状態) const;
    };

}
