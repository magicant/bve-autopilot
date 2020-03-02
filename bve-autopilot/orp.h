// orp.h : ORP の照査に抵触しないように減速します
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
#include "制御指令.h"
#include "減速パターン.h"
#include "物理量.h"

namespace autopilot
{

    class 共通状態;
    class 信号順守;

    class orp
    {
    public:
        using 信号インデックス = int;

        orp();
        ~orp() = default;

        void リセット();
        void 設定(mps 初期照査速度, m 初期位置, m 限界位置);

        void 信号現示変化(信号インデックス 指示);
        void 地上子通過(
            const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態,
            const 信号順守 &信号);

        void 経過(const 共通状態 &状態);

        bool 制御中() const;
        bool 照査中() const;

        自動制御指令 出力ノッチ() const { return _出力ノッチ; }

        mps 照査速度() const { return _照査速度; }

    private:
        信号インデックス _信号指示;
        減速パターン _照査パターン, _運転パターン;
        自動制御指令 _出力ノッチ;
        mps _照査速度;
    };

}
