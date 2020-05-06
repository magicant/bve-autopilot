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

        static constexpr 信号インデックス orp信号インデックス = 35;

        orp() noexcept;
        ~orp() = default;

        void リセット() noexcept;
        void 設定(mps 初期照査速度, m 初期位置, m 限界位置) noexcept;
        void 設定(int 地上子値, m 初期位置) noexcept;
        void 設定(mps 直前閉塞速度, m 初期位置) noexcept;

        void 経過(const 共通状態 &状態);

        bool 制御中() const noexcept {
            return _運転パターン.目標位置 < m::無限大();
        }

        自動制御指令 出力ノッチ() const noexcept { return _出力ノッチ; }

        mps 照査速度() const noexcept { return _照査速度; }

    private:
        減速パターン _照査パターン, _運転パターン;
        自動制御指令 _出力ノッチ;
        mps _照査速度;
    };

}
