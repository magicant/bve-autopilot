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
    class 運動状態;

    class orp
    {
    public:
        using 信号インデックス = int;

        static constexpr 信号インデックス orp信号インデックス = 35;

        orp() noexcept;
        ~orp() = default;

        void リセット() noexcept;
        void 設定(mps 開始照査速度, m 開始位置, m 限界位置);
        void 設定(int 地上子値, m 開始位置);
        void 設定(mps 直前閉塞速度, m 開始位置);

        void 経過(const 共通状態 &状態);

        bool 制御中() const noexcept {
            return _照査パターン.通過地点() < m::無限大();
        }

        自動制御指令 出力ノッチ() const noexcept { return _出力ノッチ; }

        mps 照査速度() const noexcept { return _照査速度; }

    private:
        m _開始位置;
        mps _開始照査速度;
        減速パターン _照査パターン;
        bool _照査速度下限到達;
        自動制御指令 _出力ノッチ;
        mps _照査速度;

        mps2 照査下パターン出力減速度(const 運動状態 &運動状態) const;
        mps2 接近パターン出力減速度(
            const 運動状態 &運動状態, const 共通状態 &状態) const;
        mps2 下限照査出力減速度(const 運動状態 &運動状態) const;
        自動制動自然数ノッチ 出力制動ノッチ(
            const 運動状態 &運動状態, const 共通状態 &状態) const;
    };

}
