// 減速目標.h : 制限速度を定め、それを維持するブレーキを計算します
//
// Copyright © 2020 Watanabe, Yuki
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
#include "制動指令計算.h"
#include "減速パターン.h"
#include "物理量.h"
#include "運動状態.h"

namespace autopilot
{

    class 勾配グラフ;
    class 共通状態;

    class 減速目標 : public 制動指令計算
    {
    public:
        constexpr static mps 速度マージン = 0.5_kmph;

        constexpr 減速目標(m 位置, mps 速度, mps2 基準減速度) noexcept :
            _位置{位置}, _速度{速度}, _基準減速度{基準減速度} {}

        constexpr m 位置() const noexcept { return _位置; }
        constexpr mps 速度() const noexcept { return _速度; }
        constexpr mps2 基準減速度() const noexcept { return _基準減速度; }

#if 0
        /// 指定した速度におけるパターン上の位置と時刻を返します。
        /// 時刻は、減速目標に到達する時刻を引数の時刻とし、
        /// パターン上の時刻はそれより前になります。
        /// 指定した速度が目標速度以下ならパターン終了時の状態を返します。
        運動状態 パターン到達状態(mps 速度, 時刻 終端時刻 = {}) const;
#endif

        /// 制限速度を維持するために適したノッチを計算します
        自動制動自然数ノッチ 出力制動ノッチ(
            const 運動状態 &運動状態, const 共通状態 &状態) const
            final override;

        区間 最低許容速度区間(区間 範囲) const final override;

    private:
        m _位置;
        mps _速度;
        mps2 _基準減速度;

        減速パターン 主パターン(const 勾配グラフ &勾配) const;
        mps2 副パターン減速度(const 運動状態 &運動状態) const;
        mps2 減速用出力減速度(
            const 運動状態 &運動状態, const 勾配グラフ &勾配) const;
        mps2 収束用出力減速度(mps 現在速度) const;

    };

}
