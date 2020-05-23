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
#include <utility>
#include "制動特性.h"
#include "制御指令.h"
#include "物理量.h"
#include "走行モデル.h"

namespace autopilot
{

    class 勾配グラフ;
    class 共通状態;

    struct 減速パターン
    {
        m 目標位置;
        mps 目標速度;
        mps2 目標減速度;
        bool 素早い速度超過回復;

        constexpr 減速パターン(
            m 目標位置, mps 目標速度, mps2 目標減速度,
            bool 素早い速度超過回復 = false) noexcept :
            目標位置(目標位置), 目標速度(目標速度),
            目標減速度(目標減速度), 素早い速度超過回復{素早い速度超過回復} {}

        ~減速パターン() = default;

        /// 目標到達後も減速を続けた場合の期待状態
        std::pair<mps, mps2> 延長期待状態(
            m 現在位置, const 勾配グラフ &勾配) const;
        /// 目標到達後は目標速度を維持する場合の期待状態
        std::pair<mps, mps2> 期待状態(
            m 現在位置, const 勾配グラフ &勾配) const;
        mps 延長期待速度(m 現在位置, const 勾配グラフ &勾配) const {
            return 延長期待状態(現在位置, 勾配).first;
        }
        mps 期待速度(m 現在位置, const 勾配グラフ &勾配) const {
            return 期待状態(現在位置, 勾配).first;
        }

        /// 延長期待状態に従って減速し続けるための出力減速度
        mps2 減速用出力減速度(m 現在位置, mps 現在速度, const 勾配グラフ &勾配)
            const;
        /// 目標速度に達したら減速をやめるための出力減速度の下限
        mps2 収束用出力減速度(mps 現在速度) const;
        自動制動自然数ノッチ 出力制動ノッチ(
            m 現在位置, mps 現在速度, 自動制動自然数ノッチ 現在制動ノッチ,
            const 共通状態 &状態) const;
        bool 力行する余裕あり(
            力行ノッチ 力行ノッチ, mps2 想定加速度, s 想定惰行時間,
            const 共通状態 &状態) const;
        自動制御指令 出力ノッチ(const 共通状態 &状態) const;

        /// 指定した速度におけるパターン上の位置と時刻を返します。
        /// 時刻は、減速目標に到達する時刻を 0 とし、
        /// それより前のパターン上の時刻は負になります。
        /// 指定した速度が目標速度以下ならパターン終了時の状態を返します。
        走行モデル パターン到達状態(mps 速度) const;

    };

}
