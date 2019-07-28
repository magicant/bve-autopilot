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
#include "単位.h"
#include "走行モデル.h"

namespace autopilot
{

    class 共通状態;

    struct 減速パターン
    {
        static constexpr 加速度型 標準最終加速度 = mps_from_kmph(1);

        距離型 目標位置;
        速度型 目標速度;
        加速度型 初期減速度, 最終減速度;

        // 最終減速度を省略するコンストラクター:
        // 目標速度に近付くにつれて期待減速度が小さくなるようにする
        // ただし停止目標に対してこれをやると手前に止まりがちなのでやらない
        constexpr 減速パターン(
            距離型 目標位置, 速度型 目標速度, 加速度型 初期減速度) :
            目標位置(目標位置), 目標速度(目標速度), 初期減速度(初期減速度),
            最終減速度(目標速度 > 0 ? 標準最終加速度 : 初期減速度) {}
        constexpr 減速パターン(
            距離型 目標位置, 速度型 目標速度,
            加速度型 初期減速度, 加速度型 最終減速度) :
            目標位置(目標位置), 目標速度(目標速度),
            初期減速度(初期減速度), 最終減速度(最終減速度) {}

        ~減速パターン() = default;

        std::pair<速度型, 加速度型> 期待速度と期待減速度(距離型 現在位置)
            const;
        速度型 期待速度(距離型 現在位置) const {
            return 期待速度と期待減速度(現在位置).first;
        }

        int 出力制動ノッチ(
            距離型 現在位置, 速度型 現在速度, int 現在制動ノッチ,
            const 制動特性 & 制動, 加速度型 勾配影響) const;
        int 出力ノッチ(
            距離型 現在位置, 速度型 現在速度, const 共通状態 & 状態) const;

    };

}
