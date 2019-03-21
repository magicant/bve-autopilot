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
#include "制動出力.h"
#include "単位.h"
#include "走行モデル.h"

namespace autopilot
{

    class 共通状態;

    struct 減速パターン
    {
        距離型 目標位置;
        速度型 目標速度;
        加速度型 目標減速度;

        constexpr 減速パターン(
            距離型 目標位置, 速度型 目標速度, 加速度型 目標減速度) :
            目標位置(目標位置), 目標速度(目標速度), 目標減速度(目標減速度) {}

        ~減速パターン() = default;

        速度型 期待速度(距離型 現在位置) const;
        int 出力制動ノッチ(
            距離型 現在位置, 速度型 現在速度, const 制動出力 & 制動) const;
        int 出力ノッチ(
            距離型 現在位置, 速度型 現在速度, const 共通状態 & 状態) const;

    };

}
