// ato.cpp : ATO メインモジュール
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

#include "stdafx.h"
#include "ato.h"

namespace autopilot
{

    void ato::経過(const 共通状態 & 状態)
    {
        _出力ノッチ = 状態.車両仕様().PowerNotches;

        距離型 現在位置 = 状態.現在位置();
        速度型 現在速度 = 状態.現在速度();
        const 制動出力 & 制動パラメーター = 状態.制動パラメーター();
        for (const 制限グラフ & グラフ : 状態.制限グラフ群()) {
            int グラフノッチ = グラフ.出力ノッチ(
                現在位置, 現在速度, 制動パラメーター);
            _出力ノッチ = std::min(_出力ノッチ, グラフノッチ);
        }
    }

}
