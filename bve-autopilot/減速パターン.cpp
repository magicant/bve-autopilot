// 減速パターン.cpp : 等加速度で減速するパターンを表します
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
#include "減速パターン.h"
#include <algorithm>
#include <limits>
#include "走行モデル.h"

namespace autopilot
{

    速度型 減速パターン::期待速度(距離型 現在位置) const
    {
        走行モデル 走行(目標位置, 目標速度, 0);
        走行.指定位置まで走行(現在位置, -目標減速度);
        return 走行.速度();
    }

    int 減速パターン::出力制動ノッチ(
        距離型 現在位置, 速度型 現在速度, const 制動出力 & 制動) const
    {
        速度型 期待速度 = this->期待速度(現在位置);
        距離型 残距離 = 目標位置 - 現在位置;

        int 出力制動ノッチ;

        if (残距離 >= 0) {
            加速度型 出力減速度 = std::min(
                // 単純な等加速度運動で減速するパターン
                -走行モデル::距離と速度による加速度(残距離, 現在速度, 目標速度),
                // 減速パターンに近付いたら目標減速度に漸次的に近付ける
                目標減速度 * (現在速度 / 期待速度) - (期待速度 - 現在速度) / 2);

            double 出力制動ノッチ相当 = 制動.ノッチ(出力減速度);
            if (出力減速度 < 目標減速度) {
                出力制動ノッチ相当 = std::floor(出力制動ノッチ相当);
            }
            else {
                出力制動ノッチ相当 = std::round(出力制動ノッチ相当);
            }
            出力制動ノッチ = static_cast<int>(出力制動ノッチ相当);
        }
        else {
            // 過走した!
            出力制動ノッチ = 制動.常用ノッチ数();
        }

        // 目標速度に近付いたらノッチを緩めて衝撃を抑える
        double 緩めノッチ = std::ceil(制動.ノッチ((現在速度 - 目標速度) / 2.0));
        出力制動ノッチ = std::min(出力制動ノッチ, static_cast<int>(緩めノッチ));

        if (出力制動ノッチ <= 制動.無効ノッチ数()) {
            出力制動ノッチ = 0;
        }
        else if (出力制動ノッチ > 制動.常用ノッチ数()) {
            出力制動ノッチ = 制動.常用ノッチ数();
        }

        return 出力制動ノッチ;
    }

    int 減速パターン::出力ノッチ(
        距離型 現在位置, 速度型 現在速度, const 制動出力 & 制動) const
    {
        int 出力制動ノッチ = this->出力制動ノッチ(現在位置, 現在速度, 制動);
        if (出力制動ノッチ > 0) {
            return -出力制動ノッチ;
        }

        // もうすぐブレーキをかけ始める必要があるなら力行をやめさせる
        走行モデル モデル(現在位置, 現在速度, 0);
        モデル.指定時間走行(5);
        出力制動ノッチ = this->出力制動ノッチ(モデル.位置(), モデル.速度(), 制動);
        if (出力制動ノッチ > 0) {
            return 0;
        }
        return std::numeric_limits<int>::max();
    }

}
