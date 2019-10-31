// 制限グラフ.cpp : 区間ごとに定められる制限速度の変化を表します
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
#include "制限グラフ.h"
#include <algorithm>
#include <limits>
#include "共通状態.h"

#pragma warning(disable:4819)

namespace autopilot
{

    void 制限グラフ::消去()
    {
        _区間リスト.clear();
    }

    void 制限グラフ::制限区間追加(m 減速目標地点, m 始点, 速度型 速度)
    {
        // 新しい制限区間に上書きされる区間を消す
        _区間リスト.remove_if([始点](const 制限区間 & 区間) {
            return 区間.始点 >= 始点;
        });

        // 新しい制限区間に重なる既存の区間を縮める
        for (制限区間 & 区間 : _区間リスト) {
            区間.終点 = std::min(区間.終点, 始点);
        }

        m 終点 = m::無限大();
        _区間リスト.emplace_front(減速目標地点, 始点, 終点, 速度);
    }

    void 制限グラフ::通過(m 位置)
    {
        // 通過済みの区間を消す
        _区間リスト.remove_if([位置](const 制限区間 & 区間) {
            return 区間.通過済(位置);
        });
    }

    速度型 制限グラフ::制限速度(区間 対象区間) const
    {
        速度型 制限速度 = std::numeric_limits<速度型>::infinity();
        for (const 制限区間 &区間 : _区間リスト) {
            if (重なる(区間, 対象区間)) {
                制限速度 = std::min(制限速度, 区間.速度);
            }
        }
        return 制限速度;
    }

    速度型 制限グラフ::現在常用パターン速度(const 共通状態 &状態) const
    {
        速度型 速度 = std::numeric_limits<速度型>::infinity();
        加速度型 標準減速度 = 状態.制動().基準最大減速度();

        for (const 制限区間 &区間 : _区間リスト) {
            加速度型 勾配影響 = std::max(状態.進路勾配加速度(区間.始点), 0.0);
            加速度型 目標減速度 = 標準減速度 - 勾配影響;
            減速パターン パターン = 区間.限界パターン(目標減速度);
            速度 = std::min(速度, パターン.期待速度(状態.現在位置()));
        }
        return 速度;
    }

    int 制限グラフ::出力ノッチ(const 共通状態 &状態) const
    {
        int ノッチ = std::numeric_limits<int>::max();

        for (制限区間 区間 : _区間リスト) {
            加速度型 勾配影響 = std::max(状態.進路勾配加速度(区間.始点), 0.0);
            加速度型 目標減速度 = 状態.目安減速度() - 勾配影響;
            減速パターン パターン = 区間.目標パターン(目標減速度);
            int パターンノッチ = パターン.出力ノッチ(状態);
            ノッチ = std::min(ノッチ, パターンノッチ);
        }

        return ノッチ;
    }

}
