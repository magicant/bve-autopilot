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
#include <cassert>
#include <iterator>
#include <limits>
#include <numeric>
#include <utility>
#include "共通状態.h"
#include "区間.h"
#include "減速パターン.h"
#include "物理量.h"

#pragma warning(disable:4819)

namespace autopilot
{

    struct 制限グラフ::制限区間
    {
        m 減速目標地点;
        mps 速度;

        void 減速目標地点を再設定(m 新しい減速目標地点);

        減速パターン 目標パターン(mps2 目標減速度) const;
    };

    制限グラフ::制限グラフ() = default;
    制限グラフ::~制限グラフ() = default;

    void 制限グラフ::消去() noexcept
    {
        _区間リスト.clear();
    }

    void 制限グラフ::制限区間追加(m 減速目標地点, m 始点, mps 速度)
    {
        _区間リスト.insert_or_assign(始点, 制限区間{減速目標地点, 速度});
    }

    void 制限グラフ::通過(m 位置)
    {
        if (_区間リスト.empty()) {
            return;
        }

        // 通過済みの区間を消す
        auto i = _区間リスト.begin();
        while (true) {
            auto j = std::next(i);
            if (j == _区間リスト.end() || j->first > 位置) {
                break;
            }
            i = j;
        }
        i = _区間リスト.erase(_区間リスト.begin(), i);
        assert(!_区間リスト.empty());
        assert(i == _区間リスト.begin());
    }

    mps 制限グラフ::制限速度(区間 対象区間) const
    {
        if (対象区間.空である()) {
            return mps::無限大();
        }

        auto i = _区間リスト.upper_bound(対象区間.始点);
        if (i != _区間リスト.begin()) {
            --i;
        }
        auto j = _区間リスト.upper_bound(対象区間.終点);

        return std::accumulate(i, j, mps::無限大(),
            [](mps 制限速度, const std::pair<const m, 制限区間> &p) {
                return std::min(制限速度, p.second.速度);
            });
    }

    mps 制限グラフ::現在常用パターン速度(const 共通状態 &状態) const
    {
        if (!_事前減速) {
            return 制限速度(状態.現在範囲());
        }

        mps パターン速度 = mps::無限大();
        mps 最小目標減速度 = mps::無限大();
        mps2 標準減速度 = 状態.制動().基準最大減速度();

        for (const auto &[位置, 区間] : _区間リスト) {
            if (最小目標減速度 <= 区間.速度) {
                continue; // 意味のない計算は省く
            }
            最小目標減速度 = 区間.速度;

            減速パターン パターン{位置, 区間.速度, 標準減速度};
            パターン速度 =
                std::min(パターン速度, パターン.期待速度(状態.現在位置()));
        }
        return パターン速度;
    }

    自動制御指令 制限グラフ::出力ノッチ(const 共通状態 &状態) const
    {
        自動制御指令 ノッチ = 力行ノッチ{std::numeric_limits<unsigned>::max()};
        mps 最小目標減速度 = mps::無限大();

        for (const auto &[位置, 区間] : _区間リスト) {
            if (最小目標減速度 <= 区間.速度) {
                continue; // 意味のない計算は省く
            }
            最小目標減速度 = 区間.速度;

            mps2 目標減速度;
            if (_事前減速) {
                目標減速度 = 状態.目安減速度();
            }
            else {
                // 目標位置を超えてから減速が始まるようにする。
                目標減速度 = mps2::無限大();
            }
            減速パターン パターン = 区間.目標パターン(目標減速度);
            自動制御指令 パターンノッチ = パターン.出力ノッチ(状態);
            ノッチ = std::min(ノッチ, パターンノッチ);
        }

        return ノッチ;
    }

    void 制限グラフ::制限区間::減速目標地点を再設定(m 新しい減速目標地点)
    {
        減速目標地点 = std::min(減速目標地点, 新しい減速目標地点);
    }

    減速パターン 制限グラフ::制限区間::目標パターン(mps2 目標減速度) const
    {
        constexpr mps 速度マージン = 0.5_kmph;
        mps 目標速度 = std::max(速度 - 速度マージン, 0.0_mps);
        return 減速パターン{減速目標地点, 目標速度, 目標減速度};
    }

}
