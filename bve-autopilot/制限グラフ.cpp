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

        減速パターン 目標パターン(mps2 初期減速度) const;
    };

    制限グラフ::制限グラフ() = default;
    制限グラフ::~制限グラフ() = default;

    void 制限グラフ::消去()
    {
        _区間リスト.clear();
    }

    void 制限グラフ::制限区間追加(m 減速目標地点, m 始点, mps 速度)
    {
        auto i = _区間リスト.lower_bound(始点);

        if (i != _区間リスト.end()) {
            if (速度 == i->second.速度) {
                // 既に同じ制限速度の区間があるなら区間を追加しない
                auto n = _区間リスト.extract(i++);
                assert(始点 <= n.key());
                n.key() = 始点;
                n.mapped().減速目標地点を再設定(減速目標地点);
                _区間リスト.insert(i, std::move(n));
                return;
            }

            if (始点 == i->first) {
                // 既に同じ位置に区間があるなら上書きする
                i->second.減速目標地点 = 減速目標地点;
                i->second.速度 = 速度;
                return;
            }
        }

        if (i != _区間リスト.begin()) {
            auto j = std::prev(i);
            assert(j->first < 始点);
            if (速度 == j->second.速度) {
                // 既に同じ制限速度の区間があるなら区間を追加しない
                j->second.減速目標地点を再設定(減速目標地点);
                return;
            }
        }
        else if (速度 == mps::無限大()) {
            // 制限区間のない位置で制限速度を解除するのは無意味
            return;
        }

        auto j =
            _区間リスト.try_emplace(i, 始点, 制限区間{減速目標地点, 速度});
        assert(std::next(j) == i);
    }

    void 制限グラフ::通過(m 位置)
    {
        // 通過済みの区間を消す
        auto i = _区間リスト.upper_bound(位置);
        if (i == _区間リスト.begin()) {
            return;
        }
        --i;
        _区間リスト.erase(_区間リスト.begin(), i);
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
        auto 速度 = mps::無限大();
        mps2 標準減速度 = 状態.制動().基準最大減速度();

        for (const auto &[位置, 区間] : _区間リスト) {
            mps2 勾配影響 = std::max(状態.進路勾配加速度(位置), 0.0_mps2);
            mps2 目標減速度 = 標準減速度 - 勾配影響;
            減速パターン パターン{位置, 区間.速度, 目標減速度};
            速度 = std::min(速度, パターン.期待速度(状態.現在位置()));
        }
        return 速度;
    }

    自動制御指令 制限グラフ::出力ノッチ(const 共通状態 &状態) const
    {
        自動制御指令 ノッチ = 力行ノッチ{std::numeric_limits<unsigned>::max()};

        for (const auto &[位置, 区間] : _区間リスト) {
            mps2 勾配影響 = std::max(状態.進路勾配加速度(位置), 0.0_mps2);
            mps2 目標減速度 = 状態.目安減速度() - 勾配影響;
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

    減速パターン 制限グラフ::制限区間::目標パターン(mps2 初期減速度) const
    {
        constexpr mps 速度マージン = 0.5_kmph;
        mps 目標速度 = std::max(速度 - 速度マージン, 0.0_mps);
        mps2 最終減速度 = 目標速度 == 0.0_mps ?
            減速パターン::停止最終減速度 : 減速パターン::標準最終減速度;
        return 減速パターン{減速目標地点, 目標速度, 初期減速度, 最終減速度};
    }

}
