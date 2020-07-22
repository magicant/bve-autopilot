// orp.cpp : ORP の照査に抵触しないように減速します
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
#include "orp.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include "共通状態.h"
#include "出力制御.h"
#include "物理量.h"
#include "運動状態.h"

namespace autopilot
{

    namespace
    {

        constexpr 自動制御指令 緩解指令 =
            力行ノッチ{std::numeric_limits<unsigned>::max()};
        constexpr mps 照査速度下限 = 7.5_kmph;
        constexpr mps 最終目標速度 = 照査速度下限 - static_cast<mps>(0.5_kmph);
        constexpr s 上接近時間 = 1.0_s;
        constexpr s 下接近時間 = 2.0_s;

    }

    orp::orp() noexcept :
        _照査パターン{m::無限大(), 0.0_mps, 0.0_mps2},
        _照査速度下限到達{false},
        _出力ノッチ{緩解指令},
        _照査速度{mps::無限大()}
    {
    }

    void orp::リセット() noexcept
    {
        _照査パターン = {m::無限大(), 0.0_mps, 0.0_mps2};
        _照査速度下限到達 = false;
        _出力ノッチ = 緩解指令;
        _照査速度 = mps::無限大();
    }

    void orp::設定(mps 開始照査速度, m 開始位置, m 限界位置)
    {
        _照査パターン = 減速パターン::二点間パターン(
            限界位置, 0.0_mps, 開始位置, 開始照査速度, 勾配グラフ::平坦グラフ);
        _照査速度下限到達 = false;
    }

    void orp::設定(int 地上子値, m 開始位置)
    {
        mps 初速度 = 地上子値 <= 48 ? 25.0_kmph : 35.0_kmph;
        m 残距離 = 地上子値 <= 48 ? 48.0_m : 79.0_m;
        設定(初速度, 開始位置, 開始位置 + 残距離);
    }

    void orp::設定(mps 直前閉塞速度, m 開始位置)
    {
        int 予想地上子値 =
            直前閉塞速度 < static_cast<mps>(34.9_kmph) ? 48 : 79;
        設定(予想地上子値, 開始位置);
    }

    void orp::経過(const 共通状態 &状態)
    {
        if (!制御中()) {
            _出力ノッチ = 緩解指令;
            _照査速度 = mps::無限大();
            return;
        }

        if (状態.現在速度() <= 最終目標速度) {
            _照査速度下限到達 = true;
        }

        mps パターン速度 = _照査パターン.期待速度(
            状態.現在位置(), 勾配グラフ::平坦グラフ);
        _照査速度 = _照査速度下限到達 ?
            std::min(パターン速度, 照査速度下限) :
            パターン速度;

        using namespace std::placeholders;
        _出力ノッチ = 出力制御::出力ノッチ(
            std::bind(&orp::出力制動ノッチ, this, _1, _2), 状態);
    }

    mps2 orp::パターン出力減速度(const 運動状態 &運動状態) const
    {
        constexpr mps マージン変化点上 = 12.0_kmph;
        constexpr mps マージン変化点下 = 6.0_kmph;
        constexpr mps 最大速度マージン = 4.0_kmph;
        constexpr mps 最小速度マージン = 0.0_kmph;
        mps パターン速度 = _照査パターン.期待速度(
            運動状態.位置(), 勾配グラフ::平坦グラフ);
        mps 速度マージン = std::clamp(
            (パターン速度 - マージン変化点下) /
            (マージン変化点上 - マージン変化点下) *
            (最大速度マージン - 最小速度マージン),
            最小速度マージン,
            最大速度マージン);
        mps 期待速度 = パターン速度 - 速度マージン;
        mps2 照査減速度 = _照査パターン.基準減速度();
        // = _照査パターン.期待減速度(運動状態.位置(), 勾配グラフ::平坦グラフ);
        mps2 期待減速度 = 照査減速度 * (期待速度 / パターン速度);
        s 接近時間 = 運動状態.速度() >= 期待速度 ? 上接近時間 : 下接近時間;
        mps2 出力減速度 =
            期待減速度 * (運動状態.速度() / 期待速度) +
            (運動状態.速度() - 期待速度) / 接近時間;
        return 出力減速度;

        // TODO 現在位置が照査開始位置よりだいぶ手前なら照査パターンではなく
        // より高い基準減速度のパターンを使う
    }

    mps2 orp::下限照査出力減速度(const 運動状態 &運動状態) const
    {
        if (!_照査速度下限到達) {
            return 0.0_mps2;
        }
        return (運動状態.速度() - 最終目標速度) / 上接近時間;
    }

    自動制動自然数ノッチ orp::出力制動ノッチ(
        const 運動状態 &運動状態, const 共通状態 &状態) const
    {
        mps2 平坦時減速度1 = パターン出力減速度(運動状態);
        mps2 平坦時減速度2 = 下限照査出力減速度(運動状態);
        mps2 平坦時減速度 = std::max(平坦時減速度1, 平坦時減速度2);
        mps2 勾配加速度 = 状態.勾配().列車勾配加速度(運動状態.位置());
        mps2 補正減速度 = 平坦時減速度 + 勾配加速度;
        自動制動実数ノッチ ノッチ = 状態.制動().自動ノッチ(補正減速度);
        return std::min(ノッチ.ceil(), 状態.制動().自動最大ノッチ());
    }

 }
