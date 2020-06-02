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
#include <cassert>
#include <cmath>
#include <limits>
#include "共通状態.h"
#include "物理量.h"
#include "走行モデル.h"

namespace autopilot
{

    namespace
    {

        constexpr 自動制御指令 緩解指令 =
            力行ノッチ{std::numeric_limits<unsigned>::max()};
        constexpr mps 照査速度下限 = 7.5_kmph;
        constexpr mps 最終目標速度 = 照査速度下限 - static_cast<mps>(0.5_kmph);
        constexpr mps 運転速度マージン = 4.0_kmph;

    }

    orp::orp() noexcept :
        _照査パターン{m::無限大(), 0.0_mps, 0.0_mps2},
        _出力ノッチ{緩解指令},
        _照査速度{mps::無限大()}
    {
    }

    void orp::リセット() noexcept
    {
        _照査パターン = {m::無限大(), 0.0_mps, 0.0_mps2};
        _出力ノッチ = 緩解指令;
    }

    void orp::設定(mps 初期照査速度, m 初期位置, m 限界位置) noexcept
    {
        /* TODO
        mps2 照査減速度 = -走行モデル::距離と速度による加速度(
            限界位置 - 初期位置, 初期照査速度, 0.0_mps);
        走行モデル 照査{初期位置, 初期照査速度};
        照査.指定速度まで走行(照査速度下限, -照査減速度);
        _照査パターン.目標位置 = 照査.位置();
        _照査パターン.初期減速度 = 照査減速度;
        _照査パターン.最終減速度 = 照査減速度;
        assert(_照査パターン.目標速度 == 照査速度下限);

        照査.指定速度まで走行(
            最終目標速度 + 運転速度マージン / 2.0, -照査減速度);

        m 減速終了位置 = 照査.位置();
        mps 初期運転速度 = 初期照査速度 - 運転速度マージン;
        mps2 運転減速度 = -走行モデル::距離と速度による加速度(
            減速終了位置 - 初期位置, 初期運転速度, 最終目標速度);
        _運転パターン.目標位置 = 減速終了位置;
        _運転パターン.初期減速度 = 運転減速度;
        _運転パターン.最終減速度 = 運転減速度 / 2.0;
        assert(_運転パターン.目標速度 == 最終目標速度);
        assert(_運転パターン.素早い速度超過回復);
        */
    }

    void orp::設定(int 地上子値, m 初期位置) noexcept
    {
        mps 初速度 = 地上子値 <= 48 ? 25.0_kmph : 35.0_kmph;
        m 残距離 = 地上子値 <= 48 ? 48.0_m : 79.0_m;
        設定(初速度, 初期位置, 初期位置 + 残距離);
    }

    void orp::設定(mps 直前閉塞速度, m 初期位置) noexcept
    {
        int 予想地上子値 =
            直前閉塞速度 < static_cast<mps>(34.9_kmph) ? 48 : 79;
        設定(予想地上子値, 初期位置);
    }

    void orp::経過(const 共通状態 &状態)
    {
        /* TODO
        if (制御中() && 状態.現在速度() < 最終目標速度) {
            // パターンを照査速度下限に固定する
            _照査パターン.目標位置 = _運転パターン.目標位置 = -m::無限大();
        }

        _出力ノッチ = 制御中() ? _運転パターン.出力ノッチ(状態) : 緩解指令;
        _照査速度 = _照査パターン.期待速度(状態.現在位置());
        */
    }

 }
