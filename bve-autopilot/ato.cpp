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
#include <algorithm>
#include <cmath>
#include <limits>
#include "tasc.h"
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        void 制限区間追加(
            制限グラフ &グラフ, int 地上子値, const 共通状態 &状態)
        {
            距離型 距離 = 地上子値 / 1000;
            速度型 速度 = mps_from_kmph(地上子値 % 1000);
            距離型 位置 = 状態.現在位置() + 距離;

            if (速度 == 0) {
                速度 = std::numeric_limits<速度型>::infinity();
            }

            グラフ.制限区間追加(位置, 速度);
        }

    }

    ato::ato() = default;
    ato::~ato() = default;

    void ato::リセット()
    {
        _制限速度1006.消去();
        _制限速度1007.消去();
        _信号.リセット();
        _照査.リセット();
        _急動作抑制.リセット();
    }

    void ato::発進(const 共通状態 &状態)
    {
        _発進中 = true;
        _信号.発進();
        _照査.発進(状態, _信号);
    }

    void ato::信号現示変化(信号インデックス 指示, const 共通状態 &状態) {
        _信号.信号現示変化(指示);
        _照査.信号現示変化(状態, _信号);
    }

    void ato::地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態)
    {
        switch (地上子.Type)
        {
        case 1006: // 制限速度設定
            制限区間追加(_制限速度1006, 地上子.Optional, 状態);
            break;
        case 1007: // 制限速度設定
            制限区間追加(_制限速度1007, 地上子.Optional, 状態);
            break;
        }
        _信号.地上子通過(地上子, 状態);
        _照査.地上子通過(地上子, 状態, _信号);
    }

    void ato::経過(const 共通状態 &状態, const tasc &tasc)
    {
        距離型 最後尾 = 状態.現在位置() - 状態.列車長();
        _制限速度1006.通過(最後尾);
        _制限速度1007.通過(最後尾);
        _信号.経過(状態);
        _照査.経過(状態);

        if (状態.現在速度() > mps_from_kmph(1) ||
            状態.逆転器ノッチ() <= 0 ||
            状態.制動ノッチ() > 0 ||
            !状態.戸閉())
        {
            _発進中 = false;
        }

        int 出力ノッチ;
        if (!_発進中 && 状態.現在速度() <= mps_from_kmph(0.05)) {
            // 停車中は制動し続ける
            出力ノッチ = -状態.車両仕様().BrakeNotches;
        }
        else {
            出力ノッチ = std::min({
                状態.車両仕様().PowerNotches,
                _制限速度1006.出力ノッチ(状態),
                _制限速度1007.出力ノッチ(状態),
                _信号.出力ノッチ(状態, tasc, _照査),
                _照査.出力ノッチ(状態),
                });
        }
        _急動作抑制.経過(出力ノッチ, 状態, _信号.is_atc());
    }

    速度型 ato::現在制限速度(const 共通状態 &状態) const
    {
        区間 列車範囲 = 状態.現在範囲();
        return std::min({
            _制限速度1006.制限速度(列車範囲),
            _制限速度1007.制限速度(列車範囲),
            _信号.現在制限速度(状態),
            _照査.現在制限速度(列車範囲),
            });
    }

    速度型 ato::現在常用パターン速度(const 共通状態 &状態) const
    {
        return std::min({
            _制限速度1006.現在常用パターン速度(状態),
            _制限速度1007.現在常用パターン速度(状態),
            _信号.現在常用パターン速度(状態),
            _照査.現在常用パターン速度(状態),
            });
    }

}
