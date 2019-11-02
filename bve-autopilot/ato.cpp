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
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        bool 発進可能(const 共通状態 &状態)
        {
            return 状態.逆転器ノッチ() > 0 &&
                状態.制動ノッチ() <= 0 &&
                状態.戸閉();
        }

        void 制限区間追加(
            制限グラフ &グラフ, int 地上子値, const 共通状態 &状態,
            mps 速度マージン = 0.0_mps)
        {
            m 距離 = static_cast<m>(地上子値 / 1000);
            mps 速度 = static_cast<kmph>(地上子値 % 1000);
            m 始点 = 状態.現在位置() + 距離;

            if (速度 == 0.0_mps) {
                速度 = mps::無限大();
            }
            速度 -= 速度マージン;

            if (速度 > 0.0_mps) {
                m 減速目標地点 = 始点 - 1.0_s * 速度;
                グラフ.制限区間追加(減速目標地点, 始点, 速度);
            }
        }

        void 制限区間終了(制限グラフ &グラフ, m 終了位置)
        {
            グラフ.制限区間追加(終了位置, 終了位置, mps::無限大());
        }

    }

    ato::ato() = default;
    ato::~ato() = default;

    void ato::リセット()
    {
        _制限速度1006.消去();
        _制限速度1007.消去();
        _制限速度6.消去();
        _制限速度8.消去();
        _制限速度9.消去();
        _制限速度10.消去();
        _信号.リセット();
        _orp.リセット();
        _急動作抑制.リセット();
    }

    void ato::発進(const 共通状態 &状態)
    {
        if (発進可能(状態)) {
            _制御状態 = 制御状態::発進;
            _信号.発進();
        }
    }

    void ato::信号現示変化(信号インデックス 指示) {
        _信号.信号現示変化(指示);
        _orp.信号現示変化(指示);
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

        if (状態.互換モード() == 互換モード型::swp2) {
            switch (地上子.Type) {
            case 6: // 制限速度設定
                制限区間追加(
                    _制限速度6, 地上子.Optional, 状態, 10.0_kmph);
                break;
            case 8: // 制限速度設定
                制限区間追加(
                    _制限速度8, 地上子.Optional, 状態, 10.0_kmph);
                break;
            case 9: // 制限速度設定
                制限区間追加(
                    _制限速度9, 地上子.Optional, 状態, 10.0_kmph);
                break;
            case 10: // 制限速度設定
                制限区間追加(
                    _制限速度10, 地上子.Optional, 状態, 10.0_kmph);
                break;
            case 16: // 制限速度解除
                制限区間終了(_制限速度6, 状態.現在位置());
                break;
            case 18: // 制限速度解除
                制限区間終了(_制限速度8, 状態.現在位置());
                break;
            case 19: // 制限速度解除
                制限区間終了(_制限速度9, 状態.現在位置());
                break;
            case 20: // 制限速度解除
                制限区間終了(_制限速度10, 状態.現在位置());
                break;
            }
        }

        _信号.地上子通過(地上子, 状態);
        _orp.地上子通過(地上子, 状態, _信号);
    }

    void ato::経過(const 共通状態 &状態)
    {
        m 最後尾 = 状態.現在位置() - 状態.列車長();
        _制限速度1006.通過(最後尾);
        _制限速度1007.通過(最後尾);
        _制限速度6.通過(最後尾);
        _制限速度8.通過(最後尾);
        _制限速度9.通過(最後尾);
        _制限速度10.通過(最後尾);
        _信号.経過(状態);
        _orp.経過(状態);

        if (_制御状態 == 制御状態::発進) {
            if (状態.現在速度() > 0.0_mps && 状態.加速度() < 0.0_mps2 ||
                !発進可能(状態))
            {
                _制御状態 = 制御状態::走行;
            }
        }
        if (_制御状態 == 制御状態::走行) {
            if (状態.停車中()) {
                _制御状態 = 制御状態::停止;
            }
        }

        if (_制御状態 == 制御状態::停止) {
            _出力ノッチ = -状態.転動防止自動ノッチ();
        }
        else {
            _急動作抑制.経過(_信号.出力ノッチ(状態), 状態, _信号.is_atc());
            _出力ノッチ = std::min({
                状態.車両仕様().PowerNotches,
                _制限速度1006.出力ノッチ(状態),
                _制限速度1007.出力ノッチ(状態),
                _制限速度6.出力ノッチ(状態),
                _制限速度8.出力ノッチ(状態),
                _制限速度9.出力ノッチ(状態),
                _制限速度10.出力ノッチ(状態),
                _orp.出力ノッチ(),
                _急動作抑制.出力ノッチ(),
                });
        }
    }

    mps ato::現在制限速度(const 共通状態 &状態) const
    {
        区間 列車範囲 = 状態.現在範囲();
        mps 速度 = std::min({
            _制限速度1006.制限速度(列車範囲),
            _制限速度1007.制限速度(列車範囲),
            _制限速度6.制限速度(列車範囲),
            _制限速度8.制限速度(列車範囲),
            _制限速度9.制限速度(列車範囲),
            _制限速度10.制限速度(列車範囲),
            _信号.現在制限速度(状態),
            });

        if (_orp.照査中() && _orp.照査速度() <= 速度) {
            return mps::無限大();
        }
        return 速度;
    }

    mps ato::現在常用パターン速度(const 共通状態 &状態) const
    {
        mps 速度 = std::min({
            _制限速度1006.現在常用パターン速度(状態),
            _制限速度1007.現在常用パターン速度(状態),
            _制限速度6.現在常用パターン速度(状態),
            _制限速度8.現在常用パターン速度(状態),
            _制限速度9.現在常用パターン速度(状態),
            _制限速度10.現在常用パターン速度(状態),
            _信号.現在常用パターン速度(状態),
            });

        if (_orp.照査中()) {
            速度 = std::min(速度, _orp.照査速度());
        }
        return 速度;
    }

    mps ato::現在orp照査速度() const
    {
        if (!_orp.照査中()) {
            return mps::無限大();
        }

        return _orp.照査速度();
    }

}
