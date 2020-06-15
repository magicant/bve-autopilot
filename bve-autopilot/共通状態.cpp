// 共通状態.cpp : プラグイン全体で使用する、ゲーム全体の状態量です
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
#include "共通状態.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "物理量.h"

#pragma warning(disable:4819)

namespace autopilot {

    namespace
    {

        constexpr s 正午 = static_cast<s>(12 * 60 * 60);
        constexpr s 一日 = static_cast<s>(24 * 60 * 60);

        /// 「停車場へ移動」でワープする間に通過する地上子の位置は信頼
        /// できないので、地上子が示す位置は 0 メートル地点と仮定する
        constexpr bool 信頼できる(区間 範囲) noexcept {
            return 範囲.始点 != 0.0_m || 範囲.終点 == 0.0_m;
        }

    }

    void 共通状態::リセット() noexcept
    {
        // _設定.リセット(); // ファイルから読み込むのでリセットしない
        _互換モード = 互換モード型::無効;
        _状態 = ATS_VEHICLESTATE{};
        _目安減速度 = 0.8 * _設定.常用最大減速度();
        _自動発進待ち時間 = s::quiet_NaN();
        _自動発進時刻 = s::quiet_NaN();
        _押しているキー.reset();
        _加速度計.リセット();
        _勾配グラフ.消去();
    }

    void 共通状態::車両仕様設定(const ATS_VEHICLESPEC & 仕様)
    {
        _車両仕様 = 仕様;
        _制動特性.性能設定(
            手動制動自然数ノッチ{static_cast<unsigned>(仕様.BrakeNotches)},
            _設定.制動最大拡張ノッチ(),
            _設定.常用最大減速度(),
            _設定.制動反応時間(),
            _設定.pressure_rates());
        _勾配グラフ.列車長を設定(列車長());
    }

    void 共通状態::地上子通過(const ATS_BEACONDATA &地上子, m 直前位置)
    {
        switch (地上子.Type)
        {
        case 1001: // 互換モード設定
            _互換モード = static_cast<互換モード型>(地上子.Optional);
            break;
        case 1002: // 目標減速度設定
            if (地上子.Optional > 0) {
                _目安減速度 = std::min(
                    static_cast<mps2>(static_cast<kmphps>(
                        0.1 * 地上子.Optional)),
                    0.95 * _設定.常用最大減速度());
            }
            break;
        case 1003: // 自動発進待ち時間設定
            if (地上子.Optional >= 0) {
                _自動発進待ち時間 = static_cast<s>(地上子.Optional * 0.1);
            }
            else {
                _自動発進待ち時間 = s::quiet_NaN();
            }
            break;
        case 1008: // 勾配設定
            勾配追加(地上子.Optional, 直前位置);
            break;
        }
    }

    void 共通状態::経過(const ATS_VEHICLESTATE & 状態)
    {
        _状態 = 状態;
        _加速度計.経過({ 現在速度(), 現在時刻() });
        _勾配グラフ.通過(現在位置());
        _制動特性.経過(*this);
    }

    void 共通状態::出力(const ATS_HANDLES & 出力) noexcept
    {
        _前回出力 = 出力;
    }

    void 共通状態::戸閉(bool 戸閉) noexcept
    {
        _戸閉 = 戸閉;

        if (戸閉) {
            _自動発進時刻 = 現在時刻() + _自動発進待ち時間;
            // _自動発進待ち時間 が NaN なら _自動発進時刻 も NaN
        }
        else {
            _自動発進時刻 = s::quiet_NaN();
        }
    }

    void 共通状態::逆転器操作(int ノッチ) noexcept
    {
        _入力逆転器ノッチ = ノッチ;
    }

    void 共通状態::力行操作(int ノッチ) noexcept
    {
        _入力力行ノッチ = ノッチ;
    }

    void 共通状態::制動操作(int ノッチ) noexcept
    {
        _入力制動ノッチ = 手動制動自然数ノッチ{static_cast<unsigned>(ノッチ)};
    }

    bool 共通状態::自動発進可能な時刻である() const
    {
        if (!isfinite(_自動発進時刻)) {
            return false;
        }

        s 現在 = 現在時刻();
        if (現在 < 正午 && 一日 <= _自動発進時刻) {
            // 日をまたぐと時刻が戻るので補正する
            現在 += 一日;
        }

        return 現在 >= _自動発進時刻;
    }

    自動制動自然数ノッチ 共通状態::転動防止自動ノッチ() const
    {
        制動力割合 割合 = _設定.転動防止制動割合();
        自動制動実数ノッチ ノッチ = _制動特性.自動ノッチ(割合);
        自動制動自然数ノッチ ノッチi{
            static_cast<unsigned>(std::ceil(ノッチ.value))};
        return std::min(ノッチi, _制動特性.自動最大ノッチ());
    }

    mps2 共通状態::車両勾配加速度() const
    {
        return _勾配グラフ.列車勾配加速度(現在位置());
    }

    void 共通状態::勾配追加(int 地上子値, m 直前位置)
    {
        if (地上子値 < -std::numeric_limits<int>::max()) {
            return; // std::abs でのオーバーフローを防止
        }

        bool 下り = 地上子値 < 0;
        地上子値 = std::abs(地上子値);

        区間 地上子のある範囲{直前位置, 現在位置()};
        m 距離 = static_cast<m>(地上子値 / 1000);
        m 勾配変化地点 = 信頼できる(地上子のある範囲) ?
            地上子のある範囲.中点() + 距離 : 0.0_m;
        double 勾配 = (地上子値 % 1000) * 0.001;
        if (下り) {
            勾配 = -勾配;
        }
        _勾配グラフ.勾配区間追加(勾配変化地点, 勾配);
    }

}
