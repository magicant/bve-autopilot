// 共通状態.h : プラグイン全体で使用する、ゲーム全体の状態量です
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

#pragma once
#include "力行特性.h"
#include "制動特性.h"
#include "制御指令.h"
#include "加速度計.h"
#include "勾配グラフ.h"
#include "区間.h"
#include "環境設定.h"
#include "物理量.h"
#include "運動状態.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot {

    enum class 互換モード型
    {
        無効,
        汎用ats,
        メトロ総合,
        swp2,
    };

    class 共通状態
    {
    public:
        void リセット() noexcept;
        void 設定ファイル読込(LPCWSTR 設定ファイル名) {
            _設定.ファイル読込(設定ファイル名);
        }
        void 車両仕様設定(const ATS_VEHICLESPEC & 仕様);
        void 地上子通過(const ATS_BEACONDATA &地上子, m 直前位置);
        void 経過(const ATS_VEHICLESTATE & 状態);
        void 出力(const ATS_HANDLES & 出力) noexcept;
        void 戸閉(bool 戸閉) noexcept;
        void 逆転器操作(int ノッチ) noexcept;
        void 力行操作(int ノッチ) noexcept;
        void 制動操作(int ノッチ) noexcept;
        void キー押し(int キー) {
            _押しているキー[キー] = true;
        }
        void キー放し(int キー) {
            _押しているキー[キー] = false;
        }

        環境設定 &設定() noexcept { return _設定; }
        const 環境設定 &設定() const noexcept { return _設定; }
        互換モード型 互換モード() const noexcept { return _互換モード; }
        const ATS_VEHICLESPEC & 車両仕様() const noexcept { return _車両仕様; }
        力行ノッチ 最大力行ノッチ() const noexcept {
            return 力行ノッチ{static_cast<unsigned>(_車両仕様.PowerNotches)};
        }
        m 列車長() const noexcept {
            return _設定.車両長() * static_cast<double>(_車両仕様.Cars);
        }
        m 現在位置() const noexcept { return static_cast<m>(_状態.Location); }
        区間 現在範囲() const noexcept {
            return 区間{現在位置() - 列車長(), 現在位置()};
        }
        時刻 現在時刻() const noexcept {
            return static_cast<時刻>(static_cast<ms>(_状態.Time));
        }
        mps 現在速度() const noexcept {
            return static_cast<kmph>(_状態.Speed);
        }
        bool 停車中() const noexcept {
            return static_cast<kmph>(_状態.Speed) < 0.05_kmph;
        }
        運動状態 現在走行状態() const noexcept {
            return 運動状態{ 現在位置(), 現在速度(), 現在時刻() };
        }
        float 現在電流() const noexcept { return _状態.Current; }
        float 現在ブレーキシリンダー圧() const noexcept {
            return _状態.BcPressure;
        }
        mps2 目安減速度() const noexcept { return _目安減速度; }
        bool 戸閉() const noexcept { return _戸閉; }
        bool 自動発進可能な時刻である() const;
        int 入力逆転器ノッチ() const noexcept { return _入力逆転器ノッチ; }
        /// 抑速ノッチでは値は負になる
        int 入力力行ノッチ() const noexcept { return _入力力行ノッチ; }
        手動制動自然数ノッチ 入力制動ノッチ() const noexcept {
            return _入力制動ノッチ;
        }
        mps2 加速度() const noexcept { return _加速度計.加速度(); }
        const 力行特性 &力行() const noexcept { return _力行特性; }
        const 制動特性 &制動() const noexcept { return _制動特性; }
        自動制動自然数ノッチ 転動防止自動ノッチ() const;
        const 勾配グラフ &勾配() const noexcept { return _勾配グラフ; }
        mps2 車両勾配加速度() const;
        int 前回逆転器ノッチ() const noexcept { return _前回出力.Reverser; }
        /// 抑速ノッチでは値は負になる
        int 前回力行ノッチ() const noexcept { return _前回出力.Power; }
        制動指令 前回制動指令() const noexcept {
            return 制動指令{_前回出力.Brake};
        }
        bool 力行をやめた直後である() const noexcept;
        キー組合せ 押しているキー() const noexcept { return _押しているキー; }

    private:
        環境設定 _設定;
        互換モード型 _互換モード = 互換モード型::無効;
        ATS_VEHICLESPEC _車両仕様 = {};
        ATS_VEHICLESTATE _状態 = {};
        mps2 _目安減速度 = {};
        bool _戸閉 = false;
        s _自動発進待ち時間 = {}; // 地上子から設定された毎回の待ち時間
        時刻 _自動発進時刻 = {}; // 現在の駅を発車する時刻
        int _入力逆転器ノッチ = 0, _入力力行ノッチ = 0;
        手動制動自然数ノッチ _入力制動ノッチ;
        キー組合せ _押しているキー;
        加速度計 _加速度計;
        力行特性 _力行特性;
        制動特性 _制動特性;
        勾配グラフ _勾配グラフ;
        ATS_HANDLES _前回出力 = {};
        時刻 _力行をやめた時刻 = static_cast<時刻>(-s::無限大());

        void 勾配追加(int 地上子値, m 直前位置);
    };

}

#pragma warning(pop)
