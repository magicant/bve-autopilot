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
#include "制動特性.h"
#include "制限区間.h"
#include "加速度計.h"
#include "勾配グラフ.h"
#include "区間.h"
#include "環境設定.h"
#include "物理量.h"
#include "走行モデル.h"

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
        void リセット();
        void 設定ファイル読込(LPCWSTR 設定ファイル名) {
            _設定.ファイル読込(設定ファイル名);
        }
        void 車両仕様設定(const ATS_VEHICLESPEC & 仕様);
        void 地上子通過(const ATS_BEACONDATA & 地上子);
        void 経過(const ATS_VEHICLESTATE & 状態);
        void 出力(const ATS_HANDLES & 出力);
        void 戸閉(bool 戸閉);
        void 逆転器操作(int ノッチ);
        void 力行操作(int ノッチ);
        void 制動操作(int ノッチ);

        const 環境設定 & 設定() const { return _設定; }
        互換モード型 互換モード() const { return _互換モード; }
        const ATS_VEHICLESPEC & 車両仕様() const { return _車両仕様; }
        m 列車長() const {
            return _設定.車両長() * static_cast<double>(_車両仕様.Cars);
        }
        m 現在位置() const { return static_cast<m>(_状態.Location); }
        区間 現在範囲() const;
        s 現在時刻() const { return static_cast<ms>(_状態.Time); }
        mps 現在速度() const { return static_cast<kmph>(_状態.Speed); }
        bool 停車中() const {
            return static_cast<kmph>(_状態.Speed) < 0.05_kmph;
        }
        走行モデル 現在走行状態() const {
            return 走行モデル{ 現在位置(), 現在速度(), 現在時刻() };
        }
        float 現在電流() const { return _状態.Current; }
        float 現在ブレーキシリンダー圧() const { return _状態.BcPressure; }
        mps2 目安減速度() const { return _目安減速度; }
        bool 戸閉() const { return _戸閉; }
        int 逆転器ノッチ() const { return _逆転器ノッチ; }
        int 力行ノッチ() const { return _力行ノッチ; }
        int 制動ノッチ() const { return _制動ノッチ; }
        mps2 加速度() const { return _加速度計.加速度(); }
        const 制動特性 & 制動() const { return _制動特性; }
        int 転動防止自動ノッチ() const;
        const 勾配グラフ &勾配() const { return _勾配グラフ; }
        mps2 進路勾配加速度(m 目標位置) const;
        mps2 車両勾配加速度() const;
        const ATS_HANDLES & 前回出力() const { return _前回出力; }

    private:
        環境設定 _設定;
        互換モード型 _互換モード = 互換モード型::無効;
        ATS_VEHICLESPEC _車両仕様 = {};
        ATS_VEHICLESTATE _状態 = {};
        mps2 _目安減速度 = {};
        bool _戸閉 = false;
        int _逆転器ノッチ = 0, _力行ノッチ = 0, _制動ノッチ = 0;
        加速度計 _加速度計;
        制動特性 _制動特性;
        勾配グラフ _勾配グラフ;
        ATS_HANDLES _前回出力 = {};

        void 勾配追加(int 地上子値);
    };

}

#pragma warning(pop)
