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

    void 共通状態::リセット()
    {
        // _設定.リセット(); // ファイルから読み込むのでリセットしない
        _互換モード = 互換モード型::無効;
        _状態 = ATS_VEHICLESTATE{};
        _目安減速度 = 0.8 * _設定.常用最大減速度();
        _加速度計.リセット();
        _勾配グラフ.消去();
    }

    void 共通状態::車両仕様設定(const ATS_VEHICLESPEC & 仕様)
    {
        _車両仕様 = 仕様;
        std::vector<制動力割合> pr; // FIXME
        for (double p : _設定.pressure_rates()) {
            pr.emplace_back(p);
        }
        _制動特性.性能設定(
            手動制動自然数ノッチ{static_cast<unsigned>(仕様.BrakeNotches)},
            // FIXME Remove unnecessary cast
            自動制動自然数ノッチ{
                static_cast<unsigned>(_設定.制動拡張ノッチ数())},
            _設定.常用最大減速度(),
            _設定.制動反応時間(),
            pr);
    }

    void 共通状態::地上子通過(const ATS_BEACONDATA & 地上子)
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
        case 1008: // 勾配設定
            勾配追加(地上子.Optional);
            break;
        }
    }

    void 共通状態::経過(const ATS_VEHICLESTATE & 状態)
    {
        _状態 = 状態;
        _加速度計.経過({ 現在速度(), 現在時刻() });
        _勾配グラフ.通過(現在位置() - 列車長());
        _制動特性.経過(*this);
    }

    void 共通状態::出力(const ATS_HANDLES & 出力)
    {
        _前回出力 = 出力;
    }

    void 共通状態::戸閉(bool 戸閉)
    {
        _戸閉 = 戸閉;
    }

    void 共通状態::逆転器操作(int ノッチ)
    {
        _逆転器ノッチ = ノッチ;
    }

    void 共通状態::力行操作(int ノッチ)
    {
        _力行ノッチ = ノッチ;
    }

    void 共通状態::制動操作(int ノッチ)
    {
        _制動ノッチ = ノッチ;
    }

    区間 共通状態::現在範囲() const
    {
        return 区間{ 現在位置() - 列車長(), 現在位置() };
    }

    int 共通状態::転動防止自動ノッチ() const
    {
        制動力割合 割合{_設定.転動防止制動割合()}; // FIXME assign
        自動制動実数ノッチ ノッチ = _制動特性.自動ノッチ(割合);
        int ノッチi = static_cast<int>(std::ceil(ノッチ.value));
        return std::min(
            ノッチi, static_cast<int>(_制動特性.自動最大ノッチ().value));
    }

    mps2 共通状態::進路勾配加速度(m 目標位置) const
    {
        区間 進路 = 現在範囲();
        進路.終点 = std::max(進路.終点, 目標位置);
        return _勾配グラフ.勾配加速度(進路);
    }

    mps2 共通状態::車両勾配加速度() const
    {
        return _勾配グラフ.勾配加速度(現在範囲());
    }

    void 共通状態::勾配追加(int 地上子値)
    {
        if (地上子値 < -std::numeric_limits<int>::max()) {
            return; // std::abs でのオーバーフローを防止
        }

        bool 下り = 地上子値 < 0;
        地上子値 = std::abs(地上子値);

        m 距離 = static_cast<m>(地上子値 / 1000);
        double 勾配 = (地上子値 % 1000) * 0.001;
        if (下り) {
            勾配 = -勾配;
        }
        _勾配グラフ.勾配区間追加(現在位置() + 距離, 勾配);
    }

}
