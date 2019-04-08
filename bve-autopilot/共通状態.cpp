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
#include <algorithm>
#include <limits>
#include "共通状態.h"
#include "単位.h"

#pragma warning(disable:4819)

namespace autopilot {

    void 共通状態::リセット()
    {
        // _設定.リセット(); // ファイルから読み込むのでリセットしない
        _状態 = ATS_VEHICLESTATE{};
        for (制限グラフ & グラフ : _制限グラフ群) {
            グラフ.消去();
        }
        _加速度計.リセット();
    }

    void 共通状態::車両仕様設定(const ATS_VEHICLESPEC & 仕様)
    {
        _車両仕様 = 仕様;
        _制動特性.性能設定(
            仕様.BrakeNotches,
            std::max(std::min(仕様.AtsNotch, 仕様.BrakeNotches) - 1, 0),
            _設定.常用最大減速度(),
            _設定.制動緩解時間());
    }

    void 共通状態::地上子通過(const ATS_BEACONDATA & 地上子)
    {
        switch (地上子.Type)
        {
        case 1006: // 制限速度設定
            制限区間追加(制限グラフ群添字::汎用1006, 地上子.Optional);
            break;
        case 1007: // 制限速度設定
            制限区間追加(制限グラフ群添字::汎用1007, 地上子.Optional);
            break;
        }
    }

    void 共通状態::経過(const ATS_VEHICLESTATE & 状態)
    {
        double 時刻 = s_from_ms(状態.Time);
        _状態 = 状態;

        距離型 最後尾 = 状態.Location - 列車長();
        for (制限グラフ & グラフ : _制限グラフ群) {
            グラフ.通過(最後尾);
        }

        _加速度計.経過({ mps_from_kmph(状態.Speed), 時刻 });
    }

    void 共通状態::出力(const ATS_HANDLES & 出力)
    {
        _前回出力 = 出力;
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

    速度型 共通状態::現在制限速度() const
    {
        速度型 制限速度 = std::numeric_limits<速度型>::infinity();
        for (const 制限グラフ & グラフ : _制限グラフ群) {
            制限速度 = std::min(制限速度, グラフ.制限速度(_状態.Location));
        }
        return 制限速度;
    }

    void 共通状態::制限区間追加(制限グラフ群添字 添字, int 地上子値)
    {
        制限グラフ & グラフ = _制限グラフ群[static_cast<std::size_t>(添字)];
        距離型 距離 = 地上子値 / 1000;
        速度型 速度 = mps_from_kmph(地上子値 % 1000);
        距離型 位置 = _状態.Location + 距離;

        if (速度 == 0) {
            速度 = std::numeric_limits<速度型>::infinity();
        }

        グラフ.制限区間追加(位置, 速度);
    }

}
