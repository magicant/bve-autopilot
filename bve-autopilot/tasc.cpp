// tasc.cpp : TASC メインモジュール
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
#include "tasc.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "単位.h"
#include "減速パターン.h"

namespace autopilot {

    tasc::tasc() :
        _目標停止位置(std::numeric_limits<double>::infinity()),
        _出力ノッチ(std::numeric_limits<int>::max())
    {
    }

    void tasc::レバー操作(const 共通状態 &状態)
    {
        if (!状態.戸閉()) {
            緩解();
        }
    }

    void tasc::戸閉()
    {
        緩解();
    }

    void tasc::地上子通過(const ATS_BEACONDATA & 地上子, const 共通状態 & 状態)
    {
        switch (地上子.Type)
        {
        case 17: // TASC 目標停止位置設定 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                _目標停止位置 = 状態.現在位置() + 11;
            }
            break;
        case 30: { // TASC 目標停止位置設定
            double 残距離 = 地上子.Optional / 1000;
            _目標停止位置 = 状態.現在位置() + 残距離;
            break;
        }
        case 32: // TASC 目標停止位置設定 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                _目標停止位置 = 状態.現在位置() + 500;
            }
            break;
        }
    }

    void tasc::経過(const 共通状態 & 状態)
    {
        速度型 現在速度 = 状態.現在速度();
        距離型 残距離 = _目標停止位置 - 状態.現在位置();
        加速度型 標準減速度 = 状態.制動().常用最大減速度() * 0.8;
        加速度型 勾配影響 = std::max(状態.進路勾配加速度(_目標停止位置), 0.0);
        加速度型 目標減速度 = 標準減速度 - 勾配影響;
        減速パターン パターン{ _目標停止位置, 0, 目標減速度 };
        _出力ノッチ = パターン.出力ノッチ(状態.現在位置(), 現在速度, 状態);

        if (残距離 <= 1 && std::abs(現在速度) < mps_from_kmph(1)) {
            // 止まりかけたらさっさと止めてしまう
            _出力ノッチ = std::min(
                _出力ノッチ, -状態.制動().無効ノッチ数() - 1);
        }
    }

    bool tasc::制御中() const
    {
        return std::isfinite(_目標停止位置);
    }

    void tasc::緩解()
    {
        _目標停止位置 = std::numeric_limits<double>::infinity();
    }

}
