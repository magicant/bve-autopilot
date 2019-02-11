// tasc.h : TASC メインモジュール
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
#include "共通状態.h"
#include "走行モデル.h"

namespace autopilot {

    class tasc
    {
    public:
        using 距離型 = 走行モデル::距離型;
        using 加速度型 = 走行モデル::加速度型;

        tasc(const ATS_VEHICLESPEC & 車両仕様);
        ~tasc() = default;

        void 地上子通過(const ATS_BEACONDATA & 地上子, const 共通状態 & 状態);
        void 経過(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2);
        void 駅出発();
        void 駅到着();

        bool 制御中() const { return _制御状態 != 制御状態::待機; }
        int 出力制動ノッチ() const { return _出力制動ノッチ; }

    private:
        enum class 制御状態 { 待機, 制動準備, 制動, 停車, };

        const ATS_VEHICLESPEC & _車両仕様;
        距離型 _目標停止位置;
        制御状態 _制御状態;
        int _出力制動ノッチ;

        void 出力計算(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2);
    };

}
