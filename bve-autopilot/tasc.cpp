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
#include <cmath>
#include <limits>

namespace autopilot {

    tasc::tasc(const ATS_VEHICLESPEC & 車両仕様) :
        _車両仕様(車両仕様),
        _目標停止位置(std::numeric_limits<double>::infinity()),
        _制御状態(制御状態::待機),
        _出力制動ノッチ(車両仕様.BrakeNotches)
    {
    }

    void tasc::地上子通過(const ATS_BEACONDATA & 地上子, const 共通状態 & 状態)
    {
        switch (地上子.Type)
        {
        case 30: // TASC 目標停止位置設定
            double 残距離 = 地上子.Optional / 1000;
            _目標停止位置 = 状態.現在位置() + 残距離;
            break;
        }
    }

    void tasc::経過(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2)
    {
        // 新しい状態を再計算
        switch (_制御状態)
        {
        case 制御状態::待機:
        case 制御状態::制動準備:
            _制御状態 = 制御状態::制動;
            break;
        case 制御状態::制動:
            //if (std::abs(mps_from_kmph(状態1.Speed)) < 0.1) {
            //    _制御状態 = 制御状態::停車;
            //}
            break;
        case 制御状態::停車:
            break;
        }

        // 状態に従って出力を計算
        switch (_制御状態)
        {
        case 制御状態::待機:
        case 制御状態::制動準備:
            _出力制動ノッチ = 0;
            break;
        case 制御状態::制動:
            出力計算(状態1, 状態2);
            break;
        case 制御状態::停車:
            _出力制動ノッチ = _車両仕様.BrakeNotches;
            break;
        }
    }

    void tasc::起動()
    {
        _制御状態 = 制御状態::待機;
    }

    void tasc::駅到着()
    {
        _目標停止位置 = std::numeric_limits<double>::infinity();
        _制御状態 = 制御状態::停車;
    }

    void tasc::出力計算(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2)
    {
        距離型 残距離 = _目標停止位置 - 状態1.Location;
        if (残距離 <= 0) {
            _出力制動ノッチ = _車両仕様.BrakeNotches;
            return;
        }
        加速度型 目標加速度 = 走行モデル::距離と速度による加速度(
            残距離, mps_from_kmph(状態1.Speed), 0);
        加速度型 常用最大減速度 = mps_from_kmph(4.0);
        double 制動ノッチ割合 = std::abs(目標加速度) / 常用最大減速度;
        int 無効ノッチ数 = _車両仕様.AtsNotch - 1;
        int 実効ノッチ数 = _車両仕様.BrakeNotches - 無効ノッチ数;
        _出力制動ノッチ =
            static_cast<int>(std::round(制動ノッチ割合 * 実効ノッチ数)) +
            無効ノッチ数;
        if (_出力制動ノッチ <= 無効ノッチ数) {
            _出力制動ノッチ = 0;
        }
        else if (_出力制動ノッチ > _車両仕様.BrakeNotches)
        {
            _出力制動ノッチ = _車両仕様.BrakeNotches;
        }
    }

}
