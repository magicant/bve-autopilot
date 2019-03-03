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
#include "減速パターン.h"

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
            if (std::abs(mps_from_kmph(状態1.Speed)) < 0.05) {
                _制御状態 = 制御状態::停車;
            }
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
            // 過走した!
            _出力制動ノッチ = _車両仕様.BrakeNotches;
        }
        else {
            出力計算_標準(状態1, 状態2);
        }

        // 停止直前はノッチを緩めて衝撃を抑える
        減速パターン::速度型 現在速度 = mps_from_kmph(状態1.Speed);
        double 緩めノッチ = std::ceil(状態2.目標制動ノッチ(現在速度 / 2.0));
        _出力制動ノッチ = std::min(_出力制動ノッチ, static_cast<int>(緩めノッチ));

        if (_出力制動ノッチ < _車両仕様.AtsNotch) {
            _出力制動ノッチ = 0;
        }
        else if (_出力制動ノッチ > _車両仕様.BrakeNotches) {
            _出力制動ノッチ = _車両仕様.BrakeNotches;
        }

        if (std::abs(現在速度) < mps_from_kmph(1)) {
            // 止まりかけたらさっさと止めてしまう
            _出力制動ノッチ = std::max(_出力制動ノッチ, _車両仕様.AtsNotch);
        }
    }

    void tasc::出力計算_標準(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2)
    {
        距離型 残距離 = _目標停止位置 - 状態1.Location;
        減速パターン::速度型 現在速度 = mps_from_kmph(状態1.Speed);
        減速パターン::加速度型 目標減速度 = 状態2.常用最大減速度() * 0.8;
        減速パターン パターン(_目標停止位置, 0, 目標減速度);
        減速パターン::速度型 期待速度 = パターン.期待速度(状態1.Location);

        減速パターン::加速度型 出力減速度 = std::min({
            // 単純な等加速度運動で減速するパターン
            -走行モデル::距離と速度による加速度(残距離, 現在速度, 0),
            // 減速パターンに近付いたら目標減速度に漸次的に近付ける
            目標減速度 * (現在速度 / 期待速度) - (期待速度 - 現在速度) / 2,
            // 速度が落ちすぎたら目標減速度に急進的に近付ける
            目標減速度 * (4 - 3 * (期待速度 + 0.5) / (現在速度 + 0.5)),
            });

        double 出力制動ノッチ = 状態2.目標制動ノッチ(出力減速度);
        if (出力減速度 < 目標減速度) {
            _出力制動ノッチ = static_cast<int>(std::floor(出力制動ノッチ));
        }
        else {
            _出力制動ノッチ = static_cast<int>(std::round(出力制動ノッチ));
        }
    }

}
