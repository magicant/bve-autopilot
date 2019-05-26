// 急動作抑制.cpp : 急制動・急加速を避けるために出力ノッチを調整します
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
#include "急動作抑制.h"
#include <cmath>
#include "共通状態.h"

namespace autopilot
{

    namespace
    {

        bool 停車中(const autopilot::共通状態 &状態)
        {
            return 状態.現在速度() < mps_from_kmph(0.05);
        }

    }

    void 急動作抑制::リセット()
    {
        *this = 急動作抑制{};
    }

    void 急動作抑制::経過(int 入力ノッチ, const 共通状態 &状態)
    {
        if (入力ノッチ == _出力ノッチ) {
            return;
        }
        if (停車中(状態)) {
            _出力ノッチ = 入力ノッチ;
            return;
        }

        時間型 現在時刻 = 状態.現在時刻();

        if (入力ノッチ < 0 || _出力ノッチ < 0) { // 制動中
            時間型 閾 = 1.5 / 状態.制動().実効ノッチ数();
            if (std::abs(現在時刻 - _最終制動操作時刻) < 閾) {
                return;
            }
            _最終制動操作時刻 = 現在時刻;
            if (入力ノッチ > _出力ノッチ) {
                _出力ノッチ++;
            }
            else {
                _出力ノッチ--;
            }
            return;
        }

        if (入力ノッチ > 0 && _出力ノッチ <= 0) { // 力行開始
            時間型 閾 = 2;
            if (std::abs(現在時刻 - _最終制動操作時刻) < 閾) {
                return;
            }
        }

        _出力ノッチ = 入力ノッチ;
    }

}
