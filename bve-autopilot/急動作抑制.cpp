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

    void 急動作抑制::リセット()
    {
        *this = 急動作抑制{};
    }

    void 急動作抑制::経過(int 入力ノッチ, const 共通状態 &状態, bool is_atc)
    {
        if (入力ノッチ == _出力ノッチ) {
            return;
        }
        if (状態.停車中()) {
            _出力ノッチ = 入力ノッチ;
            return;
        }

        s 現在時刻 = 状態.現在時刻();

        if (入力ノッチ <= 0 && _出力ノッチ > 0) { // 力行終了
            _最終力行操作時刻 = 現在時刻;
            _出力ノッチ = 0;
            return;
        }

        if (入力ノッチ < 0 && _出力ノッチ >= 0) { // 制動開始
            if (is_atc) {
                s 閾 = 2.0_s;
                if (abs(現在時刻 - _最終力行操作時刻) < 閾) {
                    return;
                }
            }
        }

        if (入力ノッチ < 0 || _出力ノッチ < 0) { // 制動中
            s 閾 = 1.5_s / static_cast<double>(状態.制動().自動ノッチ数());
            if (abs(現在時刻 - _最終制動操作時刻) < 閾) {
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
            s 閾 = 2.0_s;
            if (abs(現在時刻 - _最終制動操作時刻) < 閾) {
                return;
            }
        }

        _出力ノッチ = 入力ノッチ;
    }

}
