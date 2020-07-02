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
#include <algorithm>
#include <cmath>
#include <tuple>
#include "共通状態.h"

namespace autopilot
{

    namespace
    {
        constexpr s 閾 = 2.0_s;
    }

    void 急動作抑制::経過(
        自動制御指令 入力ノッチ, const 共通状態 &状態, bool is_atc)
    {
        std::tie(_最小出力減速度, _最大出力減速度) =
            新出力減速度(入力ノッチ, 状態, is_atc);
        _最終出力減速度計算時刻 = 状態.現在時刻();

        _出力ノッチ = 新出力ノッチ(入力ノッチ, 状態);
        if (_出力ノッチ.力行成分() > 力行ノッチ{0}) {
            _最終力行時刻 = 状態.現在時刻();
        }
        else if (_出力ノッチ.制動成分() > 自動制動自然数ノッチ{0}) {
            if (!状態.停車中()) {
                _最終制動時刻 = 状態.現在時刻();
            }
        }
    }

    std::pair<mps2, mps2> 急動作抑制::新出力減速度(
        自動制御指令 入力ノッチ, const 共通状態 &状態, bool is_atc) const
    {
        // 力行をやめた直後は制動しない
        if (is_atc && 状態.現在時刻() - _最終力行時刻 < 閾) {
            return {0.0_mps2, 0.0_mps2};
        }

        自動制動自然数ノッチ 入力制動ノッチ = 入力ノッチ.制動成分();
        mps2 入力減速度 = 状態.制動().減速度(入力制動ノッチ);

        // 急動作抑制を使わずに出力された制動ノッチも考慮したいので
        // ここで前回出力を取り込む
        自動制動自然数ノッチ 直前ノッチ =
            状態.制動().自動ノッチ(状態.前回制動指令());
        mps2 直前減速度 = 状態.制動().減速度(直前ノッチ);

        mps2 実減速度 = std::max(入力減速度, 直前減速度);

        s フレーム = std::clamp(
            状態.現在時刻() - _最終出力減速度計算時刻, 0.001_s, 0.5_s);
        mps2 最大変化 = 2.0_kmphps2 * フレーム;

        mps2 新最小出力減速度 = std::clamp(
            入力減速度,
            _最小出力減速度 - 最大変化, _最小出力減速度 + 最大変化);
        mps2 新最大出力減速度 = std::clamp(
            実減速度,
            _最大出力減速度 - 最大変化, _最大出力減速度 + 最大変化);
        return {新最小出力減速度, 新最大出力減速度};
    }

    自動制御指令 急動作抑制::新出力ノッチ(
        自動制御指令 入力ノッチ, const 共通状態 &状態) const
    {
        if (入力ノッチ.力行成分() > 力行ノッチ{0}) {
            // 制動をやめた直後は力行しない
            if (状態.現在時刻() - _最終制動時刻 >= 閾) {
                return 入力ノッチ;
            }
        }

        自動制動実数ノッチ 入力制動ノッチ = 入力ノッチ.制動成分();

        自動制動実数ノッチ 最小出力ノッチ =
            状態.制動().自動ノッチ(_最小出力減速度);
        自動制動実数ノッチ 最大出力ノッチ =
            状態.制動().自動ノッチ(_最大出力減速度);
        最小出力ノッチ.value = std::floor(最小出力ノッチ.value);
        最大出力ノッチ.value = std::ceil(最大出力ノッチ.value);

        自動制動実数ノッチ 新出力ノッチ =
            std::clamp(入力制動ノッチ, 最小出力ノッチ, 最大出力ノッチ);
        return 自動制動自然数ノッチ{static_cast<unsigned>(新出力ノッチ.value)};
    }

}
