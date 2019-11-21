// 早着防止.cpp : 予定時刻に列車が到達するように速度を調節します
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
#include "早着防止.h"
#include <algorithm>
#include "共通状態.h"
#include "減速パターン.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr s バッファ = 0.5_s;

        constexpr s 正午 = static_cast<s>(12 * 60 * 60);
        constexpr s 一日 = static_cast<s>(24 * 60 * 60);

        s 予定時刻(const 走行モデル &予定, const 共通状態 &状態) {
            s 時刻 = 予定.時刻();
            // 日をまたぐと時刻が戻るので補正する
            if (時刻 < 正午 && 正午 <= 状態.現在時刻()) {
                時刻 += 一日;
            }
            return 時刻;
        }

    }

    void 早着防止::リセット()
    {
        *this = 早着防止{};
    }

    void 早着防止::発進(const 共通状態 &状態)
    {
        // 通過済みの予定を消す
        _予定表.remove_if([&](const 走行モデル &予定) {
            return 予定.位置() <= 状態.現在位置() + 5.0_m;
            });
    }

    void 早着防止::地上子通過(
        const ATS_BEACONDATA &地上子, const 共通状態 &状態)
    {
        switch (地上子.Type) {
        case 1028:
            通過時刻設定(地上子);
            break;
        case 1029:
            通過位置設定(地上子, 状態);
            break;
        }
    }

    void 早着防止::経過(const 共通状態 &状態)
    {
        // 古い予定を消す
        _予定表.remove_if([&](const 走行モデル &予定) {
            return 予定時刻(予定, 状態) + バッファ < 状態.現在時刻();
            });

        if (加速可(状態)) {
            _出力ノッチ = 状態.最大力行ノッチ();
        }
        else if (状態.現在速度() <= static_cast<mps>(5.0_kmph)) {
            // 止まったままじっとしてるのもあれなので
            // 5 km/h までは加速するようにする
            _出力ノッチ = 力行ノッチ{1};
        }
        else {
            _出力ノッチ = 力行ノッチ{0};
        }
    }

    void 早着防止::通過時刻設定(const ATS_BEACONDATA &地上子)
    {
        _次の設定時刻 = static_cast<s>(地上子.Optional);
    }

    void 早着防止::通過位置設定(
        const ATS_BEACONDATA &地上子, const 共通状態 &状態)
    {
        m 位置 = 状態.現在位置() + static_cast<m>(地上子.Optional / 1000);
        mps 速度 = static_cast<kmph>(地上子.Optional % 1000);
        _予定表.emplace_front(位置, 速度, _次の設定時刻);
    }

    bool 早着防止::加速可(const 共通状態 &状態) const
    {
        return std::all_of(
            _予定表.begin(), _予定表.end(),
            [&](const 走行モデル &予定) {
                走行モデル 走行 = 状態.現在走行状態();

                // 現在状態から一定時間加速する動きをまずシミュレートする
                短く力行(走行, 状態.最大力行ノッチ(), 5.0_kmphps, 状態);

                // 減速にかかる時間を計算する
                減速パターン 減速{予定.位置(), 予定.速度(), 状態.目安減速度()};
                走行モデル 減速開始 = 減速.パターン到達状態(走行.速度());
                s 減速時間 = -減速開始.時刻();

                // 減速開始地点まで惰行する時間を計算する
                走行.指定位置まで走行(減速開始.位置());

                s 到達時刻 = 走行.時刻() + 減速時間;
                s 基準時刻 = 予定時刻(予定, 状態);
                // 頻繁な力行を避けるため時刻をずらす
                if (状態.前回力行ノッチ() > 0) {
                    基準時刻 -= バッファ;
                }
                else {
                    基準時刻 += バッファ;
                }
                return 到達時刻 >= 基準時刻;
            });
    }

}
