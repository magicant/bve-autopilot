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
#include "減速目標.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr s バッファ = 0.5_s;

        bool 加速可(
            力行ノッチ ノッチ, const 運動状態 &予定, const 共通状態 &状態)
        {
            運動状態 走行 = 状態.現在走行状態();

            // 現在状態から一定時間加速する動きをまずシミュレートする
            if (!短く力行(走行, ノッチ, 状態)) {
                return false;
            }

            // 減速を開始する位置と時刻を求める
            運動状態 減速開始 = 予定;
            if (減速開始.速度() < 走行.速度()) {
                減速開始.指定速度まで走行(走行.速度(), -状態.目安減速度());
            }

            // 減速開始地点まで惰行する時間を計算する
            走行.指定位置まで走行(減速開始.位置());

            時刻 目標時刻 = 減速開始.時刻();
            // 頻繁な力行を避けるため時刻をずらす
            if (状態.前回力行ノッチ() > 0) {
                目標時刻 -= バッファ;
            }
            else {
                目標時刻 += バッファ;
            }
            return 走行.時刻() >= 目標時刻;
        }

    }

    void 早着防止::リセット() noexcept
    {
        _次の設定時刻 = {};
        _予定表.clear();
        _出力ノッチ = {};
    }

    void 早着防止::発進(const 共通状態 &状態)
    {
        // 通過済みの予定を消す
        _予定表.remove_if([&](const 運動状態 &予定) {
            return 予定.位置() <= 状態.現在位置() + 5.0_m;
            });
    }

    void 早着防止::地上子通過(
        const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態)
    {
        switch (地上子.Type) {
        case 1028:
            通過時刻設定(地上子);
            break;
        case 1029:
            通過位置設定(地上子, 直前位置, 状態);
            break;
        }
    }

    void 早着防止::経過(const 共通状態 &状態)
    {
        // 古い予定を消す
        _予定表.remove_if([&](const 運動状態 &予定) {
            return 予定.時刻() + バッファ < 状態.現在時刻();
            });

        _出力ノッチ = 出力ノッチを計算(状態);

        if (_出力ノッチ == 力行ノッチ{0} &&
            状態.現在速度() <= static_cast<mps>(5.0_kmph))
        {
            // 止まったままじっとしてるのもあれなので
            // 5 km/h までは加速するようにする
            _出力ノッチ = 力行ノッチ{1};
        }
    }

    void 早着防止::通過時刻設定(const ATS_BEACONDATA &地上子) noexcept
    {
        _次の設定時刻 = static_cast<時刻>(static_cast<s>(地上子.Optional));
    }

    void 早着防止::通過位置設定(
        const ATS_BEACONDATA &地上子, m 地上子位置, const 共通状態 &状態)
    {
        if (状態.現在位置() - 地上子位置 > 1.0_m &&
            状態.現在速度() == 0.0_mps)
        {
            // 「停車場へ移動」で地上子をすっ飛ばしたときは
            // 地上子の位置が正確に分からないので無視する
            return;
        }

        m 位置 = 地上子位置 + static_cast<m>(地上子.Optional / 1000);
        mps 速度 = static_cast<kmph>(地上子.Optional % 1000);
        _予定表.emplace_front(位置, 速度, _次の設定時刻);
    }

    力行ノッチ 早着防止::出力ノッチを計算(const 共通状態 &状態) const
    {
        力行ノッチ ノッチ = 状態.最大力行ノッチ();

        // 現在よりも弱いノッチでの加速度は予測できないので除く
        // また抵抗器保護のため第一ノッチは低速域でしか使わないようにする
        力行ノッチ 最小ノッチ = 力行ノッチ{
            static_cast<unsigned>(std::max(状態.前回力行ノッチ(), 1))};
        if (状態.最大力行ノッチ() > 力行ノッチ{1} &&
            状態.現在速度() > static_cast<mps>(10.0_kmph)) {
            最小ノッチ = std::max(最小ノッチ, 力行ノッチ{2});
        }

        for (const 運動状態 &予定 : _予定表) {
            while (!加速可(ノッチ, 予定, 状態)) {
                --ノッチ.value;
                if (ノッチ < 最小ノッチ) {
                    return 力行ノッチ{0};
                }
            }
        }

        return ノッチ;
    }

}
