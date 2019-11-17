// 制動力推定.cpp : 実際の減速度に基づいて制動力を補正します
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
#include "制動力推定.h"
#include "共通状態.h"

namespace autopilot
{

    namespace
    {

        bool 圧力差が小さい(float 圧力1, float 圧力2) {
            if (圧力1 == 圧力2) {
                return true;
            }
            float 比 = 圧力1 / 圧力2;
            return 0.999f <= 比 && 比 <= 1 / 0.999f;
        }

    }

    void 制動力推定::経過(制動力割合 前回出力割合, const 共通状態 &状態)
    {
        if (_前回出力ノッチ != 状態.前回出力().Brake) {
            _前回出力ノッチ = 状態.前回出力().Brake;
            _前回出力ノッチ変化時刻 = 状態.現在時刻();
        }

        制動力割合 割合 = 前回出力割合;
        bool 割合安定 = 0.1 <= 割合.value && 割合.value <= 1.0;
        if (割合安定 && 状態.現在電流() == 0 &&
            abs(_前回出力ノッチ変化時刻 - 状態.現在時刻()) >=
                状態.制動().反応時間() &&
            圧力差が小さい(_前回観測圧力, 状態.現在ブレーキシリンダー圧()))
        {
            _推定最大圧力 = static_cast<float>(
                状態.現在ブレーキシリンダー圧() / 割合.value);
        }
        else {
            割合 = 制動力割合{状態.現在ブレーキシリンダー圧() / _推定最大圧力};
            割合安定 = 0.1 <= 割合.value && 割合.value <= 1.0;
        }

        if (!割合安定 || 状態.現在電流() != 0) {
            _前回不安定時刻 = 状態.現在時刻();
        }

        if (状態.現在速度() >= static_cast<mps>(0.1_kmph)) {
            if (abs(_前回不安定時刻 - 状態.現在時刻()) >= 0.5_s) {
                最大減速度を推定(割合, 状態);
            }
            else {
                _推定最大減速度 = _基準最大減速度;
            }
        }

        _前回観測圧力 = 状態.現在ブレーキシリンダー圧();
        _前回観測時刻 = 状態.現在時刻();
    }

    void 制動力推定::最大減速度を推定(制動力割合 割合, const 共通状態 &状態)
    {
        s フレーム = 状態.現在時刻() - _前回観測時刻;
        if (フレーム <= 0.0_s || 1.0_s <= フレーム) {
            return;
        }

        mps2 出力減速度 = 状態.車両勾配加速度() - 状態.加速度();
        if (出力減速度 < static_cast<mps2>(0.1_kmphps)) {
            return;
        }

        mps2 変化量上限 = 1.0_kmphps2 * フレーム;
        mps2 新推定値 = 出力減速度 / 割合.value;
        _推定最大減速度 = std::clamp(新推定値,
            _推定最大減速度 - 変化量上限, _推定最大減速度 + 変化量上限);
    }

}
