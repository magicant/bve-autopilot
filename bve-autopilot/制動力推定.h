// 制動力推定.h : 実際の減速度に基づいて制動力を補正します
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
#include "制御指令.h"
#include "物理量.h"

namespace autopilot
{

    class 共通状態;

    class 制動力推定
    {
    public:
        mps2 基準最大減速度() const { return _基準最大減速度; }
        void 基準最大減速度を設定(mps2 減速度) {
            _基準最大減速度 = _推定最大減速度 = 減速度;
        }
        mps2 推定最大減速度() const { return _推定最大減速度; }

        void 経過(制動力割合 前回出力割合, const 共通状態 &状態);

    private:
        /// 環境設定で指定された、最大常用ブレーキの平均減速度
        mps2 _基準最大減速度 = {};
        /// 実際の制動力から推定した最大常用ブレーキの減速度
        mps2 _推定最大減速度 = {};
        float _推定最大圧力 = 440;
        float _前回観測圧力 = 0;
        int _前回出力ノッチ = 0;
        s _前回出力ノッチ変化時刻 = {};
        s _前回不安定時刻 = {};
        s _前回観測時刻 = {};

        void 最大減速度を推定(制動力割合 割合, const 共通状態 &状態);
    };

}
