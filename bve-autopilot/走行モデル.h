// 走行モデル.h : 走行列車の位置・速度・時刻をシミュレートします。
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
#include "単位.h"

namespace autopilot {

    class 走行モデル
    {
    public:
        constexpr explicit 走行モデル(
            m 位置 = {}, mps 速度 = {}, s 時刻 = {}) :
            _位置(位置), _速度(速度), _時刻(時刻) { }
        ~走行モデル() = default;

        m 位置() const { return _位置; }
        void 位置変更(m 位置) { _位置 = 位置; }
        mps 速度() const { return _速度; }
        void 速度変更(mps 速度) { _速度 = 速度; }
        s 時刻() const { return _時刻; }
        void 時刻変更(s 時刻) { _時刻 = 時刻; }

        void 変更(m 位置 = {}, mps 速度 = {}, s 時刻 = {}) {
            _位置 = 位置;
            _速度 = 速度;
            _時刻 = 時刻;
        }

        // 等加加速度運動
        mps2 指定時間走行(s 時間, mps2 初加速度 = {}, mps3 加加速度 = {});
        mps2 指定時刻まで走行(s 時刻, mps2 初加速度 = {}, mps3 加加速度 = {});

        // 等加速度運動
        void 指定距離走行(m 距離, mps2 加速度 = {});
        void 指定位置まで走行(m 位置, mps2 加速度 = {});
        void 指定速度まで走行(mps 速度, mps2 加速度 = {});

        // 等加加速度運動
        mps2 指定位置まで走行(m 位置, mps2 初加速度, mps3 加加速度);
        void 等加加速度で指定加速度まで走行(
            mps2 初加速度, mps2 終加速度, mps3 加加速度);
        mps2 指定速度まで走行(mps 速度, mps2 初加速度, mps3 加加速度);

        // 等加速度運動
        static mps2 距離と速度による加速度(m 距離, mps 初速度, mps 終速度);

    private:
        m _位置;
        mps _速度;
        s _時刻;
    };

    class 共通状態;

    void 短く力行(
        走行モデル &モデル, int 力行ノッチ, mps2 想定加速度,
        const 共通状態 &状態);

}
