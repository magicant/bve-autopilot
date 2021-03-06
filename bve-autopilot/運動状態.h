// 運動状態.h : 走行列車の位置・速度・時刻をシミュレートします。
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

namespace autopilot {

    struct 運動状態
    {
    public:
        using 時刻型 = autopilot::時刻;

        m 位置;
        mps 速度;
        時刻型 時刻;

        constexpr explicit 運動状態(
            m 位置 = {}, mps 速度 = {}, 時刻型 時刻 = {}) noexcept :
            位置(位置), 速度(速度), 時刻(時刻) { }

        // 等加加速度運動
        mps2 指定時間走行(s 時間, mps2 初加速度 = {}, mps3 加加速度 = {})
            noexcept;
        mps2 指定時刻まで走行(
            時刻型 新時刻, mps2 初加速度 = {}, mps3 加加速度 = {})
            noexcept;

        // 等加速度運動
        void 指定距離走行(m 距離, mps2 加速度 = {}, bool 後退 = false);
        void 指定位置まで走行(m 新位置, mps2 加速度 = {}, bool 後退 = false);
        void 指定速度まで走行(mps 新速度, mps2 加速度 = {}) noexcept;

        // 等加加速度運動
#if 0
        mps2 指定位置まで走行(m 新位置, mps2 初加速度, mps3 加加速度);
        void 等加加速度で指定加速度まで走行(
            mps2 初加速度, mps2 終加速度, mps3 加加速度) noexcept;
#endif
        mps2 指定速度まで走行(
            mps 新速度, mps2 初加速度, mps3 加加速度, bool 減速);

        // 等加速度運動
        constexpr static mps2 距離と速度による加速度(
            m 距離, mps 初速度, mps 終速度) noexcept
        {
            return (終速度 * 終速度 - 初速度 * 初速度) / 距離 / 2.0;
        }

    };

    class 共通状態;

    bool 短く力行(
        運動状態 &運動状態, 力行ノッチ 力行ノッチ, const 共通状態 &状態);

}
