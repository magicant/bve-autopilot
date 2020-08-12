// 力行特性.h : 力行ノッチと加速度の関係を計算します
//
// Copyright © 2020 Watanabe, Yuki
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
#include <map>
#include <vector>
#include "制御指令.h"
#include "物理量.h"

namespace autopilot
{

    class 力行特性
    {
    public:
        using 加速度一覧型 = std::map<mps, std::vector<mps2>>;

        力行特性();
        ~力行特性();

        void 性能設定(
            const std::vector<mps2> &加速度一覧, 力行ノッチ 最大ノッチ);

        力行ノッチ 最大力行ノッチ() const noexcept {
            return _最大ノッチ;
        }
        mps2 加速度(力行ノッチ ノッチ, mps 速度) const;

    private:
        力行ノッチ _最大ノッチ;
        /// いくつかの速度において、
        /// その速度以上の範囲における各力行ノッチの最大加速度の一覧。
        /// P0 ノッチは含まない。
        加速度一覧型 _加速度一覧;
    };

}
