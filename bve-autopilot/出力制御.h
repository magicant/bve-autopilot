// 出力制御.h : 出力すべき制御指令を計算します。
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
#include <functional>
#include "制動指令計算.h"
#include "制御指令.h"
#include "物理量.h"

namespace autopilot
{

    struct 運動状態;
    class 共通状態;

    class 出力制御
    {
    public:
        static 自動制御指令 出力ノッチ(
            const 制動指令計算 &減速, const 共通状態 &状態)
        {
            return 出力制御{減速, 状態}.出力ノッチ();
        }

    private:
        const 制動指令計算 &_計算;
        const 共通状態 &_状態;

        constexpr 出力制御(
            const 制動指令計算 &計算, const 共通状態 &状態) noexcept :
            _計算{計算}, _状態{状態}
        {}

        自動制動自然数ノッチ 出力制動ノッチ(const 運動状態 &運動状態) const {
            return _計算.出力制動ノッチ(運動状態, _状態);
        }

        bool 制動を緩める余裕あり(自動制動自然数ノッチ 新制動ノッチ) const;
        自動制動自然数ノッチ 出力制動強めノッチ() const;
        s 惰行時間(力行ノッチ 力行ノッチ, mps 速度) const;
        mps パターン接点に移動(運動状態 &状態, s 惰行時間) const;
        bool 力行する余裕あり(力行ノッチ ノッチ) const;
        力行ノッチ 出力力行ノッチ() const;
        自動制御指令 出力ノッチ() const;
    };

}
