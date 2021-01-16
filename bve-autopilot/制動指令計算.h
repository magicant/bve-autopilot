// 制動指令計算.h : ブレーキの強さを計算するクラスの抽象化です
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
#include <utility>
#include "制御指令.h"
#include "区間.h"

namespace autopilot
{

    struct 運動状態;
    class 共通状態;

    class 制動指令計算
    {
    public:
        virtual 自動制動自然数ノッチ 出力制動ノッチ(
            const 運動状態 &運動状態, const 共通状態 &状態) const = 0;

        /// 引数の範囲の中で許容速度 (制限速度または減速パターンの速度) が
        /// 最低となる区間とその速度を返す。
        /// 引数は負の長さを持つ可能性があるが、戻り値の区間は必ず非負の長さ。
        /// 許容速度を計算できない場合、戻り値の速度は無限大。
        virtual std::pair<mps, 区間> 最低許容速度区間(区間 範囲) const = 0;

    protected:
        ~制動指令計算() = default;
    };

}
