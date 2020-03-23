// 制限グラフ.h : 区間ごとに定められる制限速度の変化を表します
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
#include <map>
#include "制御指令.h"
#include "区間.h"
#include "物理量.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;

    class 制限グラフ
    {
    public:
        制限グラフ();
        ~制限グラフ();

        void 消去();
        void 制限区間追加(m 減速目標地点, m 始点, mps 速度);
        void 通過(m 位置);

        mps 制限速度(区間 対象区間) const;
        mps 制限速度(m 位置) const {
            return 制限速度(区間{ 位置, 位置 });
        }

        mps 現在常用パターン速度(const 共通状態 &状態) const;

        自動制御指令 出力ノッチ(const 共通状態 &状態) const;

    private:
        struct 制限区間;

        // 区間の始点からその区間のデータへの写像
        std::map<m, 制限区間> _区間リスト;
    };

}

#pragma warning(pop)
