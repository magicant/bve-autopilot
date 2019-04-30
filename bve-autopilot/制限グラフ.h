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
#include <forward_list>
#include "制限区間.h"
#include "単位.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;

    class 制限グラフ
    {
    public:
        void 消去();
        void 制限区間追加(距離型 始点, 速度型 速度);
        void 通過(距離型 位置);

        速度型 制限速度(区間 対象区間) const;
        速度型 制限速度(距離型 位置) const {
            return 制限速度(区間{ 位置, 位置 });
        }

        速度型 現在常用パターン速度(const 共通状態 &状態) const;

        // 力行は正の値、制動は負の値
        int 出力ノッチ(
            距離型 現在位置, 速度型 現在速度, const 共通状態 & 状態,
            速度型 速度マージン = mps_from_kmph(0.5)) const;

    private:
        std::forward_list<制限区間> _区間リスト;
    };

}

#pragma warning(pop)
