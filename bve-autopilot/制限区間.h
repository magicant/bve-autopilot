// 制限区間.h : 列車の速度が制限されている区間を表します
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
#include "区間.h"
#include "単位.h"
#include "減速パターン.h"

namespace autopilot
{

    struct 制限区間 : 区間
    {
        m 減速目標地点;
        mps 速度;

        制限区間(m 減速目標地点, m 始点, m 終点, mps 速度) :
            区間{始点, 終点}, 減速目標地点{減速目標地点}, 速度{速度} { }
        ~制限区間() = default;

        減速パターン 目標パターン(mps2 初期減速度) const;
        減速パターン 限界パターン(mps2 減速度) const;
    };

}
