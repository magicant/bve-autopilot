// 急動作抑制.h : 急制動・急加速を避けるために出力ノッチを調整します
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
#include <limits>
#include "単位.h"

namespace autopilot
{

    class 共通状態;

    class 急動作抑制
    {
    public:
        void リセット();
        void 経過(int 入力ノッチ, const 共通状態 &状態);

        int 出力ノッチ() const { return _出力ノッチ; }

    private:
        int _出力ノッチ = 0;

        時間型 _最終制動操作時刻 =
            -std::numeric_limits<時間型>::infinity();
    };

}
