// 加速度計.h : 速度変化から加速度を推定します
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

namespace autopilot
{

    class 加速度計
    {
    public:
        struct 観測 {
            速度型 _速度;
            時間型 _時刻;
        };

        加速度計();

        void リセット();
        void 経過(観測 データ);

        加速度型 加速度() const { return _加速度; }
        加加速度型 加加速度() const { return _加加速度; }

    private:
        constexpr static unsigned 記録数 = 3;
        観測 _記録[記録数];
        加速度型 _加速度;
        加加速度型 _加加速度;
    };

}
