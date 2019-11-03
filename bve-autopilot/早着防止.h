// 早着防止.h : 予定時刻に列車が到達するように速度を調節します
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
#include "単位.h"
#include "走行モデル.h"

namespace autopilot
{

    class 共通状態;

    class 早着防止
    {
    public:
        void リセット();
        void 地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態);
        void 経過(const 共通状態 &状態);

        int 出力ノッチ() const { return _出力ノッチ; }

    private:
        s _次の設定時刻 = {};
        std::forward_list<走行モデル> _予定表;
        int _出力ノッチ = 0;

        void 通過時刻設定(const ATS_BEACONDATA &地上子);
        void 通過位置設定(const ATS_BEACONDATA &地上子, const 共通状態 &状態);
        bool 加速可(const 共通状態 &状態) const;
    };

}
