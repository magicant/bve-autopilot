// 共通状態.cpp : プラグイン全体で使用する、ゲーム全体の状態量です
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

#include "stdafx.h"
#include "共通状態.h"

namespace autopilot {

    void 共通状態::リセット()
    {
        _加速度計.リセット();
    }

    void 共通状態::経過(const ATS_VEHICLESTATE & 状態)
    {
        _加速度計.経過({ mps_from_kmph(状態.Speed), s_from_ms(状態.Time) });
    }

    void 共通状態::逆転器操作(int ノッチ)
    {
        _逆転器ノッチ = ノッチ;
    }

    void 共通状態::力行操作(int ノッチ)
    {
        _力行ノッチ = ノッチ;
    }

    void 共通状態::制動操作(int ノッチ)
    {
        _制動ノッチ = ノッチ;
    }

}
