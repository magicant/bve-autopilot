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
#include "信号順守.h"
#include "制御指令.h"
#include "物理量.h"
#include "運動状態.h"

namespace autopilot
{

    class 共通状態;

    class 早着防止
    {
    public:
        using 発進方式 = 信号順守::発進方式;

        void リセット() noexcept;
        void 発進(発進方式 方式, const 共通状態 &状態);
        void 地上子通過(
            const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態);
        void 経過(const 共通状態 &状態);

        自動制御指令 出力ノッチ() const noexcept { return _出力ノッチ; }

    private:
        時刻 _次の設定時刻 = {};
        std::forward_list<運動状態> _予定表;
        自動制御指令 _出力ノッチ;

        void 通過時刻設定(const ATS_BEACONDATA &地上子) noexcept;
        void 通過位置設定(
            const ATS_BEACONDATA &地上子, m 地上子位置, const 共通状態 &状態);
        力行ノッチ 出力ノッチを計算(const 共通状態 &状態) const;
    };

}
