// ato.h : ATO メインモジュール
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
#include <map>
#include "信号順守.h"
#include "制限グラフ.h"
#include "単位.h"

namespace autopilot
{

    class tasc;
    class 共通状態;

    class ato
    {
    public:
        using 信号インデックス = int;

        ato();
        ~ato();

        void リセット();
        void 発進();
        void 信号現示変化(信号インデックス 指示) { _信号.信号現示変化(指示); }
        void 地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態);
        void 経過(const 共通状態 &状態, const tasc &tasc);

        速度型 現在制限速度(const 共通状態 &状態) const;
        速度型 現在常用パターン速度(const 共通状態 &状態) const;

        // 力行は正の値、制動は負の値
        int 出力ノッチ() const { return _出力ノッチ; }

    private:
        制限グラフ _制限速度1006, _制限速度1007;
        信号順守 _信号;
        bool _発進中 = false;
        int _出力ノッチ = 0;
    };

}
