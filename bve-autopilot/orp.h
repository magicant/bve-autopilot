// orp.h : ORP の照査に抵触しないように減速します
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
#include "減速パターン.h"

namespace autopilot
{

    class 共通状態;
    class 信号順守;

    class orp
    {
    public:
        using 信号インデックス = int;

        orp();
        ~orp() = default;

        void リセット();
        void 設定(速度型 初期照査速度, 距離型 初期位置, 距離型 限界位置);

        void 信号現示変化(信号インデックス 指示);
        void 地上子通過(const ATS_BEACONDATA &地上子,
            const 共通状態 &状態, const 信号順守 &信号);

        void 経過(const 共通状態 &状態);

        // 力行は正の値、制動は負の値
        int 出力ノッチ() const { return _出力ノッチ; }

        速度型 照査速度(距離型 位置) const {
            return _照査パターン.期待速度(位置);
        }

    private:
        減速パターン _照査パターン, _運転パターン;
        int _出力ノッチ;
    };

}
