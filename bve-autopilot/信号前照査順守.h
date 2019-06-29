// 信号前照査順守.h : 速度照査に引っ掛からないように減速する機能
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
#include <vector>
#include "制限グラフ.h"
#include "単位.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;
    class 信号順守;

    class 信号前照査順守
    {
    public:
        信号前照査順守();
        ~信号前照査順守();

        void リセット();
        void 発進(const 共通状態 &状態, const 信号順守 &信号) {
            再計算(状態, 信号);
        }
        void 信号現示変化(const 共通状態 &状態, const 信号順守 &信号) {
            再計算(状態, 信号);
        }
        void 地上子通過(const ATS_BEACONDATA &地上子,
            const 共通状態 &状態, const 信号順守 &信号);

        void 経過(const 共通状態 &状態);

        // 力行は正の値、制動は負の値
        int 出力ノッチ(const 共通状態 &状態) const;

        速度型 現在制限速度(区間 現在範囲) const {
            return _制限グラフ.制限速度(現在範囲);
        }
        速度型 現在常用パターン速度(const 共通状態 &状態) const {
            return _制限グラフ.現在常用パターン速度(状態);
        }

    private:
        struct 照査型 {
            距離型 照査位置;
            速度型 照査速度;
            距離型 信号位置;
        };

        std::vector<照査型> _照査表;

        // 経過メソッドが呼ばれる度に毎回制限グラフを計算するのはメモリに
        // 優しくないので予め計算しておく
        制限グラフ _制限グラフ;

        void 再計算(const 共通状態 &状態, const 信号順守 &信号);
    };

}

#pragma warning(pop)
