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
#include "orp.h"
#include "信号順守.h"
#include "制限グラフ.h"
#include "急動作抑制.h"
#include "早着防止.h"
#include "物理量.h"

namespace autopilot
{

    class 共通状態;

    class ato
    {
    public:
        using 信号インデックス = int;

        ato();
        ~ato();

        void リセット();
        void 発進(const 共通状態 &状態);
        void 信号現示変化(信号インデックス 指示);
        void tasc目標停止位置変化(m 位置) {
            _信号.tasc目標停止位置変化(位置);
        }
        void 地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態);
        void 経過(const 共通状態 &状態);

        mps 現在制限速度(const 共通状態 &状態) const;
        mps 現在常用パターン速度(const 共通状態 &状態) const;
        mps 現在orp照査速度() const;
        bool 力行抑止中() const { return _早着防止.出力ノッチ() <= 1; }

        // 力行は正の値、制動は負の値
        int 出力ノッチ() const { return _出力ノッチ; }

    private:
        enum class 制御状態 { 停止, 発進, 走行, };

        制限グラフ _制限速度1006, _制限速度1007,
            _制限速度6, _制限速度8, _制限速度9, _制限速度10;
        信号順守 _信号;
        orp _orp;
        早着防止 _早着防止;
        制御状態 _制御状態 = 制御状態::走行;
        int _出力ノッチ = 0;
        急動作抑制 _急動作抑制;
    };

}
