// Main.cpp : プラグイン全体を統括します
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
#include "Main.h"

namespace autopilot
{

    Main::Main() :
        _車両仕様{},
        _逆転器ノッチ{},
        _力行ノッチ{},
        _制動ノッチ{}
    {
    }


    Main::~Main()
    {
    }

    void Main::車両仕様設定(const ATS_VEHICLESPEC & 車両仕様)
    {
        _車両仕様 = 車両仕様;
    }

    void Main::リセット(int 制動状態)
    {
    }

    void Main::逆転器操作(int ノッチ)
    {
        _逆転器ノッチ = ノッチ;
    }

    void Main::力行操作(int ノッチ)
    {
        _力行ノッチ = ノッチ;
    }

    void Main::制動操作(int ノッチ)
    {
        _制動ノッチ = ノッチ;
    }

    ATS_HANDLES Main::経過(const ATS_VEHICLESTATE & 状態, int * 出力値, int * 音声状態)
    {
        ATS_HANDLES ハンドル位置;
        ハンドル位置.Brake = _制動ノッチ;
        ハンドル位置.Power = _力行ノッチ;
        ハンドル位置.Reverser = _逆転器ノッチ;
        ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;
        return ハンドル位置;
    }

}
