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

    Main::Main() : _車両仕様{}, _状態{}, _tasc{_車両仕様}, _tasc制御中{false}
    {
    }

    void Main::車両仕様設定(const ATS_VEHICLESPEC & 車両仕様)
    {
        _車両仕様 = 車両仕様;
    }

    void Main::リセット(int 制動状態)
    {
        _状態.リセット();
    }

    void Main::逆転器操作(int ノッチ)
    {
        _状態.逆転器操作(ノッチ);
    }

    void Main::力行操作(int ノッチ)
    {
        _状態.力行操作(ノッチ);
    }

    void Main::制動操作(int ノッチ)
    {
        _状態.制動操作(ノッチ);
    }

    void Main::戸閉()
    {
        _tasc.駅出発();
    }

    void Main::戸開()
    {
        _tasc.駅到着();
    }

    ATS_HANDLES Main::経過(const ATS_VEHICLESTATE & 状態, int * 出力値, int * 音声状態)
    {
        _状態.経過(状態);
        _tasc.経過(状態, _状態);

        ATS_HANDLES ハンドル位置;
        if (_tasc制御中 && _tasc.制御中()) {
            ハンドル位置.Brake = _tasc.出力制動ノッチ();
            ハンドル位置.Power = 0;
            ハンドル位置.Reverser = _状態.逆転器ノッチ();
            ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;
        }
        else {
            ハンドル位置.Brake = _状態.制動ノッチ();
            ハンドル位置.Power = _状態.力行ノッチ();
            ハンドル位置.Reverser = _状態.逆転器ノッチ();
            ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;
        }
        return ハンドル位置;
    }

}
