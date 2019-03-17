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

    Main::Main() : _状態{}, _tasc{}, _tasc制御中{false}
    {
    }

    void Main::リセット(int)
    {
        _状態.リセット();
        _tasc制御中 = false;
    }

    void Main::逆転器操作(int ノッチ)
    {
        _状態.逆転器操作(ノッチ);
        _tasc制御中 = false;
    }

    void Main::力行操作(int ノッチ)
    {
        _状態.力行操作(ノッチ);
        _tasc制御中 = false;
    }

    void Main::制動操作(int ノッチ)
    {
        _状態.制動操作(ノッチ);
        _tasc制御中 = false;
    }

    void Main::警笛操作(int)
    {
    }

    void Main::キー押し(int キー)
    {
        switch (キー)
        {
        case ATS_KEY_S: // Default: Space
            break;
        case ATS_KEY_A1: // Default: Insert
            break;
        case ATS_KEY_A2: // Default: Delete
            break;
        case ATS_KEY_B1: // Default: Home
            break;
        case ATS_KEY_B2: // Default: End
            break;
        case ATS_KEY_C1: // Default: Page Up
            break;
        case ATS_KEY_C2: // Default: Page Down
            break;
        case ATS_KEY_D: // Default: 2
            break;
        case ATS_KEY_E: // Default: 3
            break;
        case ATS_KEY_F: // Default: 4
            break;
        case ATS_KEY_G: // Default: 5
            break;
        case ATS_KEY_H: // Default: 6
            break;
        case ATS_KEY_I: // Default: 7
            break;
        case ATS_KEY_J: // Default: 8
            break;
        case ATS_KEY_K: // Default: 9
            break;
        case ATS_KEY_L: // Default: 0
            _tasc制御中 = true;
            _tasc.起動();
            break;
        }
    }

    void Main::キー放し(int)
    {
    }

    void Main::戸閉()
    {
    }

    void Main::戸開()
    {
        _tasc.駅到着();
    }

    void Main::信号現示変化(int)
    {
    }

    void Main::地上子通過(const ATS_BEACONDATA & 地上子)
    {
        _状態.地上子通過(地上子);
        _tasc.地上子通過(地上子, _状態);
    }

    ATS_HANDLES Main::経過(const ATS_VEHICLESTATE & 状態, int *, int *)
    {
        _状態.経過(状態);
        _tasc.経過(_状態);

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

        _状態.出力(ハンドル位置);
        return ハンドル位置;
    }

}
