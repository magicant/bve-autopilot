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
#include <algorithm>
#include <limits>

namespace autopilot
{

    Main::Main() :
        _状態{},
        _tasc{},
        _ato{},
        _tasc有効{false},
        _ato有効{false}
    {
    }

    速度型 Main::現在制限速度() const
    {
        return _ato.現在制限速度(_状態);
    }

    速度型 Main::現在常用パターン速度() const
    {
        return _ato.現在常用パターン速度(_状態);
    }

    void Main::リセット(int)
    {
        _状態.リセット();
        _ato.リセット();
        _tasc有効 = _ato有効 = true;
    }

    void Main::逆転器操作(int ノッチ)
    {
        _状態.逆転器操作(ノッチ);
        _tasc.レバー操作(_状態);
    }

    void Main::力行操作(int ノッチ)
    {
        _状態.力行操作(ノッチ);
        _tasc.レバー操作(_状態);
    }

    void Main::制動操作(int ノッチ)
    {
        _状態.制動操作(ノッチ);
        _tasc.レバー操作(_状態);
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
            if (_状態.逆転器ノッチ() == 0 &&
                _状態.制動ノッチ() > _状態.制動().常用ノッチ数())
            {
                // ATO/TASC 有効・無効切り替え
                if (_ato有効) {
                    _ato有効 = false;
                }
                else if (_tasc有効) {
                    _tasc有効 = false;
                }
                else {
                    _ato有効 = _tasc有効 = true;
                }
            }
            if (_ato有効) {
                _ato.発進();
            }
            break;
        }
    }

    void Main::キー放し(int)
    {
    }

    void Main::戸閉()
    {
        _状態.戸閉(true);
        _tasc.戸閉();
    }

    void Main::戸開()
    {
        _状態.戸閉(false);
    }

    void Main::信号現示変化(int 信号指示)
    {
        _ato.信号現示変化(信号指示, _状態);
    }

    void Main::地上子通過(const ATS_BEACONDATA & 地上子)
    {
        _状態.地上子通過(地上子);
        _tasc.地上子通過(地上子, _状態);
        _ato.地上子通過(地上子, _状態);
    }

    ATS_HANDLES Main::経過(
        const ATS_VEHICLESTATE & 状態, int * 出力値, int *)
    {
        _状態.経過(状態);
        _tasc.経過(_状態);
        _ato.経過(_状態, _tasc);

        // TASC と ATO の出力ノッチをまとめる
        int 力行 = -1, 制動 = -1;
        if (_tasc有効) {
            int tascノッチ = _tasc.出力ノッチ();
            if (tascノッチ <= 0) {
                制動 = -tascノッチ;
            }
            else {
                //力行 = tascノッチ; // TASC が力行することは無い
            }
        }
        if (_ato有効) {
            int atoノッチ = _ato.出力ノッチ();
            if (atoノッチ <= 0) {
                制動 = std::max(制動, -atoノッチ);
            }
            else {
                力行 = atoノッチ;
            }
        }

        if (_状態.制動ノッチ() > 0 || _状態.逆転器ノッチ() <= 0) {
            力行 = -1;
        }

        ATS_HANDLES ハンドル位置;
        ハンドル位置.Brake = std::max(制動, _状態.制動ノッチ());
        ハンドル位置.Power =
            制動 >= 0 ? 0 : std::max(力行, _状態.力行ノッチ());
        ハンドル位置.Reverser = _状態.逆転器ノッチ();
        ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;

        _状態.出力(ハンドル位置);

        for (auto パネル出力 : _状態.設定().パネル出力対象登録簿()) {
            出力値[パネル出力.first] = パネル出力.second.出力(*this);
        }

        return ハンドル位置;
    }

}
