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
        _tasc有効{true},
        _ato有効{true},
        _通過済地上子{}
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

    速度型 Main::現在orp照査速度() const
    {
        return _ato.現在orp照査速度();
    }

    void Main::リセット(int)
    {
        _状態.リセット();
        _tasc.リセット();
        _ato.リセット();
        _通過済地上子.clear();
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
        _tasc.制動操作(_状態);
    }

    void Main::警笛操作(int)
    {
    }

    void Main::キー押し(int キー)
    {
        // モード切替
        if (キー == _状態.設定().キー割り当て().at(キー操作::モード切替)) {
            if (_状態.逆転器ノッチ() == 0 &&
                _状態.制動ノッチ() > _状態.制動().常用ノッチ数())
            {
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
        }

        // ATO 発進
        if (キー == _状態.設定().キー割り当て().at(キー操作::ato発進)) {
            if (_ato有効) {
                _ato.発進(_状態);
            }
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
        if (_状態.現在速度() == 0) {
            // 「停車場へ移動」の時は、移動先地点までの地上子をそれぞれ
            // 通過するがまだ経過メソッドが呼ばれていないので位置計算が
            // 狂う。通過メソッドが呼ばれるまで地上子を処理せず溜めておく。
            _通過済地上子.push_back(地上子);
        }
        else
        {
            地上子通過執行(地上子);
        }
    }

    ATS_HANDLES Main::経過(
        const ATS_VEHICLESTATE & 状態, int * 出力値, int *)
    {
        _状態.経過(状態);
        for (const ATS_BEACONDATA &地上子 : _通過済地上子) {
            地上子通過執行(地上子);
        }
        _通過済地上子.clear();
        _通過済地上子.shrink_to_fit();

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

        if (_状態.制動ノッチ() > 0 || 制動 >= 0 || _状態.逆転器ノッチ() <= 0) {
            力行 = -1;
        }

        ATS_HANDLES ハンドル位置;
        ハンドル位置.Brake = std::max(制動, _状態.制動ノッチ());
        if (_状態.力行ノッチ() >= 0) {
            ハンドル位置.Power = std::max(力行, _状態.力行ノッチ());
        }
        else {
            ハンドル位置.Power = _状態.力行ノッチ();
        }
        ハンドル位置.Reverser = _状態.逆転器ノッチ();
        ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;

        _状態.出力(ハンドル位置);

        for (auto パネル出力 : _状態.設定().パネル出力対象登録簿()) {
            出力値[パネル出力.first] = パネル出力.second.出力(*this);
        }

        return ハンドル位置;
    }

    void Main::地上子通過執行(const ATS_BEACONDATA &地上子)
    {
        _状態.地上子通過(地上子);
        _tasc.地上子通過(地上子, _状態);
        _ato.地上子通過(地上子, _状態);
    }

}
