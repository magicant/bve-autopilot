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
#include <functional>
#include <limits>

namespace autopilot
{

    namespace
    {

        制動指令 出力制動指令(
            手動制動自然数ノッチ 手動ノッチ, 自動制動自然数ノッチ 自動ノッチ,
            const 制動特性 &制動)
        {
            if (制動.非常ブレーキである(手動ノッチ)) {
                return 手動ノッチ; // 非常ブレーキは常に優先する
            }
            if (自動ノッチ == 自動制動自然数ノッチ{0}) {
                return 手動ノッチ;
            }

            mps2 手動 = 制動.減速度(手動ノッチ);
            mps2 自動 = 制動.減速度(自動ノッチ);
            if (手動 >= 自動) {
                return 手動ノッチ;
            }
            else {
                return 制動.指令(自動ノッチ);
            }
        }

    }

    Main::Main() :
        _状態{},
        _tasc{},
        _ato{},
        _tasc有効{true},
        _ato有効{true},
        _通過済地上子{},
        _音声状態{}
    {
        _tasc.目標停止位置を監視([&](区間 位置のある範囲) {
            _ato.tasc目標停止位置変化(位置のある範囲);
        });
    }

    Main::~Main()
    {
        _tasc.目標停止位置を監視(nullptr);
    }

    mps Main::現在制限速度() const
    {
        return _ato.現在制限速度(_状態);
    }

    mps Main::現在常用パターン速度() const
    {
        return _ato.現在常用パターン速度(_状態);
    }

    mps Main::現在orp照査速度() const
    {
        return _ato.現在orp照査速度();
    }

    void Main::リセット(int)
    {
        _状態.リセット();
        _tasc.リセット();
        _ato.リセット();
        _通過済地上子.clear();

        for (const auto &i : _状態.設定().音声割り当て()) {
            _音声状態[i.first].次に出力(ATS_SOUND_STOP);
        }
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
            if (_状態.入力逆転器ノッチ() == 0 &&
                _状態.制動().非常ブレーキである(_状態.入力制動ノッチ()))
            {
                if (_ato有効) {
                    _ato有効 = false;
                    _音声状態[音声::ato無効設定音].次に出力(ATS_SOUND_PLAY);
                }
                else if (_tasc有効) {
                    _tasc有効 = false;
                    _音声状態[音声::tasc無効設定音].次に出力(ATS_SOUND_PLAY);
                }
                else {
                    _ato有効 = _tasc有効 = true;
                    _音声状態[音声::ato有効設定音].次に出力(ATS_SOUND_PLAY);
                }
            }
        }

        // ATO 発進
        if (キー == _状態.設定().キー割り当て().at(キー操作::ato発進)) {
            if (_ato有効) {
                _ato.発進(_状態, ato::発進方式::手動);
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
        _ato.信号現示変化(信号指示);
    }

    void Main::地上子通過(const ATS_BEACONDATA & 地上子)
    {
        if (_状態.現在速度() == 0.0_mps) {
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
        const ATS_VEHICLESTATE &状態, int *出力値, int *音声状態)
    {
        _状態.経過(状態);
        for (const ATS_BEACONDATA &地上子 : _通過済地上子) {
            地上子通過執行(地上子);
        }
        _通過済地上子.clear();
        _通過済地上子.shrink_to_fit();

        // ATO 自動発進
        if (_ato有効 && _状態.自動発進可能な時刻である()) {
            _ato.発進(_状態, ato::発進方式::自動);
        }

        _tasc.経過(_状態);
        _ato.経過(_状態);

        // TASC と ATO の出力ノッチをまとめる
        自動制御指令 自動ノッチ = _状態.最大力行ノッチ();
        if (_tasc有効) {
            自動ノッチ = std::min(自動ノッチ, _tasc.出力ノッチ());
        }
        if (_ato有効) {
            自動ノッチ = std::min(自動ノッチ, _ato.出力ノッチ());
        }

        if (!_ato有効 || _状態.入力制動ノッチ() > 手動制動自然数ノッチ{0} ||
            _状態.入力逆転器ノッチ() <= 0)
        {
            自動ノッチ = std::min(自動ノッチ, 自動制御指令{力行ノッチ{0}});
        }

        ATS_HANDLES ハンドル位置;
        ハンドル位置.Brake = 出力制動指令(
            _状態.入力制動ノッチ(), 自動ノッチ.制動成分(), _状態.制動()).value;
        if (_状態.入力力行ノッチ() >= 0) {
            ハンドル位置.Power = std::max(
                static_cast<int>(自動ノッチ.力行成分().value),
                _状態.入力力行ノッチ());
        }
        else {
            ハンドル位置.Power = _状態.入力力行ノッチ();
        }
        ハンドル位置.Reverser = _状態.入力逆転器ノッチ();
        ハンドル位置.ConstantSpeed = ATS_CONSTANTSPEED_CONTINUE;

        _状態.出力(ハンドル位置);

        for (auto パネル出力 : _状態.設定().パネル出力対象登録簿()) {
            出力値[パネル出力.first] = パネル出力.second.出力(*this);
        }
        for (const auto &i : _状態.設定().音声割り当て()) {
            音声状態[i.second] = _音声状態[i.first].出力();
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
