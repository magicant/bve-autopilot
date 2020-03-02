// Main.h : プラグイン全体を統括します
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
#include <unordered_map>
#include "ato.h"
#include "tasc.h"
#include "共通状態.h"
#include "音声出力.h"

namespace autopilot
{

    class Main
    {
    public:
        Main();
        ~Main();

        const 共通状態 & 状態() const { return _状態; }
        const tasc & tasc状態() const { return _tasc; }
        const ato & ato状態() const { return _ato; }
        bool tasc有効() const { return _tasc有効; }
        bool ato有効() const { return _ato有効; }
        mps 現在制限速度() const;
        mps 現在常用パターン速度() const;
        mps 現在orp照査速度() const;
        bool 力行抑止中() const { return _ato.力行抑止中(); }

        void 車両仕様設定(const ATS_VEHICLESPEC & 車両仕様)
        {
            _状態.車両仕様設定(車両仕様);
        }
        void リセット(int 制動状態);
        void 設定ファイル読込(LPCWSTR 設定ファイル名) {
            _状態.設定ファイル読込(設定ファイル名);
            _tasc有効 = _状態.設定().tasc初期起動();
            _ato有効 = _状態.設定().ato初期起動();
        }

        void 逆転器操作(int ノッチ);
        void 力行操作(int ノッチ);
        void 制動操作(int ノッチ);
        void 警笛操作(int 警笛種類);
        void キー押し(int キー);
        void キー放し(int キー);

        void 戸閉();
        void 戸開();

        void 信号現示変化(int 信号指示);
        void 地上子通過(const ATS_BEACONDATA & 地上子);

        ATS_HANDLES 経過(const ATS_VEHICLESTATE & 状態, int * 出力値, int * 音声状態);

    private:
        共通状態 _状態;
        tasc _tasc;
        ato _ato;
        bool _tasc有効, _ato有効;
        std::vector<ATS_BEACONDATA> _通過済地上子;
        std::unordered_map<音声, 音声出力> _音声状態;

        void 地上子通過執行(m 直前位置);
    };

}
