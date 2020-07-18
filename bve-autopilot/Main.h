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
#include "稼働状態.h"
#include "共通状態.h"
#include "音声出力.h"

namespace autopilot
{

    class Main
    {
    public:
        Main();
        ~Main();

        const 共通状態 &状態() const noexcept { return _状態; }
        const tasc &tasc状態() const noexcept { return _tasc; }
        const ato &ato状態() const noexcept { return _ato; }
        稼働状態 現在稼働状態() const noexcept { return *_稼働状態; }
        bool tasc有効() const noexcept {
            return autopilot::tasc有効(現在稼働状態());
        }
        bool ato有効() const noexcept {
            return autopilot::ato有効(現在稼働状態());
        }
        bool インチング中() const noexcept {
            return _インチング状態 != インチング状態::切;
        }
        mps 現在制限速度() const {
            return _ato.現在制限速度(_状態);
        }
        mps 現在常用パターン速度() const {
            return _ato.現在常用パターン速度(_状態);
        }
        mps 現在orp照査速度() const noexcept {
            return _ato.現在orp照査速度(_状態);
        }
        bool ato一時停止中() const noexcept {
            return ato有効() && _ato.状態() == ato::制御状態::一時停止;
        }
        bool 力行抑止中() const noexcept { return _ato.力行抑止中(); }

        void 車両仕様設定(const ATS_VEHICLESPEC &車両仕様) {
            _状態.車両仕様設定(車両仕様);
        }
        void リセット(int 制動状態);
        void 設定ファイル読込(LPCWSTR 設定ファイル名);

        void 逆転器操作(int ノッチ) noexcept;
        void 力行操作(int ノッチ) noexcept;
        void 制動操作(int ノッチ);
        void 警笛操作(int /*警笛種類*/) noexcept { }
        void キー押し(int キー);
        void キー放し(int キー) { _状態.キー放し(キー); }

        void 戸閉() {
            _状態.戸閉(true);
            _tasc.戸閉(_状態);
        }
        void 戸開() noexcept{
            _状態.戸閉(false);
            _tasc.戸開(_状態);
        }

        void 信号現示変化(int 信号指示) { _ato.信号現示変化(信号指示); }
        void 地上子通過(const ATS_BEACONDATA & 地上子);

        ATS_HANDLES 経過(
            const ATS_VEHICLESTATE &状態, int *出力値, int *音声状態);

    private:
        enum class インチング状態
        {
            切, //! インチングを行っていない
            発進, //! 列車を発進させようとしている
            走行, //! 列車が走行している
        };

        共通状態 _状態;
        tasc _tasc;
        ato _ato;
        std::vector<稼働状態>::const_iterator _稼働状態;
        インチング状態 _インチング状態;
        std::vector<ATS_BEACONDATA> _通過済地上子;
        std::unordered_map<音声, 音声出力> _音声状態;

        void モード切替(bool 順方向, bool ループ);
        void 地上子通過執行(m 直前位置);
    };

}
