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
#include "信号順守.h"
#include "制御指令.h"
#include "制限グラフ.h"
#include "区間.h"
#include "急動作抑制.h"
#include "早着防止.h"
#include "物理量.h"

namespace autopilot
{

    class 共通状態;

    class ato
    {
    public:
        using 信号インデックス = 信号順守::信号インデックス;
        using 発進方式 = 信号順守::発進方式;

        enum class 制御状態 {
            一時停止, //! ATO が一時的に動作していない
            停止, //! 列車が停止している
            発進, //! 列車を発進させようとしている
            走行, //! 列車が走行している
        };

        static bool 発進可能(const 共通状態 &状態) noexcept;

        ato();
        ~ato();

        void リセット();
        void 発進(const 共通状態 &状態, 発進方式 方式);
        void 信号現示変化(信号インデックス 指示) {
            _信号.信号現示変化(指示);
        }
        void atc事前減速を設定(bool 事前減速) noexcept {
            _信号.atc事前減速を設定(事前減速);
        }
        void tasc目標停止位置変化(区間 位置のある範囲) {
            _信号.tasc目標停止位置変化(位置のある範囲);
        }
        void 逆転器操作(const 共通状態 &状態) noexcept {
            レバー操作(状態);
        }
        void 力行操作(const 共通状態 &状態) noexcept {
            レバー操作(状態);
        }
        void 制動操作(const 共通状態 &状態) noexcept {
            レバー操作(状態);
        }
        void 地上子通過(
            const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態);
        void 経過(const 共通状態 &状態);

        mps 現在制限速度(const 共通状態 &状態) const;
        mps 現在常用パターン速度(const 共通状態 &状態) const;
        mps 現在orp照査速度(const 共通状態 &状態) const noexcept {
            return _信号.orp照査速度(状態);
        }
        bool 力行抑止中() const noexcept {
            return _早着防止.出力ノッチ() <= 力行ノッチ{1};
        }

        制御状態 状態() const noexcept { return _制御状態; }
        自動制御指令 出力ノッチ() const noexcept { return _出力ノッチ; }

    private:
        制限グラフ _制限速度1006, _制限速度1007,
            _制限速度6, _制限速度8, _制限速度9, _制限速度10;
        信号順守 _信号;
        早着防止 _早着防止;
        制御状態 _制御状態 = 制御状態::走行;
        自動制御指令 _出力ノッチ;
        急動作抑制 _急動作抑制;
        bool _リセット直後 = true;

        void レバー操作(const 共通状態 &状態) noexcept;
    };

}
