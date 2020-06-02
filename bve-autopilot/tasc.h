// tasc.h : TASC メインモジュール
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
#include <set>
#include <utility>
#include "live.h"
#include "制御指令.h"
#include "共通状態.h"
#include "区間.h"
#include "物理量.h"
#include "走行モデル.h"

namespace autopilot {

    class tasc
    {
    public:
        tasc();
        ~tasc() = default;

        void リセット();
        void 制動操作(const 共通状態 &状態);
        void 戸閉(const 共通状態 &状態);
        void 地上子通過(
            const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態);
        void 経過(const 共通状態 & 状態);

        m 目標停止位置() const;
        bool 制御中() const;

        自動制御指令 出力ノッチ() const noexcept { return _出力ノッチ; }

        void 目標停止位置を監視(live<区間>::observer_type &&observer) {
            _次駅停止位置のある範囲.set_observer(std::move(observer));
        }

    private:
        std::set<m> _停止位置一覧;
        live<区間> _次駅停止位置のある範囲;
        m _調整した次駅停止位置;
        m _最大許容誤差;
        bool _緩解;
        自動制御指令 _出力ノッチ;

        void 停止位置を追加(m 停止位置, const 共通状態 &状態);
        void 次駅停止位置を設定(m 残距離, m 直前位置, const 共通状態 &状態);
        void 最大許容誤差を設定(m 最大許容誤差);
    };

}
