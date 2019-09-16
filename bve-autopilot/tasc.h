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
#include "共通状態.h"
#include "単位.h"
#include "走行モデル.h"

namespace autopilot {

    class tasc
    {
    public:
        tasc();
        ~tasc() = default;

        void リセット();
        void 制動操作(const 共通状態 &状態);
        void 戸閉();
        void 地上子通過(const ATS_BEACONDATA & 地上子, const 共通状態 & 状態);
        void 経過(const 共通状態 & 状態);

        距離型 目標停止位置() const { return _名目の目標停止位置; }
        bool 制御中() const;

        // 力行は正の値、制動は負の値
        int 出力ノッチ() const { return _出力ノッチ; }

    private:
        距離型 _名目の目標停止位置;
        距離型 _調整した目標停止位置;
        距離型 _直前目標停止位置受信位置;
        加速度型 _目標減速度;
        int _出力ノッチ;

        void 目標停止位置を設定(距離型 残距離, const 共通状態 &状態);
        // 直前のフレームとの現在位置変化に従い、目標停止位置が整数である
        // 可能性があるなら整数に丸める
        void 目標停止位置を補正(const 共通状態 &状態);

        加速度型 出力減速度(
            距離型 停止位置, 加速度型 勾配影響, const 共通状態 &状態) const;
    };

}
