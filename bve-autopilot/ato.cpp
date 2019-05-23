// ato.cpp : ATO メインモジュール
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
#include "ato.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "tasc.h"
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        constexpr 距離型 許容誤差 = 4;

        void 制限区間追加(
            制限グラフ &グラフ, int 地上子値, const 共通状態 &状態)
        {
            距離型 距離 = 地上子値 / 1000;
            速度型 速度 = mps_from_kmph(地上子値 % 1000);
            距離型 位置 = 状態.現在位置() + 距離;

            if (速度 == 0) {
                速度 = std::numeric_limits<速度型>::infinity();
            }

            グラフ.制限区間追加(位置, 速度);
        }

        void 信号速度設定(
            std::map<ato::信号インデックス, 速度型> &速度表, int 地上子値)
        {
            ato::信号インデックス 指示 = 地上子値 / 1000;
            if (指示 < 0 || 256 <= 指示) {
                return;
            }

            速度型 速度 = mps_from_kmph(地上子値 % 1000);
            速度表[指示] = 速度;
        }

    }

    ato::ato() = default;
    ato::~ato() = default;

    void ato::リセット()
    {
        _制限速度1006.消去();
        _制限速度1007.消去();

        _信号速度表 = {
            {0, mps_from_kmph(0)},
            {1, mps_from_kmph(25)},
            {2, mps_from_kmph(40)},
            {3, mps_from_kmph(65)},
            {4, mps_from_kmph(85)},
            {5, mps_from_kmph(160)},
            {9, mps_from_kmph(0)},
            {10, mps_from_kmph(0)},
            {11, mps_from_kmph(10)},
            {12, mps_from_kmph(10)},
            {13, mps_from_kmph(15)},
            {14, mps_from_kmph(20)},
            {15, mps_from_kmph(25)},
            {16, mps_from_kmph(30)},
            {17, mps_from_kmph(35)},
            {18, mps_from_kmph(40)},
            {19, mps_from_kmph(45)},
            {20, mps_from_kmph(50)},
            {21, mps_from_kmph(55)},
            {22, mps_from_kmph(60)},
            {23, mps_from_kmph(65)},
            {24, mps_from_kmph(70)},
            {25, mps_from_kmph(75)},
            {26, mps_from_kmph(80)},
            {27, mps_from_kmph(85)},
            {28, mps_from_kmph(90)},
            {29, mps_from_kmph(95)},
            {30, mps_from_kmph(100)},
            {31, mps_from_kmph(105)},
            {32, mps_from_kmph(110)},
            {33, mps_from_kmph(120)},
            {36, mps_from_kmph(0)},
            {39, mps_from_kmph(45)},
            {40, mps_from_kmph(40)},
            {41, mps_from_kmph(35)},
            {42, mps_from_kmph(30)},
            {43, mps_from_kmph(25)},
            {44, mps_from_kmph(20)},
            {45, mps_from_kmph(15)},
            {46, mps_from_kmph(10)},
            {47, mps_from_kmph(10)},
            {48, mps_from_kmph(01)},
            {50, mps_from_kmph(0)},
            {51, mps_from_kmph(25)},
            {52, mps_from_kmph(40)},
            {53, mps_from_kmph(65)},
            {54, mps_from_kmph(100)},
            {101, mps_from_kmph(0)},
            {102, mps_from_kmph(0)},
            {103, mps_from_kmph(15)},
            {104, mps_from_kmph(25)},
            {105, mps_from_kmph(45)},
            {106, mps_from_kmph(55)},
            {107, mps_from_kmph(65)},
            {108, mps_from_kmph(75)},
            {109, mps_from_kmph(90)},
            {110, mps_from_kmph(100)},
            {111, mps_from_kmph(110)},
            {112, mps_from_kmph(120)},
        };

        _現在閉塞 = 閉塞型{};
        _前方閉塞一覧.clear();
        _信号グラフ.消去();
    }

    void ato::発進()
    {
        _発進中 = true;

        _現在閉塞.信号速度 = std::max(_現在閉塞.信号速度, mps_from_kmph(10));
        if (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.begin()->second;
            次閉塞.信号速度 = std::max(次閉塞.信号速度, mps_from_kmph(10));
        }
        信号グラフ再計算();
    }

    void ato::信号現示変化(信号インデックス 指示)
    {
        _現在閉塞.信号指示設定(指示, _信号速度表);
        信号グラフ再計算();
    }

    void ato::地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態)
    {
        switch (地上子.Type)
        {
        case 1006: // 制限速度設定
            制限区間追加(_制限速度1006, 地上子.Optional, 状態);
            break;
        case 1007: // 制限速度設定
            制限区間追加(_制限速度1007, 地上子.Optional, 状態);
            break;
        case 1011: // 信号速度設定
            信号速度設定(_信号速度表, 地上子.Optional);
            break;
        case 1012: { // 信号現示受信
            距離型 位置 = 状態.現在位置() + 地上子.Distance;
            auto i = _前方閉塞一覧.lower_bound(位置 - 許容誤差);
            閉塞型 &閉塞 =
                i != _前方閉塞一覧.end() &&
                i->second.始点 < 位置 + 許容誤差 ?
                i->second :
                _前方閉塞一覧[位置];
            閉塞.状態更新(地上子, 状態, _信号速度表);
            信号グラフ再計算();
            break;
        }
        }
    }

    void ato::経過(const 共通状態 &状態, const tasc &tasc)
    {
        距離型 最後尾 = 状態.現在位置() - 状態.列車長();
        _制限速度1006.通過(最後尾);
        _制限速度1007.通過(最後尾);

        // 通過済みの閉塞を現在閉塞に統合して消す
        while (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.begin()->second;
            if (!次閉塞.通過済(状態.現在位置() - 許容誤差)) {
                break;
            }
            _現在閉塞.統合(次閉塞);
            _前方閉塞一覧.erase(_前方閉塞一覧.begin());
            信号グラフ再計算();
        }

        if (状態.現在速度() > mps_from_kmph(1) ||
            状態.逆転器ノッチ() <= 0 ||
            状態.制動ノッチ() > 0 ||
            !状態.戸閉())
        {
            _発進中 = false;
        }
        if (!_発進中 && 状態.現在速度() <= mps_from_kmph(0.05)) {
            // 停車中は制動し続ける
            _出力ノッチ = -状態.車両仕様().BrakeNotches;
            return;
        }

        距離型 停止マージン = tasc.目標停止位置() < 停止信号位置() ? 0 : 51;
        _出力ノッチ = std::min({
            状態.車両仕様().PowerNotches,
            _制限速度1006.出力ノッチ(状態),
            _制限速度1007.出力ノッチ(状態),
            _信号グラフ.出力ノッチ(状態, 5, 停止マージン),
            });
    }

    速度型 ato::現在制限速度(const 共通状態 &状態) const
    {
        区間 列車範囲 = 状態.現在範囲();
        return std::min({
            _制限速度1006.制限速度(列車範囲),
            _制限速度1007.制限速度(列車範囲),
            _信号グラフ.制限速度(列車範囲),
            });
    }

    速度型 ato::現在常用パターン速度(const 共通状態 &状態) const
    {
        return std::min({
            _制限速度1006.現在常用パターン速度(状態),
            _制限速度1007.現在常用パターン速度(状態),
            _信号グラフ.現在常用パターン速度(状態),
            });
    }

    void ato::信号グラフ再計算()
    {
        _信号グラフ.消去();
        _信号グラフ.制限区間追加(_現在閉塞.始点, _現在閉塞.信号速度);
        for (const auto &b : _前方閉塞一覧) {
            const 閉塞型 &閉塞 = b.second;
            _信号グラフ.制限区間追加(閉塞.始点, 閉塞.信号速度);
        }
    }

    距離型 ato::停止信号位置() const
    {
        if (_現在閉塞.信号速度 == 0) {
            return _現在閉塞.始点;
        }
        for (const auto &b : _前方閉塞一覧) {
            const 閉塞型 &閉塞 = b.second;
            if (閉塞.信号速度 == 0) {
                return 閉塞.始点;
            }
        }
        return std::numeric_limits<距離型>::infinity();
    }

    void ato::閉塞型::信号指示設定(
        信号インデックス 指示,
        const std::map<信号インデックス, 速度型> &速度表)
    {
        信号指示 = 指示;

        auto i = 速度表.find(指示);
        if (i != 速度表.end()) {
            信号速度 = i->second;
        }
    }

    void ato::閉塞型::状態更新(
        const ATS_BEACONDATA &地上子,
        const 共通状態 &状態,
        const std::map<信号インデックス, 速度型> &速度表)
    {
        始点 = 状態.現在位置() + 地上子.Distance;
        信号指示設定(地上子.Signal, 速度表);
    }

    void ato::閉塞型::統合(const 閉塞型 &統合元)
    {
        // 現在閉塞の信号指示は常に信号現示変化で受け取った値を使用する。
        // よって信号速度もここでは更新しない。

        始点 = 統合元.始点;
    }

}
