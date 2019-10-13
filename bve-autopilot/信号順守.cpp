// 信号順守.cpp : 信号に従って速度を制御する機能
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
#include "信号順守.h"
#include <algorithm>
#include <cmath>
#include "tasc.h"
#include "信号前照査順守.h"
#include "共通状態.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr 距離型 許容誤差 = 4;

        void 信号速度設定(
            std::map<信号順守::信号インデックス, 速度型> &速度表, int 地上子値)
        {
            信号順守::信号インデックス 指示 = 地上子値 / 1000;
            if (指示 < 0 || 256 <= 指示) {
                return;
            }

            速度型 速度 = mps_from_kmph(地上子値 % 1000);
            速度表[指示] = 速度;
        }

        int atc停止出力ノッチ(const 共通状態 &状態)
        {
            加速度型 目標減速度 = 状態.現在速度() / 2.0;
            加速度型 勾配影響 = 状態.車両勾配加速度();
            加速度型 出力減速度 =
                std::max(目標減速度 + 勾配影響, mps_from_kmph(1.0));
            double 制動ノッチd = 状態.制動().自動ノッチ(出力減速度);
            int 制動ノッチi = static_cast<int>(std::ceil(制動ノッチd));
            return -std::min(制動ノッチi, 状態.制動().自動ノッチ数());
        }

    }

    速度型 信号順守::閉塞型::走行速度() const
    {
        if (!停止解放) {
            return 信号速度;
        }
        return std::max(信号速度, 停止解放走行速度);
    }

    /// 先行列車がいる閉塞(のうち最も近いもの)を推定
    int 信号順守::閉塞型::先行列車位置() const
    {
        int 閉塞数 = 0, i = 信号インデックス一覧;
        for (;;) {
            if (i == 0) {
                return -1;
            }
            if (i % 10 == 信号指示) {
                return 閉塞数;
            }
            閉塞数++, i /= 10;
        }
    }

    void 信号順守::閉塞型::信号指示設定(
        信号インデックス 指示,
        const std::map<信号インデックス, 速度型> &速度表)
    {
        信号指示 = 指示;

        auto i = 速度表.find(指示);
        if (i != 速度表.end()) {
            信号速度 = i->second;
        }
    }

    void 信号順守::閉塞型::状態更新(
        const ATS_BEACONDATA &地上子,
        const 共通状態 &状態,
        const std::map<信号インデックス, 速度型> &速度表,
        bool 信号インデックスを更新する)
    {
        始点 = 状態.現在位置() + 地上子.Distance;
        if (信号インデックスを更新する && 地上子.Optional > 0) {
            信号インデックス一覧 = 地上子.Optional;
        }
        信号指示設定(地上子.Signal, 速度表);
    }

    void 信号順守::閉塞型::統合(const 閉塞型 &統合元)
    {
        // 現在閉塞の信号指示は常に信号現示変化で受け取った値を使用する。
        // よって信号速度もここでは更新しない。

        始点 = 統合元.始点;
        信号インデックス一覧 = 統合元.信号インデックス一覧;
    }

    void 信号順守::閉塞型::先行列車位置から信号指示を推定(
        int 閉塞数, const std::map<信号インデックス, 速度型> &速度表)
    {
        int i = 信号インデックス一覧;
        if (i == 0) {
            return;
        }
        while (閉塞数 > 0) {
            閉塞数--;
            if (i / 10 > 0) {
                i /= 10;
            }
        }

        int 指示 = i % 10;
        auto t = 速度表.find(指示);
        if (t != 速度表.end()) {
            auto 速度 = t->second;
            if (速度 > 信号速度) {
                信号指示 = 指示;
                信号速度 = 速度;
            }
        }
    }

    信号順守::信号順守() = default;
    信号順守::~信号順守() = default;

    void 信号順守::リセット()
    {
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

        // ATC ではリセット後に信号現示変化が来ないことがあるので
        // 現在閉塞の状態を維持する
        if (!is_atc()) {
            _現在閉塞 = 閉塞型{};
        }

        _前方閉塞一覧.clear();
        信号グラフ再計算();
    }

    void 信号順守::発進()
    {
        // 停止信号で止まった時は現示が変わってもそれを認識できていない可能性が
        // あるので少し制限を緩めて進む。7.5 km/h は C-ATS や CS-ATC ORP の
        // 最低照査速度による。
        constexpr 速度型 許容速度 = mps_from_kmph(7.5);
        _現在閉塞.信号速度 = std::max(_現在閉塞.信号速度, 許容速度);
        _現在閉塞.停止解放 = true;
        if (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.begin()->second;
            次閉塞.信号速度 = std::max(次閉塞.信号速度, 許容速度);
            次閉塞.停止解放 = true;
        }
        信号グラフ再計算();
    }

    void 信号順守::信号現示変化(信号インデックス 指示)
    {
        _現在閉塞.信号指示設定(指示, _信号速度表);
        _現在閉塞.始点 = -std::numeric_limits<距離型>::infinity();
        if (is_atc()) {
            // 前方閉塞の現示も上がっている可能性が高いが
            // 推測は無理なのできれいさっぱり忘れる
            _前方閉塞一覧.clear();
        }
        else {
            前方閉塞信号を推定();
        }
        信号グラフ再計算();
    }

    void 信号順守::地上子通過(
        const ATS_BEACONDATA &地上子, const 共通状態 &状態)
    {
        switch (地上子.Type)
        {
        case 3: // 信号現示受信 (各種 ATS-P プラグイン互換)
            if (状態.互換モード() == 互換モード型::swp2) {
                信号現示受信(地上子, 状態, false);
            }
            break;
        case 31: // 信号現示受信 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                信号現示受信(地上子, 状態, false);
            }
            break;
        case 1016: // 停止信号前速度設定
            信号現示受信(地上子, 状態, false);
            break;
        case 1011: // 信号速度設定
            信号速度設定(_信号速度表, 地上子.Optional);
            信号速度更新();
            信号グラフ再計算();
            break;
        case 1012: // 信号現示受信
            信号現示受信(地上子, 状態, true);
            break;
        }
    }

    void 信号順守::経過(const 共通状態 &状態)
    {
        // 通過済みの閉塞を現在閉塞に統合して消す
        while (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.begin()->second;
            if (!次閉塞.通過済(状態.現在位置() - 許容誤差)) {
                break;
            }
            _現在閉塞.統合(次閉塞);
            _前方閉塞一覧.erase(_前方閉塞一覧.begin());
            前方閉塞信号を推定();
            信号グラフ再計算();
        }
    }

    int 信号順守::出力ノッチ(const 共通状態 &状態,
        const tasc &tasc, const 信号前照査順守 &照査) const
    {
        距離型 停止マージン;
        時間型 時間マージン;
        if (is_atc()) {
            if (_現在閉塞.信号速度 == 0) {
                return atc停止出力ノッチ(状態);
            }

            停止マージン = -1;
            時間マージン = -1;
        }
        else {
            if (tasc.目標停止位置() < 停止信号位置() ||
                照査.制限速度(停止信号位置()) == 0)
            {
                停止マージン = 0;
            }
            else {
                停止マージン = 51;
            }
            時間マージン = 4;
        }
        return _信号グラフ.出力ノッチ(状態, 時間マージン, 停止マージン);
    }

    const 信号順守::閉塞型 &信号順守::閉塞(距離型 位置) const
    {
        auto i = _前方閉塞一覧.lower_bound(位置 + 許容誤差);
        if (i == _前方閉塞一覧.begin()) {
            return _現在閉塞;
        }
        --i;
        return i->second;
    }

    速度型 信号順守::現在制限速度(const 共通状態 &状態) const
    {
        return _信号グラフ.制限速度(状態.現在範囲());
    }

    速度型 信号順守::現在常用パターン速度(const 共通状態 &状態) const
    {
        return _信号グラフ.現在常用パターン速度(状態);
    }

    void 信号順守::信号速度更新()
    {
        _現在閉塞.信号指示設定(_現在閉塞.信号指示, _信号速度表);
        for (auto &b : _前方閉塞一覧) {
            閉塞型 &閉塞 = b.second;
            閉塞.信号指示設定(閉塞.信号指示, _信号速度表);
        }
    }

    void 信号順守::信号現示受信(
        const ATS_BEACONDATA &地上子, const 共通状態 &状態,
        bool 信号インデックスを更新する)
    {
        if (地上子.Distance == 0 && 状態.現在速度() != 0) {
            // マップファイルのバージョンが古いとおかしなデータが来ることがある
            return;
        }

        距離型 位置 = 状態.現在位置() + 地上子.Distance;
        auto i = _前方閉塞一覧.lower_bound(位置 - 許容誤差);
        閉塞型 &閉塞 =
            i != _前方閉塞一覧.end() &&
            i->second.始点 < 位置 + 許容誤差 ?
            i->second :
            _前方閉塞一覧[位置];
        閉塞.状態更新(地上子, 状態, _信号速度表, 信号インデックスを更新する);
        信号グラフ再計算();
    }

    void 信号順守::前方閉塞信号を推定()
    {
        int 位置 = _現在閉塞.先行列車位置();
        if (位置 < 0) {
            return;
        }

        for (auto &b : _前方閉塞一覧) {
            閉塞型 &閉塞 = b.second;
            if (--位置 < 0) {
                return;
            }
            閉塞.先行列車位置から信号指示を推定(位置, _信号速度表);
        }
    }

    void 信号順守::信号グラフ再計算()
    {
        _信号グラフ.消去();
        _信号グラフ.制限区間追加(_現在閉塞.始点, _現在閉塞.走行速度());
        for (const auto &b : _前方閉塞一覧) {
            const 閉塞型 &閉塞 = b.second;
            _信号グラフ.制限区間追加(閉塞.始点, 閉塞.走行速度());
        }
    }

    距離型 信号順守::停止信号位置() const
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

}
