// 信号順守.h : 信号に従って速度を制御する機能
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
#include "制限グラフ.h"
#include "単位.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;

    class 信号順守
    {
    public:
        using 信号インデックス = int;

        struct 閉塞型 {
            信号インデックス 信号指示 = -1;
            速度型 信号速度 = std::numeric_limits<速度型>::infinity();
            距離型 始点 = std::numeric_limits<距離型>::infinity();
            int 信号インデックス一覧 = 0; // 信号現示受信地上子の値
            bool 停止解放 = false;
            // この閉塞の信号速度が 0 の時にだけ有効な制限速度の一覧
            std::map<距離型, 速度型> 停止信号前照査一覧;

            bool 通過済(距離型 位置) const { return 始点 < 位置; }
            int 先行列車位置() const;

            void 制限グラフに制限区間を追加(
                制限グラフ &追加先グラフ, 距離型 始点_, 速度型 速度) const;
            void 制限グラフに追加(
                制限グラフ &追加先グラフ, 距離型 tasc目標停止位置) const;

            void 信号速度更新(
                const std::map<信号インデックス, 速度型> &速度表);
            void 信号指示設定(
                信号インデックス 指示,
                const std::map<信号インデックス, 速度型> &速度表);
            void 状態更新(
                const ATS_BEACONDATA &地上子,
                const 共通状態 &状態,
                const std::map<信号インデックス, 速度型> &速度表,
                bool 信号インデックスを更新する);
            void 停止信号前照査設定(
                const ATS_BEACONDATA &地上子, 距離型 現在位置);
            void 統合(const 閉塞型 &統合元);
            void 先行列車位置から信号指示を推定(
                int 閉塞数, const std::map<信号インデックス, 速度型> &速度表);
        };

        // 7.5 km/h は C-ATS や CS-ATC ORP の 最低照査速度による。
        static constexpr 速度型 停止解放走行速度 = mps_from_kmph(7.5);

        static constexpr bool atc用の信号指示である(信号インデックス 指示) {
            return 10 <= 指示 &&
                指示 < std::numeric_limits<信号インデックス>::max();
            // リセット前後に std::numeric_limits<信号インデックス>::max() が
            // 信号インデックスとして送られてくることがあるがそれは無視する
        }

        信号順守();
        ~信号順守();

        void リセット();
        void 発進();
        void 信号現示変化(信号インデックス 指示);
        void tasc目標停止位置変化(距離型 位置);
        void 地上子通過(const ATS_BEACONDATA &地上子, const 共通状態 &状態);

        void 経過(const 共通状態 &状態);

        // 力行は正の値、制動は負の値
        int 出力ノッチ(const 共通状態 &状態) const;

        bool is_atc() const {
            return atc用の信号指示である(_現在閉塞.信号指示);
        }
        速度型 現在制限速度(const 共通状態 &状態) const;
        速度型 現在常用パターン速度(const 共通状態 &状態) const;

    private:
        std::map<信号インデックス, 速度型> _信号速度表;
        閉塞型 _現在閉塞;
        std::map<距離型, 閉塞型> _前方閉塞一覧;

        // どうせ tasc目標停止位置変化 がすぐ呼ばれるので初期値は何でも良い
        距離型 _tasc目標停止位置 = 0.0;

        // 経過メソッドが呼ばれる度に毎回制限グラフを計算するのはメモリに
        // 優しくないので予め計算しておく
        制限グラフ _信号グラフ;

        void 信号速度更新();
        閉塞型 *信号現示受信(
            const ATS_BEACONDATA &地上子, const 共通状態 &状態,
            bool 信号インデックスを更新する);
        void 前方閉塞信号を推定();
        void 信号グラフ再計算();
    };

}

#pragma warning(pop)
