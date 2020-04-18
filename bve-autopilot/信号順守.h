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
#include <deque>
#include <limits>
#include <map>
#include "制御指令.h"
#include "制限グラフ.h"
#include "区間.h"
#include "物理量.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;

    class 信号順守
    {
    public:
        using 信号インデックス = int;

        enum class 発進方式 { 手動, 自動, };

        struct 閉塞型 {
            static constexpr 信号インデックス 無指示 =
                std::numeric_limits<信号インデックス>::min();

            信号インデックス 信号指示 = 無指示;
            mps 信号速度 = mps::無限大();
            区間 始点のある範囲 = 区間{m::無限大(), m::無限大()};
            int 信号インデックス一覧 = 0; // 信号現示受信地上子の値
            bool 停止解放 = false;
            // この閉塞の信号速度が 0 の時にだけ有効な制限速度の一覧
            std::map<m, mps> 停止信号前照査一覧;

            bool 通過済(m 位置) const { return 始点のある範囲.通過済(位置); }
            int 先行列車位置() const;

            void 制限グラフに制限区間を追加(
                制限グラフ &追加先グラフ,
                m 減速目標地点, m 始点_, mps 速度) const;
            void 制限グラフに追加(
                制限グラフ &信号グラフ, 制限グラフ &照査グラフ,
                m tasc目標停止位置, bool is_atc) const;

            void 信号速度更新(
                const std::map<信号インデックス, mps> &速度表);
            void 信号指示設定(
                信号インデックス 指示,
                const std::map<信号インデックス, mps> &速度表);
            void 状態更新(
                const ATS_BEACONDATA &地上子,
                const std::map<信号インデックス, mps> &速度表,
                bool 信号インデックスを更新する);
            void 停止信号前照査設定(const ATS_BEACONDATA &地上子, m 現在位置);
            void 統合(const 閉塞型 &統合元);
            void 先行列車位置から信号指示を推定(
                int 閉塞数, const std::map<信号インデックス, mps> &速度表);
        };

        // 7.5 km/h は C-ATS や CS-ATC ORP の 最低照査速度による。
        static constexpr mps 停止解放走行速度 = 7.5_kmph;

        信号順守();
        ~信号順守();

        void リセット();
        void 発進(発進方式 方式);
        void 信号現示変化(信号インデックス 指示);
        void tasc目標停止位置変化(区間 位置のある範囲);
        void atc事前減速を設定(bool 事前減速) noexcept {
            _atc事前減速 = 事前減速;
        }
        void 地上子通過(
            const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態);

        void 経過(const 共通状態 &状態);

        自動制御指令 出力ノッチ(const 共通状態 &状態) const;

        bool is_atc() const {
            return 10 <= _現在閉塞.信号指示 &&
                _現在閉塞.信号指示 < std::numeric_limits<int>::max();
            // リセット前後に std::numeric_limits<int>::max() が
            // 信号インデックスとして送られてくることがあるが無視する
        }
        bool 発進可能(const 共通状態 &状態) const;
        mps 現在制限速度(const 共通状態 &状態) const;
        mps 現在常用パターン速度(const 共通状態 &状態) const;

    private:
        std::map<信号インデックス, mps> _信号速度表;
        閉塞型 _現在閉塞;
        std::deque<閉塞型> _前方閉塞一覧;

        // どうせ tasc目標停止位置変化 がすぐ呼ばれるので初期値は何でも良い
        m _tasc目標停止位置 = {};

        bool _atc事前減速 = true;

        // 経過メソッドが呼ばれる度に毎回制限グラフを計算するのはメモリに
        // 優しくないので予め計算しておく
        制限グラフ _信号グラフ, _照査グラフ;

        void 信号速度更新();
        閉塞型 *信号現示受信(
            const ATS_BEACONDATA &地上子, m 直前位置,
            const 共通状態 &状態, bool 信号インデックスを更新する);
        void 前方閉塞信号を推定();
        void 信号グラフ再計算();
    };

}

#pragma warning(pop)
