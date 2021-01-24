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
#include <cassert>
#include <cmath>
#include <iterator>
#include <numeric>
#include <utility>
#include "共通状態.h"
#include "区間.h"

#pragma warning(disable:4819)

namespace autopilot
{

    namespace
    {

        constexpr m 目前距離 = 1.0_m;

        void 信号速度設定(
            std::map<信号順守::信号インデックス, mps> &速度表, int 地上子値)
        {
            信号順守::信号インデックス 指示 = 地上子値 / 1000;
            if (指示 < 0 || 256 <= 指示) {
                return;
            }

            mps 速度 = static_cast<kmph>(地上子値 % 1000);
            速度表[指示] = 速度;
        }

        自動制御指令 atc停止出力ノッチ(const 共通状態 &状態)
        {
            constexpr mps2 最小減速度 = 1.0_kmphps;
            mps2 目標減速度 = 状態.現在速度() / 2.0_s;
            mps2 勾配加速度 = 状態.車両勾配加速度();
            mps2 出力減速度 = std::max(目標減速度, 最小減速度) + 勾配加速度;
            自動制動実数ノッチ 制動ノッチ実数 =
                状態.制動().自動ノッチ(出力減速度);
            自動制動自然数ノッチ 制動ノッチ = 制動ノッチ実数.ceil();
            return std::min(制動ノッチ, 状態.制動().自動最大ノッチ());
        }

        struct 範囲比較
        {
            constexpr bool operator()(const 区間 &a, const 区間 &b)
            {
                return a.終点 < b.始点;
            }
            constexpr bool operator()(const 区間 &a, const 信号順守::閉塞型 &b)
            {
                return (*this)(a, b.始点のある範囲);
            }
            constexpr bool operator()(const 信号順守::閉塞型 &a, const 区間 &b)
            {
                return (*this)(a.始点のある範囲, b);
            }
        };

        std::deque<信号順守::閉塞型>::iterator 対応する閉塞(
            区間 始点のある範囲, std::deque<信号順守::閉塞型> &閉塞一覧)
        {
            // 始点のある範囲が重なる閉塞を全て求める
            auto [i, j] = std::equal_range(
                閉塞一覧.begin(), 閉塞一覧.end(), 始点のある範囲, 範囲比較());

            switch (std::distance(i, j)) {
            case 0:
            { // 重なる範囲がなければ新しく作る
                i = 閉塞一覧.emplace(i);
                i->始点のある範囲 = 始点のある範囲;
                break;
            }
            case 1:
            { // 重なる範囲が一つだけならそれと統合する
                auto 新しい範囲 = 重なり(始点のある範囲, i->始点のある範囲);
                if (新しい範囲.空である()) {
                    // ここには来ないはずだけど念のため
                    新しい範囲 = 始点のある範囲;
                }
                i->始点のある範囲 = 新しい範囲;
                break;
            }
            default:
            { // 複数の候補があるときはとりあえず最初の閉塞を選んでおく
                break;
            }
            }

            return i;
        }

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

    void 信号順守::閉塞型::制限グラフに制限区間を追加(
        制限グラフ &追加先グラフ,
        m 減速目標地点, m 始点_, mps 速度) const
    {
        if (停止解放) {
            速度 = std::max(速度, 停止解放走行速度);
        }
        追加先グラフ.制限区間追加(減速目標地点, 始点_, 速度);
    }

    void 信号順守::閉塞型::制限グラフに追加(
        制限グラフ &信号グラフ, 制限グラフ &照査グラフ,
        m tasc目標停止位置, bool is_atc) const
    {
        m 減速目標地点 = 始点のある範囲.始点;

        if (信号速度 == mps::無限大()) {
            return;
        }

        if (信号速度 == 0.0_mps) {
            m 停止位置 = tasc目標停止位置;

            for (const auto &照査 : 停止信号前照査一覧) {
                m 照査位置 = 照査.first;
                mps 照査速度 = 照査.second;
                制限グラフに制限区間を追加(
                    照査グラフ, 照査位置, 照査位置, 照査速度);
                if (照査速度 == 0.0_mps) {
                    停止位置 = std::min(停止位置, 照査位置);
                }
            }

            if (is_atc) {
                // ちょっと次の閉塞に入ったところで止める
                減速目標地点 += 1.0_m;
            }
            else if (減速目標地点 <= 停止位置) {
                // 閉塞境界ギリギリではなくある程度手前に止める
                減速目標地点 -= 51.0_m;
            }
        }
        else {
            // 実際には 1 秒くらい早く制限速度まで減速しきることが多いので
            // 1 秒分遅れてブレーキがかかるように目標位置をずらす
            if (is_atc) {
                減速目標地点 += 1.0_s * 信号速度;
            }
            else {
                減速目標地点 -= 4.0_s * 信号速度;
            }
        }

        制限グラフに制限区間を追加(
            信号グラフ, 減速目標地点, 始点のある範囲.始点, 信号速度);
    }

    bool 信号順守::閉塞型::経過(const 共通状態 &状態)
    {
        bool グラフ更新 = false;

        if (状態.現在時刻() >= 開通時刻) {
            開通時刻 = 時刻{s::無限大()};
            停止解放 = true;
            グラフ更新 = true;
        }

        orp.経過(状態);

        return グラフ更新;
    }

    void 信号順守::閉塞型::信号速度更新(
        const std::map<信号インデックス, mps> &速度表)
    {
        auto i = 速度表.find(信号指示);
        if (i != 速度表.end()) {
            信号速度 = i->second;
        }
    }

    void 信号順守::閉塞型::信号指示設定(
        信号インデックス 指示,
        const std::map<信号インデックス, mps> &速度表)
    {
        信号指示 = 指示;
        停止解放 = false;
        信号速度更新(速度表);
    }

    void 信号順守::閉塞型::状態更新(
        const ATS_BEACONDATA &地上子,
        const std::map<信号インデックス, mps> &速度表,
        bool 信号インデックスを更新する)
    {
        if (信号インデックスを更新する && 地上子.Optional > 0) {
            信号インデックス一覧 = 地上子.Optional;
        }
        信号指示設定(地上子.Signal, 速度表);
    }

    void 信号順守::閉塞型::停止信号前照査設定(
        const ATS_BEACONDATA &地上子, m 現在位置)
    {
        m 位置 = 現在位置 + static_cast<m>(地上子.Optional / 1000);
        mps 速度 = static_cast<kmph>(地上子.Optional % 1000);
        停止信号前照査一覧[位置] = 速度;
    }

    void 信号順守::閉塞型::orp状態更新(mps 直前閉塞速度) noexcept
    {
        if (信号指示 == orp::orp信号インデックス) {
            orp.設定(直前閉塞速度, 始点のある範囲.始点);
        }
        else {
            orp.リセット();
        }
    }

    void 信号順守::閉塞型::統合(閉塞型 &&統合元) noexcept
    {
        // 現在閉塞の信号指示は常に信号現示変化で受け取った値を使用する。
        // よって信号速度もここでは更新しない。

        始点のある範囲 = std::move(統合元.始点のある範囲);
        信号インデックス一覧 = std::move(統合元.信号インデックス一覧);
        停止解放 = std::move(統合元.停止解放);
        停止信号前照査一覧 = std::move(統合元.停止信号前照査一覧);

        if (信号指示 == orp::orp信号インデックス && !orp.制御中()) {
            orp = std::move(統合元.orp);
        }
    }

    void 信号順守::閉塞型::先行列車位置から信号指示を推定(
        int 閉塞数, const std::map<信号インデックス, mps> &速度表)
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
            {0, 0.0_kmph},
            {1, 25.0_kmph},
            {2, 40.0_kmph},
            {3, 65.0_kmph},
            {4, 85.0_kmph},
            {5, 160.0_kmph},
            {9, 0.0_kmph},
            {10, 0.0_kmph},
            {11, 10.0_kmph},
            {12, 10.0_kmph},
            {13, 15.0_kmph},
            {14, 20.0_kmph},
            {15, 25.0_kmph},
            {16, 30.0_kmph},
            {17, 35.0_kmph},
            {18, 40.0_kmph},
            {19, 45.0_kmph},
            {20, 50.0_kmph},
            {21, 55.0_kmph},
            {22, 60.0_kmph},
            {23, 65.0_kmph},
            {24, 70.0_kmph},
            {25, 75.0_kmph},
            {26, 80.0_kmph},
            {27, 85.0_kmph},
            {28, 90.0_kmph},
            {29, 95.0_kmph},
            {30, 100.0_kmph},
            {31, 105.0_kmph},
            {32, 110.0_kmph},
            {33, 120.0_kmph},
            {36, 0.0_kmph},
            {39, 45.0_kmph},
            {40, 40.0_kmph},
            {41, 35.0_kmph},
            {42, 30.0_kmph},
            {43, 25.0_kmph},
            {44, 20.0_kmph},
            {45, 15.0_kmph},
            {46, 10.0_kmph},
            {47, 10.0_kmph},
            {48, 01.0_kmph},
            {50, 0.0_kmph},
            {51, 25.0_kmph},
            {52, 40.0_kmph},
            {53, 65.0_kmph},
            {54, 100.0_kmph},
            {101, 0.0_kmph},
            {102, 0.0_kmph},
            {103, 15.0_kmph},
            {104, 25.0_kmph},
            {105, 45.0_kmph},
            {106, 55.0_kmph},
            {107, 65.0_kmph},
            {108, 75.0_kmph},
            {109, 90.0_kmph},
            {110, 100.0_kmph},
            {111, 110.0_kmph},
            {112, 120.0_kmph},
        };

        if (_現在閉塞.信号指示 == 閉塞型::無指示) {
            _現在閉塞 = 閉塞型{};
        }
        else {
            // リセット後に信号現示変化が来ないことがあるので
            // 現在閉塞の状態を維持する
            _現在閉塞 = 閉塞型{
                _現在閉塞.信号指示, _現在閉塞.信号速度, -m::無限大()};
        }

        _前方閉塞一覧.clear();
        信号グラフ再計算();
    }

    void 信号順守::発進(発進方式 方式)
    {
        if (方式 != 発進方式::手動) {
            return;
        }

        // 停止信号で止まった時は現示が変わってもそれを認識できていない可能性が
        // あるので少し制限を緩めて進む。
        _現在閉塞.停止解放 = true;
        if (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.front();
            次閉塞.停止解放 = true;
        }
        信号グラフ再計算();
    }

    void 信号順守::信号現示変化(信号インデックス 指示)
    {
        _現在閉塞.信号指示設定(指示, _信号速度表);
        _現在閉塞.始点のある範囲.始点 = -m::無限大();
        _現在閉塞.始点のある範囲.終点 = -m::無限大();

        if (指示 != orp::orp信号インデックス) {
            _現在閉塞.orp.リセット();
        }

        if (is_atc()) {
            // 前方閉塞の現示も上がっている可能性が高いが
            // 推測は無理なのできれいさっぱり忘れる
            // ただし ORP のことは覚えておく
            if (!_前方閉塞一覧.empty() &&
                !_前方閉塞一覧.front().orp.制御中())
            {
                _前方閉塞一覧.clear();
            }
        }
        else {
            前方閉塞信号を推定();
        }

        信号グラフ再計算();
    }

    void 信号順守::tasc目標停止位置変化(区間 位置のある範囲)
    {
        _tasc目標停止位置 = 位置のある範囲.始点;
        信号グラフ再計算();
    }

    void 信号順守::地上子通過(
        const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態)
    {
        switch (地上子.Type)
        {
        case 3: // 信号現示受信 (各種 ATS-P プラグイン互換)
            if (状態.互換モード() == 互換モード型::swp2) {
                信号現示受信(地上子, 直前位置, 状態, false);
            }
            break;
        case 12: // ORP 起動 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                if (_現在閉塞.信号指示 == orp::orp信号インデックス) {
                    _現在閉塞.orp.設定(地上子.Optional, 直前位置);
                }
            }
            break;
        case 31: // 信号現示受信 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                信号現示受信(地上子, 直前位置, 状態, false);
            }
            break;
        case 1016: // 停止信号前速度設定
            if (閉塞型 *閉塞 = 信号現示受信(地上子, 直前位置, 状態, false)) {
                閉塞->停止信号前照査設定(地上子, 直前位置);
                信号グラフ再計算();
            }
            break;
        case 1011: // 信号速度設定
            信号速度設定(_信号速度表, 地上子.Optional);
            信号速度更新();
            信号グラフ再計算();
            break;
        case 1012: // 信号現示受信
            信号現示受信(地上子, 直前位置, 状態, true);
            break;
        case 1018: //信号開通時刻設定
            if (閉塞型 *閉塞 = 信号現示受信(地上子, 直前位置, 状態, false)) {
                閉塞->開通時刻 =
                    static_cast<時刻>(static_cast<s>(地上子.Optional));
            }
            break;
        }
    }

    void 信号順守::経過(const 共通状態 &状態)
    {
        bool グラフ更新 = false;

        // 通過済みの閉塞を現在閉塞に統合して消す
        while (!_前方閉塞一覧.empty()) {
            閉塞型 &次閉塞 = _前方閉塞一覧.front();
            if (!次閉塞.通過済(状態.現在位置())) {
                break;
            }
            _現在閉塞.統合(std::move(次閉塞));
            _前方閉塞一覧.pop_front();
            前方閉塞信号を推定();
            グラフ更新 = true;
        }

        // 残っている閉塞を更新する
        グラフ更新 |= _現在閉塞.経過(状態);
        for (auto &閉塞 : _前方閉塞一覧) {
            グラフ更新 |= 閉塞.経過(状態);
        }

        if (グラフ更新) {
            信号グラフ再計算();
        }
    }

    自動制御指令 信号順守::出力ノッチ(const 共通状態 &状態) const
    {
        if (is_atc() && !_現在閉塞.停止解放 &&
            _現在閉塞.信号速度 == 0.0_mps)
        {
            return atc停止出力ノッチ(状態);
        }

        自動制御指令 ノッチ = std::min(
            _信号グラフ.出力ノッチ(状態), _照査グラフ.出力ノッチ(状態));

        // 停止位置が目前ならもう加速しない
        if (ノッチ > 自動制御指令{}) {
            m 目前 = 状態.現在位置() + 目前距離;
            if (!_信号グラフ.進行可能(目前) || !_照査グラフ.進行可能(目前)) {
                ノッチ = 自動制御指令{};
            }
        }

        if (状態.互換モード() == 互換モード型::メトロ総合) {
            // ORP の出力ノッチを取り込む
            ノッチ = std::min(ノッチ, _現在閉塞.orp.出力ノッチ());
            ノッチ = std::accumulate(
                _前方閉塞一覧.begin(), _前方閉塞一覧.end(), ノッチ,
                [](自動制御指令 ノッチ, const 閉塞型 &閉塞) {
                    return std::min(ノッチ, 閉塞.orp.出力ノッチ());
                });
        }

        return ノッチ;
    }

    bool 信号順守::自動発進可能(const 共通状態 &状態) const
    {
        m 目前 = 状態.現在位置() + 目前距離;
        return _信号グラフ.進行可能(目前) && _照査グラフ.進行可能(目前);
    }

    bool 信号順守::orp照査中(const 共通状態 &状態) const noexcept
    {
        return 状態.互換モード() == 互換モード型::メトロ総合 &&
            _現在閉塞.orp.制御中();
    }

    mps 信号順守::orp照査速度(const 共通状態 &状態) const noexcept
    {
        if (!orp照査中(状態)) {
            return mps::無限大();
        }
        return _現在閉塞.orp.照査速度();
    }

    mps 信号順守::現在制限速度(const 共通状態 &状態) const
    {
        区間 範囲 = 状態.現在範囲();
        return std::min(
            _信号グラフ.制限速度(範囲), _照査グラフ.制限速度(範囲));
    }

    mps 信号順守::現在常用パターン速度(const 共通状態 &状態) const
    {
        return std::min({
            _信号グラフ.現在常用パターン速度(状態),
            _照査グラフ.現在常用パターン速度(状態),
            orp照査速度(状態),
            });
    }

    void 信号順守::信号速度更新()
    {
        _現在閉塞.信号速度更新(_信号速度表);
        for (閉塞型 &閉塞 : _前方閉塞一覧) {
            閉塞.信号速度更新(_信号速度表);
        }
    }

    信号順守::閉塞型 *信号順守::信号現示受信(
        const ATS_BEACONDATA &地上子, m 直前位置,
        const 共通状態 &状態, bool 信号インデックスを更新する)
    {
        if (地上子.Distance == 0 && 状態.現在速度() != 0.0_mps) {
            // マップファイルのバージョンが古いとおかしなデータが来ることがある
            return nullptr;
        }

        if (直前位置 == 0.0_m) {
            // リセット直後は現在位置を信用する
            直前位置 = 状態.現在位置();
        }

        m 残距離 = static_cast<m>(地上子.Distance);
        区間 受信した閉塞始点のある範囲 =
            安全マージン付き区間(直前位置, 状態.現在位置(), 残距離);
        assert(!受信した閉塞始点のある範囲.空である());

        auto 閉塞 = 対応する閉塞(受信した閉塞始点のある範囲, _前方閉塞一覧);
        閉塞->状態更新(地上子, _信号速度表, 信号インデックスを更新する);

        if (状態.互換モード() == 互換モード型::メトロ総合) {
            const auto &直前閉塞 =
                閉塞 == _前方閉塞一覧.begin() ? _現在閉塞 : *std::prev(閉塞);
            閉塞->orp状態更新(直前閉塞.信号速度);

            auto 直後閉塞 = std::next(閉塞);
            if (直後閉塞 != _前方閉塞一覧.end()) {
                直後閉塞->orp状態更新(閉塞->信号速度);
            }
        }

        信号グラフ再計算();
        return &*閉塞;
    }

    void 信号順守::前方閉塞信号を推定()
    {
        int 位置 = _現在閉塞.先行列車位置();
        if (位置 < 0) {
            return;
        }

        for (閉塞型 &閉塞 : _前方閉塞一覧) {
            if (--位置 < 0) {
                return;
            }
            閉塞.先行列車位置から信号指示を推定(位置, _信号速度表);
        }
    }

    void 信号順守::信号グラフ再計算()
    {
        _信号グラフ.消去();
        _照査グラフ.消去();

        bool atc = is_atc();
        _信号グラフ.事前減速を設定(!atc || _atc事前減速);

        _現在閉塞.制限グラフに追加(
            _信号グラフ, _照査グラフ, _tasc目標停止位置, atc);
        for (const 閉塞型 &閉塞 : _前方閉塞一覧) {
            閉塞.制限グラフに追加(
                _信号グラフ, _照査グラフ, _tasc目標停止位置, atc);
        }
    }

}
