// tasc.cpp : TASC メインモジュール
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
#include "tasc.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include "出力制御.h"
#include "区間.h"
#include "減速目標.h"
#include "物理量.h"
#include "運動状態.h"

namespace autopilot {

    namespace
    {

        constexpr m デフォルト最大許容誤差 = 0.5_m;

    }

    tasc::tasc() :
        _停止位置一覧(),
        _次駅停止位置のある範囲(区間{m::無限大(), m::無限大()}),
        _調整した次駅停止位置(m::無限大()),
        _最大許容誤差(デフォルト最大許容誤差),
        _制御リセット時刻(s::無限大()),
        _緩解時刻(s::無限大()),
        _停止位置リセット時刻(s::無限大()),
        _出力ノッチ()
    {
    }

    void tasc::リセット()
    {
        _停止位置一覧.clear();
        _次駅停止位置のある範囲.set(区間{m::無限大(), m::無限大()});
        _調整した次駅停止位置 = m::無限大();
        _最大許容誤差 = デフォルト最大許容誤差;
        _制御リセット時刻 = _緩解時刻 = _停止位置リセット時刻 =
            static_cast<時刻>(s::無限大());
    }

    void tasc::戸閉(const 共通状態 &状態)
    {
        auto 現在位置 = 状態.現在位置();
        auto 現在駅停止位置 = _次駅停止位置のある範囲.get().終点;
        if (isfinite(現在駅停止位置)) {
            現在位置 = std::max(現在位置, 現在駅停止位置);
        }
        auto i = _停止位置一覧.upper_bound(現在位置);
        m 次の停止位置 = (i != _停止位置一覧.end()) ? *i : m::無限大();
        _次駅停止位置のある範囲.set(区間{次の停止位置, 次の停止位置});

        _調整した次駅停止位置 = m::無限大();
        _制御リセット時刻 = _緩解時刻 = _停止位置リセット時刻 =
            static_cast<時刻>(s::無限大());
    }

    void tasc::地上子通過(
        const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態)
    {
        switch (地上子.Type) {
        case 255: // TASC 目標停止位置設定
            停止位置を追加(static_cast<m>(地上子.Optional), 状態);
            break;
        case 200: // TASC 目標停止位置設定（追加:小田急PIのATO互換）
            if (状態.互換モード() == 互換モード型::小田急d_ats_p ||
                状態.互換モード() == 互換モード型::小田急cs_atc)
            {
                停止位置を追加(static_cast<m>(地上子.Optional / 10), 状態);
            }
            break;
        case 1031: // TASC 停止位置許容誤差設定
            最大許容誤差を設定(static_cast<cm>(地上子.Optional));
            break;
        case 1032: // TASC 駅出発時刻設定
            停止位置リセット時刻を設定(地上子.Optional);
            break;
        }

        // 「停車場へ移動」時は無視する
        if (!状態.戸閉()) {
            return;
        }
        if (状態.現在位置() == 0.0_m && 状態.現在速度() == 0.0_mps) {
            return;
        }

        switch (地上子.Type)
        {
        case 21:
        case 22:
        case 23:// TASC 目標停止位置設定（追加:メトロ対応TASC互換）
            if (状態.互換モード() == 互換モード型::メトロtasc) {
                次駅停止位置を設定(
                    static_cast<m>(地上子.Optional % 1000), 直前位置, 状態);
            }
            break;
        case 30: // TASC 目標停止位置設定
            if (状態.互換モード() == 互換モード型::汎用ats) {
                次駅停止位置を設定(
                    static_cast<m>(地上子.Optional / 1000), 直前位置, 状態);
            }
            break;
        case 1030: // TASC 目標停止位置設定
            次駅停止位置を設定(
                static_cast<m>(地上子.Optional / 1000.0), 直前位置, 状態);
            break;
        }
    }

    void tasc::経過(const 共通状態 & 状態)
    {
        constexpr auto 最低減速度 = 0.3_kmphps;

        m 名目の目標停止位置 = 目標停止位置();
        m 残距離 = 名目の目標停止位置 - 状態.現在位置();
        if (残距離 <= _最大許容誤差) {
            // 目標停止位置に近付いたらさっさと車両を止めるように
            // 目標停止位置を手前に接近させる
            運動状態 減速モデル = 状態.現在走行状態();
            減速モデル.指定速度まで走行(0.0_mps, -最低減速度);
            _調整した次駅停止位置 =
                std::min(_調整した次駅停止位置, 減速モデル.位置);
        }
        else
        {
            _調整した次駅停止位置 = m::無限大();
        }

        m 計算用目標停止位置 =
            std::min(名目の目標停止位置, _調整した次駅停止位置);
        減速目標 目標{計算用目標停止位置, 0.0_mps, 状態.目安減速度()};
        自動制御指令 前回ノッチ =
            std::exchange(_出力ノッチ, 出力制御::出力ノッチ(目標, 状態));

        if (残距離 > _最大許容誤差) {
            return; // まだ目標停止位置に十分近付いていない
        }

        if (!状態.停車中()) {
            // 目標停止位置に近付いたらもう力行しない
            _出力ノッチ = std::min(_出力ノッチ, 自動制御指令{});

            // さらに、一定以上のブレーキをかけたらもう緩解しない
            自動制御指令 最低ノッチ =
                状態.制動().自動ノッチ(最低減速度).ceil();
            if (前回ノッチ <= 最低ノッチ) {
                _出力ノッチ = std::min(_出力ノッチ, 最低ノッチ);
            }
        }
        else {
            イベント発動(イベント::停止, 状態);
            if (状態.現在時刻() < _緩解時刻) {
                // 目標停止位置に停止したらしばらくブレーキをかけ続ける
                _出力ノッチ = std::min(
                    _出力ノッチ, 自動制御指令{状態.転動防止自動ノッチ()});
            }
            else {
                // 時間がたったらブレーキをやめる
                _出力ノッチ = 状態.最大力行ノッチ();
            }

            if (状態.現在時刻() >= _停止位置リセット時刻) {
                戸閉(状態);
            }
        }
    }

    m tasc::目標停止位置() const {
        // 大抵の路線データでは停止位置は整数なので
        // 範囲に整数があればそれを優先する
        auto &範囲 = _次駅停止位置のある範囲.get();
        m 整数位置 = ceil(範囲.始点);
        if (整数位置 <= 範囲.終点) {
            return 整数位置;
        }
        return 範囲.始点;
    }

    bool tasc::制御中(時刻 現在時刻) const
    {
        return 現在時刻 < _制御リセット時刻 &&
            isfinite(_次駅停止位置のある範囲.get().始点);
    }

    bool tasc::定位置内(m 位置) const
    {
        m 残距離 = 目標停止位置() - 位置;
        return abs(残距離) <= _最大許容誤差;
    }

    bool tasc::発進可能(const 共通状態 &状態) const
    {
        m 名目の目標停止位置 = 目標停止位置();
        m 残距離 = 名目の目標停止位置 - 状態.現在位置();
        return 残距離 > _最大許容誤差;
    }

    bool tasc::インチング可能(const 共通状態 &状態) const
    {
        if (!状態.停車中()) {
            return false;
        }
        if (!状態.戸閉()) {
            return false;
        }
        if (状態.入力逆転器ノッチ() <= 0) {
            return false;
        }
        if (状態.入力力行ノッチ() != 0) {
            return false;
        }
        if (状態.入力制動ノッチ() != 手動制動自然数ノッチ{0}) {
            return false;
        }

        m 残距離 = 目標停止位置() - 状態.現在位置();
        if (残距離 > 5.0_m) {
            return false; // 目標停止位置が未設定か遠すぎる
        }
        if (残距離 <= _最大許容誤差) {
            return false; // もう十分目標停止位置に近いか超えている
        }

        return true;
    }

    void tasc::停止位置を追加(m 停止位置, const 共通状態 &状態)
    {
        if (停止位置 < 状態.現在位置()) {
            return;
        }

        _停止位置一覧.insert(停止位置);
        if (状態.戸閉() && 停止位置 < _次駅停止位置のある範囲.get().始点) {
            _次駅停止位置のある範囲.set(区間{停止位置, 停止位置});
        }
    }

    void tasc::次駅停止位置を設定(m 残距離, m 直前位置, const 共通状態 &状態)
    {
        区間 受信した目標停止位置のある範囲 =
            安全マージン付き区間(直前位置, 状態.現在位置(), 残距離);
        assert(!受信した目標停止位置のある範囲.空である());

        if (状態.現在速度() == 0.0_mps &&
            受信した目標停止位置のある範囲.長さ() > 1.0_m)
        {
            // 「停車場へ移動」で地上子をすっ飛ばしたときは
            // 範囲の長さがとんでもなく長くなったりするので無視する
            return;
        }

        // これまでに分かっている範囲と組み合わせる
        区間 絞り込んだ範囲 = 重なり(
            受信した目標停止位置のある範囲, _次駅停止位置のある範囲.get());

        if (絞り込んだ範囲.空である()) {
            // これまで位置が設定されていなかったか、あるいは新しい位置が
            // これまでの位置と全然違っている場合 → これまでのことは無視する
            絞り込んだ範囲 = 受信した目標停止位置のある範囲;
        }

        if (!絞り込んだ範囲.含む(_次駅停止位置のある範囲.get())) {
            _次駅停止位置のある範囲.set(絞り込んだ範囲);
        }
    }

    void tasc::最大許容誤差を設定(m 最大許容誤差)
    {
        _最大許容誤差 = std::max(0.0_m, 最大許容誤差);
        _調整した次駅停止位置 = std::max(
            _調整した次駅停止位置, 目標停止位置() - _最大許容誤差);
    }

    void tasc::停止位置リセット時刻を設定(int 地上子値)
    {
        _停止位置リセット時刻 = static_cast<時刻>(
            地上子値 < 0 ? -s::無限大() : static_cast<s>(地上子値));
    }

    void tasc::手動ブレーキ条件ならイベント発動(const 共通状態 &状態)
    {
        if (!状態.戸閉()) {
            mps2 手動 = 状態.制動().減速度(状態.入力制動ノッチ());
            mps2 自動 = 状態.制動().減速度(_出力ノッチ.制動成分());
            if (手動 >= 自動) {
                イベント発動(イベント::手動ブレーキ, 状態);
            }
        }
    }

    void tasc::イベント発動(イベント イベント, const 共通状態 &状態)
    {
        if (状態.設定().tasc制御リセット条件().タイミング == イベント) {
            _制御リセット時刻 = std::min(
                _制御リセット時刻,
                状態.現在時刻() + 状態.設定().tasc制御リセット条件().遅延);
        }
        if (状態.設定().tasc緩解条件().タイミング == イベント) {
            _緩解時刻 = std::min(
                _緩解時刻, 状態.現在時刻() + 状態.設定().tasc緩解条件().遅延);
        }
    }

}
