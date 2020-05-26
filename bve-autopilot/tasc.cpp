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
#include <limits>
#include "区間.h"
#include "減速パターン.h"
#include "物理量.h"
#include "走行モデル.h"

namespace autopilot {

    namespace
    {

        constexpr m デフォルト最大許容誤差 = 0.5_m;
        constexpr 自動制御指令 緩解指令 =
            力行ノッチ{std::numeric_limits<unsigned>::max()};

    }

    tasc::tasc() :
        _停止位置一覧(),
        _次駅停止位置のある範囲(区間{m::無限大(), m::無限大()}),
        _調整した次駅停止位置(m::無限大()),
        _最大許容誤差(デフォルト最大許容誤差),
        _目標減速度(2.5_kmphps),
        _緩解(false),
        _出力ノッチ(緩解指令)
    {
    }

    void tasc::リセット()
    {
        _停止位置一覧.clear();
        _次駅停止位置のある範囲.set(区間{m::無限大(), m::無限大()});
        _調整した次駅停止位置 = m::無限大();
        _最大許容誤差 = デフォルト最大許容誤差;
        _緩解 = false;
    }

    void tasc::制動操作(const 共通状態 &状態)
    {
        if (!状態.戸閉()) {
            mps2 手動 = 状態.制動().減速度(状態.入力制動ノッチ());
            mps2 自動 = 状態.制動().減速度(_出力ノッチ.制動成分());
            if (手動 >= 自動) {
                _緩解 = true;
            }
        }
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
        _緩解 = false;
    }

    void tasc::地上子通過(
        const ATS_BEACONDATA &地上子, m 直前位置, const 共通状態 &状態)
    {
        switch (地上子.Type) {
        case 255: // TASC 目標停止位置設定
            停止位置を追加(static_cast<m>(地上子.Optional), 状態);
            break;
        case 1031: // TASC 停止位置許容誤差設定
            最大許容誤差を設定(static_cast<cm>(地上子.Optional));
            break;
        }

        if (!状態.戸閉()) {
            return; // 「停車場へ移動」時は無視する
        }
        switch (地上子.Type)
        {
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
        if (_緩解) {
            _出力ノッチ = 緩解指令;
            return;
        }

        m 名目の目標停止位置 = 目標停止位置();
        m 残距離 = 名目の目標停止位置 - 状態.現在位置();
        if (残距離 <= _最大許容誤差) {
            // 目標停止位置に近付いたらさっさと車両を止めるように
            // 目標停止位置を手前に接近させる
            走行モデル 減速モデル = 状態.現在走行状態();
            減速モデル.指定速度まで走行(0.0_mps, -0.3_kmphps);
            _調整した次駅停止位置 =
                std::min(_調整した次駅停止位置, 減速モデル.位置());
        }
        else
        {
            _調整した次駅停止位置 = m::無限大();
        }

        m 計算用目標停止位置 =
            std::min(名目の目標停止位置, _調整した次駅停止位置);
        mps2 計算用勾配影響 = 状態.進路勾配加速度(計算用目標停止位置);
        mps2 目標減速度 = 出力減速度(計算用目標停止位置, 計算用勾配影響, 状態);
        // 新しい減速度に追従する十分な時間があるなら新しい減速度を使う
        if (状態.現在速度() / 目標減速度 >= 5.0_s) {
            _目標減速度 = 目標減速度;
        }

        減速パターン パターン{ 計算用目標停止位置, 0.0_mps, _目標減速度 };
        _出力ノッチ = パターン.出力ノッチ(状態);

        if (残距離 > _最大許容誤差) {
            return; // まだ目標停止位置に十分近付いていない
        }

        if (!状態.停車中()) {
            // 目標停止位置に近付いたらもう力行しない
            _出力ノッチ = std::min(_出力ノッチ, 自動制御指令{});
        }
        else {
            // 停車中は制動し続ける
            _出力ノッチ = std::min(
                _出力ノッチ, 自動制御指令{状態.転動防止自動ノッチ()});
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

    bool tasc::制御中() const
    {
        return isfinite(_次駅停止位置のある範囲.get().始点);
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

    mps2 tasc::出力減速度(
        m 停止位置, mps2 勾配影響, const 共通状態 &状態) const
    {
        mps 現在速度 = 状態.現在速度();
        m 残距離 = 停止位置 - 状態.現在位置();
        mps2 減速度 =
            -走行モデル::距離と速度による加速度(残距離, 現在速度, 0.0_mps);

        // 短い時間で急に止まろうとするとブレーキの強さ加減が不安定に
        // なるので、8 秒以上かけて止まるようにする
        減速度 = std::max(減速度, 現在速度 / 8.0_s);

        // とはいえあまりにもゆっくりした減速度で止まるのも避ける
        減速度 = std::max(減速度, static_cast<mps2>(0.5_kmphps));

        減速度 += 勾配影響;

        // 高すぎる減速度は無理なので
        減速度 = std::min(減速度, 状態.目安減速度());

        return 減速度;
    }

}
