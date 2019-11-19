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
#include <cmath>
#include <limits>
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
        _名目の目標停止位置(m::無限大()),
        _調整した目標停止位置(m::無限大()),
        _直前目標停止位置受信位置(m::quiet_NaN()),
        _最大許容誤差(デフォルト最大許容誤差),
        _目標減速度(2.5_kmphps),
        _緩解(false),
        _出力ノッチ(緩解指令)
    {
    }

    void tasc::リセット()
    {
        _名目の目標停止位置.set(m::無限大());
        _調整した目標停止位置 = m::無限大();
        _直前目標停止位置受信位置 = m::quiet_NaN();
        _最大許容誤差 = デフォルト最大許容誤差;
        _緩解 = false;
    }

    void tasc::制動操作(const 共通状態 &状態)
    {
        if (!状態.戸閉()) {
            mps2 手動 = 状態.制動().減速度(
                // FIXME Remove unnecessary cast
                手動制動自然数ノッチ{static_cast<unsigned>(状態.制動ノッチ())});
            mps2 自動 = 状態.制動().減速度(_出力ノッチ.制動成分());
            if (手動 >= 自動) {
                _緩解 = true;
            }
        }
    }

    void tasc::戸閉()
    {
        リセット();
    }

    void tasc::地上子通過(const ATS_BEACONDATA & 地上子, const 共通状態 & 状態)
    {
        switch (地上子.Type) {
        case 1031: // TASC 停止位置許容誤差設定
            最大許容誤差を設定(static_cast<cm>(地上子.Optional));
            break;
        }

        if (!状態.戸閉()) {
            return; // 「停車場へ移動」時は無視する
        }
        switch (地上子.Type)
        {
        case 17: // TASC 目標停止位置設定 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                目標停止位置を設定(11.0_m, 状態);
            }
            break;
        case 30: // TASC 目標停止位置設定
            if (状態.互換モード() == 互換モード型::汎用ats) {
                目標停止位置を設定(
                    static_cast<m>(地上子.Optional / 1000), 状態);
            }
            break;
        case 32: // TASC 目標停止位置設定 (メトロ総合プラグイン互換)
            if (状態.互換モード() == 互換モード型::メトロ総合) {
                目標停止位置を設定(500.0_m, 状態);
            }
            break;
        case 1030: // TASC 目標停止位置設定
            目標停止位置を設定(
                static_cast<m>(地上子.Optional / 1000), 状態);
            break;
        }
    }

    void tasc::経過(const 共通状態 & 状態)
    {
        if (_緩解) {
            _出力ノッチ = 緩解指令;
            return;
        }

        目標停止位置を補正(状態);

        m 残距離 = _名目の目標停止位置.get() - 状態.現在位置();
        if (残距離 <= _最大許容誤差) {
            // 目標停止位置に近付いたらさっさと車両を止めるように
            // 目標停止位置を手前に接近させる
            走行モデル 減速モデル = 状態.現在走行状態();
            減速モデル.指定速度まで走行(0.0_mps, -0.3_kmphps);
            _調整した目標停止位置 =
                std::min(_調整した目標停止位置, 減速モデル.位置());
        }
        else
        {
            _調整した目標停止位置 = m::無限大();
        }

        m 計算用目標停止位置 =
            std::min(_名目の目標停止位置.get(), _調整した目標停止位置);
        mps2 実勾配影響 = 状態.進路勾配加速度(計算用目標停止位置);
        mps2 計算用勾配影響 = std::max(実勾配影響, 0.0_mps2);
        mps2 目標減速度 =
            出力減速度(計算用目標停止位置, 計算用勾配影響, 状態) -
            計算用勾配影響;
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
                _出力ノッチ, 自動制御指令{-状態.転動防止自動ノッチ()});
        }
    }

    bool tasc::制御中() const
    {
        return isfinite(_名目の目標停止位置.get());
    }

    void tasc::目標停止位置を設定(m 残距離, const 共通状態 &状態)
    {
        m 現在位置 = 状態.現在位置();
        _名目の目標停止位置.set(現在位置 + 残距離);
        _直前目標停止位置受信位置 = 現在位置;
    }

    void tasc::目標停止位置を補正(const 共通状態 &状態)
    {
        if (isnan(_直前目標停止位置受信位置)) {
            return;
        }

        m 直前位置 = _直前目標停止位置受信位置;
        m 現在位置 = 状態.現在位置();
        m 最大誤差 = 現在位置 - 直前位置;
        m 補正前の目標停止位置 = _名目の目標停止位置.get();
        m 補正した目標停止位置 = ceil(補正前の目標停止位置);
        m 補正距離 = 補正した目標停止位置 - 補正前の目標停止位置;
        if (補正距離 <= 最大誤差) {
            _名目の目標停止位置.set(補正した目標停止位置);
        }

        _直前目標停止位置受信位置 = m::quiet_NaN();
    }

    void tasc::最大許容誤差を設定(m 最大許容誤差)
    {
        _最大許容誤差 = std::max(0.0_m, 最大許容誤差);
        _調整した目標停止位置 = std::max(
            _調整した目標停止位置, _名目の目標停止位置.get() - _最大許容誤差);
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
