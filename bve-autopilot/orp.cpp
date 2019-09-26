// orp.cpp : ORP の照査に抵触しないように減速します
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
#include "orp.h"
#include <cassert>
#include <cmath>
#include <limits>
#include "信号順守.h"
#include "共通状態.h"
#include "単位.h"
#include "走行モデル.h"

namespace autopilot
{

    namespace
    {

        constexpr 距離型 無限遠 = std::numeric_limits<距離型>::infinity();
        constexpr orp::信号インデックス orp信号インデックス = 35;
        constexpr 速度型 照査速度下限 = mps_from_kmph(7.5);
        constexpr 速度型 最終目標速度 = 照査速度下限 - mps_from_kmph(0.5);
        constexpr 速度型 運転速度マージン = mps_from_kmph(4);

    }

    orp::orp() :
        _信号指示{-1},
        _照査パターン{無限遠, 照査速度下限, 0, 0},
        _運転パターン{無限遠, 最終目標速度, 0, 0},
        _出力ノッチ{std::numeric_limits<int>::max()},
        _照査速度{std::numeric_limits<速度型>::infinity()}
    {
    }

    void orp::リセット()
    {
        _照査パターン.目標位置 = _運転パターン.目標位置 = 無限遠;
        _出力ノッチ = std::numeric_limits<int>::max();
    }

    void orp::設定(速度型 初期照査速度, 距離型 初期位置, 距離型 限界位置)
    {
        加速度型 照査減速度 = -走行モデル::距離と速度による加速度(
            限界位置 - 初期位置, 初期照査速度, 0);
        走行モデル 照査{初期位置, 初期照査速度};
        照査.指定速度まで走行(照査速度下限, -照査減速度);
        _照査パターン.目標位置 = 照査.位置();
        _照査パターン.初期減速度 = 照査減速度;
        _照査パターン.最終減速度 = 照査減速度;
        assert(_照査パターン.目標速度 == 照査速度下限);

        照査.指定速度まで走行(
            最終目標速度 + 運転速度マージン / 2, -照査減速度);

        距離型 減速終了位置 = 照査.位置();
        速度型 初期運転速度 = 初期照査速度 - 運転速度マージン;
        加速度型 運転減速度 = -走行モデル::距離と速度による加速度(
            減速終了位置 - 初期位置, 初期運転速度, 最終目標速度);
        _運転パターン.目標位置 = 減速終了位置;
        _運転パターン.初期減速度 = 運転減速度;
        _運転パターン.最終減速度 = 運転減速度 / 2;
        assert(_運転パターン.目標速度 == 最終目標速度);
    }

    void orp::信号現示変化(信号インデックス 指示)
    {
        _信号指示 = 指示;

        if (指示 != orp信号インデックス) {
            リセット();
        }
    }

    void orp::地上子通過(const ATS_BEACONDATA &地上子,
        const 共通状態 &状態, const 信号順守 &信号)
    {
        if (状態.互換モード() != 互換モード型::メトロ総合) {
            リセット();
            return;
        }

        switch (地上子.Type) {
        case 12: { // ORP 動作開始 (メトロ総合プラグイン互換)
            速度型 初速度 = 信号.現在制限速度(状態);
            距離型 残距離 = 地上子.Optional <= 48 ? 48 : 79;
            設定(初速度, 状態.現在位置(), 状態.現在位置() + 残距離);
            break;
        }
        case 31: // 信号現示受信 (メトロ総合プラグイン互換)
        case 1012: // 信号現示受信
            if (地上子.Signal == orp信号インデックス &&
                地上子.Distance > 0)
            {
                速度型 初速度 = 信号.現在制限速度(状態);
                距離型 開始位置 = 状態.現在位置() + 地上子.Distance;
                距離型 orp距離 = 初速度 <= mps_from_kmph(30) ? 48 : 79;
                設定(初速度, 開始位置, 開始位置 + orp距離);
            }
            break;
        }
    }

    void orp::経過(const 共通状態 &状態)
    {
        if (制御中() && 状態.現在速度() < 最終目標速度) {
            // パターンを照査速度下限に固定する
            _照査パターン.目標位置 = _運転パターン.目標位置 = -無限遠;
        }

        _出力ノッチ =
            制御中() ?
            _運転パターン.出力ノッチ(状態.現在位置(), 状態.現在速度(), 状態) :
            std::numeric_limits<int>::max();
        _照査速度 = _照査パターン.期待速度(状態.現在位置());
    }

    bool orp::制御中() const
    {
        return _運転パターン.目標位置 < 無限遠;
    }

    bool orp::照査中() const
    {
        return _信号指示 == orp信号インデックス && 制御中();
    }

}
