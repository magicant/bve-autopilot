// 減速パターン.cpp : 等加速度で減速するパターンを表します
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
#include "減速パターン.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include "共通状態.h"
#include "走行モデル.h"

namespace autopilot
{

    namespace
    {
        constexpr mps3 終盤加加速度 = 0.5_kmphps2;
    }

    std::pair<mps, mps2> 減速パターン::期待速度と期待減速度(
        m 現在位置) const
    {
        if (現在位置 >= 目標位置) {
            return { 目標速度, 0.0_mps2 };
        }

        // 目標位置・目標速度の状態から逆算する
        走行モデル 走行(目標位置, 目標速度);

        if (初期減速度 > 最終減速度) {
            走行.等加加速度で指定加速度まで走行(
                -最終減速度, -初期減速度, 終盤加加速度);

            if (走行.位置() <= 現在位置) {
                // 行き過ぎたので計算し直す
                走行.変更(目標位置, 目標速度);
                mps2 期待減速度 = -走行.指定位置まで走行(
                    現在位置, -最終減速度, 終盤加加速度);
                return { 走行.速度(), 期待減速度 };
            }
        }

        // 現在位置がもっと手前なら初期減速度で減速する
        走行.指定位置まで走行(現在位置, -初期減速度);
        return { 走行.速度(), 初期減速度 };
    }

    mps2 減速パターン::出力減速度(m 現在位置, mps 現在速度) const
    {
        if (現在位置 >= 目標位置) {
            if (目標速度 > 0.0_mps) {
                return (現在速度 - 目標速度) / 2.0_s;
            }
            else {
                return mps2::無限大();
            }
        }

        mps 期待速度;
        mps2 期待減速度;
        std::tie(期待速度, 期待減速度) = 期待速度と期待減速度(現在位置);

        // 期待(減)速度に漸次的に近付けるように減速度を調整する。
        // 基本的には 2 秒後に期待速度に到達するような減速度を出力する。
        // ただし現在速度が期待速度と異なることにより
        // 期待速度の実際の減速度も期待減速度とは異なるので
        // 期待減速度に現在速度と期待速度の比を掛けて補正する。
        mps2 出力減速度1 =
            期待減速度 * (現在速度 / 期待速度) - (期待速度 - 現在速度) / 2.0_s;

        if (現在速度 > 期待速度) {
            // 目標位置までの途中でパターンに戻れるような減速度にする
            m 中間地点 = (目標位置 + 現在位置) / 2.0;
            mps 中間速度 = this->期待速度(中間地点);
            mps2 出力減速度2 = -走行モデル::距離と速度による加速度(
                中間地点 - 現在位置, 現在速度, 中間速度);

            if (目標速度 == 0.0_mps) {
                // 出力減速度1 だとわずかに過走することがあるので
                return 出力減速度2;
            }

            // 目標位置までまだ距離があるときは出力減速度2 で十分。
            // 一方、目標位置が目前に迫った時は出力減速度2 だと強すぎる。
            return std::min(出力減速度1, 出力減速度2);
        }

        if (目標速度 > 0.0_mps) {
            return 出力減速度1;
        }

        // 手前に止まってしまうような強い減速は避ける
        // (停止直前に「あと 2 秒で期待速度に到達」では間に合わない)
        mps2 出力減速度2 = -走行モデル::距離と速度による加速度(
            目標位置 - 現在位置, 現在速度, 0.0_mps);
        return std::min(出力減速度1, 出力減速度2);
    }

    自動制動自然数ノッチ 減速パターン::出力制動ノッチ(
        m 現在位置, mps 現在速度, 自動制動自然数ノッチ 現在制動ノッチ,
        mps2 勾配影響, const 共通状態 &状態) const
    {
        const 制動特性 &制動 = 状態.制動();
        mps2 出力減速度 = this->出力減速度(現在位置, 現在速度);
        自動制動実数ノッチ 出力実数 = 制動.自動ノッチ(出力減速度 + 勾配影響);

        if (目標位置 >= 現在位置) {
            // 小刻みに制動ノッチを変化させるのを防ぐ
            double 補正ノッチ =
                0.25 * std::min(1.0, 現在速度 / (5.0_s * 初期減速度));
            if (出力実数 >= 自動制動実数ノッチ{現在制動ノッチ}) {
                出力実数.value -= 補正ノッチ;
            }
            else {
                出力実数.value += 補正ノッチ;
            }

            出力実数 = 制動.自動ノッチ丸め(出力実数);
        }
        else {
            出力実数.value = std::ceil(出力実数.value);
        }

        if (出力実数.value > std::numeric_limits<int>::max()) {
            return 制動.自動最大ノッチ();
        }
        return std::clamp(
            自動制動自然数ノッチ{static_cast<unsigned>(出力実数.value)},
            自動制動自然数ノッチ{0},
            制動.自動最大ノッチ());
    }

    bool 減速パターン::力行する余裕あり(
        力行ノッチ 力行ノッチ, mps2 想定加速度, s 想定惰行時間,
        mps2 勾配影響, const 共通状態 &状態) const
    {
        走行モデル モデル = 状態.現在走行状態();

        // 現在状態から一定時間加速する動きをまずシミュレートする
        短く力行(モデル, 力行ノッチ, 想定加速度, 状態);

        // その後、惰行して……
        モデル.指定時間走行(想定惰行時間, 勾配影響);

        // ブレーキをかける必要があるなら力行しない
        auto 制動ノッチ = 出力制動ノッチ(
            モデル.位置(), モデル.速度(),
            自動制動自然数ノッチ{0}, 勾配影響, 状態);
        return 制動ノッチ == 自動制動自然数ノッチ{0};
    }

    自動制御指令 減速パターン::出力ノッチ(const 共通状態 &状態) const
    {
        mps 現在速度 = 状態.現在速度();
        自動制動自然数ノッチ 現在制動ノッチ =
            状態.制動().自動ノッチ(状態.前回出力().Brake);
        mps2 勾配影響 = 状態.車両勾配加速度();
        自動制動自然数ノッチ 出力制動ノッチ = this->出力制動ノッチ(
            状態.現在位置(), 現在速度, 現在制動ノッチ, 勾配影響, 状態);

        // 制動を弱めてもまたすぐ強くすることになるなら弱めない
        if (現在速度 >= static_cast<mps>(10.0_kmph) &&
            出力制動ノッチ < 現在制動ノッチ)
        {
            mps2 出力減速度 = 状態.制動().減速度(出力制動ノッチ) - 勾配影響;
            s 猶予 = 出力減速度 > 0.0_mps2 ?
                std::clamp(
                    (現在速度 - 目標速度) / 4.0 / 出力減速度, 1.0_s, 3.0_s) :
                1.0_s;
            走行モデル モデル = 状態.現在走行状態();
            モデル.指定時間走行(猶予, -出力減速度);
            自動制動自然数ノッチ 猶予後出力制動ノッチ = this->出力制動ノッチ(
                モデル.位置(), モデル.速度(), 出力制動ノッチ,
                勾配影響, 状態);
            if (猶予後出力制動ノッチ > 出力制動ノッチ) {
                出力制動ノッチ.value++;
            }
        }

        // 停止直前はノッチを緩めて衝撃を抑える
        if (目標速度 == 0.0_mps) {
            走行モデル 空走 = 状態.現在走行状態();
            空走.指定時間走行(状態.制動().反応時間(), 状態.加速度());
            走行モデル 緩め走行{目標位置};
            mps2 緩め減速度 = std::max(
                -緩め走行.指定速度まで走行(
                    空走.速度(), 0.0_mps2, 2.0_kmphps2, true),
                static_cast<mps2>(0.3_kmphps));
            自動制動実数ノッチ 緩め制動ノッチ実数 =
                状態.制動().自動ノッチ(緩め減速度 + 勾配影響);
            自動制動自然数ノッチ 緩め制動ノッチ{
                static_cast<unsigned>(std::ceil(緩め制動ノッチ実数.value))};
            出力制動ノッチ = std::min(出力制動ノッチ, 緩め制動ノッチ);
        }

        if (出力制動ノッチ > 自動制動自然数ノッチ{0}) {
            return 出力制動ノッチ;
        }

        // 制限速度まで余裕があるなら全力で力行する
        力行ノッチ 最大力行ノッチ = 状態.最大力行ノッチ();
        mps2 想定勾配影響 = std::max(勾配影響, 0.0_mps2);
        if (力行する余裕あり(
            最大力行ノッチ, 5.0_kmphps, 5.0_s, 想定勾配影響, 状態))
        {
            return 最大力行ノッチ;
        }

        // ある程度速度が出ているなら弱い力行ノッチは無意味なので
        constexpr 自動制御指令 惰行{};
        if (現在速度 >= static_cast<mps>(10.0_kmph)) {
            return 惰行;
        }

        // 止まりかけならとにかく前に進む
        constexpr 力行ノッチ 最弱力行ノッチ{1};
        if (現在速度 < static_cast<mps>(0.5_kmph)) {
            return 最弱力行ノッチ;
        }

        if (力行する余裕あり(
            最弱力行ノッチ, 2.5_kmphps, 1.0_s, 想定勾配影響, 状態))
        {
            return 最弱力行ノッチ;
        }
        return 惰行;
    }

    走行モデル 減速パターン::パターン到達状態(mps 速度) const
    {
        走行モデル 走行{目標位置, 目標速度};
        if (速度 <= 目標速度) {
            return 走行;
        }

        if (初期減速度 > 最終減速度) {
            走行.等加加速度で指定加速度まで走行(
                -最終減速度, -初期減速度, 終盤加加速度);

            if (走行.速度() > 速度) {
                // 行き過ぎたので計算し直す
                走行.変更(目標位置, 目標速度);
                走行.指定速度まで走行(速度, -最終減速度, 終盤加加速度, true);
                return 走行;
            }
        }

        // パターンに当たる場所がもっと手前なら初期減速度で減速する
        走行.指定速度まで走行(速度, -初期減速度);
        return 走行;
    }

}
