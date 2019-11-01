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

    std::pair<mps, mps2> 減速パターン::期待速度と期待減速度(
        m 現在位置) const
    {
        if (現在位置 >= 目標位置) {
            return { 目標速度, 0.0_mps2 };
        }

        // 目標位置・目標速度の状態から逆算する
        走行モデル 走行(目標位置, 目標速度);

        constexpr 加加速度型 終盤加加速度 = mps_from_kmph(0.5);
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

    int 減速パターン::出力制動ノッチ(
        m 現在位置, mps 現在速度, int 現在制動ノッチ,
        mps2 勾配影響, const 共通状態 &状態) const
    {
        const 制動特性 &制動 = 状態.制動();
        mps2 出力減速度 = this->出力減速度(現在位置, 現在速度);
        double 出力制動ノッチ相当 = 制動.自動ノッチ(出力減速度 + 勾配影響);

        if (目標位置 >= 現在位置) {
            // 小刻みに制動ノッチを変化させるのを防ぐ
            double 補正ノッチ =
                0.25 * std::min(1.0, 現在速度 / (5.0_s * 初期減速度));
            if (出力制動ノッチ相当 >= 現在制動ノッチ) {
                出力制動ノッチ相当 -= 補正ノッチ;
            }
            else {
                出力制動ノッチ相当 += 補正ノッチ;
            }

            出力制動ノッチ相当 = 制動.自動ノッチ丸め(出力制動ノッチ相当);
        }
        else {
            出力制動ノッチ相当 = std::ceil(出力制動ノッチ相当);
        }

        if (出力制動ノッチ相当 > std::numeric_limits<int>::max()) {
            return 制動.自動ノッチ数();
        }
        int 出力制動ノッチ = static_cast<int>(出力制動ノッチ相当);
        return std::clamp(出力制動ノッチ, 0, 制動.自動ノッチ数());
    }

    bool 減速パターン::力行する余裕あり(
        int 力行ノッチ, mps2 想定加速度, s 想定惰行時間,
        mps2 勾配影響, const 共通状態 &状態) const
    {
        走行モデル モデル = 状態.現在走行状態();

        // 現在状態から一定時間加速する動きをまずシミュレートする
        if (状態.前回出力().Power >= 力行ノッチ) {
            // 現在行っている力行を直ちにやめる動き
            モデル.指定時間走行(状態.設定().加速終了遅延(), 状態.加速度());
        }
        else {
            // 力行をこれから短い時間行う動き
            s 最低加速時間 = std::max(1.0_s, 状態.設定().加速終了遅延());
            想定加速度 = std::max(想定加速度, 状態.加速度()); // *1
            モデル.指定時間走行(最低加速時間, 想定加速度);

            // *1 強い力行をやめた直後に弱い力行をシミュレートするときは
            // まだ前の力行の加速度が残っているのでそれも考慮する
            // (すぐ無駄に弱い力行をしないために)
        }

        // その後、惰行して……
        モデル.指定時間走行(想定惰行時間, 勾配影響);

        // ブレーキをかける必要があるなら力行しない
        int 制動ノッチ = 出力制動ノッチ(
            モデル.位置(), モデル.速度(), 0, 勾配影響, 状態);
        return 制動ノッチ <= 0;
    }

    int 減速パターン::出力ノッチ(const 共通状態 &状態) const
    {
        mps 現在速度 = 状態.現在速度();
        int 現在制動ノッチ =
            状態.制動().自動ノッチインデクス(状態.前回出力().Brake);
        mps2 勾配影響 = 状態.車両勾配加速度();
        int 出力制動ノッチ = this->出力制動ノッチ(
            状態.現在位置(), 現在速度, 現在制動ノッチ, 勾配影響, 状態);

        // 制動を弱めてもまたすぐ強くすることになるなら弱めない
        if (現在速度 >= static_cast<mps>(10.0_kmph) &&
            出力制動ノッチ < 現在制動ノッチ)
        {
            mps2 出力減速度 =
                状態.制動().自動ノッチ減速度(出力制動ノッチ) - 勾配影響;
            s 猶予 = 出力減速度 > 0.0_mps2 ?
                std::clamp(
                    (現在速度 - 目標速度) / 4.0 / 出力減速度, 1.0_s, 3.0_s) :
                1.0_s;
            走行モデル モデル = 状態.現在走行状態();
            モデル.指定時間走行(猶予, -出力減速度);
            int 猶予後出力制動ノッチ = this->出力制動ノッチ(
                モデル.位置(), モデル.速度(), 出力制動ノッチ,
                勾配影響, 状態);
            if (猶予後出力制動ノッチ > 出力制動ノッチ) {
                出力制動ノッチ++;
            }
        }

        // 停止直前はノッチを緩めて衝撃を抑える
        if (目標速度 == 0.0_mps) {
            走行モデル 空走 = 状態.現在走行状態();
            空走.指定時間走行(状態.制動().反応時間(), 状態.加速度());
            走行モデル 緩め走行{目標位置};
            mps2 緩め減速度 = std::max(
                緩め走行.指定速度まで走行(
                    空走.速度(), 0.0_mps2, mps_from_kmph(2)),
                static_cast<mps2>(0.3_kmphps));
            double 緩め制動ノッチ相当 =
                状態.制動().自動ノッチ(緩め減速度 + 勾配影響);
            int 緩め制動ノッチ =
                static_cast<int>(std::ceil(緩め制動ノッチ相当));
            出力制動ノッチ = std::min(出力制動ノッチ, 緩め制動ノッチ);
        }

        if (出力制動ノッチ > 0) {
            return -出力制動ノッチ;
        }

        // 制限速度まで余裕があるなら全力で力行する
        mps2 想定勾配影響 = std::max(勾配影響, 0.0_mps2);
        if (力行する余裕あり(
            状態.車両仕様().PowerNotches, 5.0_kmphps, 5.0_s,
            想定勾配影響, 状態))
        {
            return 状態.車両仕様().PowerNotches;
        }

        // ある程度速度が出ているなら弱い力行ノッチは無意味なので
        if (現在速度 >= static_cast<mps>(10.0_kmph)) {
            return 0;
        }

        // 止まりかけならとにかく前に進む
        if (現在速度 < static_cast<mps>(0.5_kmph)) {
            return 1;
        }

        if (力行する余裕あり(1, 2.5_kmphps, 1.0_s, 想定勾配影響, 状態)) {
            return 1;
        }
        return 0;
    }

}
