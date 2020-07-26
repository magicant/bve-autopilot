// 出力制御.cpp : 出力すべき制御指令を計算します。
//
// Copyright © 2020 Watanabe, Yuki
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
#include "出力制御.h"
#include "共通状態.h"
#include "運動状態.h"

namespace autopilot
{

    namespace
    {

        自動制動自然数ノッチ 緩め制動ノッチ(const 共通状態 &状態)
        {
            運動状態 空走 = 状態.現在走行状態();
            空走.指定時間走行(状態.制動().反応時間(), 状態.加速度());
            運動状態 緩め走行;
            mps2 理想減速度 = -緩め走行.指定速度まで走行(
                空走.速度(), 0.0_mps2, 2.0_kmphps2, true);
            mps2 緩め減速度 =
                std::max(理想減速度, static_cast<mps2>(0.3_kmphps));
            mps2 勾配加速度 = 状態.車両勾配加速度();
            自動制動実数ノッチ 緩めノッチ =
                状態.制動().自動ノッチ(緩め減速度 + 勾配加速度);
            return 緩めノッチ.ceil();
        }

    }

    /// 制動ノッチを本当に現在制動ノッチより緩めるかどうか計算する。
    /// (制動を弱めてもまたすぐ強くすることになるなら弱めない。)
    bool 出力制御::制動を緩める余裕あり(自動制動自然数ノッチ 新制動ノッチ)
        const
    {
        // 現在の減速度を保って減速し続けた場合に、数秒後に制動ノッチが
        // 強くなるかどうかを判定する。ただし停止直前は敏速に制動を制御
        // する必要があるので、低速度での判定は行わない。
        //   最初から 7.5 km/h 以下
        //     → チェック不要
        //   1 秒後には 7.5 km/h 以下
        //     → 7.5 km/h でチェック
        //   1 秒後は 7.5 km/h 以上だが 3 秒後には 7.5 km/h 以下
        //     → 1 秒後と 7.5 km/h でチェック
        //   3 秒後でも 7.5 km/h 以上
        //     → 1 秒後と 3 秒後でチェック

        constexpr mps 判定速度下限 = 7.5_kmph;

        運動状態 運動状態 = _状態.現在走行状態();
        if (運動状態.速度() <= 判定速度下限) {
            return true;
        }

        mps2 出力減速度 = _状態.制動().減速度(新制動ノッチ);
        mps2 勾配加速度 = _状態.勾配().列車勾配加速度(運動状態.位置());
        mps2 実加速度 = 勾配加速度 - 出力減速度;
        運動状態.指定時間走行(1.0_s, 実加速度);
        if (運動状態.速度() > 判定速度下限) {
            自動制動自然数ノッチ 制動ノッチ1 = 出力制動ノッチ(運動状態);
            if (制動ノッチ1 > 新制動ノッチ) {
                return false;
            }

            運動状態.指定時間走行(2.0_s, 実加速度);
            if (運動状態.速度() > 判定速度下限) {
                自動制動自然数ノッチ 制動ノッチ2 = 出力制動ノッチ(運動状態);
                return 制動ノッチ2 <= 新制動ノッチ;
            }
        }

        運動状態 = _状態.現在走行状態();
        運動状態.指定速度まで走行(判定速度下限, 実加速度);
        自動制動自然数ノッチ 制動ノッチ3 = 出力制動ノッチ(運動状態);
        return 制動ノッチ3 <= 新制動ノッチ;
    }

    自動制動自然数ノッチ 出力制御::出力制動強めノッチ() const
    {
        // 現在状態での制動ノッチを求める
        運動状態 運動状態 = _状態.現在走行状態();
        自動制動自然数ノッチ 制動ノッチ = 出力制動ノッチ(運動状態);
        if (制動を緩める余裕あり(制動ノッチ)) {
            return 制動ノッチ;
        }

        自動制動自然数ノッチ 現在制動ノッチ =
            _状態.制動().自動ノッチ(_状態.前回制動指令());
        if (制動ノッチ >= 現在制動ノッチ) {
            return 制動ノッチ;
        }

        自動制動自然数ノッチ 強めノッチ =
            _状態.制動().次に強いノッチ(制動ノッチ);
        return std::min(強めノッチ, 現在制動ノッチ);
    }

    bool 出力制御::力行する余裕あり(
        力行ノッチ ノッチ, mps2 想定加速度, s 想定惰行時間) const
    {
        // 現在よりも弱いノッチでの加速度は予測できないので拒否する
        if (_状態.前回力行ノッチ() > static_cast<int>(ノッチ.value)) {
            return false;
        }

        運動状態 運動状態 = _状態.現在走行状態();

        // 現在状態から一定時間加速する動きをまずシミュレートする
        短く力行(運動状態, ノッチ, 想定加速度, _状態);

        // その後、惰行して……
        m 元の位置 = 運動状態.位置();
        運動状態.指定時間走行(想定惰行時間);

        // 惰行した間の下り勾配による加速を考慮する
        m2ps2 勾配比エネ差 =
            _状態.勾配().下り勾配比エネルギー({元の位置, 運動状態.位置()});
        運動状態.速度変更(加算(運動状態.速度(), 勾配比エネ差));

        // その時点でブレーキをかける必要があるなら力行しない
        自動制動自然数ノッチ 制動ノッチ = 出力制動ノッチ(運動状態);
        return 制動ノッチ == 自動制動自然数ノッチ{0};
    }

    力行ノッチ 出力制御::出力力行ノッチ() const
    {
        // 前の加速が終わらないうちは正しい計算ができない可能性があるので
        if (_状態.力行をやめた直後である()) {
            return 力行ノッチ{0};
        }

        // 制限速度まで余裕があるなら全力で力行する
        if (力行する余裕あり(_状態.最大力行ノッチ(), 5.0_kmphps, 5.0_s)) {
            return _状態.最大力行ノッチ();
        }

        // 止まりかけならとにかく前に進む
        if (_状態.現在速度() < static_cast<mps>(0.5_kmph)) {
            return 力行ノッチ{1};
        }

        // 余裕があれば弱く力行
        if (力行する余裕あり(力行ノッチ{1}, 2.5_kmphps, 1.0_s)) {
            return 力行ノッチ{1};
        }

        return 力行ノッチ{0};
    }

    自動制御指令 出力制御::出力ノッチ() const
    {
        自動制動自然数ノッチ 制動ノッチ = 出力制動強めノッチ();

        // 停止直前はノッチを緩めて衝撃を抑える
        自動制動自然数ノッチ 緩めノッチ = 緩め制動ノッチ(_状態);
        制動ノッチ = std::min(制動ノッチ, 緩めノッチ);

        if (制動ノッチ > 自動制動自然数ノッチ{0}) {
            return 制動ノッチ;
        }

        // 力行または惰行
        return 出力力行ノッチ();
    }

}
