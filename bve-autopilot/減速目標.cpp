// 減速目標.cpp : 制限速度を定め、それを維持するブレーキを計算します
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
#include "減速目標.h"
#include <algorithm>
#include <cmath>
#include "勾配グラフ.h"

namespace autopilot
{

    namespace
    {

        constexpr s 接近時間 = 2.0_s;

    }

    走行モデル 減速目標::パターン到達状態(mps 速度) const
    {
        走行モデル 走行{_位置, _速度};
        if (速度 > _速度) {
            走行.指定速度まで走行(速度, -_基準減速度);
        }
        return 走行;

        // より正確なアルゴリズムは以下のようになるが、ここではそこまで
        // 正確さは求められていない
#if 0
        if (速度 <= _速度 + 速度マージン) {
            return 走行;
        }

        減速パターン パターン = 主パターン(平坦グラフ);

        if (速度 < パターン.通過速度()) {
            // 到達状態がブレーキを緩める途中の場合
            mps 速度差 = 速度 - _速度;
            double 速度倍率 = 速度差 / 速度マージン;
            s 時間差 = 接近時間 * std::log(速度倍率);
            m 距離 = _速度 * 時間差 + 速度差 * 接近時間;
            走行.変更(
                走行.位置() - 距離,
                走行.速度() + 速度差,
                走行.時刻() - 時間差);
            return 走行;
        }

        // パターンに当たる場所がもっと手前なら基準減速度で減速する
        走行.変更(パターン.通過地点(), パターン.通過速度(), -緩解時間);
        走行.指定速度まで走行(速度, -パターン.基準減速度());
        return 走行;
#endif
    }

    減速パターン 減速目標::主パターン(const 勾配グラフ &勾配) const
    {
        if (_速度 <= 0.0_mps) {
            return 減速パターン(_位置, 0.0_mps, _基準減速度);
        }

        // このクラスでは、基本的には 減速パターン クラスに従った等加速度運動
        // による減速を行う。ただし、速度が目標速度に近付いたところで緩やかに
        // ブレーキを緩めるため、目標速度に到達する直前の減速度は減速パターンの
        // 減速度より小さくなる。そのため、減速パターンの曲線がぴったり
        // 目標速度・目標位置を通過するようにするとわずかに速度を超えてしまう。
        // ブレーキを緩め始める瞬間の位置と速度を計算し、そこを通過するような
        // 減速パターンを用いることにする。

        // 微分方程式を解くと、ブレーキを緩めてゆく運動は指数関数の式になる
        // ことが分かる。ブレーキを緩め始める地点はこの指数関数が等加速度運動の
        // パターン (二次関数) と接する点になるので、以下「接点」と呼ぶ。

        // まず目標位置における勾配加速度を求める。
        // 下り勾配ではこの分だけ基準減速度から差し引く。
        mps2 勾配加速度 = 0.0_mps2; // TODO 勾配.列車勾配加速度(_位置);

        // 接点での減速度を求める
        constexpr mps2 接点減速度下限 = 速度マージン / 接近時間;
        mps2 接点減速度 = std::max(
            _基準減速度 - std::max(勾配加速度, 0.0_mps2), 接点減速度下限);

        // 接点での速度が目標速度よりどれだけ高いかを求める
        mps 接点速度差 = 接点減速度 * 接近時間;

        // ブレーキを緩めるのにかかる時間(の上界)を求める。つまり、減速度が
        // 接点減速度から 接点減速度下限 以下まで下がる時間。
        // (微分方程式を解くと、減速度は 1 秒当たり exp(-1 / 接近時間) 倍に
        // 小さくなることが分かる。そこからこのような計算式になる)
        double 減速度倍率 = 接点減速度 / 接点減速度下限;
        s 緩解時間 = 接近時間 * std::log(減速度倍率);

        // 緩解時間の間に列車が進む距離を求める
        m 緩解距離 = _速度 * 緩解時間 + 接点速度差 * 接近時間;

        // 接点を通る減速パターンを作る
        return 減速パターン(_位置 - 緩解距離, _速度 + 接点速度差, _基準減速度);
    }

}
