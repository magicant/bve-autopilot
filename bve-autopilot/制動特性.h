// 制動特性.h : 期待する減速度を得るために制動ノッチを加減します
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
#include <vector>
#include "制動力推定.h"
#include "加速度計.h"
#include "物理量.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 共通状態;

    /// このクラスは制動ノッチと減速度の関係を計算します。
    /// 実際に使用されている制動ノッチと実際の加速度を元に、
    /// 所望の加速度を得るためのノッチを推測します。
    class 制動特性
    {
    public:
        制動特性();
        ~制動特性();

        void 性能設定(
            手動制動自然数ノッチ 標準最大ノッチ,
            自動制動自然数ノッチ 拡張最大ノッチ,
            mps2 基準最大減速度, s 反応時間,
            const std::vector<制動力割合> &pressure_rates);

        /// 標準の常用ブレーキのノッチの最大値です。
        手動制動自然数ノッチ 標準最大ノッチ() const { return _標準最大ノッチ; }
        /// TASC/ATO が使用する想定のノッチの最大値です。
        /// 拡張ノッチが設定されていない場合は標準最大ノッチに一致します。
        自動制動自然数ノッチ 自動最大ノッチ() const;
        mps2 基準最大減速度() const {
            return _制動力推定.基準最大減速度();
        }
        mps2 推定最大減速度() const {
            return _制動力推定.推定最大減速度();
        }
        s 反応時間() const { return _反応時間; }

        /// 割合に対応する自動ノッチを単純に返します。
        自動制動実数ノッチ 自動ノッチ(制動力割合 割合) const;

        /// 所望の減速度に対応する自動ノッチを得ます。
        /// 引数の大きさによっては常用最大を超えるノッチを返すことがあります。
        自動制動実数ノッチ 自動ノッチ(mps2 減速度) const;

        /// 標準ノッチに対して得られるであろう減速度を得ます。
        mps2 減速度(手動制動自然数ノッチ ノッチ) const;
        /// 自動ノッチに対して得られるであろう減速度を得ます。
        mps2 減速度(自動制動実数ノッチ ノッチ) const;

        /// 自動ノッチに対応する、BVE 本体側に実際に出力するノッチ番号
        /// (PressureRates 全体におけるノッチ位置) を得ます。
        int 自動ノッチ番号(自動制動自然数ノッチ ノッチ) const;
        /// ノッチ番号に対応する自動ノッチを返します。
        /// 引数が自動ノッチでない場合、対応する自動ノッチに切り上げます。
        自動制動自然数ノッチ 自動ノッチ(int ノッチ番号) const;

        /// 自動ノッチを整数に丸めます。
        /// 弱いノッチはより弱く、強いノッチはより強くなる方向に
        /// バイアスをかけます。
        自動制動自然数ノッチ 自動ノッチ丸め(自動制動実数ノッチ ノッチ) const;

        void 経過(const 共通状態 &状態);

    private:
        /// 車両パラメーターファイルの PressureRates と同様に、ノッチごとの
        /// ブレーキ力の割合を示す数列です。
        /// 最初の要素は 0 である必要があります。
        class pressure_rates : public std::vector<制動力割合> {
        public:
            using std::vector<制動力割合>::vector;

            pressure_rates &operator=(const std::vector<制動力割合> &v) {
                static_cast<std::vector<制動力割合> &>(*this) = v;
                return *this;
            }

            void 穴埋めする(size_type 常用ノッチ数);

            /// 指定した割合に相当するノッチを返します。
            double ノッチ(制動力割合 割合) const;
            /// 指定したノッチに相当する制割合と、ノッチを切り上げるか
            /// 切り捨てるか判断する目安となる基準の割合を返します。
            std::pair<制動力割合, 制動力割合> 割合と丸め閾値(double ノッチ)
                const;
            /// 指定したノッチに相当する割合を返します。
            制動力割合 割合(double ノッチ) const;
            /// ノッチを整数に丸めます。
            /// 弱いノッチはより弱く、強いノッチはより強くなる方向に
            /// バイアスをかけます。
            自動制動自然数ノッチ 丸め(double ノッチ) const;
        };

        手動制動自然数ノッチ _標準最大ノッチ;
        s _反応時間 = {};
        制動力推定 _制動力推定;

        pressure_rates _標準ノッチ列, _拡張ノッチ列;

        const pressure_rates &有効ノッチ列() const noexcept {
            return _拡張ノッチ列.empty() ? _標準ノッチ列 : _拡張ノッチ列;
        }

        制動力割合 割合(int ノッチ番号) const;
    };

}

#pragma warning(pop)
