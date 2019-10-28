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
#include "単位.h"

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
            int 標準ノッチ数, int 拡張ノッチ数,
            加速度型 基準最大減速度, 時間型 反応時間,
            const std::vector<double> &pressure_rates);

        /// 標準の常用ブレーキのノッチ数です。
        /// 緩めノッチと非常ブレーキノッチを含みません。
        int 標準ノッチ数() const { return _標準ノッチ数; }
        /// 非常ブレーキノッチの後のノッチ番号を使用して定義された
        /// 拡張ノッチの数です。緩めノッチと非常ブレーキノッチを含みません。
        int 拡張ノッチ数() const;
        /// TASC/ATO が使用する想定のノッチ数です。
        /// 標準ノッチ数または拡張ノッチ数のどちらかです。
        /// 緩めノッチと非常ブレーキノッチを含みません。
        int 自動ノッチ数() const;
        加速度型 基準最大減速度() const {
            return _制動力推定.基準最大減速度();
        }
        加速度型 推定最大減速度() const {
            return _制動力推定.推定最大減速度();
        }
        時間型 反応時間() const { return _反応時間; }

        /// 所望の減速度に対応する標準ノッチを得ます。
        /// 引数の大きさによっては常用最大を超えるノッチを返すことがあります。
        double 標準ノッチ(加速度型 減速度) const;
        /// 所望の減速度に対応する自動ノッチを得ます。
        /// 引数の大きさによっては常用最大を超えるノッチを返すことがあります。
        double 自動ノッチ(加速度型 減速度) const;

        /// 割合に対応する自動ノッチを単純に返します。
        double 割合自動ノッチ(double 割合) const;

        /// 標準ノッチに対して得られるであろう減速度を得ます。
        加速度型 標準ノッチ減速度(double ノッチ) const;
        /// 自動ノッチに対して得られるであろう減速度を得ます。
        加速度型 自動ノッチ減速度(double ノッチ) const;

        /// 自動ノッチに対応する、BVE 本体側に実際に出力するノッチ番号
        /// (PressureRates 全体におけるノッチ位置) を得ます。
        int 自動ノッチ番号(int 自動ノッチインデクス) const;
        /// ノッチ番号に対応する自動ノッチを返します。
        /// 引数が自動ノッチでない場合、対応する自動ノッチに切り上げます。
        int 自動ノッチインデクス(int ノッチ番号) const;

        /// 自動ノッチを整数に丸めます。
        /// 弱いノッチはより弱く、強いノッチはより強くなる方向に
        /// バイアスをかけます。
        double 自動ノッチ丸め(double ノッチ) const;

        void 経過(const 共通状態 &状態);

    private:
        /// 車両パラメーターファイルの PressureRates と同様に、ノッチごとの
        /// ブレーキ力の割合を示す数列です。
        /// 最初の要素は 0 である必要があります。
        class pressure_rates : public std::vector<double> {
        public:
            using std::vector<double>::vector;

            pressure_rates &operator=(const std::vector<double> &v) {
                static_cast<std::vector<double> &>(*this) = v;
                return *this;
            }

            void 穴埋めする(size_type 常用ノッチ数);

            /// 指定した割合に相当するノッチを返します。
            double ノッチ(double 割合) const;
            std::pair<double, double> 割合と丸め閾値(double ノッチ) const;
            /// 指定したノッチに相当する割合を返します。
            double 割合(double ノッチ) const;
            /// ノッチを整数に丸めます。
            /// 弱いノッチはより弱く、強いノッチはより強くなる方向に
            /// バイアスをかけます。
            double 丸め(double ノッチ) const;
        };

        int _標準ノッチ数 = 0;
        時間型 _反応時間 = 0;
        制動力推定 _制動力推定;

        pressure_rates _標準ノッチ列, _拡張ノッチ列;

        double 割合(int ノッチ番号) const;
    };

}
