// 制動出力.h : 期待する減速度を得るために制動ノッチを加減します
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

namespace autopilot
{

    class 制動出力
    {
    public:
        using 加速度型 = double;

        制動出力() = default;
        ~制動出力() = default;

        void 性能設定(
            int 常用ノッチ数, int 無効ノッチ数, 加速度型 標準常用最大減速度);

        /// 所望の減速度を得るために設定すべき常用制動ノッチを得ます。
        /// 引数の大きさによっては常用最大を超えるノッチを返すことがあります。
        double ノッチ(加速度型 減速度) const;

    private:
        int _常用ノッチ数, _無効ノッチ数;
        加速度型 _標準常用最大減速度;
    };

}
