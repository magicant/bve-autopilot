// 勾配グラフ.h : 勾配による列車の挙動への影響を計算します
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
#include <map>
#include "区間.h"
#include "物理量.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    class 勾配加速度グラフ
    {
    public:
        using 勾配 = double;

        bool empty() const noexcept {
            return _変化点リスト.empty();
        }
        mps2 勾配加速度(m 位置) const;
        m2ps2 比エネルギー差(区間 変位) const {
            return 比エネルギー(変位.終点) - 比エネルギー(変位.始点);
        }
        m2ps2 下り勾配比エネルギー差(区間 変位) const {
            return 下り勾配比エネルギー(変位.終点) -
                下り勾配比エネルギー(変位.始点);
        }

        void clear() noexcept {
            _変化点リスト.clear();
            _累積比エネルギー未計算位置 = _変化点リスト.begin();
            _累積下り勾配比エネルギー未計算位置 = _変化点リスト.begin();
        }
        void 勾配変化追加(区間 変化区間, 勾配 勾配変化量);

    private:
        struct 変化点 {
            mps2 勾配加速度;
            mutable m2ps2 累積比エネルギー = m2ps2::quiet_NaN();
            mutable m2ps2 累積下り勾配比エネルギー = m2ps2::quiet_NaN();

            constexpr explicit 変化点() noexcept = default;
            constexpr explicit 変化点(const mps2 &勾配加速度) noexcept :
                勾配加速度{勾配加速度} {}
        };

        using const_iterator = std::map<m, 変化点>::const_iterator;
        using iterator = std::map<m, 変化点>::iterator;

        /// 加速度が a1 から a2 に変化するときの比エネルギーを求める。
        static m2ps2 比エネルギー差(mps2 a2, mps2 a1, m 変位);
        /// 加速度が a1 から a2 に変化するときの比エネルギーを求める。
        /// ただし加速度が正の範囲のみ計算に加える。
        static m2ps2 下り勾配比エネルギー差(mps2 a2, mps2 a1, m 変位);

        std::map<m, 変化点> _変化点リスト;
        mutable const_iterator _累積比エネルギー未計算位置 =
            _変化点リスト.begin();
        mutable const_iterator _累積下り勾配比エネルギー未計算位置 =
            _変化点リスト.begin();

        /// i == upper_bound(位置)
        mps2 勾配加速度(const_iterator i, m 位置) const;
        m2ps2 比エネルギー(m 位置) const;
        m2ps2 下り勾配比エネルギー(m 位置) const;
    };

    class 勾配グラフ
    {
    public:
        using 勾配 = 勾配加速度グラフ::勾配;

        勾配グラフ();
        ~勾配グラフ();

        static const 勾配グラフ 平坦グラフ;

        void 消去() noexcept;
        void 列車長を設定(m 列車長) noexcept;
        void 勾配区間追加(m 始点, 勾配 勾配);
        void 通過(m 位置);

        mps2 列車勾配加速度(m 列車先頭位置) const;
        m2ps2 比エネルギー差(区間 変位) const;
        m2ps2 下り勾配比エネルギー差(区間 変位) const;

    private:
        // 区間の始点からその区間の勾配への写像
        std::map<m, 勾配> _区間リスト;

        m _列車長 = 0.0_m;

        /// 区間リストに対応する加速度のキャッシュ。
        /// ただし使用されるときに初めて構築される。
        mutable 勾配加速度グラフ _加速度キャッシュ;

        void 加速度キャッシュ構築() const;
    };

}

#pragma warning(pop)
