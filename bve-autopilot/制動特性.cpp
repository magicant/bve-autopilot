// 制動特性.cpp : 期待する減速度を得るために制動ノッチを加減します
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
#include "制動特性.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include "共通状態.h"

#pragma warning(disable:4819)

namespace autopilot
{

    制動特性::制動特性() = default;
    制動特性::~制動特性() = default;

    void 制動特性::性能設定(
        手動制動自然数ノッチ 標準最大ノッチ,
        自動制動自然数ノッチ 拡張最大ノッチ,
        mps2 基準最大減速度, s 反応時間,
        const std::vector<制動力割合> &pressure_rates)
    {
        _標準最大ノッチ = 標準最大ノッチ;
        _反応時間 = 反応時間;
        _制動力推定.基準最大減速度を設定(基準最大減速度);

        auto 標準ノッチ数 =
            static_cast<pressure_rates::size_type>(標準最大ノッチ.value);
        auto 標準ノッチ列最大長 = 標準ノッチ数 + 2;
        _標準ノッチ列 = pressure_rates;
        _標準ノッチ列.穴埋めする(標準ノッチ数);
        if (_標準ノッチ列.size() > 標準ノッチ列最大長) {
            // 拡張ノッチ列相当部分は取り除く
            _標準ノッチ列.resize(標準ノッチ列最大長);
        }

        auto 拡張ノッチ列最大長 =
            static_cast<pressure_rates::size_type>(拡張最大ノッチ.value) + 1;
        _拡張ノッチ列.clear();
        if (pressure_rates.size() > 標準ノッチ列最大長) {
            _拡張ノッチ列.emplace_back(0.0);
            std::copy(
                pressure_rates.begin() + 標準ノッチ列最大長,
                pressure_rates.end(),
                std::back_inserter(_拡張ノッチ列));
            if (_拡張ノッチ列.size() > 拡張ノッチ列最大長) {
                // 余った分は取り除く
                _拡張ノッチ列.resize(拡張ノッチ列最大長);
            }
            if (_拡張ノッチ列.size() <= 1) {
                // 緩めノッチしかないのはダメ
                _拡張ノッチ列.clear();
            }
        }
    }

    自動制動自然数ノッチ 制動特性::自動最大ノッチ() const noexcept {
        if (_拡張ノッチ列.empty()) {
            return 自動制動自然数ノッチ{標準最大ノッチ().value};
        }
        return 自動制動自然数ノッチ{
            static_cast<unsigned>(_拡張ノッチ列.size() - 1)};
    }

    自動制動実数ノッチ 制動特性::自動ノッチ(制動力割合 割合) const
    {
        return 自動制動実数ノッチ{有効ノッチ列().ノッチ(割合)};
    }

    自動制動実数ノッチ 制動特性::自動ノッチ(mps2 減速度) const
    {
        制動力割合 割合{減速度 / 推定最大減速度()};
        return 自動ノッチ(割合);
    }

    mps2 制動特性::減速度(手動制動自然数ノッチ ノッチ) const
    {
        制動力割合 割合 = _標準ノッチ列.割合(ノッチ.value);
        return 推定最大減速度() * 割合.value;
    }

    mps2 制動特性::減速度(自動制動実数ノッチ ノッチ) const
    {
        制動力割合 割合 = 有効ノッチ列().割合(ノッチ.value);
        return 推定最大減速度() * 割合.value;
    }

    制動指令 制動特性::指令(自動制動自然数ノッチ ノッチ) const
    {
        if (ノッチ.value == 0) {
            return 制動指令{0};
        }
        if (_拡張ノッチ列.empty()) {
            return 手動制動自然数ノッチ{ノッチ.value};
        }
        return 制動指令{static_cast<int>(
            ノッチ.value + _標準最大ノッチ.value + 1)};
    }

    自動制動自然数ノッチ 制動特性::自動ノッチ(制動指令 ノッチ) const
    {
        auto ノッチ番号 = static_cast<unsigned>(ノッチ.value);
        auto 標準最大 = static_cast<unsigned>(_標準最大ノッチ.value);
        if (ノッチ番号 > 標準最大 + 1) {
            return 自動制動自然数ノッチ{ノッチ番号 - (標準最大 + 1)};
        }
        if (_拡張ノッチ列.empty()) {
            return 自動制動自然数ノッチ{ノッチ番号};
        }

        制動力割合 割合 = _標準ノッチ列.割合(ノッチ番号);
        double インデクス = _拡張ノッチ列.ノッチ(割合);
        return 自動制動自然数ノッチ{
            static_cast<unsigned>(std::ceil(インデクス))};
    }

    自動制動自然数ノッチ 制動特性::自動ノッチ丸め(
        自動制動実数ノッチ ノッチ) const
    {
        return 有効ノッチ列().丸め(ノッチ.value);
    }

    自動制動自然数ノッチ 制動特性::次に強いノッチ(
        自動制動自然数ノッチ ノッチ) const
    {
        auto n = static_cast<pressure_rates::size_type>(ノッチ.value);
        auto n2 = static_cast<unsigned>(有効ノッチ列().次に強いノッチ(n));
        return 自動制動自然数ノッチ{n2};
    }

    void 制動特性::経過(const 共通状態 &状態)
    {
        制動力割合 割合 = this->割合(状態.前回制動指令());
        _制動力推定.経過(割合, 状態);
    }

    制動力割合 制動特性::割合(制動指令 ノッチ) const
    {
        auto ノッチ番号 = static_cast<unsigned>(ノッチ.value);
        auto 標準ノッチ列最大長 = _標準最大ノッチ.value + 2;
        if (ノッチ番号 >= 標準ノッチ列最大長) {
            auto 拡張ノッチ番号 = ノッチ番号 - 標準ノッチ列最大長;
            return _拡張ノッチ列.割合(拡張ノッチ番号);
        }
        else {
            return _標準ノッチ列.割合(ノッチ番号);
        }
    }

    void 制動特性::pressure_rates::穴埋めする(size_type 常用ノッチ数)
    {
        if (empty()) {
            emplace_back(0);
        }
        while (size() <= 常用ノッチ数) {
            auto 分割数 = 常用ノッチ数 - size() + 1;
            auto 前の値 = back().value;
            auto 次の値 = 前の値 + (1 - 前の値) / 分割数;
            emplace_back(次の値);
        }
    }

    double 制動特性::pressure_rates::ノッチ(制動力割合 割合) const
    {
        auto i = std::lower_bound(begin(), end(), 割合);
        if (i == begin()) {
            return 0.0;
        }
        if (i == end()) {
            return static_cast<double>(size() - 1);
        }

        double 次ノッチ割合 = i->value;
        --i;
        double 前ノッチ割合 = i->value;
        return std::distance(begin(), i) +
            (割合.value - 前ノッチ割合) / (次ノッチ割合 - 前ノッチ割合);
    }

    std::pair<制動力割合, 制動力割合> 制動特性::pressure_rates::割合と丸め閾値(
        double ノッチ) const
    {
        assert(size() > 0);

        if (ノッチ <= 0.0) {
            return {制動力割合{0.0}, 制動力割合{0.0}};
        }

        auto 最大ノッチ = size() - 1;
        if (ノッチ >= 最大ノッチ) {
            制動力割合 最大ノッチ割合 = back();
            制動力割合 割合 = 最大ノッチ割合;
            割合.value *= ノッチ / 最大ノッチ;
            return {割合, 最大ノッチ割合};
        }

        size_type i = static_cast<size_type>(ノッチ);
        assert(i < 最大ノッチ);
        double 前ノッチ割合 = (*this)[i].value;
        double 次ノッチ割合 = (*this)[i + 1].value;
        double 割合 = 前ノッチ割合, 丸め閾値 = 次ノッチ割合;
        double 割合差 = 次ノッチ割合 - 前ノッチ割合;
        if (割合差 > 0.0) {
            double 最大割合 = back().value;
            割合 += (ノッチ - i) * 割合差;
            丸め閾値 *= 最大割合 / (最大割合 + 割合差);
        }
        return {制動力割合{割合}, 制動力割合{丸め閾値}};
    }

    制動力割合 制動特性::pressure_rates::割合(double ノッチ) const
    {
        return 割合と丸め閾値(ノッチ).first;
    }

    自動制動自然数ノッチ 制動特性::pressure_rates::丸め(double ノッチ) const
    {
        制動力割合 割合, 丸め閾値;
        std::tie(割合, 丸め閾値) = 割合と丸め閾値(ノッチ);
        if (割合 <= 丸め閾値) {
            ノッチ = std::floor(ノッチ);
        }
        else {
            ノッチ = std::ceil(ノッチ);
        }

        constexpr double max = std::numeric_limits<int>::max();
        ノッチ = std::clamp(ノッチ, 0.0, max);
        return 自動制動自然数ノッチ{static_cast<unsigned>(ノッチ)};
    }

    制動特性::pressure_rates::size_type
        制動特性::pressure_rates::次に強いノッチ(size_type ノッチ) const
    {
        制動力割合 割合 = (*this)[ノッチ];
        while (ノッチ < size() && 割合 >= (*this)[ノッチ]) {
            ノッチ++;
        }
        return ノッチ;
    }

}
