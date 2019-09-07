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

namespace autopilot
{

    制動特性::制動特性() = default;
    制動特性::~制動特性() = default;

    void 制動特性::性能設定(
        int 常用ノッチ数, int 無効ノッチ数, int 拡張ノッチ数,
        加速度型 常用最大減速度, 時間型 反応時間,
        const std::vector<double> &pressure_rates_config)
    {
        _常用ノッチ数 = 常用ノッチ数;
        _無効ノッチ数 = 無効ノッチ数;
        _拡張ノッチ数 = 拡張ノッチ数;
        _常用最大減速度 = 常用最大減速度;
        _反応時間 = 反応時間;
        _標準ノッチ列 = pressure_rates_config;

        _標準ノッチ列.穴埋めする(常用ノッチ数, 無効ノッチ数);
    }

    double 制動特性::ノッチ(加速度型 減速度) const
    {
        double 割合 = 減速度 / _常用最大減速度;
        return _標準ノッチ列.ノッチ(割合);
    }

    加速度型 制動特性::減速度(double ノッチ) const
    {
        double 割合 = _標準ノッチ列.割合(ノッチ);
        return _常用最大減速度 * 割合;
    }

    void 制動特性::pressure_rates::穴埋めする(
        size_type 常用ノッチ数, size_type 無効ノッチ数)
    {
        while (size() <= 常用ノッチ数) {
            if (size() <= 無効ノッチ数) {
                push_back(0);
            }
            else {
                auto 分割数 = 常用ノッチ数 - size() + 1;
                auto 前の値 = back();
                auto 次の値 = 前の値 + (1 - 前の値) / 分割数;
                push_back(次の値);
            }
        }
    }

    double 制動特性::pressure_rates::ノッチ(double 割合) const
    {
        auto i = std::lower_bound(begin(), end(), 割合);
        if (i == begin()) {
            return 0;
        }
        if (i == end()) {
            return 割合 * size();
        }

        double 次ノッチ割合 = *i;
        --i;
        double 前ノッチ割合 = *i;
        return std::distance(begin(), i) +
            (割合 - 前ノッチ割合) / (次ノッチ割合 - 前ノッチ割合);
    }

    double 制動特性::pressure_rates::割合(double ノッチ) const
    {
        assert(ノッチ >= 0);
        assert(size() > 0);

        if (ノッチ >= size() - 1) {
            return ノッチ / (size() - 1);
        }

        size_type i = static_cast<size_type>(ノッチ);
        assert(i < size() - 1);
        double 前ノッチ割合 = (*this)[i];
        double 次ノッチ割合 = (*this)[i + 1];
        double 割合 = 前ノッチ割合;
        if (前ノッチ割合 < 次ノッチ割合) {
            割合 += (ノッチ - i) / (次ノッチ割合 - 前ノッチ割合);
        }
        return 割合;
    }

}
