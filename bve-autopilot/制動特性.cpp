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
        int 常用ノッチ数, int 無効ノッチ数, 加速度型 常用最大減速度,
        時間型 緩解時間, const std::vector<double> &pressure_rates)
    {
        _常用ノッチ数 = 常用ノッチ数;
        _無効ノッチ数 = 無効ノッチ数;
        _常用最大減速度 = 常用最大減速度;
        _緩解時間 = 緩解時間;
        _pressure_rates = pressure_rates;

        pressure_rates_を穴埋め();
    }

    double 制動特性::ノッチ(加速度型 減速度) const
    {
        double 割合 = 減速度 / _常用最大減速度;
        auto i = std::lower_bound(
            _pressure_rates.begin(), _pressure_rates.end(), 割合);
        if (i == _pressure_rates.begin()) {
            return 0;
        }
        if (i == _pressure_rates.end()) {
            return 割合 * 実効ノッチ数() + _無効ノッチ数;
        }

        double 次ノッチ割合 = *i;
        --i;
        double 前ノッチ割合 = *i;
        return std::distance(_pressure_rates.begin(), i) +
            (割合 - 前ノッチ割合) / (次ノッチ割合 - 前ノッチ割合);

    }

    加速度型 制動特性::減速度(double ノッチ) const
    {
        assert(ノッチ >= 0);

        double 割合;
        if (ノッチ >= _常用ノッチ数) {
            割合 = (ノッチ - _無効ノッチ数) / 実効ノッチ数();
        }
        else {
            using size_type = std::vector<double>::size_type;
            size_type i = static_cast<size_type>(ノッチ);
            assert(i + 1 < _pressure_rates.size());
            double 前ノッチ割合 = _pressure_rates[i];
            double 次ノッチ割合 = _pressure_rates[i + 1];
            割合 = 前ノッチ割合;
            if (前ノッチ割合 < 次ノッチ割合) {
                割合 += (ノッチ - i) / (次ノッチ割合 - 前ノッチ割合);
            }
        }
        return _常用最大減速度 * 割合;
    }

    void 制動特性::pressure_rates_を穴埋め()
    {
        while (_pressure_rates.size() <=
            static_cast<std::vector<double>::size_type>(_常用ノッチ数))
        {
            if (_pressure_rates.size() <=
                static_cast<std::vector<double>::size_type>(_無効ノッチ数))
            {
                _pressure_rates.push_back(0);
            }
            else {
                auto 分割数 = _常用ノッチ数 - _pressure_rates.size() + 1;
                auto 前の値 = _pressure_rates.back();
                auto 次の値 = 前の値 + (1 - 前の値) / 分割数;
                _pressure_rates.push_back(次の値);
            }
        }
    }

}
