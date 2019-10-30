// 単位.h : 物理量の単位を扱います
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
#include <algorithm>
#include <cmath>
#include <limits>

namespace autopilot
{

    // メートル
    using 距離型 = double;
    // メートル毎秒
    using 速度型 = double;
    // メートル毎秒毎秒
    using 加速度型 = double;
    // メートル毎秒毎秒毎秒
    using 加加速度型 = double;

    constexpr 速度型 mps_from_kmph(double kmph) {
        return kmph / 3.6;
    }
    constexpr double kmph_from_mps(速度型 mps) {
        return mps * 3.6;
    }

    template<typename Value, typename Self>
    struct 物理量
    {
        using value_type = Value;

        value_type value;

        static constexpr Self 無限大() {
            return static_cast<Self>(
                std::numeric_limits<value_type>::infinity());
        }

        constexpr 物理量() : value{} {}
        constexpr explicit 物理量(const value_type &v) : value{v} {}
    };

    template<typename Value, typename Self>
    constexpr Self operator+(const 物理量<Value, Self> &v) {
        return static_cast<Self>(+v.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator-(const 物理量<Value, Self> &v) {
        return static_cast<Self>(-v.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator+(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return static_cast<Self>(a.value + b.value);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator+=(
        物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        a.value += b.value;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Self operator-(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return static_cast<Self>(a.value - b.value);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator-=(
        物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        a.value -= b.value;
        return a;
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator*(
        const 物理量<Value, Self> &a, const Value2 &b)
    {
        return static_cast<Self>(a.value * b);
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator*=(物理量<Value, Self> &a, const Value2 &b)
    {
        a.value *= b;
        return a;
    }

    template<typename Value2, typename Value, typename Self>
    constexpr Self operator*(
        const Value2 &a, const 物理量<Value, Self> &b)
    {
        return static_cast<Self>(a * b.value);
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator/(
        const 物理量<Value, Self> &a, const Value2 &b)
    {
        return static_cast<Self>(a.value / b);
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator/=(物理量<Value, Self> &a, const Value2 &b)
    {
        a.value /= b;
        return a;
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator%(
        const 物理量<Value, Self> &a, const Value2 &b)
    {
        return static_cast<Self>(a.value % b);
    }

    template<typename Value, typename Self, typename Value2>
    constexpr Self operator%=(物理量<Value, Self> &a, const Value2 &b)
    {
        a.value %= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr bool operator==(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value == b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator!=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value != b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator<(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value < b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator<=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value <= b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator>(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value > b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator>=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value >= b.value;
    }

    template<typename Value, typename Self>
    Self abs(const 物理量<Value, Self> &v) {
        return static_cast<Self>(std::abs(v.value));
    }

    template<typename Value, typename Self>
    constexpr Self max(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return static_cast<Self>(std::max(a.value, b.value));
    }

    template<typename Value, typename Self>
    constexpr Self min(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return static_cast<Self>(std::min(a.value, b.value));
    }

    // 時間

    struct 秒;
    struct ミリ秒;

    struct 秒 : 物理量<double, 秒>
    {
        using 物理量::物理量;
        constexpr 秒(const ミリ秒 &v);
    };

    struct ミリ秒 : 物理量<double, ミリ秒>
    {
        using 物理量::物理量;
        constexpr ミリ秒(const 秒 &v);
    };

    constexpr 秒::秒(const ミリ秒 &v) : 物理量(v.value / 1000.0) {}
    constexpr ミリ秒::ミリ秒(const 秒 &v) : 物理量(v.value * 1000.0) {}

    constexpr 秒 operator"" _s(long double v) {
        return static_cast<秒>(static_cast<double>(v));
    }
    constexpr ミリ秒 operator"" _ms(long double v) {
        return static_cast<ミリ秒>(static_cast<double>(v));
    }

}
