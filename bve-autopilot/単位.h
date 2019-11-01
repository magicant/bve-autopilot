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
#include <type_traits>

namespace autopilot
{

    template<typename Value, typename Self>
    struct 物理量
    {
        using value_type = Value;

        value_type value;

        static constexpr Self 無限大() {
            return static_cast<Self>(
                std::numeric_limits<value_type>::infinity());
        }
        static constexpr Self quiet_NaN() {
            return static_cast<Self>(
                std::numeric_limits<value_type>::quiet_NaN());
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

    template<typename Value, typename Self>
    constexpr Self operator*(const 物理量<Value, Self> &a, const Value &b) {
        return static_cast<Self>(a.value * b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator*=(
        物理量<Value, Self> &a, const Value &b)
    {
        a.value *= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Self operator*(const Value &a, const 物理量<Value, Self> &b) {
        return static_cast<Self>(a * b.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator/(const 物理量<Value, Self> &a, const Value &b) {
        return static_cast<Self>(a.value / b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator/=(
        物理量<Value, Self> &a, const Value &b)
    {
        a.value /= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Value operator/(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b)
    {
        return a.value / b.value;
    }

    template<typename Value, typename Self>
    constexpr Self operator%(const 物理量<Value, Self> &a, const Value &b) {
        return static_cast<Self>(a.value % b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator%=(
        物理量<Value, Self> &a, const Value &b)
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
    bool isfinite(const 物理量<Value, Self> &v) {
        return std::isfinite(v.value);
    }

    template<typename Value, typename Self>
    bool isinf(const 物理量<Value, Self> &v) {
        return std::isinf(v.value);
    }

    template<typename Value, typename Self>
    bool isnormal(const 物理量<Value, Self> &v) {
        return std::isnormal(v.value);
    }

    template<typename Value, typename Self>
    bool isnan(const 物理量<Value, Self> &v) {
        return std::isnan(v.value);
    }

    template<typename Value, typename Self>
    Self abs(const 物理量<Value, Self> &v) {
        return static_cast<Self>(std::abs(v.value));
    }

    template<typename Value, typename Self>
    Self ceil(const 物理量<Value, Self> &v) {
        return static_cast<Self>(std::ceil(v.value));
    }

    template<typename Value, typename Self>
    Self floor(const 物理量<Value, Self> &v) {
        return static_cast<Self>(std::floor(v.value));
    }

    // 時間

    struct s;
    struct ms;

    // 秒
    struct s : 物理量<double, s>
    {
        using 物理量::物理量;
        constexpr s(const ms &v);
    };

    // ミリ秒
    struct ms : 物理量<double, ms>
    {
        using 物理量::物理量;
        constexpr ms(const s &v);
    };

    constexpr s::s(const ms &v) : 物理量(v.value / 1000.0) {}
    constexpr ms::ms(const s &v) : 物理量(v.value * 1000.0) {}

    constexpr s operator"" _s(long double v) {
        return static_cast<s>(static_cast<double>(v));
    }
    constexpr ms operator"" _ms(long double v) {
        return static_cast<ms>(static_cast<double>(v));
    }

    // 距離

    struct m;
    struct cm;

    /// メートル
    struct m : 物理量<double, m>
    {
        using 物理量::物理量;
        constexpr m(const cm &v);
    };

    /// センチメートル
    struct cm : 物理量<double, cm>
    {
        using 物理量::物理量;
        constexpr cm(const m &v);
    };

    constexpr m::m(const cm &v) : 物理量(v.value / 100.0) {}
    constexpr cm::cm(const m &v) : 物理量(v.value * 100.0) {}

    constexpr m operator"" _m(long double v) {
        return static_cast<m>(static_cast<double>(v));
    }
    constexpr cm operator"" _cm(long double v) {
        return static_cast<cm>(static_cast<double>(v));
    }

    // 速度

    struct mps;
    struct kmph;

    /// メートル毎秒
    struct mps : 物理量<double, mps>
    {
        using 物理量::物理量;
        constexpr mps(const kmph &v);
    };

    /// キロメートル毎時
    struct kmph : 物理量<double, kmph>
    {
        using 物理量::物理量;
        constexpr kmph(const mps &v);
    };

    constexpr mps::mps(const kmph &v) : 物理量(v.value / 3.6) {}
    constexpr kmph::kmph(const mps &v) : 物理量(v.value * 3.6) {}

    constexpr mps operator"" _mps(long double v) {
        return static_cast<mps>(static_cast<double>(v));
    }
    constexpr kmph operator"" _kmph(long double v) {
        return static_cast<kmph>(static_cast<double>(v));
    }

    constexpr mps operator/(const m &a, const s &b) {
        return static_cast<mps>(a.value / b.value);
    }
    constexpr m operator*(const mps &a, const s &b) {
        return static_cast<m>(a.value * b.value);
    }
    constexpr m operator*(const s &a, const mps &b) {
        return static_cast<m>(a.value * b.value);
    }
    constexpr s operator/(const m &a, const mps &b) {
        return static_cast<s>(a.value / b.value);
    }

    // 加速度

    struct mps2;
    struct kmphps;

    /// メートル毎秒毎秒
    struct mps2 : 物理量<double, mps2>
    {
        using 物理量::物理量;
        constexpr mps2(const kmphps &v);
    };

    /// キロメートル毎時毎秒
    struct kmphps : 物理量<double, kmphps>
    {
        using 物理量::物理量;
        constexpr kmphps(const mps2 &v);
    };

    constexpr mps2::mps2(const kmphps &v) : 物理量(v.value / 3.6) {}
    constexpr kmphps::kmphps(const mps2 &v) : 物理量(v.value * 3.6) {}

    constexpr mps2 operator"" _mps2(long double v) {
        return static_cast<mps2>(static_cast<double>(v));
    }
    constexpr kmphps operator"" _kmphps(long double v) {
        return static_cast<kmphps>(static_cast<double>(v));
    }

    constexpr mps2 operator/(const mps &a, const s &b) {
        return static_cast<mps2>(a.value / b.value);
    }
    constexpr mps operator*(const mps2 &a, const s &b) {
        return static_cast<mps>(a.value * b.value);
    }
    constexpr mps operator*(const s &a, const mps2 &b) {
        return static_cast<mps>(a.value * b.value);
    }
    constexpr s operator/(const mps &a, const mps2 &b) {
        return static_cast<s>(a.value / b.value);
    }

    // 加加速度

    struct mps3;
    struct kmphps2;

    /// メートル毎秒毎秒毎秒
    struct mps3 : 物理量<double, mps3>
    {
        using 物理量::物理量;
        constexpr mps3(const kmphps2 &v);
    };

    /// キロメートル毎時毎秒毎秒
    struct kmphps2 : 物理量<double, kmphps2>
    {
        using 物理量::物理量;
        constexpr kmphps2(const mps3 &v);
    };

    constexpr mps3::mps3(const kmphps2 &v) : 物理量(v.value / 3.6) {}
    constexpr kmphps2::kmphps2(const mps3 &v) : 物理量(v.value * 3.6) {}

    constexpr mps3 operator"" _mps3(long double v) {
        return static_cast<mps3>(static_cast<double>(v));
    }
    constexpr kmphps2 operator"" _kmphps2(long double v) {
        return static_cast<kmphps2>(static_cast<double>(v));
    }

    constexpr mps3 operator/(const mps2 &a, const s &b) {
        return static_cast<mps3>(a.value / b.value);
    }
    constexpr mps2 operator*(const mps3 &a, const s &b) {
        return static_cast<mps2>(a.value * b.value);
    }
    constexpr mps2 operator*(const s &a, const mps3 &b) {
        return static_cast<mps2>(a.value * b.value);
    }
    constexpr s operator/(const mps2 &a, const mps3 &b) {
        return static_cast<s>(a.value / b.value);
    }

}
