// 物理量.h : 物理量の単位を扱います
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
#include <chrono>
#include <cmath>
#include <limits>
#include <ratio>

namespace autopilot
{

    template<typename Value, typename Self>
    struct 物理量
    {
        using value_type = Value;

        value_type value;

        static constexpr Self 無限大() noexcept {
            static_assert(std::numeric_limits<value_type>::has_infinity);
            return static_cast<Self>(
                std::numeric_limits<value_type>::infinity());
        }
        static constexpr Self quiet_NaN() noexcept {
            static_assert(std::numeric_limits<value_type>::has_quiet_NaN);
            return static_cast<Self>(
                std::numeric_limits<value_type>::quiet_NaN());
        }

        constexpr 物理量() noexcept : value{} {}
        constexpr explicit 物理量(const value_type &v) noexcept : value{v} {}
    };

    template<typename Value, typename Self>
    constexpr Self operator+(const 物理量<Value, Self> &v) noexcept {
        return static_cast<Self>(+v.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator-(const 物理量<Value, Self> &v) noexcept {
        return static_cast<Self>(-v.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator+(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return static_cast<Self>(a.value + b.value);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator+=(
        物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        a.value += b.value;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Self operator-(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return static_cast<Self>(a.value - b.value);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator-=(
        物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        a.value -= b.value;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Self operator*(const 物理量<Value, Self> &a, const Value &b)
        noexcept
    {
        return static_cast<Self>(a.value * b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator*=(
        物理量<Value, Self> &a, const Value &b) noexcept
    {
        a.value *= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Self operator*(const Value &a, const 物理量<Value, Self> &b)
        noexcept
    {
        return static_cast<Self>(a * b.value);
    }

    template<typename Value, typename Self>
    constexpr Self operator/(const 物理量<Value, Self> &a, const Value &b)
        noexcept
    {
        return static_cast<Self>(a.value / b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator/=(
        物理量<Value, Self> &a, const Value &b) noexcept
    {
        a.value /= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr Value operator/(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value / b.value;
    }

    template<typename Value, typename Self>
    constexpr Self operator%(const 物理量<Value, Self> &a, const Value &b)
        noexcept
    {
        return static_cast<Self>(a.value % b);
    }

    template<typename Value, typename Self>
    constexpr 物理量<Value, Self> &operator%=(
        物理量<Value, Self> &a, const Value &b) noexcept
    {
        a.value %= b;
        return a;
    }

    template<typename Value, typename Self>
    constexpr bool operator==(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value == b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator!=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value != b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator<(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value < b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator<=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value <= b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator>(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
    {
        return a.value > b.value;
    }

    template<typename Value, typename Self>
    constexpr bool operator>=(
        const 物理量<Value, Self> &a, const 物理量<Value, Self> &b) noexcept
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

    /// 秒
    struct s : 物理量<double, s>
    {
        using 物理量::物理量;
        template<typename R, typename P>
        constexpr s(const std::chrono::duration<R, P> &d) :
            物理量(std::chrono::duration<double>(d).count()) {}

        constexpr static s 半日() noexcept {
            return std::chrono::hours(12);
        }
        constexpr static s 一日() noexcept {
            return std::chrono::hours(24);
        }
    };

    constexpr s operator"" _s(long double v) noexcept {
        return static_cast<s>(static_cast<double>(v));
    }

    /// ミリ秒
    using ms = std::chrono::duration<int, std::milli>;

    // 時刻

    class 時刻
    {
    public:
        constexpr 時刻() noexcept : _経過時間{} {}
        constexpr explicit 時刻(const s &v) noexcept :_経過時間{v} {}

        constexpr s 経過時間() const noexcept { return _経過時間; }

        constexpr 時刻 &operator+=(const s &v) noexcept {
            _経過時間 += v;
            return *this;
        }
        constexpr 時刻 &operator-=(const s &v) noexcept {
            _経過時間 -= v;
            return *this;
        }

    private:
        s _経過時間;
    };

    constexpr 時刻 operator+(const 時刻 &a, const s &b) noexcept {
        return static_cast<時刻>(a.経過時間() + b);
    }
    constexpr 時刻 operator+(const s &a, const 時刻 &b) noexcept {
        return static_cast<時刻>(a + b.経過時間());
    }
    constexpr 時刻 operator-(const 時刻 &a, const s &b) noexcept {
        return static_cast<時刻>(a.経過時間() - b);
    }
    constexpr s operator-(const 時刻 &a, const 時刻 &b) noexcept {
        // BVE では正子をまたいで日付が変わると時刻が 00:00:00 に戻るので
        // 単純に引き算しただけでは必ずしも正しい結果にならない。
        // ここでは 12 時間を超えるシナリオはないと仮定し、二日目の午前を
        // 表している可能性のある時刻を補正する。

        s 差 = a.経過時間() - b.経過時間();
        if (差 < -s::半日()) {
            return 差 + s::一日();
        }
        if (差 > s::半日()) {
            return 差 - s::一日();
        }
        return 差;
    }
    constexpr bool operator==(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b == 0.0_s;
    }
    constexpr bool operator!=(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b != 0.0_s;
    }
    constexpr bool operator<(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b < 0.0_s;
    }
    constexpr bool operator<=(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b <= 0.0_s;
    }
    constexpr bool operator>(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b > 0.0_s;
    }
    constexpr bool operator>=(const 時刻 &a, const 時刻 &b) noexcept {
        return a - b >= 0.0_s;
    }

    // 距離

    struct m;
    struct cm;

    /// メートル
    struct m : 物理量<double, m>
    {
        using 物理量::物理量;
        constexpr m(const cm &v) noexcept;
    };

    /// センチメートル
    struct cm : 物理量<double, cm>
    {
        using 物理量::物理量;
        constexpr cm(const m &v) noexcept;
    };

    constexpr m::m(const cm &v) noexcept : 物理量(v.value / 100.0) {}
    constexpr cm::cm(const m &v) noexcept : 物理量(v.value * 100.0) {}

    constexpr m operator"" _m(long double v) noexcept {
        return static_cast<m>(static_cast<double>(v));
    }
    constexpr cm operator"" _cm(long double v) noexcept {
        return static_cast<cm>(static_cast<double>(v));
    }

    // 速度

    struct mps;
    struct kmph;

    /// メートル毎秒
    struct mps : 物理量<double, mps>
    {
        using 物理量::物理量;
        constexpr mps(const kmph &v) noexcept;
    };

    /// キロメートル毎時
    struct kmph : 物理量<double, kmph>
    {
        using 物理量::物理量;
        constexpr kmph(const mps &v) noexcept;
    };

    constexpr mps::mps(const kmph &v) noexcept : 物理量(v.value / 3.6) {}
    constexpr kmph::kmph(const mps &v) noexcept : 物理量(v.value * 3.6) {}

    constexpr mps operator"" _mps(long double v) noexcept {
        return static_cast<mps>(static_cast<double>(v));
    }
    constexpr kmph operator"" _kmph(long double v) noexcept {
        return static_cast<kmph>(static_cast<double>(v));
    }

    constexpr mps operator/(const m &a, const s &b) noexcept {
        return static_cast<mps>(a.value / b.value);
    }
    constexpr m operator*(const mps &a, const s &b) noexcept {
        return static_cast<m>(a.value * b.value);
    }
    constexpr m operator*(const s &a, const mps &b) noexcept {
        return static_cast<m>(a.value * b.value);
    }
    constexpr s operator/(const m &a, const mps &b) noexcept {
        return static_cast<s>(a.value / b.value);
    }

    // 加速度

    struct mps2;
    struct kmphps;

    /// メートル毎秒毎秒
    struct mps2 : 物理量<double, mps2>
    {
        using 物理量::物理量;
        constexpr mps2(const kmphps &v) noexcept;
    };

    /// キロメートル毎時毎秒
    struct kmphps : 物理量<double, kmphps>
    {
        using 物理量::物理量;
        constexpr kmphps(const mps2 &v) noexcept;
    };

    constexpr mps2::mps2(const kmphps &v) noexcept : 物理量(v.value / 3.6) {}
    constexpr kmphps::kmphps(const mps2 &v) noexcept : 物理量(v.value * 3.6) {}

    constexpr mps2 operator"" _mps2(long double v) noexcept {
        return static_cast<mps2>(static_cast<double>(v));
    }
    constexpr kmphps operator"" _kmphps(long double v) noexcept {
        return static_cast<kmphps>(static_cast<double>(v));
    }

    constexpr mps2 operator/(const mps &a, const s &b) noexcept {
        return static_cast<mps2>(a.value / b.value);
    }
    constexpr mps operator*(const mps2 &a, const s &b) noexcept {
        return static_cast<mps>(a.value * b.value);
    }
    constexpr mps operator*(const s &a, const mps2 &b) noexcept {
        return static_cast<mps>(a.value * b.value);
    }
    constexpr s operator/(const mps &a, const mps2 &b) noexcept {
        return static_cast<s>(a.value / b.value);
    }

    // 加加速度

    struct mps3;
    struct kmphps2;

    /// メートル毎秒毎秒毎秒
    struct mps3 : 物理量<double, mps3>
    {
        using 物理量::物理量;
        constexpr mps3(const kmphps2 &v) noexcept;
    };

    /// キロメートル毎時毎秒毎秒
    struct kmphps2 : 物理量<double, kmphps2>
    {
        using 物理量::物理量;
        constexpr kmphps2(const mps3 &v) noexcept;
    };

    constexpr mps3::mps3(const kmphps2 &v) noexcept :
        物理量(v.value / 3.6) {}
    constexpr kmphps2::kmphps2(const mps3 &v) noexcept :
        物理量(v.value * 3.6) {}

    constexpr mps3 operator"" _mps3(long double v) noexcept {
        return static_cast<mps3>(static_cast<double>(v));
    }
    constexpr kmphps2 operator"" _kmphps2(long double v) noexcept {
        return static_cast<kmphps2>(static_cast<double>(v));
    }

    constexpr mps3 operator/(const mps2 &a, const s &b) noexcept {
        return static_cast<mps3>(a.value / b.value);
    }
    constexpr mps2 operator*(const mps3 &a, const s &b) noexcept {
        return static_cast<mps2>(a.value * b.value);
    }
    constexpr mps2 operator*(const s &a, const mps3 &b) noexcept {
        return static_cast<mps2>(a.value * b.value);
    }
    constexpr s operator/(const mps2 &a, const mps3 &b) noexcept {
        return static_cast<s>(a.value / b.value);
    }

    // 比エネルギー

    /// メートルメートル毎秒毎秒
    struct m2ps2 : 物理量<double, m2ps2>
    {
        using 物理量::物理量;
    };

    constexpr m2ps2 operator"" _m2ps2(long double v) noexcept {
        return static_cast<m2ps2>(static_cast<double>(v));
    }

    constexpr m2ps2 operator*(const mps &a, const mps &b) noexcept {
        return static_cast<m2ps2>(a.value * b.value);
    }
    constexpr m2ps2 operator*(const m &a, const mps2 &b) noexcept {
        return static_cast<m2ps2>(a.value * b.value);
    }
    constexpr m2ps2 operator*(const mps2 &a, const m &b) noexcept {
        return static_cast<m2ps2>(a.value * b.value);
    }
    constexpr mps2 operator/(const m2ps2 &a, const m &b) noexcept {
        return static_cast<mps2>(a.value / b.value);
    }

    inline mps sqrt(const m2ps2 &v) {
        return static_cast<mps>(std::sqrt(v.value));
    }
    inline mps 加算(const mps &v, const m2ps2 &e) {
        m2ps2 e2 = v * v + 2.0 * e;
        if (e2 < 0.0_m2ps2) {
            return 0.0_mps;
        }
        return sqrt(e2);
    }

}
