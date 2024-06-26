// 制御指令.h : 力行や制動の強さを表すデータを定義します
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

    /// 車両の動きを制御する何らかの指令を表す自然数
    template<typename Self>
    struct 自然数ノッチ
    {
        unsigned value;
        constexpr 自然数ノッチ() noexcept : value{} {}
        constexpr explicit 自然数ノッチ(unsigned v) noexcept : value(v) {}
    };

    template<typename Self>
    constexpr bool operator==(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value == b.value;
    }

    template<typename Self>
    constexpr bool operator!=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value != b.value;
    }

    template<typename Self>
    constexpr bool operator<(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value < b.value;
    }

    template<typename Self>
    constexpr bool operator<=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value <= b.value;
    }

    template<typename Self>
    constexpr bool operator>(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value > b.value;
    }

    template<typename Self>
    constexpr bool operator>=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b) noexcept
    {
        return a.value >= b.value;
    }

    /// ブレーキレバーによって手動でかけられた制動の強さ
    struct 手動制動自然数ノッチ : 自然数ノッチ<手動制動自然数ノッチ>
    {
        using 自然数ノッチ::自然数ノッチ;
    };

    /// TASC/ATO の制御によりかけられる制動の強さ。
    /// 拡張制動ノッチが設定されている場合は値はその中から選ばれます。
    /// 拡張制動ノッチが設定されていなければ手動制動と同じノッチを使用します。
    struct 自動制動自然数ノッチ : 自然数ノッチ<自動制動自然数ノッチ>
    {
        using 自然数ノッチ::自然数ノッチ;
    };

    /// BVE 本体側と実際にやり取りする制動指令の値
    struct 制動指令 {
        int value;
        constexpr explicit 制動指令(int v = 0) noexcept : value{v} {}
        constexpr 制動指令(手動制動自然数ノッチ v) noexcept :
            value{static_cast<int>(v.value)} {}
    };

    constexpr bool operator==(const 制動指令 &a, const 制動指令 &b) noexcept {
        return a.value == b.value;
    }

    constexpr bool operator!=(const 制動指令 &a, const 制動指令 &b) noexcept {
        return a.value != b.value;
    }

    /// 力行の強さ
    struct 力行ノッチ : 自然数ノッチ<力行ノッチ>
    {
        using 自然数ノッチ::自然数ノッチ;
    };

    class 自動制御指令 {
    public:
        constexpr 自動制御指令() noexcept : _value{} {}
        constexpr 自動制御指令(const 自動制動自然数ノッチ &ノッチ) noexcept :
            _value{-to_int(ノッチ.value)} {}
        constexpr 自動制御指令(const 力行ノッチ &ノッチ) noexcept :
            _value(to_int(ノッチ.value)) {}

        constexpr 自動制動自然数ノッチ 制動成分() const noexcept {
            return static_cast<自動制動自然数ノッチ>(
                _value <= 0 ? static_cast<unsigned>(-_value) : 0);
        }
        constexpr 力行ノッチ 力行成分() const noexcept {
            return static_cast<力行ノッチ>(
                _value >= 0 ? static_cast<unsigned>(_value) : 0);
        }

        constexpr bool operator==(const 自動制御指令 &v) const noexcept {
            return _value == v._value;
        }
        constexpr bool operator!=(const 自動制御指令 &v) const noexcept {
            return _value != v._value;
        }
        constexpr bool operator<(const 自動制御指令 &v) const noexcept {
            return _value < v._value;
        }
        constexpr bool operator<=(const 自動制御指令 &v) const noexcept {
            return _value <= v._value;
        }
        constexpr bool operator>(const 自動制御指令 &v) const noexcept {
            return _value > v._value;
        }
        constexpr bool operator>=(const 自動制御指令 &v) const noexcept {
            return _value >= v._value;
        }

    private:
        constexpr static int to_int(const unsigned &v) noexcept {
            return static_cast<int>(std::min(
                v, static_cast<unsigned>(std::numeric_limits<int>::max())));
        }

        // 力行なら正、制動なら負
        int _value;
    };

    /// 制動の強さを何らかの単位で表す実数
    template<typename Self>
    struct 制動力
    {
        double value;
        constexpr 制動力() noexcept : value{} {}
        constexpr explicit 制動力(double v) noexcept : value(v) {}
    };

    template<typename Self>
    constexpr bool operator==(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value == b.value;
    }

    template<typename Self>
    constexpr bool operator!=(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value != b.value;
    }

    template<typename Self>
    constexpr bool operator<(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value < b.value;
    }

    template<typename Self>
    constexpr bool operator<=(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value <= b.value;
    }

    template<typename Self>
    constexpr bool operator>(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value > b.value;
    }

    template<typename Self>
    constexpr bool operator>=(const 制動力<Self> &a, const 制動力<Self> &b)
        noexcept
    {
        return a.value >= b.value;
    }

    /// 自動制動自然数ノッチと同じ尺度で制動の強さを表す実数。
    /// 主に減速度や制動力割合を自動制動自然数ノッチに変換する中間値として
    /// 使用します。
    struct 自動制動実数ノッチ : 制動力<自動制動実数ノッチ>
    {
        using 制動力::制動力;

        constexpr 自動制動実数ノッチ(const 自動制動自然数ノッチ &v) noexcept :
            制動力(v.value) {}

        自動制動自然数ノッチ ceil() const
        {
            constexpr double max = std::numeric_limits<int>::max();
            double r = std::clamp(value, 0.0, max);
            unsigned n = static_cast<unsigned>(std::ceil(r));
            return 自動制動自然数ノッチ{n};
        }

        自動制動自然数ノッチ floor() const
        {
            constexpr double max = std::numeric_limits<int>::max();
            double r = std::clamp(value, 0.0, max);
            unsigned n = static_cast<unsigned>(std::floor(r));
            return 自動制動自然数ノッチ{n};
        }
    };

    /// 制動の強さを最大常用ブレーキに対する割合で表したもの
    struct 制動力割合 : 制動力<制動力割合>
    {
        using 制動力::制動力;
    };

}
