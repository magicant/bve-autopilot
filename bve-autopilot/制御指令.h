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
#include <variant>

namespace autopilot
{

    /// 車両の動きを制御する何らかの指令を表す自然数
    template<typename Self>
    struct 自然数ノッチ
    {
        unsigned value;
        constexpr explicit 自然数ノッチ(unsigned v) : value(v) {}
    };

    template<typename Self>
    constexpr bool operator==(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
    {
        return a.value == b.value;
    }

    template<typename Self>
    constexpr bool operator!=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
    {
        return a.value != b.value;
    }

    template<typename Self>
    constexpr bool operator<(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
    {
        return a.value < b.value;
    }

    template<typename Self>
    constexpr bool operator<=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
    {
        return a.value <= b.value;
    }

    template<typename Self>
    constexpr bool operator>(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
    {
        return a.value > b.value;
    }

    template<typename Self>
    constexpr bool operator>=(
        const 自然数ノッチ<Self> &a, const 自然数ノッチ<Self> &b)
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

    /// 力行の強さ
    struct 力行ノッチ : 自然数ノッチ<力行ノッチ>
    {
        using 自然数ノッチ::自然数ノッチ;
    };

    using 制御指令 =
        std::variant<手動制動自然数ノッチ, 自動制動自然数ノッチ, 力行ノッチ>;
    using 自動制御指令 = std::variant<自動制動自然数ノッチ, 力行ノッチ>;

    /// 制動の強さを何らかの単位で表す実数
    template<typename Self>
    struct 制動力
    {
        double value;
        constexpr explicit 制動力(double v) : value(v) {}
    };

    template<typename Self>
    constexpr bool operator==(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value == b.value;
    }

    template<typename Self>
    constexpr bool operator!=(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value != b.value;
    }

    template<typename Self>
    constexpr bool operator<(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value < b.value;
    }

    template<typename Self>
    constexpr bool operator<=(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value <= b.value;
    }

    template<typename Self>
    constexpr bool operator>(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value > b.value;
    }

    template<typename Self>
    constexpr bool operator>=(const 制動力<Self> &a, const 制動力<Self> &b)
    {
        return a.value >= b.value;
    }

    /// 自動制動自然数ノッチと同じ尺度で制動の強さを表す実数。
    /// 主に減速度や制動力割合を自動制動自然数ノッチに変換する中間値として
    /// 使用します。
    struct 自動制動実数ノッチ : 制動力<自動制動実数ノッチ>
    {
        using 制動力::制動力;
    };

    /// 制動の強さを最大常用ブレーキに対する割合で表したもの
    struct 制動力割合 : 制動力<制動力割合>
    {
        using 制動力::制動力;
    };

}
