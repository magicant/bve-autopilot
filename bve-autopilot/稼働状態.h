// 稼働状態.h : ATO/TASC がそれぞれ有効になっているかどうかを表します
//
// Copyright © 2020 Watanabe, Yuki
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

    enum class 稼働状態
    {
        切,
        tascのみ有効,
        ato有効,
    };

    constexpr bool tasc有効(稼働状態 s) noexcept {
        return s != 稼働状態::切;
    }
    constexpr bool ato有効(稼働状態 s) noexcept {
        return s == 稼働状態::ato有効;
    }

}
