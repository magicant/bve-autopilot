// 環境設定.cpp : ライブラリの動作を制御する設定を管理します
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
#include "環境設定.h"
#include <cmath>
#include <cstddef>
#include <cwchar>
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    void 環境設定::リセット()
    {
        _常用最大減速度 = mps_from_kmph(3.0);
    }

    void 環境設定::ファイル読込(LPCWSTR 設定ファイル名)
    {
        constexpr std::size_t buffer_size = 32;
        WCHAR buffer[buffer_size];

        // 常用最大減速度
        DWORD size = GetPrivateProfileStringW(
            L"braking", L"maxdeceleration", L"", &buffer[0], buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            加速度型 減速度 = std::wcstod(buffer, nullptr);
            if (0 < 減速度 && std::isfinite(減速度)) {
                _常用最大減速度 = mps_from_kmph(減速度);
            }
        }
    }

}
