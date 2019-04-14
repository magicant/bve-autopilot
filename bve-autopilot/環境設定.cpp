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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        std::vector<std::pair<std::wstring, std::wstring>>
            セクション内全設定(LPCWSTR 設定ファイル名, LPCWSTR セクション名) {

            constexpr std::size_t buffer1_size = 4096;
            WCHAR buffer1[buffer1_size];
            DWORD size = GetPrivateProfileStringW(
                セクション名, NULL, L"", buffer1, buffer1_size,
                設定ファイル名);
            PCWSTR key = buffer1;
            std::vector<std::pair<std::wstring, std::wstring>> 設定;

            if (size > buffer1_size) {
                return 設定;
            }
            while (*key != L'\0') {
                constexpr std::size_t buffer2_size = 256;
                WCHAR buffer2[buffer2_size];
                size = GetPrivateProfileStringW(
                    セクション名, key, L"", buffer2, buffer2_size,
                    設定ファイル名);
                if (0 < size && size < buffer2_size - 1) {
                    設定.emplace_back(key, buffer2);
                }

                key += std::char_traits<WCHAR>::length(key) + 1;
            }
            return 設定;
        }

    }

    環境設定::環境設定() :
        _車両長(20),
        _常用最大減速度(mps_from_kmph(3)),
        _制動緩解時間(1),
        _パネル出力対象登録簿()
    {
    }

    環境設定::~環境設定() = default;

    void 環境設定::リセット()
    {
        *this = 環境設定();
    }

    void 環境設定::ファイル読込(LPCWSTR 設定ファイル名)
    {
        constexpr std::size_t buffer_size = 32;
        WCHAR buffer[buffer_size];
        DWORD size;

        // 車両長
        size = GetPrivateProfileStringW(
            L"dynamics", L"carlength", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            距離型 車両長 = std::wcstod(buffer, nullptr);
            if (0 < 車両長 && std::isfinite(車両長)) {
                _車両長 = mps_from_kmph(車両長);
            }
        }

        // 常用最大減速度
        size = GetPrivateProfileStringW(
            L"braking", L"maxdeceleration", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            加速度型 減速度 = std::wcstod(buffer, nullptr);
            if (0 < 減速度 && std::isfinite(減速度)) {
                _常用最大減速度 = mps_from_kmph(減速度);
            }
        }

        // 制動緩解時間
        size = GetPrivateProfileStringW(
            L"braking", L"releasedelay", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            時間型 緩解時間 = std::wcstod(buffer, nullptr);
            if (緩解時間 == 0) {
                _制動緩解時間 = 0; // 負の 0 は正の 0 にする
            }
            else if (0 < 緩解時間 && std::isfinite(緩解時間)) {
                _制動緩解時間 = 緩解時間;
            }
        }

        // パネル出力対象
        for (auto 設定 : セクション内全設定(設定ファイル名, L"panel")) {
            try {
                int index = std::stoi(設定.first);
                if (index < 0 || 256 <= index) {
                    continue;
                }
                _パネル出力対象登録簿.insert_or_assign(
                    index, パネル出力対象::対象(設定.second));
            }
            catch (const std::invalid_argument &) {
            }
            catch (const std::out_of_range &) {
            }
        }
    }

}
