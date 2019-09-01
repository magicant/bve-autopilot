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
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "共通状態.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        std::vector<double> 実数列(LPCWSTR s) {
            std::vector<double> values;
            while (*s != L'\0') {
                LPWSTR s2;
                double value = std::wcstod(s, &s2);
                if (s2 == s) {
                    ++s;
                    continue;
                }
                values.push_back(value);
                s = s2;
            }
            return values;
        }

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
        _tasc初期起動(true),
        _ato初期起動(true),
        _車両長(20),
        _加速終了遅延(1),
        _常用最大減速度(mps_from_kmph(3)),
        _制動反応時間(0.2),
        _pressure_rates{},
        _キー割り当て{
            {キー操作::モード切替, ATS_KEY_L},
            {キー操作::ato発進, ATS_KEY_L}, },
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
        using namespace std::string_view_literals;

        constexpr std::size_t buffer_size = 256;
        WCHAR buffer[buffer_size];
        DWORD size;

        // 初期モード
        size = GetPrivateProfileStringW(
            L"init", L"mode", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            if (buffer == L"off"sv) {
                _tasc初期起動 = false;
                _ato初期起動 = false;
            }
            else if (buffer == L"tasc"sv) {
                _tasc初期起動 = true;
                _ato初期起動 = false;
            }
            else {
                _tasc初期起動 = true;
                _ato初期起動 = true;
            }
        }

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

        // 加速終了遅延
        size = GetPrivateProfileStringW(
            L"power", L"offdelay", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            時間型 遅延 = std::wcstod(buffer, nullptr);
            if (0 <= 遅延 && std::isfinite(遅延)) {
                _加速終了遅延 = 遅延;
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

        // 制動反応時間
        size = GetPrivateProfileStringW(
            L"braking", L"effectlag", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            時間型 反応時間 = std::wcstod(buffer, nullptr);
            if (反応時間 == 0) {
                _制動反応時間 = 0; // 負の 0 は正の 0 にする
            }
            else if (0 < 反応時間 && std::isfinite(反応時間)) {
                _制動反応時間 = 反応時間;
            }
        }

        // ブレーキ指令の強さ (pressure rates)
        size = GetPrivateProfileStringW(
            L"braking", L"pressurerates", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            _pressure_rates = 実数列(buffer);
        }

        // キー割り当て
        for (auto &i : std::initializer_list<std::pair<キー操作, LPCWSTR>>
            { {キー操作::モード切替, L"mode"},
              {キー操作::ato発進, L"atostart"} })
        {
            size = GetPrivateProfileStringW(
                L"key", i.second, L"", buffer, buffer_size, 設定ファイル名);
            if (0 < size && size < buffer_size - 1) {
                try {
                    int key = std::stoi(buffer);
                    if (key < ATS_KEY_S || ATS_KEY_L < key) {
                        key = -1;
                    }
                    _キー割り当て[i.first] = key;
                }
                catch (const std::invalid_argument &) {
                }
                catch (const std::out_of_range &) {
                }
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
