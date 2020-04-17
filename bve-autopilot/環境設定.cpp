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
#include <cwctype>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "共通状態.h"
#include "物理量.h"

namespace autopilot
{

    namespace
    {

        キー組合せ デフォルトキー組合せ() {
            キー組合せ 組合せ;
            組合せ[ATS_KEY_L] = true;
            return 組合せ;
        }

        std::vector<制動力割合> 実数列(LPCWSTR s) {
            std::vector<制動力割合> values;
            while (*s != L'\0') {
                LPWSTR s2;
                double value = std::wcstod(s, &s2);
                if (s2 == s) {
                    ++s;
                    continue;
                }
                values.emplace_back(value);
                s = s2;
            }
            return values;
        }

        // 無効なキーは std::out_of_range を投げる
        キー組合せ キー組合せを解析(std::wstring s) {
            キー組合せ 組合せ;

            while (std::iswdigit(s[0])) {
                std::size_t i;
                int キー = std::stoi(s, &i);
                if (キー < ATS_KEY_S || ATS_KEY_L < キー) {
                    throw std::out_of_range("invalid key");
                }
                組合せ.set(キー);

                // 空白を飛ばす
                while (std::iswblank(s[i])) {
                    ++i;
                }

                // & を飛ばす
                if (s[i] != L'&') {
                    break;
                }
                ++i;

                // もう一度 空白を飛ばす
                while (std::iswblank(s[i])) {
                    ++i;
                }

                // 飛ばした部分まで消す
                s.erase(0, i);
            }

            return 組合せ;
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
        _加速終了遅延(2.0_s),
        _常用最大減速度(3.0_kmphps),
        _制動反応時間(0.2_s),
        _制動最大拡張ノッチ{0},
        _転動防止制動割合(0.5),
        _pressure_rates{},
        _キー割り当て{
            {キー操作::モード切替, デフォルトキー組合せ()},
            {キー操作::ato発進, デフォルトキー組合せ()}, },
        _パネル出力対象登録簿(),
        _音声割り当て{}
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
            double 車両長 = std::wcstod(buffer, nullptr);
            if (0 < 車両長 && std::isfinite(車両長)) {
                _車両長 = static_cast<m>(車両長);
            }
        }

        // 加速終了遅延
        size = GetPrivateProfileStringW(
            L"power", L"offdelay", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            double 遅延 = std::wcstod(buffer, nullptr);
            if (0 <= 遅延 && std::isfinite(遅延)) {
                _加速終了遅延 = static_cast<s>(遅延);
            }
        }

        // 常用最大減速度
        size = GetPrivateProfileStringW(
            L"braking", L"maxdeceleration", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            double 減速度 = std::wcstod(buffer, nullptr);
            if (0 < 減速度 && std::isfinite(減速度)) {
                _常用最大減速度 = static_cast<kmphps>(減速度);
            }
        }

        // 制動反応時間
        size = GetPrivateProfileStringW(
            L"braking", L"effectlag", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            double 反応時間 = std::wcstod(buffer, nullptr);
            if (反応時間 == 0) {
                _制動反応時間 = 0.0_s; // 負の 0 は正の 0 にする
            }
            else if (0 < 反応時間 && std::isfinite(反応時間)) {
                _制動反応時間 = static_cast<s>(反応時間);
            }
        }

        // 制動拡張ノッチ数
        size = GetPrivateProfileStringW(
            L"braking", L"extendednotches", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            int count = std::stoi(buffer);
            if (count >= 0) {
                _制動最大拡張ノッチ =
                    自動制動自然数ノッチ{static_cast<unsigned>(count)};
            }
        }

        // 転動防止制動割合
        size = GetPrivateProfileStringW(
            L"braking", L"standbybrakerate", L"", buffer, buffer_size,
            設定ファイル名);
        if (0 < size && size < buffer_size - 1) {
            double 割合 = std::wcstod(buffer, nullptr);
            if (割合 == 0.0) {
                _転動防止制動割合 = 制動力割合{0.0}; // 負の 0 は正の 0 にする
            }
            else if (0.0 <= 割合 && 割合 <= 1.0) {
                _転動防止制動割合 = 制動力割合{割合};
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
                    _キー割り当て[i.first] = キー組合せを解析(buffer);
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

        // 音声出力対象
        for (auto &i : std::initializer_list<std::pair<音声, LPCWSTR>>
            { {音声::tasc無効設定音, L"tascdisabled"},
              {音声::ato無効設定音, L"atodisabled"},
              {音声::ato有効設定音, L"atoenabled"},
              {音声::ato発進音, L"atostart"} })
        {
            size = GetPrivateProfileStringW(
                L"sound", i.second, L"", buffer, buffer_size, 設定ファイル名);
            if (size <= 0 || buffer_size - 1 <= size) {
                continue;
            }
            int index = std::stoi(buffer);
            if (index < 0 || 256 <= index) {
                continue;
            }
            _音声割り当て[i.first] = index;
        }
    }

}
