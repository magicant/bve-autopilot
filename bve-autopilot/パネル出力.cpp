// パネル出力.cpp : 運転台パネルへ出力する値を定義します
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
#include "パネル出力.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include "Main.h"
#include "単位.h"

namespace autopilot
{

    namespace
    {

        const std::unordered_map<std::wstring, パネル出力対象> 対象名簿 = {
            {L"brake", パネル出力対象([](const Main & main) {
                return main.状態().前回出力().Brake;
            })},
            {L"power", パネル出力対象([](const Main & main) {
                return main.状態().前回出力().Power;
            })},

            {L"tascenabled", パネル出力対象([](const Main & main) {
                return main.tasc有効();
            })},
            {L"tascmonitor", パネル出力対象([](const Main & main) {
                return main.tasc有効() && main.tasc状態().制御中();
            })},
            {L"tascbrake", パネル出力対象([](const Main & main) {
                if (!main.tasc有効()) {
                    return 0;
                }
                return std::max(-main.tasc状態().出力ノッチ(), 0);
            })},
            {L"atoenabled", パネル出力対象([](const Main & main) {
                return main.ato有効();
            })},
            {L"speedlimit", パネル出力対象([](const Main & main) {
                速度型 制限速度 = main.現在制限速度();
                double 出力 = kmph_from_mps(制限速度);
                if (!std::isfinite(出力)) {
                    出力 = -20;
                }
                return static_cast<int>(出力);
            })},
        };

        const パネル出力対象 無対象{ [](const Main &) { return 0; } };

    }

    パネル出力対象 パネル出力対象::対象(const std::wstring & 名前)
    {
        auto i = 対象名簿.find(名前);
        if (i == 対象名簿.end()) {
            return 無対象;
        }
        return i->second;
    }

}
