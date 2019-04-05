// 環境設定.h : ライブラリの動作を制御する設定を管理します
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
#include <unordered_map>
#include "パネル出力.h"
#include "単位.h"

namespace autopilot
{

    class 環境設定
    {
    public:
        環境設定() { リセット(); }
        ~環境設定() = default;

        void リセット();
        void ファイル読込(LPCWSTR 設定ファイル名);

        距離型 車両長() const { return _車両長; }
        加速度型 常用最大減速度() const { return _常用最大減速度; }

        const std::unordered_map<int, パネル出力対象> & パネル出力対象登録簿()
            const {
            return _パネル出力対象登録簿;
        }

    private:
        距離型 _車両長;
        加速度型 _常用最大減速度;

        std::unordered_map<int, パネル出力対象> _パネル出力対象登録簿;
    };

}
