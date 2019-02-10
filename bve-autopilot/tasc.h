// tasc.h : TASC メインモジュール
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
#include "共通状態.h"

namespace autopilot {

    class tasc
    {
    public:
        tasc(const ATS_VEHICLESPEC & 車両仕様);
        ~tasc();

        void 経過(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2);

        int 出力制動ノッチ() const { return _出力制動ノッチ; }

    private:
        const ATS_VEHICLESPEC & _車両仕様;
        int _出力制動ノッチ;
    };

}
