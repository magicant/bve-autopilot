// tasc.cpp : TASC メインモジュール
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
#include "tasc.h"

namespace autopilot {

    tasc::tasc(const ATS_VEHICLESPEC & 車両仕様) :
        _車両仕様(車両仕様), _出力制動ノッチ{}
    {
    }

    tasc::~tasc()
    {
    }

    void tasc::経過(const ATS_VEHICLESTATE & 状態1, const 共通状態 & 状態2)
    {
        // TODO
    }

}
