// 音声出力.h : 音声出力の状態を制御する値を切り替えます
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
#include <utility>

namespace autopilot
{

    class 音声出力
    {
    public:
        void 次に出力(int 次の出力) {
            _次の出力 = 次の出力;
        }

        int 出力() {
            return std::exchange(_次の出力, ATS_SOUND_CONTINUE);
        }

    private:
        int _次の出力 = ATS_SOUND_CONTINUE;
    };

}
