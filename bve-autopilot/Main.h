// Main.h : プラグイン全体を統括します
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

namespace autopilot
{

    class Main
    {
    public:
        Main();
        ~Main();

        void 車両仕様設定(const ATS_VEHICLESPEC & 車両仕様);
        void リセット(int 制動状態);

        void 逆転器操作(int ノッチ);
        void 力行操作(int ノッチ);
        void 制動操作(int ノッチ);

        ATS_HANDLES 経過(const ATS_VEHICLESTATE & 状態, int * 出力値, int * 音声状態);

    private:
        ATS_VEHICLESPEC _車両仕様;
        int _逆転器ノッチ, _力行ノッチ, _制動ノッチ;
    };

}
