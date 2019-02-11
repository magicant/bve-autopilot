// 走行モデル.h : 走行列車の位置・速度・時刻をシミュレートします。
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

namespace autopilot {

    class 走行モデル
    {
    public:
        using 加速度型 = double;
        using 速度型 = double;
        using 距離型 = double;
        using 時間型 = double;

        走行モデル(距離型 位置, 速度型 速度, 時間型 時刻) :
            _位置(位置), _速度(速度), _時刻(時刻) { }
        ~走行モデル() { }

        距離型 位置() const { return _位置; }
        void 位置変更(距離型 位置) { _位置 = 位置; }
        速度型 速度() const { return _速度; }
        void 速度変更(速度型 速度) { _速度 = 速度; }
        時間型 時刻() const { return _時刻; }
        void 時刻変更(時間型 時刻) { _時刻 = 時刻; }

        void 指定時間走行(時間型 時間, 加速度型 加速度 = 0);
        void 指定時刻まで走行(時間型 時刻, 加速度型 加速度 = 0);
        void 指定距離走行(距離型 距離, 加速度型 加速度 = 0);
        void 指定位置まで走行(距離型 位置, 加速度型 加速度 = 0);
        void 指定速度まで走行(速度型 速度, 加速度型 加速度 = 0);

        static 加速度型 距離と速度による加速度(
            距離型 距離, 速度型 初速度, 速度型 終速度);

    private:
        距離型 _位置;
        速度型 _速度;
        時間型 _時刻;
    };

}
