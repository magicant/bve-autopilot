// 信号前照査順守.cpp : 速度照査に引っ掛からないように減速する機能
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
#include "信号前照査順守.h"
#include <algorithm>
#include "信号順守.h"
#include "共通状態.h"

namespace autopilot
{

    信号前照査順守::信号前照査順守() = default;
    信号前照査順守::~信号前照査順守() = default;

    void 信号前照査順守::リセット()
    {
        _照査表.clear();
        _制限グラフ.消去();
    }

    void 信号前照査順守::地上子通過(const ATS_BEACONDATA &地上子,
        const 共通状態 &状態, const 信号順守 &信号)
    {
        switch (地上子.Type)
        {
        case 1016: { // 停止信号前速度設定
            照査型 照査;
            照査.照査位置 = 状態.現在位置() + 地上子.Optional / 1000;
            照査.照査速度 = mps_from_kmph(地上子.Optional % 1000);
            照査.信号位置 = 状態.現在位置() + 地上子.Distance;
            _照査表.push_back(照査);
            再計算(状態, 信号);
            break;
        }
        }
    }

    void 信号前照査順守::経過(const 共通状態 &状態)
    {
        距離型 最後尾 = 状態.現在位置() - 状態.列車長();
        _制限グラフ.通過(最後尾);
    }

    void 信号前照査順守::再計算(const 共通状態 &状態, const 信号順守 &信号)
    {
        _制限グラフ.消去();

        // 通過済の照査を削除
        auto i = std::remove_if(_照査表.begin(), _照査表.end(),
            [現在位置 = 状態.現在位置()](const 照査型 &照査) {
                return 照査.信号位置 < 現在位置;
            });
        _照査表.erase(i, _照査表.end());

        // 制限グラフを再作成
        for (const 照査型 &照査 : _照査表) {
            if (信号.制限速度(照査.信号位置) == 0) {
                _制限グラフ.制限区間追加(照査.照査位置, 照査.照査速度);
            }
        }
    }

}
