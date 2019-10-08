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
#include <vector>
#include "パネル出力.h"
#include "単位.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    enum class キー操作
    {
        モード切替,
        ato発進,
    };

    using キー番号 = int;

    class 環境設定
    {
    public:
        環境設定();
        ~環境設定();

        void リセット();
        void ファイル読込(LPCWSTR 設定ファイル名);

        bool tasc初期起動() const { return _tasc初期起動; }
        bool ato初期起動() const { return _ato初期起動; }
        距離型 車両長() const { return _車両長; }
        時間型 加速終了遅延() const { return _加速終了遅延; }
        加速度型 常用最大減速度() const { return _常用最大減速度; }
        時間型 制動反応時間() const { return _制動反応時間; }
        int 制動拡張ノッチ数() const { return _制動拡張ノッチ数; }
        double 転動防止制動割合() const { return _転動防止制動割合; }
        const std::vector<double> &pressure_rates() const {
            return _pressure_rates;
        }

        const std::unordered_map<キー操作, キー番号> &キー割り当て() const {
            return _キー割り当て;
        }

        const std::unordered_map<int, パネル出力対象> & パネル出力対象登録簿()
            const {
            return _パネル出力対象登録簿;
        }

    private:
        bool _tasc初期起動, _ato初期起動;
        距離型 _車両長;
        時間型 _加速終了遅延;
        加速度型 _常用最大減速度;
        時間型 _制動反応時間;
        int _制動拡張ノッチ数;
        double _転動防止制動割合;
        std::vector<double> _pressure_rates;

        std::unordered_map<キー操作, キー番号> _キー割り当て;
        std::unordered_map<int, パネル出力対象> _パネル出力対象登録簿;
    };

}

#pragma warning(pop)
