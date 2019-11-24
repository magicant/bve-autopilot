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
#include "制御指令.h"
#include "パネル出力.h"
#include "物理量.h"

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

    enum class 音声
    {
        tasc無効設定音,
        ato無効設定音,
        ato有効設定音,
    };

    using 音声出力先 = int;

    class 環境設定
    {
    public:
        環境設定();
        ~環境設定();

        void リセット();
        void ファイル読込(LPCWSTR 設定ファイル名);

        bool tasc初期起動() const { return _tasc初期起動; }
        bool ato初期起動() const { return _ato初期起動; }
        m 車両長() const { return _車両長; }
        s 加速終了遅延() const { return _加速終了遅延; }
        mps2 常用最大減速度() const { return _常用最大減速度; }
        s 制動反応時間() const { return _制動反応時間; }
        自動制動自然数ノッチ 制動最大拡張ノッチ() const {
            return _制動最大拡張ノッチ;
        }
        制動力割合 転動防止制動割合() const { return _転動防止制動割合; }
        const std::vector<制動力割合> &pressure_rates() const {
            return _pressure_rates;
        }

        const std::unordered_map<キー操作, キー番号> &キー割り当て() const {
            return _キー割り当て;
        }

        const std::unordered_map<int, パネル出力対象> & パネル出力対象登録簿()
            const {
            return _パネル出力対象登録簿;
        }

        const std::unordered_map<音声, 音声出力先> &音声割り当て() const {
            return _音声割り当て;
        }

    private:
        bool _tasc初期起動, _ato初期起動;
        m _車両長;
        s _加速終了遅延;
        mps2 _常用最大減速度;
        s _制動反応時間;
        自動制動自然数ノッチ _制動最大拡張ノッチ;
        制動力割合 _転動防止制動割合;
        std::vector<制動力割合> _pressure_rates;

        std::unordered_map<キー操作, キー番号> _キー割り当て;
        std::unordered_map<int, パネル出力対象> _パネル出力対象登録簿;
        std::unordered_map<音声, 音声出力先> _音声割り当て;
    };

}

#pragma warning(pop)
