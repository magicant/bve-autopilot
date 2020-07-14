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
#include <bitset>
#include <cstddef>
#include <unordered_map>
#include <vector>
#include "パネル出力.h"
#include "稼働状態.h"
#include "制御指令.h"
#include "物理量.h"

#pragma warning(push)
#pragma warning(disable:4819)

namespace autopilot
{

    enum class キー操作
    {
        モード切替,
        モード切替逆,
        モード切替次,
        モード切替前,
        ato発進,
    };

    constexpr std::size_t キー種類数 = 16;
    using キー組合せ = std::bitset<キー種類数>;

    enum class イベント
    {
        なし,
        停止,
        戸開,
        手動ブレーキ,
    };

    struct リセット条件 {
        イベント タイミング;
        s 遅延;
    };

    enum class 音声
    {
        tasc無効設定音,
        ato無効設定音,
        ato有効設定音,
        ato発進音,
    };

    using 音声出力先 = int;

    class 環境設定
    {
    public:
        環境設定();
        ~環境設定();

        // void リセット();
        void ファイル読込(LPCWSTR 設定ファイル名);

        稼働状態 初期稼働状態() const noexcept { return _初期稼働状態; }
        const std::vector<稼働状態> &稼働状態切替順序() const noexcept {
            return _稼働状態切替順序;
        }
        m 車両長() const noexcept { return _車両長; }
        s 加速終了遅延() const noexcept { return _加速終了遅延; }
        mps2 常用最大減速度() const noexcept { return _常用最大減速度; }
        s 制動反応時間() const noexcept { return _制動反応時間; }
        自動制動自然数ノッチ 制動最大拡張ノッチ() const noexcept {
            return _制動最大拡張ノッチ;
        }
        制動力割合 転動防止制動割合() const noexcept {
            return _転動防止制動割合;
        }
        const std::vector<制動力割合> &pressure_rates() const noexcept {
            return _pressure_rates;
        }
        const リセット条件 &tasc制御リセット条件() const noexcept {
            return _tasc制御リセット条件;
        }
        void tasc制御リセット条件を設定(const リセット条件 &条件) noexcept {
            _tasc制御リセット条件 = 条件;
        }
        const リセット条件 &tasc緩解条件() const noexcept {
            return _tasc緩解条件;
        }
        void tasc緩解条件を設定(const リセット条件 &条件) noexcept {
            _tasc緩解条件 = 条件;
        }
        bool atc事前減速() const noexcept { return _atc事前減速; }
        bool ato一時停止あり() const noexcept { return _ato一時停止あり; }

        const std::unordered_map<キー操作, キー組合せ> &キー割り当て() const
            noexcept
        {
            return _キー割り当て;
        }

        const std::unordered_map<int, パネル出力対象> &パネル出力対象登録簿()
            const noexcept
        {
            return _パネル出力対象登録簿;
        }

        const std::unordered_map<音声, 音声出力先> &音声割り当て() const
            noexcept
        {
            return _音声割り当て;
        }

    private:
        稼働状態 _初期稼働状態;
        std::vector<稼働状態> _稼働状態切替順序;
        m _車両長;
        s _加速終了遅延;
        mps2 _常用最大減速度;
        s _制動反応時間;
        自動制動自然数ノッチ _制動最大拡張ノッチ;
        制動力割合 _転動防止制動割合;
        std::vector<制動力割合> _pressure_rates;
        リセット条件 _tasc制御リセット条件, _tasc緩解条件;
        bool _atc事前減速, _ato一時停止あり;

        std::unordered_map<キー操作, キー組合せ> _キー割り当て;
        std::unordered_map<int, パネル出力対象> _パネル出力対象登録簿;
        std::unordered_map<音声, 音声出力先> _音声割り当て;
    };

}

#pragma warning(pop)
