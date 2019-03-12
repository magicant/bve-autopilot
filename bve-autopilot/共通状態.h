// 共通状態.h : プラグイン全体で使用する、ゲーム全体の状態量です
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
#include <forward_list>
#include "制動出力.h"
#include "制限区間.h"
#include "加速度計.h"
#include "単位.h"
#include "環境設定.h"

namespace autopilot {

    class 共通状態
    {
    public:
        void リセット();
        void 設定ファイル読込(LPCWSTR 設定ファイル名) {
            _設定.ファイル読込(設定ファイル名);
        }
        void 車両仕様設定(const ATS_VEHICLESPEC & 仕様);
        void 経過(const ATS_VEHICLESTATE & 状態);
        void 出力(const ATS_HANDLES & 出力);
        void 逆転器操作(int ノッチ);
        void 力行操作(int ノッチ);
        void 制動操作(int ノッチ);

        const 環境設定 & 設定() const { return _設定; }
        加速度型 常用最大減速度() const {
            return _制動出力.常用最大減速度();
        }
        距離型 現在位置() const { return _状態.Location; }
        速度型 現在速度() const { return mps_from_kmph(_状態.Speed); }
        int 逆転器ノッチ() const { return _逆転器ノッチ; }
        int 力行ノッチ() const { return _力行ノッチ; }
        int 制動ノッチ() const { return _制動ノッチ; }
        加速度型 加速度() const { return _加速度計.加速度(); }

        double 目標制動ノッチ(加速度型 減速度) const {
            return _制動出力.ノッチ(減速度);
        }

    private:
        環境設定 _設定;
        ATS_VEHICLESTATE _状態;
        std::forward_list<制限区間> _制限区間リスト;
        int _逆転器ノッチ, _力行ノッチ, _制動ノッチ;
        加速度計 _加速度計;
        制動出力 _制動出力;
        ATS_HANDLES _前回出力;
    };

}
