// 制動特性.cpp : 期待する減速度を得るために制動ノッチを加減します
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
#include "制動特性.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include "共通状態.h"

namespace autopilot
{

    namespace
    {

        bool 圧力差が小さい(float 圧力1, float 圧力2) {
            if (圧力1 == 圧力2) {
                return true;
            }
            float 比 = 圧力1 / 圧力2;
            return 0.999f <= 比 && 比 <= 1 / 0.999f;
        }

    }

    制動特性::制動特性() = default;
    制動特性::~制動特性() = default;

    void 制動特性::性能設定(
        int 標準ノッチ数, int 拡張ノッチ数,
        加速度型 基準最大減速度, 時間型 反応時間,
        const std::vector<double> &pressure_rates)
    {
        _標準ノッチ数 = 標準ノッチ数;
        _基準最大減速度 = 基準最大減速度;
        _反応時間 = 反応時間;

        auto 標準ノッチ列最大長 =
            static_cast<pressure_rates::size_type>(標準ノッチ数) + 2;
        _標準ノッチ列 = pressure_rates;
        _標準ノッチ列.穴埋めする(標準ノッチ数);
        if (_標準ノッチ列.size() > 標準ノッチ列最大長) {
            // 拡張ノッチ列相当部分は取り除く
            _標準ノッチ列.resize(標準ノッチ列最大長);
        }

        auto 拡張ノッチ列最大長 =
            static_cast<pressure_rates::size_type>(拡張ノッチ数) + 1;
        _拡張ノッチ列.clear();
        if (pressure_rates.size() > 標準ノッチ列最大長) {
            _拡張ノッチ列.push_back(0);
            std::copy(
                pressure_rates.begin() + 標準ノッチ列最大長,
                pressure_rates.end(),
                std::back_inserter(_拡張ノッチ列));
            if (_拡張ノッチ列.size() > 拡張ノッチ列最大長) {
                // 余った分は取り除く
                _拡張ノッチ列.resize(拡張ノッチ列最大長);
            }
            if (_拡張ノッチ列.size() <= 1) {
                // 緩めノッチしかないのはダメ
                _拡張ノッチ列.clear();
            }
        }

        _推定最大減速度 = 基準最大減速度;
    }

    int 制動特性::拡張ノッチ数() const
    {
        if (_拡張ノッチ列.empty()) {
            return 0;
        }
        return _拡張ノッチ列.size() - 1;
    }

    int 制動特性::自動ノッチ数() const {
        int c = 拡張ノッチ数();
        return c > 0 ? c : _標準ノッチ数;
    }

    double 制動特性::標準ノッチ(加速度型 減速度) const
    {
        double 割合 = 減速度 / _推定最大減速度;
        return _標準ノッチ列.ノッチ(割合);
    }

    double 制動特性::自動ノッチ(加速度型 減速度) const
    {
        if (_拡張ノッチ列.empty()) {
            return 標準ノッチ(減速度);
        }
        double 割合 = 減速度 / _推定最大減速度;
        return _拡張ノッチ列.ノッチ(割合);
    }

    加速度型 制動特性::標準ノッチ減速度(double ノッチ) const
    {
        double 割合 = _標準ノッチ列.割合(ノッチ);
        return _推定最大減速度 * 割合;
    }

    加速度型 制動特性::自動ノッチ減速度(double ノッチ) const
    {
        double 割合;
        if (_拡張ノッチ列.empty()) {
            割合 = _標準ノッチ列.割合(ノッチ);
        }
        else {
            割合 = _拡張ノッチ列.割合(ノッチ);
        }
        return _推定最大減速度 * 割合;
    }

    int 制動特性::自動ノッチ番号(int 自動ノッチ) const
    {
        if (自動ノッチ == 0) {
            return 0;
        }
        if (_拡張ノッチ列.empty()) {
            return 自動ノッチ;
        }
        return 自動ノッチ + _標準ノッチ数 + 1;
    }

    int 制動特性::自動ノッチインデクス(int ノッチ番号) const
    {
        if (ノッチ番号 > _標準ノッチ数 + 1) {
            return ノッチ番号 - _標準ノッチ数 - 1;
        }
        if (_拡張ノッチ列.empty()) {
            return ノッチ番号;
        }

        double 割合 = _標準ノッチ列.割合(ノッチ番号);
        double インデクス = _拡張ノッチ列.ノッチ(割合);
        return static_cast<int>(std::ceil(インデクス));
    }

    double 制動特性::自動ノッチ丸め(double ノッチ) const
    {
        if (_拡張ノッチ列.empty()) {
            return _標準ノッチ列.丸め(ノッチ);
        }
        else {
            return _拡張ノッチ列.丸め(ノッチ);
        }
    }

    void 制動特性::経過(const 共通状態 &状態)
    {
        if (_前回出力ノッチ != 状態.前回出力().Brake) {
            _前回出力ノッチ = 状態.前回出力().Brake;
            _前回出力ノッチ変化時刻 = 状態.現在時刻();
        }

        double 割合 = this->割合(状態.前回出力().Brake);
        bool 割合安定 = 0.1 <= 割合 && 割合 <= 1.0;
        if (割合安定 && 状態.現在電流() == 0 &&
            std::abs(_前回出力ノッチ変化時刻 - 状態.現在時刻()) >= _反応時間 &&
            圧力差が小さい(_前回観測圧力, 状態.現在ブレーキシリンダー圧()))
        {
            _推定最大圧力 = static_cast<float>(
                状態.現在ブレーキシリンダー圧() / 割合);
        }
        else {
            割合 = 状態.現在ブレーキシリンダー圧() / _推定最大圧力;
            割合安定 = 0.1 <= 割合 && 割合 <= 1.0;
        }

        if (!割合安定 || 状態.現在電流() != 0) {
            _前回不安定時刻 = 状態.現在時刻();
        }

        if (状態.現在速度() >= mps_from_kmph(0.1)) {
            if (std::abs(_前回不安定時刻 - 状態.現在時刻()) >= 0.5) {
                最大減速度を推定(割合, 状態);
            }
            else {
                _推定最大減速度 = _基準最大減速度;
            }
        }

        _前回観測圧力 = 状態.現在ブレーキシリンダー圧();
        _前回観測時刻 = 状態.現在時刻();
    }

    double 制動特性::割合(int ノッチ番号) const
    {
        auto 標準ノッチ列最大長 = _標準ノッチ数 + 2;
        if (ノッチ番号 >= 標準ノッチ列最大長) {
            return _拡張ノッチ列.割合(ノッチ番号 - 標準ノッチ列最大長);
        }
        else {
            return _標準ノッチ列.割合(ノッチ番号);
        }
    }

    void 制動特性::最大減速度を推定(double 割合, const 共通状態 &状態)
    {
        時間型 フレーム = 状態.現在時刻() - _前回観測時刻;
        if (フレーム <= 0.0 || 1.0 <= フレーム) {
            return;
        }

        加速度型 出力減速度 = 状態.車両勾配加速度() - 状態.加速度();
        if (出力減速度 < mps_from_kmph(0.1)) {
            return;
        }

        加速度型 変化量上限 = mps_from_kmph(1.0) * フレーム;
        加速度型 新推定値 = 出力減速度 / 割合;
        _推定最大減速度 = std::clamp(新推定値,
            _推定最大減速度 - 変化量上限, _推定最大減速度 + 変化量上限);
    }

    void 制動特性::pressure_rates::穴埋めする(size_type 常用ノッチ数)
    {
        if (empty()) {
            push_back(0);
        }
        while (size() <= 常用ノッチ数) {
            auto 分割数 = 常用ノッチ数 - size() + 1;
            auto 前の値 = back();
            auto 次の値 = 前の値 + (1 - 前の値) / 分割数;
            push_back(次の値);
        }
    }

    double 制動特性::pressure_rates::ノッチ(double 割合) const
    {
        auto i = std::lower_bound(begin(), end(), 割合);
        if (i == begin()) {
            return 0;
        }
        if (i == end()) {
            return 割合 * size();
        }

        double 次ノッチ割合 = *i;
        --i;
        double 前ノッチ割合 = *i;
        return std::distance(begin(), i) +
            (割合 - 前ノッチ割合) / (次ノッチ割合 - 前ノッチ割合);
    }

    std::pair<double, double> 制動特性::pressure_rates::割合と丸め閾値(
        double ノッチ) const
    {
        assert(size() > 0);

        if (ノッチ <= 0) {
            return {0, 0};
        }

        auto s1 = size() - 1;
        if (ノッチ >= s1) {
            return {ノッチ / s1, s1};
        }

        size_type i = static_cast<size_type>(ノッチ);
        assert(i < s1);
        double 前ノッチ割合 = (*this)[i];
        double 次ノッチ割合 = (*this)[i + 1];
        double 割合 = 前ノッチ割合;
        if (前ノッチ割合 < 次ノッチ割合) {
            割合 += (ノッチ - i) * (次ノッチ割合 - 前ノッチ割合);
        }
        double 丸め閾値 = 次ノッチ割合 / (次ノッチ割合 - 前ノッチ割合 + 1);
        return {割合, 丸め閾値};
    }

    double 制動特性::pressure_rates::割合(double ノッチ) const
    {
        return 割合と丸め閾値(ノッチ).first;
    }

    double 制動特性::pressure_rates::丸め(double ノッチ) const
    {
        double 割合, 丸め閾値;
        std::tie(割合, 丸め閾値) = 割合と丸め閾値(ノッチ);
        return 割合 <= 丸め閾値 ? std::floor(ノッチ) : std::ceil(ノッチ);
    }

}
