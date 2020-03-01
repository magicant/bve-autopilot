// live.h : 他のクラスから変更を監視可能な汎用オブジェクトです
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
#include <functional>
#include <utility>

namespace autopilot
{

    template<typename T>
    class live
    {
    public:
        using observer_type = std::function<void(const T &)>;

        constexpr explicit live(const T &value) :_value(value) { }
        constexpr explicit live(T &&value) : _value{std::move(value)} { }

        constexpr const T &get() const { return _value; }

        template<typename U>
        void set(U &&new_value) {
            _value = std::forward<U>(new_value);
            notify();
        }

        template<typename U>
        void set_observer(U &&new_observer) {
            _observer = std::forward<U>(new_observer);
            notify();
        }

    private:
        T _value;
        observer_type _observer; // Support only one observer for now.

        void notify() const {
            if (_observer) {
                _observer(get());
            }
        }
    };

}
