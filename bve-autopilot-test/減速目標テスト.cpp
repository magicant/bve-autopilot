// 減速目標テスト.cpp : 減速目標をテストします
//
// Copyright © 2020 Watanabe, Yuki
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

#include "pch.h"
#include "CppUnitTest.h"
#include "bve-autopilot/減速目標.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace autopilot;

namespace bveautopilottest
{
    TEST_CLASS(減速目標テスト)
    {
    public:

        TEST_METHOD(無限大な基準減速度での最低速度区間)
        {
            減速目標 t{100.0_m, 40.0_kmph, mps2::無限大()};
            {
                区間 i = t.最低速度区間({80.0_m, 90.0_m});
                Assert::AreEqual(90.0, i.始点.value, 0.001);
                Assert::AreEqual(90.0, i.終点.value, 0.001);
            }
            {
                区間 i = t.最低速度区間({90.0_m, 110.0_m});
                Assert::AreEqual(100.0, i.始点.value, 0.001);
                Assert::AreEqual(110.0, i.終点.value, 0.001);
            }
            {
                区間 i = t.最低速度区間({110.0_m, 120.0_m});
                Assert::AreEqual(110.0, i.始点.value, 0.001);
                Assert::AreEqual(120.0, i.終点.value, 0.001);
            }
        }

    };
}
