// 勾配グラフテスト.cpp : 勾配グラフをテストします
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
#include "bve-autopilot/勾配グラフ.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace autopilot;

namespace bveautopilottest
{
    TEST_CLASS(勾配加速度グラフテスト)
    {
    public:

        TEST_METHOD(空のグラフの勾配加速度)
        {
            勾配加速度グラフ g;
            Assert::IsTrue(g.empty());
            Assert::AreEqual(0.0, g.勾配加速度(0.0_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(1.0_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(5.0_m).value, 0.0);
        }

        TEST_METHOD(勾配一つのグラフの勾配加速度)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({10.0_m, 20.0_m}, 0.02);
            Assert::IsFalse(g.empty());
            Assert::AreEqual(0.0, g.勾配加速度(0.0_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(9.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(10.0_m).value, 0.0);
            Assert::AreEqual(-0.147, g.勾配加速度(20.0_m).value, 0.001);
            Assert::AreEqual(-0.147, g.勾配加速度(30.0_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(15.0_m).value, 0.001);
            Assert::AreEqual(-0.037, g.勾配加速度(12.5_m).value, 0.001);
        }

        TEST_METHOD(離れた勾配二つのグラフの勾配加速度_昇順)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({10.0_m, 20.0_m}, 0.01);
            g.勾配変化追加({40.0_m, 45.0_m}, -0.02);
            Assert::AreEqual(0.0, g.勾配加速度(9.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(10.0_m).value, 0.0);
            Assert::AreEqual(-0.0184, g.勾配加速度(12.5_m).value, 0.0001);
            Assert::AreEqual(-0.074, g.勾配加速度(20.0_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(20.1_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(40.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(45.0_m).value, 0.001);
            Assert::AreEqual(0.0, g.勾配加速度(42.5_m).value, 0.001);
        }

        TEST_METHOD(離れた勾配二つのグラフの勾配加速度_降順)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({40.0_m, 45.0_m}, -0.02);
            g.勾配変化追加({10.0_m, 20.0_m}, 0.01);
            Assert::AreEqual(0.0, g.勾配加速度(9.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(10.0_m).value, 0.0);
            Assert::AreEqual(-0.0184, g.勾配加速度(12.5_m).value, 0.0001);
            Assert::AreEqual(-0.074, g.勾配加速度(20.0_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(20.1_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(40.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(45.0_m).value, 0.001);
            Assert::AreEqual(0.0, g.勾配加速度(42.5_m).value, 0.001);
        }

        TEST_METHOD(重なった勾配二つのグラフの勾配加速度_昇順)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({100.0_m, 120.0_m}, 0.005);
            g.勾配変化追加({110.0_m, 130.0_m}, -0.015);
            Assert::AreEqual(0.0, g.勾配加速度(99.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(100.0_m).value, 0.0);
            Assert::AreEqual(-0.009, g.勾配加速度(105.0_m).value, 0.001);
            Assert::AreEqual(-0.018, g.勾配加速度(110.0_m).value, 0.001);
            Assert::AreEqual(0.0, g.勾配加速度(115.0_m).value, 0.001);
            Assert::AreEqual(0.018, g.勾配加速度(120.0_m).value, 0.001);
            Assert::AreEqual(0.046, g.勾配加速度(125.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(130.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(130.1_m).value, 0.001);
        }

        TEST_METHOD(重なった勾配二つのグラフの勾配加速度_降順)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({110.0_m, 130.0_m}, -0.015);
            g.勾配変化追加({100.0_m, 120.0_m}, 0.005);
            Assert::AreEqual(0.0, g.勾配加速度(99.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(100.0_m).value, 0.0);
            Assert::AreEqual(-0.009, g.勾配加速度(105.0_m).value, 0.001);
            Assert::AreEqual(-0.018, g.勾配加速度(110.0_m).value, 0.001);
            Assert::AreEqual(0.0, g.勾配加速度(115.0_m).value, 0.001);
            Assert::AreEqual(0.018, g.勾配加速度(120.0_m).value, 0.001);
            Assert::AreEqual(0.046, g.勾配加速度(125.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(130.0_m).value, 0.001);
            Assert::AreEqual(0.074, g.勾配加速度(130.1_m).value, 0.001);
        }

        TEST_METHOD(一致する勾配二つのグラフの勾配加速度)
        {
            勾配加速度グラフ g;
            g.勾配変化追加({10.0_m, 20.0_m}, 0.04);
            g.勾配変化追加({10.0_m, 20.0_m}, -0.02);
            Assert::AreEqual(0.0, g.勾配加速度(9.9_m).value, 0.0);
            Assert::AreEqual(0.0, g.勾配加速度(10.0_m).value, 0.0);
            Assert::AreEqual(-0.147, g.勾配加速度(20.0_m).value, 0.001);
            Assert::AreEqual(-0.147, g.勾配加速度(20.1_m).value, 0.001);
            Assert::AreEqual(-0.074, g.勾配加速度(15.0_m).value, 0.001);
            Assert::AreEqual(-0.037, g.勾配加速度(12.5_m).value, 0.001);
        }

    };
}
