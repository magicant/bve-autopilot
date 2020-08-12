// 力行特性テスト.cpp : 力行特性をテストします
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
#include "bve-autopilot/力行特性.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace autopilot;

namespace bveautopilottest
{
    TEST_CLASS(力行特性テスト)
    {
    public:

        TEST_METHOD(デフォルトの加速度_0kmph_性能設定なし)
        {
            力行特性 c;
            c.性能設定({}, 力行ノッチ{5});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.5_kmphps).value,
                c.加速度(力行ノッチ{1}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{5}, 0.0_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトの加速度_0kmph_性能設定あり)
        {
            力行特性 c;
            c.性能設定(
                {{20.0_kmph, {1.0_kmphps, 2.0_kmphps, 3.0_kmphps}}},
                力行ノッチ{5});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.5_kmphps).value,
                c.加速度(力行ノッチ{1}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{5}, 0.0_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトの加速度_10kmph)
        {
            力行特性 c;
            c.性能設定({}, 力行ノッチ{5});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.5_kmphps).value,
                c.加速度(力行ノッチ{1}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{5}, 10.0_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトの加速度_11kmph)
        {
            力行特性 c;
            c.性能設定({}, 力行ノッチ{5});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 10.1_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{1}, 10.1_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 10.1_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{5}, 10.1_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトでない加速度_0kmph)
        {
            力行特性 c;
            c.性能設定(
                {{0.0_kmph, {1.0_kmphps, 2.0_kmphps, 3.0_kmphps, 4.0_kmphps}}},
                力行ノッチ{4});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(1.0_kmphps).value,
                c.加速度(力行ノッチ{1}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 0.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(4.0_kmphps).value,
                c.加速度(力行ノッチ{4}, 0.0_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトでない加速度_10kmph_to_0kmph)
        {
            力行特性 c;
            c.性能設定(
                {{0.0_kmph, {1.0_kmphps, 2.0_kmphps, 3.0_kmphps}}},
                力行ノッチ{4});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(1.0_kmphps).value,
                c.加速度(力行ノッチ{1}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(3.0_kmphps).value,
                c.加速度(力行ノッチ{3}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{4}, 10.0_kmph).value,
                0.001);
        }

        TEST_METHOD(デフォルトでない加速度_10kmph_to_9kmph)
        {
            力行特性 c;
            c.性能設定(
                {{9.99_kmph, {1.0_kmphps, 2.0_kmphps, 3.0_kmphps}}},
                力行ノッチ{4});
            Assert::AreEqual(
                static_cast<mps2>(0.0_kmphps).value,
                c.加速度(力行ノッチ{0}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(1.0_kmphps).value,
                c.加速度(力行ノッチ{1}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(2.0_kmphps).value,
                c.加速度(力行ノッチ{2}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(3.0_kmphps).value,
                c.加速度(力行ノッチ{3}, 10.0_kmph).value,
                0.001);
            Assert::AreEqual(
                static_cast<mps2>(5.0_kmphps).value,
                c.加速度(力行ノッチ{4}, 10.0_kmph).value,
                0.001);
        }

    };
}
