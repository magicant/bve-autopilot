// tascテスト.cpp : TASC をテストします
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
#include "bve-autopilot/tasc.h"
#include <chrono>
#include <ratio>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace autopilot;

namespace Microsoft {
    namespace VisualStudio {
        namespace CppUnitTestFramework {

            template<> inline std::wstring ToString<イベント>(
                const イベント &e)
            {
                switch (e) {
                case イベント::なし:
                    return L"なし";
                case イベント::停止:
                    return L"停止";
                case イベント::戸開:
                    return L"戸開";
                case イベント::手動ブレーキ:
                    return L"手動ブレーキ";
                default:
                    return ToString(static_cast<int>(e));
                }
            }

        }
    }
}

namespace bveautopilottest
{
    TEST_CLASS(tascテスト)
    {
    public:

        TEST_METHOD_INITIALIZE(init)
        {
            _状態.リセット();
            _tasc.リセット();

            ATS_VEHICLESPEC 仕様;
            仕様.BrakeNotches = 8;
            仕様.PowerNotches = 4;
            仕様.AtsNotch = 1;
            仕様.B67Notch = 8;
            仕様.Cars = 1;
            _状態.車両仕様設定(仕様);
        }

        TEST_METHOD(位置設定前は非制御中)
        {
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));

            _状態.戸閉(true);
            _tasc.戸閉(_状態);
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(位置設定したら制御中)
        {
            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(制御リセット条件なしなら戸閉まで制御中)
        {
            auto デフォルト条件 = _状態.設定().tasc制御リセット条件();
            Assert::AreEqual(イベント::なし, デフォルト条件.タイミング);
            Assert::AreEqual(0.0, デフォルト条件.遅延.value);

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.制動操作(7);
            _tasc.制動操作(_状態);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.戸閉(true);
            _tasc.戸閉(_状態);
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(手動ブレーキで制御リセット)
        {
            _状態.設定().tasc制御リセット条件を設定(
                {イベント::手動ブレーキ, 1.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.制動操作(3);
            _tasc.制動操作(_状態);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.制動操作(4);
            _tasc.制動操作(_状態);
            状態更新(100.0_m, 0.0_kmph, 60.5_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            状態更新(100.0_m, 0.0_kmph, 61.0_s);
            _tasc.経過(_状態);
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(戸開で制御リセット)
        {
            _状態.設定().tasc制御リセット条件を設定(
                {イベント::戸開, 1.5_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            状態更新(100.0_m, 0.0_kmph, 61.0_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            状態更新(100.0_m, 0.0_kmph, 61.5_s);
            _tasc.経過(_状態);
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(停止で制御リセット)
        {
            _状態.設定().tasc制御リセット条件を設定(
                {イベント::停止, 0.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 1.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::IsTrue(_tasc.制御中(_状態.現在時刻()));

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::IsFalse(_tasc.制御中(_状態.現在時刻()));
        }

        TEST_METHOD(位置設定前は緩解)
        {
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);

            _状態.戸閉(true);
            _tasc.戸閉(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);

            状態更新(5.0_m, 10.0_kmph, 5.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);
        }

        TEST_METHOD(停止許容範囲内に停止したら転動防止)
        {
            auto デフォルト条件 = _状態.設定().tasc緩解条件();
            Assert::AreEqual(
                イベント::手動ブレーキ, デフォルト条件.タイミング);
            Assert::AreEqual(0.0, デフォルト条件.遅延.value);

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);
        }

        TEST_METHOD(停止許容範囲手前なら停止しても転動防止しない)
        {
            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 許容範囲設定地上子{1031, 0, 0.0f, 10};
            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(許容範囲設定地上子, 0.0_m, _状態);
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(99.75_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(0u, _tasc.出力ノッチ().制動成分().value);
        }

        TEST_METHOD(緩解条件なしなら戸閉まで転動防止)
        {
            _状態.設定().tasc緩解条件を設定({イベント::なし, 0.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            _状態.制動操作(7);
            _tasc.制動操作(_状態);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);

            _状態.戸閉(true);
            _tasc.戸閉(_状態);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);
        }

        TEST_METHOD(手動ブレーキで緩解)
        {
            _状態.設定().tasc緩解条件を設定({イベント::手動ブレーキ, 1.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            _状態.制動操作(3);
            _tasc.制動操作(_状態);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);

            _状態.制動操作(4);
            _tasc.制動操作(_状態);
            状態更新(100.0_m, 0.0_kmph, 60.5_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);

            状態更新(100.0_m, 0.0_kmph, 61.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);
        }

        TEST_METHOD(戸開で緩解)
        {
            _状態.設定().tasc緩解条件を設定({イベント::戸開, 4.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);

            _状態.戸閉(false);
            _tasc.戸開(_状態);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 63.5_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);

            状態更新(100.0_m, 0.0_kmph, 64.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);
        }

        TEST_METHOD(停止で緩解)
        {
            _状態.設定().tasc緩解条件を設定({イベント::停止, 2.0_s});

            _状態.戸閉(true);
            _tasc.戸閉(_状態);

            ATS_BEACONDATA 位置設定地上子{255, 0, 0.0f, 100};
            _tasc.地上子通過(位置設定地上子, 0.0_m, _状態);
            _tasc.経過(_状態);

            状態更新(99.9_m, 1.0_kmph, 50.0_s);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 60.0_s);
            _tasc.経過(_状態);

            状態更新(100.0_m, 0.0_kmph, 61.5_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().制動成分().value);

            状態更新(100.0_m, 0.0_kmph, 62.0_s);
            _tasc.経過(_状態);
            Assert::AreEqual(4u, _tasc.出力ノッチ().力行成分().value);
        }

    private:

        共通状態 _状態;
        tasc _tasc;

        void 状態更新(m 位置, kmph 速度, s 時間)
        {
            using ms = std::chrono::duration<int, std::milli>;

            ATS_VEHICLESTATE state;
            state.Location = 位置.value;
            state.Speed = static_cast<float>(速度.value);
            state.Time = std::chrono::duration_cast<ms>(
                std::chrono::duration<double>(時間.value)).count();
            state.BcPressure = 0;
            state.MrPressure = 0;
            state.ErPressure = 0;
            state.BpPressure = 0;
            state.SapPressure = 0;
            state.Current = 0;
            _状態.経過(state);
        }

    };
}
