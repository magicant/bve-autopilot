// bve-autopilot.cpp : DLL アプリケーション用にエクスポートされる関数を定義します。
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
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include "Main.h"

namespace {

    std::unique_ptr<autopilot::Main> main;

    std::filesystem::path dll_file_name;

    std::filesystem::path get_module_file_name(HMODULE hModule)
    {
        std::wstring name;
        name.resize(1u << 8);
        while (name.size() < 1u << 16) {
            DWORD s = GetModuleFileNameW(hModule, name.data(), name.size());
            if (0 < s && s < name.size()) {
                name.resize(s);
                name.shrink_to_fit();
                return name;
            }
            name.resize(name.size() * 2);
        }
        name.clear();
        name.shrink_to_fit();
        return std::move(name);
    }

    std::filesystem::path ini_file_name()
    {
        std::filesystem::path name = dll_file_name;
        if (!name.has_filename()) {
            name.clear();
        }
        else {
            name.replace_filename(L"autopilot.ini");
        }
        return name;
    }

}

BOOL APIENTRY DllMain(
    HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
        dll_file_name = get_module_file_name(hModule);
        break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
        break;
    case DLL_PROCESS_DETACH:
        dll_file_name.clear();
        break;
    }
    return TRUE;
}

ATS_API int WINAPI GetPluginVersion() {
    return ATS_VERSION;
}

ATS_API void WINAPI Load() {
    main = std::make_unique<autopilot::Main>();
}

ATS_API void WINAPI Dispose() {
    main = nullptr;
}

ATS_API void WINAPI SetVehicleSpec(ATS_VEHICLESPEC spec) {
    if (main != nullptr) {
        main->設定ファイル読込(ini_file_name().c_str());
        main->車両仕様設定(spec);
    }
}

ATS_API void WINAPI Initialize(int brake) {
    if (main != nullptr) {
        main->リセット(brake);
    }
}

ATS_API ATS_HANDLES WINAPI Elapse(
    ATS_VEHICLESTATE state, int *panelValues, int *soundStates) {
    if (main != nullptr) {
        return main->経過(state, panelValues, soundStates);
    }
    return ATS_HANDLES{};
}

ATS_API void WINAPI SetPower(int notch) {
    if (main != nullptr) {
        main->力行操作(notch);
    }
}

ATS_API void WINAPI SetBrake(int notch) {
    if (main != nullptr) {
        main->制動操作(notch);
    }
}

ATS_API void WINAPI SetReverser(int notch) {
    if (main != nullptr) {
        main->逆転器操作(notch);
    }
}

ATS_API void WINAPI KeyDown(int key) {
    if (main != nullptr) {
        main->キー押し(key);
    }
}

ATS_API void WINAPI KeyUp(int key) {
    if (main != nullptr) {
        main->キー放し(key);
    }
}

ATS_API void WINAPI HornBlow(int type) {
    if (main != nullptr) {
        main->警笛操作(type);
    }
}

ATS_API void WINAPI DoorOpen() {
    if (main != nullptr) {
        main->戸開();
    }
}

ATS_API void WINAPI DoorClose() {
    if (main != nullptr) {
        main->戸閉();
    }
}

ATS_API void WINAPI SetSignal(int aspect) {
    if (main != nullptr) {
        main->信号現示変化(aspect);
    }
}

ATS_API void WINAPI SetBeaconData(ATS_BEACONDATA data) {
    if (main != nullptr) {
        main->地上子通過(data);
    }
}
