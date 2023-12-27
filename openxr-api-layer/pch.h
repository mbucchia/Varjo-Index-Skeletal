// MIT License
//
// Copyright(c) 2022-2023 Matthieu Bucchianeri
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

// Standard library.
#include <algorithm>
#include <cstdarg>
#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <set>

using namespace std::chrono_literals;

// Windows header files.
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#define NOMINMAX
#include <windows.h>
#include <unknwn.h>
#include <wrl.h>
#include <wil/resource.h>
#include <traceloggingactivity.h>
#include <traceloggingprovider.h>

using Microsoft::WRL::ComPtr;

// OpenXR + Windows-specific definitions.
#define XR_NO_PROTOTYPES
#define XR_USE_PLATFORM_WIN32
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

// OpenXR loader interfaces.
#include <loader_interfaces.h>

// OpenXR/DirectX utilities.
#include <XrError.h>
#include <XrMath.h>
#include <XrSide.h>
#include <XrToString.h>

// OpenVR.
#include <openvr.h>

// FMT formatter.
#include <fmt/format.h>

// Detours.
#include <detours.h>

// Helper to detour a class method.
template <class T, typename TMethod>
void DetourMethodAttach(T* instance, unsigned int methodOffset, TMethod hooked, TMethod& original) {
    if (original) {
        // Already hooked.
        return;
    }

    LPVOID* vtable = *((LPVOID**)instance);
    LPVOID target = vtable[methodOffset];

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());

    original = (TMethod)target;
    DetourAttach((PVOID*)&original, hooked);

    CHECK_MSG(DetourTransactionCommit() == NO_ERROR, "Detour failed");
}

template <class T, typename TMethod>
void DetourMethodDetach(T* instance, unsigned int methodOffset, TMethod hooked, TMethod& original) {
    if (!original) {
        // Not hooked.
        return;
    }

    LPVOID* vtable = *((LPVOID**)instance);
    LPVOID target = vtable[methodOffset];

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());

    DetourDetach((PVOID*)&original, hooked);

    CHECK_MSG(DetourTransactionCommit() == NO_ERROR, "Detour failed");

    original = nullptr;
}

#define DECLARE_DETOUR_FUNCTION(ReturnType, Callconv, FunctionName, ...)                                               \
    inline ReturnType(Callconv* g_original_##FunctionName)(##__VA_ARGS__) = nullptr;                                   \
    ReturnType Callconv hooked_##FunctionName(##__VA_ARGS__)
