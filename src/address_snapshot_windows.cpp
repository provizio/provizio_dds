// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Symmetric with network_monitor_windows.cpp — pull the Windows SDK
// headers in here so they get processed before address_snapshot.h /
// common.h / Fast-DDS can include them with a different include order.
// _WIN32_WINNT / NTDDI_VERSION / NOMINMAX / WIN32_LEAN_AND_MEAN are
// pinned project-wide via add_compile_definitions in CMakeLists.txt.
#if defined(_WIN32)

#include <winsock2.h>
// clang-format off
#include <ws2tcpip.h>
#include <iphlpapi.h>
// clang-format on
#endif  // _WIN32

#include "provizio/dds/detail/address_snapshot.h"

#if defined(_WIN32)

#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Ws2_32.lib")

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace provizio::dds::detail
{
    namespace
    {
        // Friendly-name / description substrings that flag a virtual / container
        // adapter on Windows. Matched case-sensitively against the Windows-localised
        // FriendlyName and Description fields; vendors are stable enough that this
        // works in practice. Extended as new container runtimes are encountered.
        constexpr std::array<std::string_view, 7> excluded_description_substrings{
            "Hyper-V Virtual", "WSL",         "VirtualBox Host-Only",
            "VMware Virtual",  "TAP-Windows", "Loopback Pseudo-Interface",
            "Tunnel adapter",
        };

        bool description_excluded(const std::string &description)
        {
            for (const auto &needle : excluded_description_substrings)
            {
                if (description.find(needle) != std::string::npos)
                {
                    return true;
                }
            }
            return false;
        }

        std::string wide_to_utf8(const wchar_t *wide)
        {
            if (wide == nullptr || *wide == L'\0')
            {
                return {};
            }
            // WideCharToMultiByte with cchWideChar=-1 (NUL-terminated input)
            // returns the byte count INCLUDING the NUL terminator and writes
            // the NUL into the output buffer. Allocate the full `len` bytes,
            // let the API write the NUL, then trim it — sizing the buffer to
            // `len - 1` and asking the API for `len` bytes would write one
            // byte past the std::string's storage (heap corruption).
            const int len = ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, nullptr, 0, nullptr, nullptr);
            if (len <= 1)
            {
                return {};
            }
            std::string out(static_cast<std::size_t>(len), '\0');
            ::WideCharToMultiByte(CP_UTF8, 0, wide, -1, out.data(), len, nullptr, nullptr);
            out.resize(static_cast<std::size_t>(len) - 1);  // drop the NUL std::string already has.
            return out;
        }
    }  // namespace

    address_snapshot capture_address_snapshot()
    {
        address_snapshot snapshot;

        ULONG buffer_size = 16 * 1024;
        std::vector<std::uint8_t> buffer(buffer_size);
        DWORD result = 0;

        // GetAdaptersAddresses may need a larger buffer; loop while it tells us so.
        for (int attempts = 0; attempts < 3; ++attempts)
        {
            result = ::GetAdaptersAddresses(
                AF_UNSPEC, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr,
                reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data()), &buffer_size);
            if (result == ERROR_BUFFER_OVERFLOW)
            {
                buffer.resize(buffer_size);
                continue;
            }
            break;
        }

        if (result != NO_ERROR)
        {
            return snapshot;
        }

        for (auto *adapter = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data()); adapter != nullptr;
             adapter = adapter->Next)
        {
            if (adapter->OperStatus != IfOperStatusUp)
            {
                continue;
            }

            const bool is_ethernet_or_wifi = adapter->IfType == IF_TYPE_ETHERNET_CSMACD ||
                                             adapter->IfType == IF_TYPE_IEEE80211 || adapter->IfType == IF_TYPE_PPP;
            if (!is_ethernet_or_wifi)
            {
                continue;
            }

            const std::string friendly = wide_to_utf8(adapter->FriendlyName);
            const std::string description = wide_to_utf8(adapter->Description);
            if (description_excluded(friendly) || description_excluded(description))
            {
                continue;
            }

            const std::string adapter_name{adapter->AdapterName != nullptr ? adapter->AdapterName : ""};

            for (auto *uni = adapter->FirstUnicastAddress; uni != nullptr; uni = uni->Next)
            {
                if (uni->DadState != IpDadStatePreferred)
                {
                    continue;  // tentative / deprecated / duplicate
                }

                const auto *sa = uni->Address.lpSockaddr;
                if (sa == nullptr)
                {
                    continue;
                }
                if (sa->sa_family != AF_INET && sa->sa_family != AF_INET6)
                {
                    continue;
                }

                std::array<char, INET6_ADDRSTRLEN> addr_text{};
                if (sa->sa_family == AF_INET)
                {
                    const auto *sin = reinterpret_cast<const sockaddr_in *>(sa);
                    if (::inet_ntop(AF_INET, &sin->sin_addr, addr_text.data(),
                                    static_cast<std::size_t>(addr_text.size())) == nullptr)
                    {
                        continue;
                    }
                }
                else
                {
                    const auto *sin6 = reinterpret_cast<const sockaddr_in6 *>(sa);
                    if (IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr))
                    {
                        continue;
                    }
                    if (::inet_ntop(AF_INET6, &sin6->sin6_addr, addr_text.data(),
                                    static_cast<std::size_t>(addr_text.size())) == nullptr)
                    {
                        continue;
                    }
                }

                snapshot.insert({adapter_name.empty() ? friendly : adapter_name, std::string{addr_text.data()}});
            }
        }

        return snapshot;
    }
}  // namespace provizio::dds::detail

#endif  // _WIN32
