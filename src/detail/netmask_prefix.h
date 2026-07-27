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

#ifndef DDS_DETAIL_NETMASK_PREFIX
#define DDS_DETAIL_NETMASK_PREFIX

#if !defined(_WIN32)

#include <cstddef>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

namespace provizio::dds::detail
{
    /**
     * @file netmask_prefix.h
     * @brief Shared by the Linux and macOS @c capture_address_snapshot backends, which
     * both obtain their netmask as a @c sockaddr from @c getifaddrs. Kept in one place so
     * the two platforms cannot drift on how a prefix length is derived — a divergence
     * would make otherwise-identical hosts produce different snapshot identities.
     */

    /**
     * @brief Convert a @c getifaddrs netmask to its CIDR prefix length.
     *
     * @param netmask The @c ifa_netmask entry; may be null (some point-to-point and
     * tunnel devices report none) and may be of any family.
     * @return Number of leading set bits, e.g. 24 for 255.255.255.0. 0 for a null mask
     * or a family other than @c AF_INET / @c AF_INET6.
     */
    inline unsigned int prefix_length_from_netmask(const sockaddr *netmask)
    {
        if (netmask == nullptr)
        {
            return 0;
        }

        const unsigned char *bytes = nullptr;
        std::size_t byte_count = 0;
        std::size_t address_offset = 0;
        if (netmask->sa_family == AF_INET)
        {
            // sockaddr → sockaddr_in is the canonical BSD-sockets idiom; there is no
            // safe alternative at this layer.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            const auto *mask4 = reinterpret_cast<const sockaddr_in *>(netmask);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            bytes = reinterpret_cast<const unsigned char *>(&mask4->sin_addr);
            byte_count = sizeof(mask4->sin_addr);
            address_offset = offsetof(sockaddr_in, sin_addr);
        }
        else if (netmask->sa_family == AF_INET6)
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            const auto *mask6 = reinterpret_cast<const sockaddr_in6 *>(netmask);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            bytes = reinterpret_cast<const unsigned char *>(&mask6->sin6_addr);
            byte_count = sizeof(mask6->sin6_addr);
            address_offset = offsetof(sockaddr_in6, sin6_addr);
        }
        else
        {
            return 0;
        }

#if defined(__APPLE__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
        // BSD-family getifaddrs copies each sockaddr using only its self-reported
        // sa_len, and a netmask is the canonical short one: trailing all-zero bytes are
        // simply omitted, so a /8 mask can be as little as sin_len = offsetof(sin_addr)
        // + 1. Reading the full 4 / 16 bytes would run past the object — usually into
        // the neighbouring sockaddr in the same buffer (a garbage prefix, hence spurious
        // or missed snapshot deltas), and past the allocation for the last entry. The
        // omitted bytes are zero by definition, and prefix counting stops at the first
        // zero bit anyway, so clamping loses nothing. Linux has no sa_len and glibc
        // always materialises a full-size mask, so this does not apply there.
        const auto reported_len = static_cast<std::size_t>(netmask->sa_len);
        const std::size_t available = reported_len > address_offset ? reported_len - address_offset : 0;
        if (available < byte_count)
        {
            byte_count = available;
        }
#else
        (void)address_offset;
#endif

        unsigned int bits = 0;
        for (std::size_t i = 0; i < byte_count; ++i)
        {
            // The kernel only ever produces contiguous masks, so counting whole 0xFF
            // bytes and then the leading bits of the first partial byte is exact.
            if (bytes[i] == 0xFFU)
            {
                bits += 8;
                continue;
            }
            for (unsigned int bit = 0; bit < 8; ++bit)
            {
                if ((bytes[i] & (0x80U >> bit)) == 0)
                {
                    break;
                }
                ++bits;
            }
            break;
        }
        return bits;
    }
}  // namespace provizio::dds::detail

#endif  // !_WIN32

#endif  // DDS_DETAIL_NETMASK_PREFIX
