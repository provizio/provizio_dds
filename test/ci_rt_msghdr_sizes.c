/* Copyright 2026 Provizio Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * TEMPORARY investigation probe. src/network_monitor_macos.cpp looks at a routing message
 * only when recv() returned MORE than sizeof(struct rt_msghdr) bytes. rt_msghdr is the
 * ROUTE-ENTRY header and carries a whole rt_metrics; an address event uses ifa_msghdr,
 * which is a fraction of that size, plus a few sockaddrs. If a real RTM_NEWADDR comes to
 * fewer bytes than sizeof(struct rt_msghdr), that check silently discards every address
 * event on macOS -- and the C++ network monitor then never sees an address appear or
 * disappear. This prints the numbers that settle it on the actual runner.
 */

#include <net/if.h>
#include <net/if_dl.h>
#include <net/route.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>

int main(void)
{
    printf("sizeof(struct rt_msghdr)   = %zu   <- network_monitor_macos.cpp's acceptance threshold\n",
           sizeof(struct rt_msghdr));
    printf("sizeof(struct ifa_msghdr)  = %zu   <- the header an RTM_NEWADDR/RTM_DELADDR actually uses\n",
           sizeof(struct ifa_msghdr));
    printf("sizeof(struct if_msghdr)   = %zu   <- RTM_IFINFO\n", sizeof(struct if_msghdr));
    printf("sizeof(struct sockaddr_in) = %zu\n", sizeof(struct sockaddr_in));
    printf("sizeof(struct sockaddr_dl) = %zu\n", sizeof(struct sockaddr_dl));

    /* A typical IPv4 address event on a /24 carries RTAX_NETMASK, RTAX_IFP, RTAX_IFA and
     * RTAX_BRD: a truncated sockaddr_in netmask (sa_len 8), a sockaddr_dl for a short
     * interface name (12 after the 4-byte round-up), and two full sockaddr_in. */
    printf("a typical IPv4 RTM_NEWADDR = %zu (ifa_msghdr + 8 netmask + 12 ifp + 16 ifa + 16 brd)\n",
           sizeof(struct ifa_msghdr) + 8 + 12 + 16 + 16);
    printf("=> such a message is %s than the threshold\n",
           (sizeof(struct ifa_msghdr) + 8 + 12 + 16 + 16) > sizeof(struct rt_msghdr) ? "LONGER (accepted)"
                                                                                    : "SHORTER (DISCARDED)");
    return 0;
}
