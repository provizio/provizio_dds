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

#ifndef DDS_DETAIL_SNAPSHOT_POLICY
#define DDS_DETAIL_SNAPSHOT_POLICY

#include "provizio/dds/detail/vpn_interfaces.h"

namespace provizio::dds::detail
{
    /**
     * @file snapshot_policy.h
     * @brief The order in which every platform's snapshot filter asks its questions.
     *
     * What each question means IS per platform -- Linux has an rtnetlink kind, macOS has
     * only a device name, Windows has three identities and an adapter type -- but the order
     * and the reasoning behind it are not. Only one backend compiles per platform, so three
     * hand-kept copies of that order could never clash at build time; they would simply
     * drift, one platform at a time, into the one outcome none of them may produce -- the
     * snapshot and the transports disagreeing about a single interface.
     */

    /**
     * @brief Whether change detection should drop an interface from its snapshot, given the
     * four answers only the platform can supply.
     *
     * The order, and why it is this order:
     *
     * 1. A tunnel this library kept OFF the transports is dropped before anything else, and
     *    regardless of force-inclusion: the transports refuse to bind it, so its address
     *    churn -- a Tailscale re-auth, a tunnel reconnect -- can no longer move any locator,
     *    and rebuilding every participant over it would be pure disruption. Gated on the
     *    exclusion having actually reached the transports, because where it did not (the
     *    caller owns the transports, a participant-level netmask filter of OFF rules a
     *    blocklist out, or the host's interfaces could not be read when the participant was
     *    configured) DDS binds and announces the tunnel after all, and dropping it here
     *    would leave a re-auth or a reconnect with a dead locator that no rebuild replaces.
     *    Before the exclusion existed, tunnels stayed in the snapshot for exactly that
     *    reason.
     * 2. A tunnel that nonetheless STAYS -- the override re-admitted it, or the exclusion
     *    never reached the transports -- is kept, and the platform heuristics below must not
     *    then drop it again. Those heuristics are written to exclude tunnels ("utun" is a
     *    macOS name prefix; IF_TYPE_TUNNEL is none of the Windows types kept), so consulting
     *    them here would leave the snapshot ignoring an interface the transports do bind.
     * 3. A force-included interface skips the heuristics, but not the loopback / carrier /
     *    link-local checks the walk has already applied. See @c force_included_interfaces.
     * 4. Otherwise the platform's own container-plumbing and virtual-adapter heuristics
     *    decide.
     *
     * @param excluded_as_vpn Whether this identity is a tunnel that
     *        @c PROVIZIO_DDS_ALLOW_VPN_INTERFACES did NOT re-admit. Invoked only where the
     *        exclusion reached the transports, deliberately: the classifier behind it
     *        records which override names matched, and a decision that will not be acted on
     *        must not feed that bookkeeping.
     * @param is_vpn Whether this identity is a tunnel at all, override or not.
     * @param force_included Whether the force-include list names this interface.
     * @param platform_excludes The platform's remaining heuristics.
     * @return @c true when the interface has no place in the snapshot.
     */
    template <typename excluded_as_vpn_fn, typename is_vpn_fn, typename force_included_fn,
              typename platform_excludes_fn>
    bool snapshot_policy_excludes(excluded_as_vpn_fn excluded_as_vpn, is_vpn_fn is_vpn,
                                  force_included_fn force_included, platform_excludes_fn platform_excludes)
    {
        if (vpn_exclusion_applies_to_transports() && excluded_as_vpn())
        {
            return true;
        }

        if (is_vpn())
        {
            return false;
        }

        if (force_included())
        {
            return false;
        }

        return platform_excludes();
    }
}  // namespace provizio::dds::detail

#endif  // DDS_DETAIL_SNAPSHOT_POLICY
