# Make TopicPayloadPoolRegistry hand out a pool it actually holds.
#
# Fast-DDS shares one payload pool per topic name (and memory policy) among every DataWriter
# and DataReader of a process, through a process-wide registry of weak_ptrs. Looking one up goes
#
#     if (ptr.expired()) { create a new proxy; return it; }
#     return ptr.lock();
#
# under the registry's mutex -- but the endpoints holding the shared_ptrs release them WITHOUT
# that mutex (a plain shared_ptr going out of scope in DataWriterImpl::release_payload_pool /
# DataReaderImpl::release_payload_pool, on whatever thread is destroying the endpoint). So
# between expired() (false: the last DataWriter on the topic still holds it) and lock(), that
# writer's destruction on another thread can drop the last reference, and lock() returns nullptr.
# The caller does not expect that: DataReaderImpl::get_payload_pool() dereferences the result at
# once (reserve_history) and the process dies with a SEGV on the zero page from inside
# create_datareader.
#
# provizio_dds hits this whenever an endpoint on a topic is created while the last other
# endpoint of the same process on that topic is being destroyed: a match-publisher subscriber's
# deferred build racing the publisher's destruction (repro_deadlock: ~4000 create/destroy cycles
# under ASan), and network-recovery resets that destroy and recreate endpoints while the
# application creates more (`network_recovery_concurrent_make_publisher_during_reset`, which is
# where CI first showed it, on macOS). Present in v3.6.2 and unchanged on upstream master at the
# time of writing.
#
# The fix is the canonical one: lock() first, and only create when that yielded nothing. A proxy
# whose last owner is mid-release is then simply replaced by a fresh one -- two pools for the
# same topic exist for the instant it takes the old one to finish dying, which is harmless (the
# sharing is an optimisation; each pool is self-contained).
#
# Like the other scripts in this directory it runs as the Fast-DDS ExternalProject PATCH_COMMAND,
# is idempotent, and FAILs loudly when its anchor has moved. A FAST_DDS_VERSION bump must re-check
# it; drop it once upstream fixes the lookup.
#
# Invoked as:
#   cmake -DREGISTRY_HPP=<path-to-TopicPayloadPoolRegistry_impl/TopicPayloadPoolRegistry.hpp>
#         -P topic_payload_pool_registry_lock_first.cmake

if(NOT DEFINED REGISTRY_HPP)
    message(FATAL_ERROR "topic_payload_pool_registry_lock_first.cmake: REGISTRY_HPP must be defined")
endif()
if(NOT EXISTS "${REGISTRY_HPP}")
    message(FATAL_ERROR "topic_payload_pool_registry_lock_first.cmake: file not found: ${REGISTRY_HPP}")
endif()

file(READ "${REGISTRY_HPP}" _contents)

# The replacement carries this tag in a comment; a file that has it is already patched.
string(FIND "${_contents}" "[provizio_dds]" _already_pos)
if(NOT _already_pos EQUAL -1)
    message(STATUS "topic_payload_pool_registry_lock_first: TopicPayloadPoolRegistry.hpp already patched -- no-op")
    return()
endif()

set(_anchor [==[
        if (ptr.expired())
        {
            auto new_ptr = std::make_shared<TopicPayloadPoolProxy>(topic_name, config);
            ptr = new_ptr;
            return new_ptr;
        }

        return ptr.lock();
]==])
set(_patched [==[
        // [provizio_dds] lock() first: the owners release their shared_ptrs without taking mutex_,
        // so between an expired() that says "alive" and a lock() the last owner (an endpoint being
        // destroyed on another thread) can let go, and lock() then yields nullptr -- which
        // DataReaderImpl::get_payload_pool dereferences. Whatever lock() returns is a reference of
        // our own; if it is nothing, the pool is gone or going, and a fresh one replaces it.
        std::shared_ptr<TopicPayloadPoolProxy> existing = ptr.lock();
        if (existing)
        {
            return existing;
        }
        auto new_ptr = std::make_shared<TopicPayloadPoolProxy>(topic_name, config);
        ptr = new_ptr;
        return new_ptr;
]==])

string(FIND "${_contents}" "${_anchor}" _pos)
if(_pos EQUAL -1)
    message(FATAL_ERROR
        "topic_payload_pool_registry_lock_first.cmake: anchor for 'do_get' not found in ${REGISTRY_HPP}. "
        "Fast-DDS has changed shape; re-check this patch against the new sources "
        "(or drop it if the lookup now locks before it decides).")
endif()
string(REPLACE "${_anchor}" "${_patched}" _contents "${_contents}")

file(WRITE "${REGISTRY_HPP}" "${_contents}")
message(STATUS "topic_payload_pool_registry_lock_first: patched TopicPayloadPoolRegistry.hpp")
