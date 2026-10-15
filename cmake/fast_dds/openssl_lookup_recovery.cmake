# Copyright 2026 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Given first to every configure of the Fast-DDS build that provizio_dds runs (cmake -C,
# PROVIZIO_DDS_FAST_DDS_OPENSSL_RECOVERY in openssl_package.cmake), ahead of its command line and of
# any of Fast-DDS's own code: where a configure stopped inside the lookup of an OpenSSL package,
# puts the cache back as it was before that lookup from the journal it left there (see
# openssl_lookup_journal.cmake). The command line then sets what it sets, as on any other configure.

include("${CMAKE_CURRENT_LIST_DIR}/openssl_lookup_journal.cmake")
_provizio_dds_openssl_journal_end()
