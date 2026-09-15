# Copyright 2025 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

add_executable(ros_interop_service ros_interop_service.cpp)
# Threads::Threads for the std::condition_variable this waits on: on a glibc older
# than 2.34 the pthread symbols behind it live outside libc, so linking provizio_dds
# alone fails there and succeeds on every newer host.
find_package(Threads REQUIRED)
target_link_libraries(ros_interop_service PUBLIC provizio_dds Threads::Threads)
