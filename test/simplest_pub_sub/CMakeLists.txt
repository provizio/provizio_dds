# Copyright 2023 Provizio Ltd.
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

add_subdirectory(simplest_publisher)
add_subdirectory(simplest_subscriber)

# TODO: Windows version
add_test(NAME simplest_pub_sub COMMAND
    sh -c "$<TARGET_FILE:simplest_publisher> & $<TARGET_FILE:simplest_subscriber>"
)
