// Copyright 2025 Provizio Ltd.
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

#ifndef DDS_IGNORE_REQUEST
#define DDS_IGNORE_REQUEST

#include <exception>

namespace provizio::dds
{
    /**
     * @file ignore_request.h
     * @brief Exception to indicate that a request should be ignored by a service.
     *
     * Throw this from a request handler (sync or async) to drop the request silently.
     */
    class ignore_request : public std::exception
    {
      public:
        const char *what() const noexcept override { return "ignore_request"; }
    };
} // namespace provizio::dds

#endif // DDS_IGNORE_REQUEST


