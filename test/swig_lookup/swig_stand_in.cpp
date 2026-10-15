// Copyright 2026 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//
// A SWIG for swig_lookup_test.cmake to find, where a shell script cannot be one: on Windows, FindSWIG
// runs SWIG through CreateProcess, which starts a program image and nothing else (no #! script). The
// test copies this executable once per SWIG it needs and appends to each copy what that SWIG
// answers, which also gives every copy content of its own -- the lookup tells one SWIG from another
// by its path and its content. It answers what FindSWIG asks: -version, -swiglib and -help.
//
// The appended trailer is "<version>|<library directory>", then the length of that text in bytes
// as eight decimal digits. It is read from argv[0], the file the program was started as, which
// FindSWIG gives as the full path find_program found.

#include <cctype>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
    /// How many decimal digits the trailer's length is written in.
    constexpr std::size_t k_length_digits = 8;

    /// What one copy answers, as appended to it.
    struct answers
    {
        std::string version;
        std::string library;
    };

    /// @brief Reads the answers appended to the file at @p path.
    /// @param path The file this program was started as.
    /// @param out Where the answers go.
    /// @return Whether a well-formed trailer was read.
    bool read_answers(const std::string &path, answers &out)
    {
        std::ifstream file{path, std::ios::binary};
        if (!file)
        {
            return false;
        }
        const std::string content{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
        if (content.size() < k_length_digits)
        {
            return false;
        }
        const std::string length_text = content.substr(content.size() - k_length_digits);
        std::size_t length = 0;
        for (const char digit : length_text)
        {
            if (std::isdigit(static_cast<unsigned char>(digit)) == 0)
            {
                return false;
            }
            constexpr std::size_t k_base = 10;
            length = length * k_base + static_cast<std::size_t>(digit - '0');
        }
        if (length > content.size() - k_length_digits)
        {
            return false;
        }
        const std::string text = content.substr(content.size() - k_length_digits - length, length);
        const std::size_t separator = text.find('|');
        if (separator == std::string::npos)
        {
            return false;
        }
        out.version = text.substr(0, separator);
        out.library = text.substr(separator + 1);
        return true;
    }
}  // namespace

int main(int argc, char **argv)
{
    const std::vector<std::string> args(
        argv, std::next(argv, argc));  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (args.size() < 2)
    {
        std::cerr << "swig stand-in: expected one of -version, -swiglib or -help\n";
        return 1;
    }
    answers own;
    if (!read_answers(args.at(0), own))
    {
        std::cerr << "swig stand-in: " << args.at(0)
                  << " carries no answers; it must be a copy the test appended them to\n";
        return 1;
    }
    const std::string &option = args.at(1);
    if (option == "-version")
    {
        std::cout << "\nSWIG Version " << own.version << '\n';
    }
    else if (option == "-swiglib")
    {
        std::cout << own.library << '\n';
    }
    else if (option == "-help")
    {
        std::cout << "     -python         - Generate Python wrappers\n";
    }
    else
    {
        std::cerr << "swig stand-in: unexpected option " << option << '\n';
        return 1;
    }
    return 0;
}
