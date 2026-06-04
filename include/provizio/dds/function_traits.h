// Copyright 2014 Open Source Robotics Foundation, Inc.
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
//
// It's a modified version of https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/function_traits.hpp
// which is also licensed under Apache Licence 2.0:
// https://github.com/ros2/rclcpp/blob/jazzy/LICENSE

#ifndef DDS_FUNCTION_TRAITS
#define DDS_FUNCTION_TRAITS

#include <functional>
#include <memory>
#include <tuple>

namespace provizio::dds
{
    /**
     * @file function_traits.h
     * @brief Templates to introspect callable types and adapt them across standard libraries.
     *
     * Provides utilities to:
     * - Inspect a callable's arity, argument and return types
     * - Check argument list equality between two callables
     * - Convert a callable type to an equivalent std::function signature
     * - Support lambdas, free functions, member functions and std::bind wrappers
     */

    // Remove the first item in a tuple
    template <typename T> struct tuple_tail;

    template <typename head, typename... tail> struct tuple_tail<std::tuple<head, tail...>>
    {
        using type = std::tuple<tail...>;
    };

    // std::function
    template <typename function_type> struct function_traits
    {
        /**
         * @brief Tuple of function arguments.
         */
        using arguments =
            typename tuple_tail<typename function_traits<decltype(&function_type::operator())>::arguments>::type;

        /**
         * @brief Number of function arguments.
         */
        static constexpr std::size_t arity = std::tuple_size<arguments>::value;

        /**
         * @brief Nth argument type.
         * @tparam index Zero-based index of the argument in the signature
         */
        template <std::size_t index> using argument_type = typename std::tuple_element<index, arguments>::type;

        /**
         * @brief Function return type.
         */
        using return_type = typename function_traits<decltype(&function_type::operator())>::return_type;
    };

    // Free functions
    template <typename ret_type, typename... args> struct function_traits<ret_type(args...)>
    {
        /**
         * @brief Tuple of function arguments.
         */
        using arguments = std::tuple<args...>;

        /**
         * @brief Number of function arguments.
         */
        static constexpr std::size_t arity = std::tuple_size<arguments>::value;

        /**
         * @brief Nth argument type.
         * @tparam index Zero-based index of the argument in the signature
         */
        template <std::size_t index> using argument_type = typename std::tuple_element<index, arguments>::type;

        /**
         * @brief Function return type.
         */
        using return_type = ret_type;
    };

    // Function pointers
    template <typename ret_type, typename... args>
    struct function_traits<ret_type (*)(args...)> : function_traits<ret_type(args...)>
    {
    };

    // std::bind for object methods
    template <typename class_type, typename ret_type, typename... args, typename... function_args>
#if defined DOXYGEN_ONLY
    struct function_traits<std::bind<ret_type (class_type::*)(args...), function_args...>>
#elif defined _LIBCPP_VERSION   // libc++ (Clang)
    struct function_traits<std::__bind<ret_type (class_type::*)(args...), function_args...>>
#elif defined _GLIBCXX_RELEASE  // glibc++ (GNU C++ >= 7.1)
    struct function_traits<std::_Bind<ret_type (class_type::*(function_args...))(args...)>>
#elif defined __GLIBCXX__       // glibc++ (GNU C++)
    struct function_traits<std::_Bind<std::_Mem_fn<ret_type (class_type::*)(args...)>(function_args...)>>
#elif defined _MSC_VER          // MS Visual Studio
    struct function_traits<std::_Binder<std::_Unforced, ret_type (class_type::*)(args...), function_args...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
        : function_traits<ret_type(args...)>
    {
    };

    // std::bind for object const methods
    template <typename class_type, typename ret_type, typename... args, typename... function_args>
#if defined DOXYGEN_ONLY
    struct function_traits<std::bind<ret_type (class_type::*)(args...) const, function_args...>>
#elif defined _LIBCPP_VERSION   // libc++ (Clang)
    struct function_traits<std::__bind<ret_type (class_type::*)(args...) const, function_args...>>
#elif defined _GLIBCXX_RELEASE  // glibc++ (GNU C++ >= 7.1)
    struct function_traits<std::_Bind<ret_type (class_type::*(function_args...))(args...) const>>
#elif defined __GLIBCXX__       // glibc++ (GNU C++)
    struct function_traits<std::_Bind<std::_Mem_fn<ret_type (class_type::*)(args...) const>(function_args...)>>
#elif defined _MSC_VER          // MS Visual Studio
    struct function_traits<std::_Binder<std::_Unforced, ret_type (class_type::*)(args...) const, function_args...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
        : function_traits<ret_type(args...)>
    {
    };

    // std::bind for free functions
    template <typename ret_type, typename... args, typename... function_args>
#if defined DOXYGEN_ONLY
    struct function_traits<std::bind<ret_type (&)(args...), function_args...>>
#elif defined _LIBCPP_VERSION  // libc++ (Clang)
    struct function_traits<std::__bind<ret_type (&)(args...), function_args...>>
#elif defined __GLIBCXX__      // glibc++ (GNU C++)
    struct function_traits<std::_Bind<ret_type (*(function_args...))(args...)>>
#elif defined _MSC_VER         // MS Visual Studio
    struct function_traits<std::_Binder<std::_Unforced, ret_type (&)(args...), function_args...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
        : function_traits<ret_type(args...)>
    {
    };

    // Lambdas
    template <typename class_type, typename ret_type, typename... args>
    struct function_traits<ret_type (class_type::*)(args...) const> : function_traits<ret_type(class_type &, args...)>
    {
    };

    template <typename function_type> struct function_traits<function_type &> : function_traits<function_type>
    {
    };

    template <typename function_type> struct function_traits<function_type &&> : function_traits<function_type>
    {
    };

    template <std::size_t arity, typename functor_type>
    struct arity_comparator : std::integral_constant<bool, (arity == function_traits<functor_type>::arity)>
    {
    };

    template <typename functor_type, typename... args>
    struct check_arguments : std::is_same<typename function_traits<functor_type>::arguments, std::tuple<args...>>
    {
    };

    template <typename functor_a_type, typename functor_b_type>
    struct same_arguments : std::is_same<typename function_traits<functor_a_type>::arguments,
                                         typename function_traits<functor_b_type>::arguments>
    {
    };

    namespace detail
    {
        template <typename ret_type, typename... args> struct as_std_function_helper;

        template <typename ret_type, typename... args> struct as_std_function_helper<ret_type, std::tuple<args...>>
        {
            using type = std::function<ret_type(args...)>;
        };
    }  // namespace detail

    template <typename functor_type, typename function_traits_type = function_traits<functor_type>>
    struct as_std_function
    {
        /**
         * @brief Type alias to an std::function with the same signature as the functor.
         * @tparam functor_type Callable type to convert
         * @tparam function_traits_type Traits implementation to use (defaults to provizio::dds::function_traits)
         */
        using type = typename detail::as_std_function_helper<typename function_traits_type::return_type,
                                                             typename function_traits_type::arguments>::type;
    };
}  // namespace provizio::dds

#endif  // DDS_FUNCTION_TRAITS
