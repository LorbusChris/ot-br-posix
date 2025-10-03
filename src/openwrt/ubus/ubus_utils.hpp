/*
 *  Copyright (c) 2026, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OTBR_AGENT_UBUS_UTILS_HPP_
#define OTBR_AGENT_UBUS_UTILS_HPP_

#include "common/code_utils.hpp"

#include <stddef.h>
#include <type_traits>

#include <libubus.h>

namespace otbr {
namespace ubus {

// Wraps C++ instance methods in ubus_handler_t functions at compile time.
struct UbusMethodHandler
{
    // Prepares a method invocation by resolving the C++ handler object that
    // owns the given ubus_object, and performs any necessary per-request
    // setup. By default the method is forwarded to the handler class, but
    // this template can be specialized externally instead if necessary.
    template <typename Handler>
    static Handler &PrepareInvocation(ubus_object *aObj, ubus_request_data *aReq, const char *aMethod)
    {
        return Handler::PrepareInvocation(aObj, aReq, aMethod);
    }

    template <typename M> struct MethodTraits;

    // Handlers for methods without arguments take only the ubus_request_data.
    template <typename C> struct MethodTraits<int (C::*)(ubus_request_data *)>
    {
        using HandlerType               = C;
        static constexpr size_t NumArgs = 0;
    };

    // Handlers for methods with arguments receive them as blob_attr *(&aArgs)[N].
    // Passing by reference is necessary to be able to deduce the size of the array,
    // which is statically validated against the size of the policy array by the adapter.
    template <typename C, size_t N> struct MethodTraits<int (C::*)(ubus_request_data *, blob_attr *(&)[N])>
    {
        using HandlerType               = C;
        static constexpr size_t NumArgs = N;
    };

    // Generates a ubus_handler_t adapter function for an ubus method without arguments.
    template <typename M, M Method> static constexpr ubus_handler_t Adapter()
    {
        return [](ubus_context *aContext, ubus_object *aObj, ubus_request_data *aReq, const char *aMethod,
                  blob_attr *aMsg) {
            OT_UNUSED_VARIABLE(aContext);
            OT_UNUSED_VARIABLE(aMsg);

            using HandlerType = typename MethodTraits<M>::HandlerType;
            static_assert(MethodTraits<M>::NumArgs == 0, "Invalid no-arg method signature");

            return (PrepareInvocation<HandlerType>(aObj, aReq, aMethod).*Method)(aReq);
        };
    }

    // Generates a ubus_handler_t adapter function for an ubus method with arguments.
    template <typename M, M Method, typename P, P Policy> static constexpr ubus_handler_t Adapter()
    {
        return [](ubus_context *aContext, ubus_object *aObj, ubus_request_data *aRequest, const char *aMethod,
                  blob_attr *aMsg) {
            OT_UNUSED_VARIABLE(aContext);

            constexpr size_t NumArgs = MethodTraits<M>::NumArgs;
            static_assert(NumArgs > 0, "Invalid method signature");
            static_assert(std::is_array<P>::value, "Policy must be a blobmsg_policy[N]");
            static_assert(NumArgs == std::extent<P>::value, "Method signature / policy mismatch");
            using HandlerType = typename MethodTraits<M>::HandlerType;

            blob_attr *args[NumArgs]; // fully initialized by blobmsg_parse
            blobmsg_parse(Policy, NumArgs, args, blob_data(aMsg), blob_len(aMsg));
            return (PrepareInvocation<HandlerType>(aObj, aRequest, aMethod).*Method)(aRequest, args);
        };
    }
};

#define OTBR_UBUS_METHOD_NOARG(NAME, METHOD) \
    UBUS_METHOD_NOARG(NAME, (UbusMethodHandler::Adapter<decltype(METHOD), METHOD>()))

#define OTBR_UBUS_METHOD(NAME, METHOD, POLICY) \
    UBUS_METHOD(NAME, (UbusMethodHandler::Adapter<decltype(METHOD), METHOD, decltype(POLICY), POLICY>()), POLICY)

} // namespace ubus
} // namespace otbr

#endif // OTBR_AGENT_UBUS_UTILS_HPP_
