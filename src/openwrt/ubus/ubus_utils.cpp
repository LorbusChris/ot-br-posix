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

#include "openwrt/ubus/ubus_utils.hpp"
#include "common/code_utils.hpp"

#include <limits.h>

namespace otbr {
namespace ubus {

int blobmsg_add_hex_string(blob_buf *aBuf, const char *aName, const uint8_t *aData, unsigned int aLength)
{
    VerifyOrReturn(aLength > 0, blobmsg_add_string(aBuf, aName, ""));
    char *hex = static_cast<char *>(blobmsg_alloc_string_buffer(aBuf, aName, aLength * 2 + 1));
    VerifyOrReturn(hex != nullptr, -1);

    for (unsigned int i = 0; i < aLength; i++)
    {
        snprintf(&hex[i * 2], 3, "%02x", aData[i]);
    }
    blobmsg_add_string_buffer(aBuf);
    return 0;
}

static int HexDigit(char c)
{
    int result = -1;
    if ('0' <= c && c <= '9')
    {
        result = c - '0';
    }
    else if ('a' <= c && c <= 'f')
    {
        result = c - 'a' + 10;
    }
    else if ('A' <= c && c <= 'F')
    {
        result = c - 'A' + 10;
    }
    return result;
}

int blobmsg_get_hex_string(blob_attr *aAttr, uint8_t *aOut, int aOutSize)
{
    VerifyOrReturn(aOut != nullptr && 0 <= aOutSize && aOutSize <= INT_MAX, -1);

    char const *hex = blobmsg_get_string(aAttr); // handles null by returning null
    VerifyOrReturn(hex != nullptr, -1);

    size_t hexLength = strlen(hex);
    VerifyOrReturn(hexLength % 2 == 0, -1);
    size_t binLength = hexLength / 2;
    VerifyOrReturn(binLength <= static_cast<size_t>(aOutSize), -1);

    char const *hexEnd = hex + hexLength;
    while (hex < hexEnd)
    {
        int hi = HexDigit(*hex++);
        int lo = HexDigit(*hex++);
        VerifyOrReturn(hi >= 0 && lo >= 0, -1);
        *aOut++ = (hi << 4) | lo;
    }
    return binLength;
}

} // namespace ubus
} // namespace otbr
