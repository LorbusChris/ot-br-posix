/*
 *  Copyright (c) 2019, The OpenThread Authors.
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

#if __ORDER_BIG_ENDIAN__
#define BYTE_ORDER_BIG_ENDIAN 1
#endif

#define OTBR_LOG_TAG "UBUS"

#include "openwrt/ubus/otubus.hpp"

#include <functional>

#include <openthread/commissioner.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>

#include "common/logging.hpp"
#include "common/time.hpp"
#include "host/rcp_host.hpp"
#include "openwrt/ubus/ubus_utils.hpp"

namespace otbr {
namespace ubus {

static constexpr uint32_t kDefaultJoinerTimeout = 120;

// === UbusServer ===

UbusServer::UbusServer(ubus_context &aContext, Host::RcpHost &aHost)
    : mContext(aContext)
    , mHost(&aHost)
{
    Object().name      = "otbr";
    Object().type      = &sObjectType;
    Object().methods   = sObjectType.methods;
    Object().n_methods = sObjectType.n_methods;
    blob_buf_init(&mNetworkdataBuf, 0);
}

void UbusServer::Init()
{
    if (ubus_add_object(&mContext, &Object()) != 0)
    {
        otbrLogErr("Failed to publish object");
    }
}

UbusServer::~UbusServer()
{
    // blob_buf_free() is safe even if blob_buf_init() was not called due to zero
    // initialization (which is always needed due to how blob_buf_init() works).
    blob_buf_free(&mBuf);
    blob_buf_free(&mNetworkdataBuf);
    blob_buf_free(&mScanBuf);
}

UbusServer &UbusServer::PrepareInvocation(ubus_object *aObj, ubus_request_data *aRequest, const char *aMethod)
{
    auto *self = static_cast<UbusServer *>(aObj);
    otbrLogDebug("Handling %s.%s from %08x (%s)", aObj->name, aMethod, aRequest->peer, aRequest->acl.user);
    blob_buf_init(&self->mBuf, 0);
    return *self;
}

void UbusServer::HandleActiveScanResult(otActiveScanResult *aResult, void *aContext)
{
    static_cast<UbusServer *>(aContext)->HandleActiveScanResultDetail(aResult);
}

void UbusServer::AppendResult(otError aError, struct ubus_context *aContext, struct ubus_request_data *aRequest)
{
    blobmsg_add_u16(&mBuf, "Error", aError);
    ubus_send_reply(aContext, aRequest, mBuf.head);
}

void UbusServer::HandleActiveScanResultDetail(otActiveScanResult *aResult)
{
    void *jsonList = nullptr;

    if (aResult == nullptr)
    {
        blobmsg_close_array(&mScanBuf, mScanArray);
        blobmsg_add_u16(&mScanBuf, "Error", OT_ERROR_NONE);
        ubus_send_reply(&mContext, &mScanRequest, mScanBuf.head);
        ubus_complete_deferred_request(&mContext, &mScanRequest, 0);
        goto exit;
    }

    jsonList = blobmsg_open_table(&mScanBuf, nullptr);

    blobmsg_add_string(&mScanBuf, "NetworkName", aResult->mNetworkName.m8);
    blobmsg_add_hex_string(&mScanBuf, "ExtendedPanId", aResult->mExtendedPanId.m8, OT_EXT_PAN_ID_SIZE);
    blobmsg_printf(&mScanBuf, "PanId", "0x%04x", aResult->mPanId);
    blobmsg_add_u32(&mScanBuf, "Channel", aResult->mChannel);
    blobmsg_add_u32(&mScanBuf, "Rssi", aResult->mRssi);
    blobmsg_add_u32(&mScanBuf, "Lqi", aResult->mLqi);

    blobmsg_close_table(&mScanBuf, jsonList);

exit:
    return;
}

int UbusServer::HandleScan(ubus_request_data *aRequest)
{
    otError  error;
    uint32_t scanChannels = 0;
    uint16_t scanDuration = 0;

    // Note: This returns kErrorBusy if a scan is already in progress.
    SuccessOrExit(error = otLinkActiveScan(mHost->GetInstance(), scanChannels, scanDuration,
                                           &UbusServer::HandleActiveScanResult, this));

    ubus_defer_request(&mContext, aRequest, &mScanRequest);
    blob_buf_init(&mScanBuf, 0);
    mScanArray = blobmsg_open_array(&mScanBuf, "scan_list");

exit:
    if (error != OT_ERROR_NONE)
    {
        AppendResult(error, &mContext, aRequest);
    }
    return 0;
}

int UbusServer::HandleLeave(ubus_request_data *aRequest)
{
    ubus_request_data request;

    AppendResult(OT_ERROR_NONE, &mContext, aRequest);

    // Complete the request immediately because otInstanceFactoryReset() won't return.
    ubus_defer_request(&mContext, aRequest, &request);
    ubus_complete_deferred_request(&mContext, &request, 0);

    otInstanceFactoryReset(mHost->GetInstance());
    return 0;
}

int UbusServer::HandleThreadStart(ubus_request_data *aRequest)
{
    otError error = OT_ERROR_NONE;

    SuccessOrExit(error = otIp6SetEnabled(mHost->GetInstance(), true));
    SuccessOrExit(error = otThreadSetEnabled(mHost->GetInstance(), true));

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleThreadStop(ubus_request_data *aRequest)
{
    otError error = OT_ERROR_NONE;

    SuccessOrExit(error = otThreadSetEnabled(mHost->GetInstance(), false));
    SuccessOrExit(error = otIp6SetEnabled(mHost->GetInstance(), false));

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleParent(ubus_request_data *aRequest)
{
    otError      error = OT_ERROR_NONE;
    otRouterInfo parentInfo;
    void        *jsonList  = nullptr;
    void        *jsonArray = nullptr;

    SuccessOrExit(error = otThreadGetParentInfo(mHost->GetInstance(), &parentInfo));

    jsonArray = blobmsg_open_array(&mBuf, "parent_list");
    jsonList  = blobmsg_open_table(&mBuf, "parent");

    blobmsg_add_string(&mBuf, "Role", "R");
    blobmsg_printf(&mBuf, "Rloc16", "0x%04x", parentInfo.mRloc16);
    blobmsg_printf(&mBuf, "Age", "%3d", parentInfo.mAge);
    blobmsg_add_hex_string(&mBuf, "ExtAddress", parentInfo.mExtAddress.m8, sizeof(parentInfo.mExtAddress.m8));
    blobmsg_add_u16(&mBuf, "LinkQualityIn", parentInfo.mLinkQualityIn);

    blobmsg_close_table(&mBuf, jsonList);
    blobmsg_close_array(&mBuf, jsonArray);

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleNeighbor(ubus_request_data *aRequest)
{
    otError                error = OT_ERROR_NONE;
    otNeighborInfo         neighborInfo;
    otNeighborInfoIterator iterator  = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    void                  *jsonList  = nullptr;
    void                  *jsonTable = nullptr;
    char                   mode[5]   = "";

    jsonList = blobmsg_open_array(&mBuf, "neighbor_list");

    while (otThreadGetNextNeighborInfo(mHost->GetInstance(), &iterator, &neighborInfo) == OT_ERROR_NONE)
    {
        jsonTable = blobmsg_open_table(&mBuf, nullptr);

        blobmsg_add_string(&mBuf, "Role", neighborInfo.mIsChild ? "C" : "R");
        blobmsg_printf(&mBuf, "Rloc16", "0x%04x", neighborInfo.mRloc16);
        blobmsg_printf(&mBuf, "Age", "%3d", neighborInfo.mAge);
        blobmsg_printf(&mBuf, "AvgRssi", "%8d", neighborInfo.mAverageRssi);
        blobmsg_printf(&mBuf, "LastRssi", "%9d", neighborInfo.mLastRssi);

        if (neighborInfo.mRxOnWhenIdle)
        {
            strcat(mode, "r");
        }

        if (neighborInfo.mFullThreadDevice)
        {
            strcat(mode, "d");
        }

        if (neighborInfo.mFullNetworkData)
        {
            strcat(mode, "n");
        }
        blobmsg_add_string(&mBuf, "Mode", mode);
        blobmsg_add_hex_string(&mBuf, "ExtAddress", neighborInfo.mExtAddress.m8, sizeof(neighborInfo.mExtAddress.m8));
        blobmsg_add_u16(&mBuf, "LinkQualityIn", neighborInfo.mLinkQualityIn);

        blobmsg_close_table(&mBuf, jsonTable);

        memset(mode, 0, sizeof(mode));
    }

    blobmsg_close_array(&mBuf, jsonList);
    AppendResult(error, &mContext, aRequest);
    return 0;
}

enum
{
    NETWORKKEY,
    NETWORKNAME,
    EXTPANID,
    PANID,
    CHANNEL,
    PSKC,
};

static constexpr blobmsg_policy kMgmtSetPolicy[] = {
    [NETWORKKEY]  = {.name = "networkkey", .type = BLOBMSG_TYPE_STRING},
    [NETWORKNAME] = {.name = "networkname", .type = BLOBMSG_TYPE_STRING},
    [EXTPANID]    = {.name = "extpanid", .type = BLOBMSG_TYPE_STRING},
    [PANID]       = {.name = "panid", .type = BLOBMSG_TYPE_STRING},
    [CHANNEL]     = {.name = "channel", .type = BLOBMSG_TYPE_STRING},
    [PSKC]        = {.name = "pskc", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleMgmtSet(ubus_request_data *aRequest, blob_attr *(&tb)[6])
{
    otError              error = OT_ERROR_NONE;
    otOperationalDataset dataset;
    uint8_t              tlvs[128];
    long                 value;
    int                  length = 0;

    SuccessOrExit(error = otDatasetGetActive(mHost->GetInstance(), &dataset));

    if (tb[NETWORKKEY] != nullptr)
    {
        dataset.mComponents.mIsNetworkKeyPresent = true;
        VerifyOrExit((length = Hex2Bin(blobmsg_get_string(tb[NETWORKKEY]), dataset.mNetworkKey.m8,
                                       sizeof(dataset.mNetworkKey.m8))) == OT_NETWORK_KEY_SIZE,
                     error = OT_ERROR_PARSE);
        length = 0;
    }
    if (tb[NETWORKNAME] != nullptr)
    {
        dataset.mComponents.mIsNetworkNamePresent = true;
        VerifyOrExit((length = static_cast<int>(strlen(blobmsg_get_string(tb[NETWORKNAME])))) <=
                         OT_NETWORK_NAME_MAX_SIZE,
                     error = OT_ERROR_PARSE);
        memset(&dataset.mNetworkName, 0, sizeof(dataset.mNetworkName));
        memcpy(dataset.mNetworkName.m8, blobmsg_get_string(tb[NETWORKNAME]), static_cast<size_t>(length));
        length = 0;
    }
    if (tb[EXTPANID] != nullptr)
    {
        dataset.mComponents.mIsExtendedPanIdPresent = true;
        VerifyOrExit(Hex2Bin(blobmsg_get_string(tb[EXTPANID]), dataset.mExtendedPanId.m8,
                             sizeof(dataset.mExtendedPanId.m8)) == OT_EXT_PAN_ID_SIZE,
                     error = OT_ERROR_PARSE);
    }
    if (tb[PANID] != nullptr)
    {
        dataset.mComponents.mIsPanIdPresent = true;
        SuccessOrExit(error = ParseLong(blobmsg_get_string(tb[PANID]), value));
        dataset.mPanId = static_cast<otPanId>(value);
    }
    if (tb[CHANNEL] != nullptr)
    {
        dataset.mComponents.mIsChannelPresent = true;
        SuccessOrExit(error = ParseLong(blobmsg_get_string(tb[CHANNEL]), value));
        dataset.mChannel = static_cast<uint16_t>(value);
    }
    if (tb[PSKC] != nullptr)
    {
        dataset.mComponents.mIsPskcPresent = true;
        VerifyOrExit((length = Hex2Bin(blobmsg_get_string(tb[PSKC]), dataset.mPskc.m8, sizeof(dataset.mPskc.m8))) ==
                         OT_PSKC_MAX_SIZE,
                     error = OT_ERROR_PARSE);
        length = 0;
    }
    dataset.mActiveTimestamp.mSeconds++;
    if (otCommissionerGetState(mHost->GetInstance()) == OT_COMMISSIONER_STATE_DISABLED)
    {
        otCommissionerStop(mHost->GetInstance());
    }
    SuccessOrExit(error = otDatasetSendMgmtActiveSet(mHost->GetInstance(), &dataset, tlvs, static_cast<uint8_t>(length),
                                                     /* aCallback */ nullptr,
                                                     /* aContext */ nullptr));
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleCommissionerStart(ubus_request_data *aRequest)
{
    otError error = OT_ERROR_NONE;

    if (otCommissionerGetState(mHost->GetInstance()) == OT_COMMISSIONER_STATE_DISABLED)
    {
        error = otCommissionerStart(mHost->GetInstance(), &UbusServer::HandleStateChanged,
                                    &UbusServer::HandleJoinerEvent, this);
    }
    AppendResult(error, &mContext, aRequest);
    return 0;
}

enum
{
    PSKD,
    EUI64,
};

static constexpr blobmsg_policy kJoinerAddPolicy[] = {
    [PSKD]  = {.name = "pskd", .type = BLOBMSG_TYPE_STRING},
    [EUI64] = {.name = "eui64", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleJoinerAdd(ubus_request_data *aRequest, blob_attr *(&tb)[2])
{
    otError             error = OT_ERROR_NONE;
    otExtAddress        addr;
    const otExtAddress *addrPtr = nullptr;
    char               *pskd    = nullptr;

    if (tb[PSKD] != nullptr)
    {
        pskd = blobmsg_get_string(tb[PSKD]);
    }
    if (tb[EUI64] != nullptr)
    {
        if (!strcmp(blobmsg_get_string(tb[EUI64]), "*"))
        {
            addrPtr = nullptr;
            memset(&addr, 0, sizeof(addr));
        }
        else
        {
            VerifyOrExit(Hex2Bin(blobmsg_get_string(tb[EUI64]), addr.m8, sizeof(addr)) == sizeof(addr),
                         error = OT_ERROR_PARSE);
            addrPtr = &addr;
        }
    }

    SuccessOrExit(error = otCommissionerAddJoiner(mHost->GetInstance(), addrPtr, pskd, kDefaultJoinerTimeout));

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kJoinerRemovePolicy[] = {
    [0] = {.name = "eui64", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleJoinerRemove(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError             error = OT_ERROR_NONE;
    otExtAddress        addr;
    const otExtAddress *addrPtr = nullptr;

    if (aArgs[0] != nullptr)
    {
        if (strcmp(blobmsg_get_string(aArgs[0]), "*") == 0)
        {
            addrPtr = nullptr;
        }
        else
        {
            VerifyOrExit(Hex2Bin(blobmsg_get_string(aArgs[0]), addr.m8, sizeof(addr)) == sizeof(addr),
                         error = OT_ERROR_PARSE);
            addrPtr = &addr;
        }
    }

    SuccessOrExit(error = otCommissionerRemoveJoiner(mHost->GetInstance(), addrPtr));

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

void UbusServer::HandleStateChanged(otCommissionerState aState, void *aContext)
{
    static_cast<UbusServer *>(aContext)->HandleStateChanged(aState);
}

void UbusServer::HandleStateChanged(otCommissionerState aState)
{
    switch (aState)
    {
    case OT_COMMISSIONER_STATE_DISABLED:
        otbrLogInfo("Commissioner state disabled");
        break;
    case OT_COMMISSIONER_STATE_ACTIVE:
        otbrLogInfo("Commissioner state active");
        break;
    case OT_COMMISSIONER_STATE_PETITION:
        otbrLogInfo("Commissioner state petition");
        break;
    }
}

void UbusServer::HandleJoinerEvent(otCommissionerJoinerEvent aEvent,
                                   const otJoinerInfo       *aJoinerInfo,
                                   const otExtAddress       *aJoinerId,
                                   void                     *aContext)
{
    static_cast<UbusServer *>(aContext)->HandleJoinerEvent(aEvent, aJoinerInfo, aJoinerId);
}

void UbusServer::HandleJoinerEvent(otCommissionerJoinerEvent aEvent,
                                   const otJoinerInfo       *aJoinerInfo,
                                   const otExtAddress       *aJoinerId)
{
    OT_UNUSED_VARIABLE(aJoinerInfo);
    OT_UNUSED_VARIABLE(aJoinerId);

    switch (aEvent)
    {
    case OT_COMMISSIONER_JOINER_START:
        otbrLogInfo("Joiner start");
        break;
    case OT_COMMISSIONER_JOINER_CONNECTED:
        otbrLogInfo("Joiner connected");
        break;
    case OT_COMMISSIONER_JOINER_FINALIZE:
        otbrLogInfo("Joiner finalize");
        break;
    case OT_COMMISSIONER_JOINER_END:
        otbrLogInfo("Joiner end");
        break;
    case OT_COMMISSIONER_JOINER_REMOVED:
        otbrLogInfo("Joiner remove");
        break;
    }
}

int UbusServer::HandleNetworkName(ubus_request_data *aRequest)
{
    blobmsg_add_string(&mBuf, "NetworkName", otThreadGetNetworkName(mHost->GetInstance()));
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleInterfaceName(ubus_request_data *aRequest)
{
    blobmsg_add_string(&mBuf, "InterfaceName", mHost->GetInterfaceName());
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleState(ubus_request_data *aRequest)
{
    char state[10];
    GetState(mHost->GetInstance(), state);
    blobmsg_add_string(&mBuf, "State", state);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleChannel(ubus_request_data *aRequest)
{
    blobmsg_add_u32(&mBuf, "Channel", otLinkGetChannel(mHost->GetInstance()));
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandlePanId(ubus_request_data *aRequest)
{
    blobmsg_printf(&mBuf, "PanId", "0x%04x", otLinkGetPanId(mHost->GetInstance()));
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleRloc16(ubus_request_data *aRequest)
{
    blobmsg_printf(&mBuf, "rloc16", "0x%04x", otThreadGetRloc16(mHost->GetInstance()));
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleNetworkKey(ubus_request_data *aRequest)
{
    otNetworkKey key;

    otThreadGetNetworkKey(mHost->GetInstance(), &key);
    blobmsg_add_hex_string(&mBuf, "Networkkey", key.m8, OT_NETWORK_KEY_SIZE);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandlePskc(ubus_request_data *aRequest)
{
    otPskc pskc;

    otThreadGetPskc(mHost->GetInstance(), &pskc);
    blobmsg_add_hex_string(&mBuf, "pskc", pskc.m8, OT_PSKC_MAX_SIZE);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleExtPanId(ubus_request_data *aRequest)
{
    const uint8_t *extPanId = reinterpret_cast<const uint8_t *>(otThreadGetExtendedPanId(mHost->GetInstance()));
    blobmsg_add_hex_string(&mBuf, "ExtPanId", extPanId, OT_EXT_PAN_ID_SIZE);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleMode(ubus_request_data *aRequest)
{
    otLinkModeConfig linkMode;
    char             mode[5] = "";

    memset(&linkMode, 0, sizeof(otLinkModeConfig));

    linkMode = otThreadGetLinkMode(mHost->GetInstance());

    if (linkMode.mRxOnWhenIdle)
    {
        strcat(mode, "r");
    }

    if (linkMode.mDeviceType)
    {
        strcat(mode, "d");
    }

    if (linkMode.mNetworkData)
    {
        strcat(mode, "n");
    }
    blobmsg_add_string(&mBuf, "Mode", mode);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandlePartitionId(ubus_request_data *aRequest)
{
    blobmsg_add_u32(&mBuf, "Partitionid", otThreadGetPartitionId(mHost->GetInstance()));
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleLeaderData(ubus_request_data *aRequest)
{
    otError      error = OT_ERROR_NONE;
    void        *jsonTable;
    otLeaderData leaderData;

    SuccessOrExit(error = otThreadGetLeaderData(mHost->GetInstance(), &leaderData));

    jsonTable = blobmsg_open_table(&mBuf, "leaderdata");

    blobmsg_add_u32(&mBuf, "PartitionId", leaderData.mPartitionId);
    blobmsg_add_u32(&mBuf, "Weighting", leaderData.mWeighting);
    blobmsg_add_u32(&mBuf, "DataVersion", leaderData.mDataVersion);
    blobmsg_add_u32(&mBuf, "StableDataVersion", leaderData.mStableDataVersion);
    blobmsg_add_u32(&mBuf, "LeaderRouterId", leaderData.mLeaderRouterId);

    blobmsg_close_table(&mBuf, jsonTable);

exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleNetworkData(ubus_request_data *aRequest)
{
    ubus_send_reply(&mContext, aRequest, mNetworkdataBuf.head);
    if (time(nullptr) - mLastNetworkDataTime > 10)
    {
        static constexpr uint16_t kMaxTlvs = 35;

        otIp6Address address;
        uint8_t      tlvTypes[kMaxTlvs];
        uint8_t      count             = 0;
        char         multicastAddr[10] = "ff03::2";

        blob_buf_init(&mNetworkdataBuf, 0);

        SuccessOrExit(otIp6AddressFromString(multicastAddr, &address));

        tlvTypes[count++] = static_cast<uint8_t>(OT_NETWORK_DIAGNOSTIC_TLV_ROUTE);
        tlvTypes[count++] = static_cast<uint8_t>(OT_NETWORK_DIAGNOSTIC_TLV_CHILD_TABLE);

        mNetworkDataIndex = 0;
        otThreadSendDiagnosticGet(mHost->GetInstance(), &address, tlvTypes, count,
                                  &UbusServer::HandleDiagnosticGetResponse, this);
        mLastNetworkDataTime = time(nullptr);
    }
exit:
    return 0;
}

int UbusServer::HandleJoinerNum(ubus_request_data *aRequest)
{
    void        *jsonTable = nullptr;
    void        *jsonArray = nullptr;
    otJoinerInfo joinerInfo;
    uint16_t     iterator  = 0;
    int          joinerNum = 0;

    jsonArray = blobmsg_open_array(&mBuf, "joinerList");
    while (otCommissionerGetNextJoinerInfo(mHost->GetInstance(), &iterator, &joinerInfo) == OT_ERROR_NONE)
    {
        jsonTable = blobmsg_open_table(&mBuf, nullptr);

        blobmsg_add_string(&mBuf, "pskd", joinerInfo.mPskd.m8);

        switch (joinerInfo.mType)
        {
        case OT_JOINER_INFO_TYPE_ANY:
            blobmsg_add_u16(&mBuf, "isAny", 1);
            break;
        case OT_JOINER_INFO_TYPE_EUI64:
            blobmsg_add_u16(&mBuf, "isAny", 0);
            blobmsg_add_hex_string(&mBuf, "eui64", joinerInfo.mSharedId.mEui64.m8,
                                   sizeof(joinerInfo.mSharedId.mEui64.m8));
            break;
        case OT_JOINER_INFO_TYPE_DISCERNER:
            blobmsg_add_u16(&mBuf, "isAny", 0);
            blobmsg_add_u64(&mBuf, "discernerValue", joinerInfo.mSharedId.mDiscerner.mValue);
            blobmsg_add_u16(&mBuf, "discernerLength", joinerInfo.mSharedId.mDiscerner.mLength);
            break;
        }

        blobmsg_close_table(&mBuf, jsonTable);

        joinerNum++;
    }
    blobmsg_close_array(&mBuf, jsonArray);

    blobmsg_add_u32(&mBuf, "joinernum", joinerNum);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleMacFilterState(ubus_request_data *aRequest)
{
    otMacFilterAddressMode mode = otLinkFilterGetAddressMode(mHost->GetInstance());

    if (mode == OT_MAC_FILTER_ADDRESS_MODE_DISABLED)
    {
        blobmsg_add_string(&mBuf, "state", "disable");
    }
    else if (mode == OT_MAC_FILTER_ADDRESS_MODE_ALLOWLIST)
    {
        blobmsg_add_string(&mBuf, "state", "allowlist");
    }
    else if (mode == OT_MAC_FILTER_ADDRESS_MODE_DENYLIST)
    {
        blobmsg_add_string(&mBuf, "state", "denylist");
    }
    else
    {
        blobmsg_add_string(&mBuf, "state", "error");
    }

    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleMacFilterAddr(ubus_request_data *aRequest)
{
    otMacFilterEntry    entry;
    otMacFilterIterator iterator = OT_MAC_FILTER_ITERATOR_INIT;
    void               *jsonArray;

    jsonArray = blobmsg_open_array(&mBuf, "addrlist");

    while (otLinkFilterGetNextAddress(mHost->GetInstance(), &iterator, &entry) == OT_ERROR_NONE)
    {
        blobmsg_add_hex_string(&mBuf, "addr", entry.mExtAddress.m8, sizeof(entry.mExtAddress.m8));
    }

    blobmsg_close_array(&mBuf, jsonArray);
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

void UbusServer::HandleDiagnosticGetResponse(otError              aError,
                                             otMessage           *aMessage,
                                             const otMessageInfo *aMessageInfo,
                                             void                *aContext)
{
    static_cast<UbusServer *>(aContext)->HandleDiagnosticGetResponse(aError, aMessage, aMessageInfo);
}

static bool IsRoutingLocator(const otIp6Address *aAddress)
{
    enum
    {
        kAloc16Mask            = 0xfc, ///< The mask for Aloc16.
        kRloc16ReservedBitMask = 0x02, ///< The mask for the reserved bit of Rloc16.
    };

    return (aAddress->mFields.m32[2] == htonl(0x000000ff) && aAddress->mFields.m16[6] == htons(0xfe00) &&
            aAddress->mFields.m8[14] < kAloc16Mask && (aAddress->mFields.m8[14] & kRloc16ReservedBitMask) == 0);
}

void UbusServer::HandleDiagnosticGetResponse(otError aError, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    uint16_t              rloc16;
    uint16_t              sockRloc16 = 0;
    void                 *jsonTable  = nullptr;
    void                 *jsonArray  = nullptr;
    void                 *jsonItem   = nullptr;
    otNetworkDiagTlv      diagTlv;
    otNetworkDiagIterator iterator = OT_NETWORK_DIAGNOSTIC_ITERATOR_INIT;

    SuccessOrExit(aError);

    char networkdata[20];
    sprintf(networkdata, "networkdata%d", mNetworkDataIndex++);
    jsonTable = blobmsg_open_table(&mNetworkdataBuf, networkdata);

    if (IsRoutingLocator(&aMessageInfo->mSockAddr))
    {
        sockRloc16 = ntohs(aMessageInfo->mPeerAddr.mFields.m16[7]);
        blobmsg_printf(&mNetworkdataBuf, "rloc", "0x%04x", sockRloc16);
    }

    while (otThreadGetNextDiagnosticTlv(aMessage, &iterator, &diagTlv) == OT_ERROR_NONE)
    {
        switch (diagTlv.mType)
        {
        case OT_NETWORK_DIAGNOSTIC_TLV_ROUTE:
        {
            const otNetworkDiagRoute &route = diagTlv.mData.mRoute;

            jsonArray = blobmsg_open_array(&mNetworkdataBuf, "routedata");

            for (uint16_t i = 0; i < route.mRouteCount; ++i)
            {
                uint8_t in, out;
                in  = route.mRouteData[i].mLinkQualityIn;
                out = route.mRouteData[i].mLinkQualityOut;
                if (in != 0 && out != 0)
                {
                    jsonItem = blobmsg_open_table(&mNetworkdataBuf, "router");
                    rloc16   = route.mRouteData[i].mRouterId << 10;
                    blobmsg_add_u32(&mNetworkdataBuf, "routerid", route.mRouteData[i].mRouterId);
                    blobmsg_printf(&mNetworkdataBuf, "rloc", "0x%04x", rloc16);
                    blobmsg_close_table(&mNetworkdataBuf, jsonItem);
                }
            }
            blobmsg_close_array(&mNetworkdataBuf, jsonArray);
            break;
        }

        case OT_NETWORK_DIAGNOSTIC_TLV_CHILD_TABLE:
        {
            jsonArray = blobmsg_open_array(&mNetworkdataBuf, "childdata");
            for (uint16_t i = 0; i < diagTlv.mData.mChildTable.mCount; ++i)
            {
                enum
                {
                    kModeRxOnWhenIdle     = 1 << 3, ///< If the device has its receiver on when not transmitting.
                    kModeFullThreadDevice = 1 << 1, ///< If the device is an FTD.
                    kModeFullNetworkData  = 1 << 0, ///< If the device requires the full Network Data.
                };
                const otNetworkDiagChildEntry &entry = diagTlv.mData.mChildTable.mTable[i];

                uint8_t mode = 0;

                jsonItem = blobmsg_open_table(&mNetworkdataBuf, "child");
                blobmsg_printf(&mNetworkdataBuf, "rloc", "0x%04x", (sockRloc16 | entry.mChildId));

                mode = (entry.mMode.mRxOnWhenIdle ? kModeRxOnWhenIdle : 0) |
                       (entry.mMode.mDeviceType ? kModeFullThreadDevice : 0) |
                       (entry.mMode.mNetworkData ? kModeFullNetworkData : 0);
                blobmsg_add_u16(&mNetworkdataBuf, "mode", mode);
                blobmsg_close_table(&mNetworkdataBuf, jsonItem);
            }
            blobmsg_close_array(&mNetworkdataBuf, jsonArray);
            break;
        }

        default:
            // Ignore other network diagnostics data.
            break;
        }
    }

    blobmsg_close_table(&mNetworkdataBuf, jsonTable);

exit:
    if (aError != OT_ERROR_NONE)
    {
        otbrLogWarning("Failed to receive diagnostic response: %s", otThreadErrorToString(aError));
    }
}

static constexpr blobmsg_policy kSetNetworkNamePolicy[] = {
    [0] = {.name = "networkname", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetNetworkName(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;
    if (aArgs[0] != nullptr)
    {
        char *newName = blobmsg_get_string(aArgs[0]);
        SuccessOrExit(error = otThreadSetNetworkName(mHost->GetInstance(), newName));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetChannelPolicy[] = {
    [0] = {.name = "channel", .type = BLOBMSG_TYPE_INT32},
};

int UbusServer::HandleSetChannel(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] != nullptr)
    {
        uint32_t channel = blobmsg_get_u32(aArgs[0]);
        SuccessOrExit(error = otLinkSetChannel(mHost->GetInstance(), static_cast<uint8_t>(channel)));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetPanIdPolicy[] = {
    [0] = {.name = "panid", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetPanId(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] != nullptr)
    {
        long  value;
        char *panid = blobmsg_get_string(aArgs[0]);
        SuccessOrExit(error = ParseLong(panid, value));
        error = otLinkSetPanId(mHost->GetInstance(), static_cast<otPanId>(value));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetNetworkkeyPolicy[] = {
    [0] = {.name = "networkkey", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetNetworkKey(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] != nullptr)
    {
        otNetworkKey key;
        char        *networkkey = blobmsg_get_string(aArgs[0]);

        VerifyOrExit(Hex2Bin(networkkey, key.m8, sizeof(key.m8)) == OT_NETWORK_KEY_SIZE, error = OT_ERROR_PARSE);
        SuccessOrExit(error = otThreadSetNetworkKey(mHost->GetInstance(), &key));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetPskcPolicy[] = {
    [0] = {.name = "pskc", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetPskc(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] != nullptr)
    {
        otPskc pskc;

        VerifyOrExit(Hex2Bin(blobmsg_get_string(aArgs[0]), pskc.m8, sizeof(pskc)) == OT_PSKC_MAX_SIZE,
                     error = OT_ERROR_PARSE);
        SuccessOrExit(error = otThreadSetPskc(mHost->GetInstance(), &pskc));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetExtPanIdPolicy[] = {
    [0] = {.name = "extpanid", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetExtPanId(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError error = OT_ERROR_INVALID_ARGS;

    if (aArgs[0] != nullptr)
    {
        otExtendedPanId extPanId;
        char           *input = blobmsg_get_string(aArgs[0]);
        VerifyOrExit(Hex2Bin(input, extPanId.m8, sizeof(extPanId.m8)) == OT_EXT_PAN_ID_SIZE, error = OT_ERROR_PARSE);
        error = otThreadSetExtendedPanId(mHost->GetInstance(), &extPanId);
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kSetModePolicy[] = {
    [0] = {.name = "mode", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetMode(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError          error = OT_ERROR_INVALID_ARGS;
    otLinkModeConfig linkMode{};

    if (aArgs[0] != nullptr)
    {
        char *inputMode = blobmsg_get_string(aArgs[0]);
        for (char *ch = inputMode; *ch != '\0'; ch++)
        {
            switch (*ch)
            {
            case 'r':
                linkMode.mRxOnWhenIdle = 1;
                break;

            case 'd':
                linkMode.mDeviceType = 1;
                break;

            case 'n':
                linkMode.mNetworkData = 1;
                break;

            default:
                ExitNow(error = OT_ERROR_PARSE);
            }
        }

        SuccessOrExit(error = otThreadSetLinkMode(mHost->GetInstance(), linkMode));
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kMacfilterAddRemovePolicy[] = {
    [0] = {.name = "addr", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleMacFilterAdd(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError      error = OT_ERROR_INVALID_ARGS;
    otExtAddress extAddr;

    if (aArgs[0] != nullptr)
    {
        char *addr = blobmsg_get_string(aArgs[0]);

        VerifyOrExit(Hex2Bin(addr, extAddr.m8, OT_EXT_ADDRESS_SIZE) == OT_EXT_ADDRESS_SIZE, error = OT_ERROR_PARSE);

        error = otLinkFilterAddAddress(mHost->GetInstance(), &extAddr);

        VerifyOrExit(error == OT_ERROR_NONE || error == OT_ERROR_ALREADY);
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleMacFilterRemove(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError      error = OT_ERROR_INVALID_ARGS;
    otExtAddress extAddr;

    if (aArgs[0] != nullptr)
    {
        char *addr = blobmsg_get_string(aArgs[0]);
        VerifyOrExit(Hex2Bin(addr, extAddr.m8, OT_EXT_ADDRESS_SIZE) == OT_EXT_ADDRESS_SIZE, error = OT_ERROR_PARSE);

        otLinkFilterRemoveAddress(mHost->GetInstance(), &extAddr);
        error = OT_ERROR_NONE;
    }
exit:
    AppendResult(error, &mContext, aRequest);
    return 0;
}

static constexpr blobmsg_policy kMacfilterSetStatePolicy[] = {
    [0] = {.name = "state", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleMacFilterSetState(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    if (aArgs[0] != nullptr)
    {
        char *state = blobmsg_get_string(aArgs[0]);

        if (strcmp(state, "disable") == 0)
        {
            otLinkFilterSetAddressMode(mHost->GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_DISABLED);
        }
        else if (strcmp(state, "allowlist") == 0)
        {
            otLinkFilterSetAddressMode(mHost->GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_ALLOWLIST);
        }
        else if (strcmp(state, "denylist") == 0)
        {
            otLinkFilterSetAddressMode(mHost->GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_DENYLIST);
        }
    }
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

int UbusServer::HandleMacFilterClear(ubus_request_data *aRequest)
{
    otLinkFilterClearAddresses(mHost->GetInstance());
    AppendResult(OT_ERROR_NONE, &mContext, aRequest);
    return 0;
}

const ubus_method UbusServer::sMethods[] = {
    OTBR_UBUS_METHOD_NOARG("channel", &UbusServer::HandleChannel),
    OTBR_UBUS_METHOD("setchannel", &UbusServer::HandleSetChannel, kSetChannelPolicy),
    OTBR_UBUS_METHOD_NOARG("networkname", &UbusServer::HandleNetworkName),
    OTBR_UBUS_METHOD("setnetworkname", &UbusServer::HandleSetNetworkName, kSetNetworkNamePolicy),
    OTBR_UBUS_METHOD_NOARG("panid", &UbusServer::HandlePanId),
    OTBR_UBUS_METHOD("setpanid", &UbusServer::HandleSetPanId, kSetPanIdPolicy),
    OTBR_UBUS_METHOD_NOARG("extpanid", &UbusServer::HandleExtPanId),
    OTBR_UBUS_METHOD("setextpanid", &UbusServer::HandleSetExtPanId, kSetExtPanIdPolicy),
    OTBR_UBUS_METHOD_NOARG("networkkey", &UbusServer::HandleNetworkKey),
    OTBR_UBUS_METHOD("setnetworkkey", &UbusServer::HandleSetNetworkKey, kSetNetworkkeyPolicy),
    OTBR_UBUS_METHOD_NOARG("pskc", &UbusServer::HandlePskc),
    OTBR_UBUS_METHOD("setpskc", &UbusServer::HandleSetPskc, kSetPskcPolicy),
    OTBR_UBUS_METHOD_NOARG("mode", &UbusServer::HandleMode),
    OTBR_UBUS_METHOD("setmode", &UbusServer::HandleSetMode, kSetModePolicy),

    OTBR_UBUS_METHOD_NOARG("interfacename", &UbusServer::HandleInterfaceName),
    OTBR_UBUS_METHOD_NOARG("leaderdata", &UbusServer::HandleLeaderData),
    OTBR_UBUS_METHOD_NOARG("neighbor", &UbusServer::HandleNeighbor),
    OTBR_UBUS_METHOD_NOARG("networkdata", &UbusServer::HandleNetworkData),
    OTBR_UBUS_METHOD_NOARG("parent", &UbusServer::HandleParent),
    OTBR_UBUS_METHOD_NOARG("partitionid", &UbusServer::HandlePartitionId),
    OTBR_UBUS_METHOD_NOARG("rloc16", &UbusServer::HandleRloc16),
    OTBR_UBUS_METHOD_NOARG("state", &UbusServer::HandleState),

    OTBR_UBUS_METHOD_NOARG("scan", &UbusServer::HandleScan),
    OTBR_UBUS_METHOD_NOARG("leave", &UbusServer::HandleLeave),
    OTBR_UBUS_METHOD("mgmtset", &UbusServer::HandleMgmtSet, kMgmtSetPolicy),
    OTBR_UBUS_METHOD_NOARG("threadstart", &UbusServer::HandleThreadStart),
    OTBR_UBUS_METHOD_NOARG("threadstop", &UbusServer::HandleThreadStop),

    OTBR_UBUS_METHOD_NOARG("commissionerstart", &UbusServer::HandleCommissionerStart),
    OTBR_UBUS_METHOD("joineradd", &UbusServer::HandleJoinerAdd, kJoinerAddPolicy),
    OTBR_UBUS_METHOD_NOARG("joinernum", &UbusServer::HandleJoinerNum),
    OTBR_UBUS_METHOD("joinerremove", &UbusServer::HandleJoinerRemove, kJoinerRemovePolicy),

    OTBR_UBUS_METHOD("macfilteradd", &UbusServer::HandleMacFilterAdd, kMacfilterAddRemovePolicy),
    OTBR_UBUS_METHOD("macfilterremove", &UbusServer::HandleMacFilterRemove, kMacfilterAddRemovePolicy),
    OTBR_UBUS_METHOD_NOARG("macfilterclear", &UbusServer::HandleMacFilterClear),
    OTBR_UBUS_METHOD_NOARG("macfilterstate", &UbusServer::HandleMacFilterState),
    OTBR_UBUS_METHOD("macfiltersetstate", &UbusServer::HandleMacFilterSetState, kMacfilterSetStatePolicy),
    OTBR_UBUS_METHOD_NOARG("macfilteraddr", &UbusServer::HandleMacFilterAddr),
};

ubus_object_type UbusServer::sObjectType = UBUS_OBJECT_TYPE("otbr", sMethods);

void UbusServer::GetState(otInstance *aInstance, char *aState)
{
    switch (otThreadGetDeviceRole(aInstance))
    {
    case OT_DEVICE_ROLE_DISABLED:
        strcpy(aState, "disabled");
        break;

    case OT_DEVICE_ROLE_DETACHED:
        strcpy(aState, "detached");
        break;

    case OT_DEVICE_ROLE_CHILD:
        strcpy(aState, "child");
        break;

    case OT_DEVICE_ROLE_ROUTER:
        strcpy(aState, "router");
        break;

    case OT_DEVICE_ROLE_LEADER:
        strcpy(aState, "leader");
        break;
    default:
        strcpy(aState, "invalid aState");
        break;
    }
}

otError UbusServer::ParseLong(char *aString, long &aLong)
{
    char *endptr;
    aLong = strtol(aString, &endptr, 0);
    return (*endptr == '\0') ? OT_ERROR_NONE : OT_ERROR_PARSE;
}

int UbusServer::Hex2Bin(const char *aHex, uint8_t *aBin, uint16_t aBinLength)
{
    size_t      hexLength = strlen(aHex);
    const char *hexEnd    = aHex + hexLength;
    uint8_t    *cur       = aBin;
    uint8_t     numChars  = hexLength & 1;
    uint8_t     byte      = 0;
    int         rval;

    VerifyOrExit((hexLength + 1) / 2 <= aBinLength, rval = -1);

    while (aHex < hexEnd)
    {
        if ('A' <= *aHex && *aHex <= 'F')
        {
            byte |= 10 + (*aHex - 'A');
        }
        else if ('a' <= *aHex && *aHex <= 'f')
        {
            byte |= 10 + (*aHex - 'a');
        }
        else if ('0' <= *aHex && *aHex <= '9')
        {
            byte |= *aHex - '0';
        }
        else
        {
            ExitNow(rval = -1);
        }

        aHex++;
        numChars++;

        if (numChars >= 2)
        {
            numChars = 0;
            *cur++   = byte;
            byte     = 0;
        }
        else
        {
            byte <<= 4;
        }
    }

    rval = static_cast<int>(cur - aBin);

exit:
    return rval;
}

// === UloopProcessor ===

UloopProcessor *UloopProcessor::sInstance = nullptr;

UloopProcessor::~UloopProcessor()
{
    if (uloop_fd_set_cb == &ULoopFDHandler)
    {
        otbrLogDebug("Shutting down uloop");
        uloop_done();
        uloop_fd_set_cb = nullptr;
        sInstance       = nullptr;
    }
}

void UloopProcessor::Init()
{
    VerifyOrDie(sInstance == nullptr, "Cannot initialize multiple uloop instances");
    VerifyOrDie(uloop_fd_set_cb == nullptr, "An uloop fd set callback is already installed");
    otbrLogDebug("Initializing uloop");
    sInstance       = this;
    uloop_fd_set_cb = &ULoopFDHandler;
    uloop_init();
}

void UloopProcessor::ULoopFDHandler(uloop_fd *aFd, unsigned int aFlags)
{
    VerifyOrDie((aFlags & ~(ULOOP_READ | ULOOP_WRITE | ULOOP_BLOCKING)) == 0, "Unsupported uloop fd flags");
    if (aFlags & (ULOOP_READ | ULOOP_WRITE))
    {
        // flags will be saved by uloop in aFd->flags
        UloopProcessor::sInstance->mFds.insert(aFd);
    }
    else
    {
        UloopProcessor::sInstance->mFds.erase(aFd);
    }
}

void UloopProcessor::Update(MainloopContext &aMainloop)
{
    for (auto &fd : mFds)
    {
        uint8_t sets = 0;
        if (fd->flags & ULOOP_READ)
        {
            sets |= MainloopContext::kReadFdSet;
        }
        if (fd->flags & ULOOP_WRITE)
        {
            sets |= MainloopContext::kWriteFdSet;
        }
        aMainloop.AddFdToSet(fd->fd, sets);
    }

    std::chrono::duration<int, std::milli> timeout(uloop_get_next_timeout());
    if (timeout.count() > 0)
    {
        auto timeval = ToTimeval(timeout);
        if (timercmp(&timeval, &aMainloop.mTimeout, <))
        {
            aMainloop.mTimeout = timeval;
        }
    }
}

void UloopProcessor::Process(const MainloopContext &aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);
    uloop_run_timeout(0);
}

// === UBusAgent ===

UBusAgent::UBusAgent(otbr::Host::RcpHost &aHost)
    : ubus_context{}
    , uloop_timeout{}
    , mServer(Context(), aHost)
{
}

UBusAgent::~UBusAgent()
{
    uloop_timeout_cancel(&ReconnectTimer());
    ubus_shutdown(&Context());
}

void UBusAgent::Init()
{
    UloopProcessor::Init();

    // Hard-fail on initial connection error; later disconnection is handled gracefully.
    VerifyOrDie(ubus_connect_ctx(&Context(), nullptr) == 0, "Unable to connect to ubus");
    UbusConnected();

    mServer.Init();
}

void UBusAgent::UbusConnected()
{
    otbrLogInfo("Connected to ubus (peer id %08x)", Context().local_id);
    Context().connection_lost = [](ubus_context *aCtx) { static_cast<UBusAgent *>(aCtx)->OnConnectionLost(); };
    ubus_add_uloop(&Context());
}

void UBusAgent::OnConnectionLost()
{
    otbrLogInfo("Connection to ubus lost");
    // Make the first attempt immediately
    OnReconnectTimer();
}

void UBusAgent::OnReconnectTimer()
{
    otbrLogDebug("Reconnecting...");
    if (ubus_reconnect(&Context(), nullptr) == 0)
    {
        UbusConnected(); // our objects have been republished by ubus_reconnect()
    }
    else
    {
        ReconnectTimer().cb = [](uloop_timeout *aTimeout) { static_cast<UBusAgent *>(aTimeout)->OnReconnectTimer(); };
        uloop_timeout_set(&ReconnectTimer(), 2000 /* ms */);
    }
}

} // namespace ubus
} // namespace otbr
