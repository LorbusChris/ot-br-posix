/*
 *  Copyright (c) 2019-2026, The OpenThread Authors.
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

#define OTBR_LOG_TAG "UBUS"

#include "openwrt/ubus/otubus.hpp"

#include <openthread/netdiag.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>

#include "common/logging.hpp"
#include "common/time.hpp"
#include "host/rcp_host.hpp"
#include "openwrt/ubus/ubus_utils.hpp"

#include <functional>

using std::placeholders::_1;

namespace otbr {
namespace ubus {

static constexpr uint32_t kDefaultJoinerTimeout = 120;

static otError ParseLong(char *aString, long &aLong);

static char const *DeviceRoleToString(otDeviceRole aRole)
{
    const char *string = "invalid";
    switch (aRole)
    {
    case OT_DEVICE_ROLE_DISABLED:
        string = "disabled";
        break;

    case OT_DEVICE_ROLE_DETACHED:
        string = "detached";
        break;

    case OT_DEVICE_ROLE_CHILD:
        string = "child";
        break;

    case OT_DEVICE_ROLE_ROUTER:
        string = "router";
        break;

    case OT_DEVICE_ROLE_LEADER:
        string = "leader";
        break;
    }
    return string;
}

static bool IsAttached(otDeviceRole aRole)
{
    switch (aRole)
    {
    case OT_DEVICE_ROLE_CHILD:
    case OT_DEVICE_ROLE_ROUTER:
    case OT_DEVICE_ROLE_LEADER:
        return true;
    default:
        return false;
    }
}

// === UbusServer ===

UbusServer::UbusServer(ubus_context &aContext, Host::RcpHost &aHost)
    : mContext(aContext)
    , mHost(aHost)
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
        ExitNow();
    }

    mHost.GetThreadHelper()->AddDeviceRoleHandler(std::bind(&UbusServer::HandleDeviceRoleChanged, this, _1));
    mHost.GetThreadHelper()->AddActiveDatasetChangeHandler(
        std::bind(&UbusServer::HandleActiveDatasetChanged, this, _1));
    mHost.GetThreadHelper()->AddPendingDatasetChangeHandler(
        std::bind(&UbusServer::HandlePendingDatasetChanged, this, _1));

exit:
    return;
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

void UbusServer::SendInvokeResponse(ubus_request_data *aRequest, blob_buf *aBuf, otError aError)
{
    blobmsg_add_u16(aBuf, "Error", aError);
    ubus_send_reply(&mContext, aRequest, aBuf->head);
}

std::function<void(otError, const std::string &)> UbusServer::DeferResponse(ubus_request_data *aRequest)
{
    // The deferred request has to outlive this handler, so it is owned by the
    // receiver and released once the response has been sent.
    auto deferred = std::make_shared<ubus_request_data>();

    ubus_defer_request(&mContext, aRequest, deferred.get());

    return [this, deferred](otError aError, const std::string &aInfo) {
        blob_buf buf{};

        if (aError != OT_ERROR_NONE && !aInfo.empty())
        {
            otbrLogWarning("Deferred ubus request failed: %s", aInfo.c_str());
        }

        blob_buf_init(&buf, 0);
        // Not mBuf: that buffer belongs to whichever request is being handled
        // synchronously by the time this callback runs.
        SendInvokeResponse(deferred.get(), &buf, aError);
        ubus_complete_deferred_request(&mContext, deferred.get(), 0);
        blob_buf_free(&buf);
    };
}

void UbusServer::HandleActiveScanResultDetail(otActiveScanResult *aResult)
{
    void *jsonList = nullptr;

    if (aResult == nullptr)
    {
        blobmsg_close_array(&mScanBuf, mScanArray);
        SendInvokeResponse(&mScanRequest, &mScanBuf, OT_ERROR_NONE);
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
    SuccessOrExit(error = otLinkActiveScan(mHost.GetInstance(), scanChannels, scanDuration,
                                           &UbusServer::HandleActiveScanResult, this));

    ubus_defer_request(&mContext, aRequest, &mScanRequest);
    blob_buf_init(&mScanBuf, 0);
    mScanArray = blobmsg_open_array(&mScanBuf, "scan_list");

exit:
    if (error != OT_ERROR_NONE)
    {
        SendInvokeResponse(aRequest, &mBuf, error);
    }
    return 0;
}

int UbusServer::HandleLeave(ubus_request_data *aRequest)
{
    ubus_request_data request;

    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);

    // Complete the request immediately because otInstanceFactoryReset() won't return.
    ubus_defer_request(&mContext, aRequest, &request);
    ubus_complete_deferred_request(&mContext, &request, 0);

    otInstanceFactoryReset(mHost.GetInstance());
    return 0;
}

int UbusServer::HandleThreadStart(ubus_request_data *aRequest)
{
    auto respond = DeferResponse(aRequest);

    // Going through the host abstraction keeps RcpHost's enabled-state
    // machine in sync; enabling the stack directly leaves it saying
    // disabled, which makes ScheduleMigration() refuse to run and Leave()
    // silently skip its dataset erase. SetThreadEnabled() only starts the
    // stack when a dataset is already committed, so follow up with the
    // direct calls to keep this method's contract of actually starting
    // Thread; both are no-ops when the stack is already up.
    mHost.SetThreadEnabled(true, [this, respond](otError aError, const std::string &aInfo) {
        otError error = aError;

        if (error == OT_ERROR_NOT_IMPLEMENTED)
        {
            // NCP does not implement SetThreadEnabled and does not need it.
            error = OT_ERROR_NONE;
        }

        if (error == OT_ERROR_NONE)
        {
            otOperationalDatasetTlvs dataset;

            // Preserve the old contract: starting without a dataset is an
            // error. Forcing MLE up without one leaves the stack scanning in
            // the detached role forever, and blocks provision, which requires
            // the disabled role.
            if (otDatasetGetActiveTlvs(mHost.GetInstance(), &dataset) != OT_ERROR_NONE)
            {
                error = OT_ERROR_INVALID_STATE;
            }
        }

        if (error == OT_ERROR_NONE)
        {
            error = otIp6SetEnabled(mHost.GetInstance(), true);
        }

        if (error == OT_ERROR_NONE)
        {
            error = otThreadSetEnabled(mHost.GetInstance(), true);
        }

        respond(error, aInfo);
    });

    return 0;
}

int UbusServer::HandleThreadStop(ubus_request_data *aRequest)
{
    // The counterpart of threadstart: SetThreadEnabled(false) detaches
    // gracefully before disabling the stack, and moves the host state
    // machine along with it, where the direct calls would leave it saying
    // enabled.
    mHost.SetThreadEnabled(false, DeferResponse(aRequest));

    return 0;
}

int UbusServer::HandleParent(ubus_request_data *aRequest)
{
    otError      error = OT_ERROR_NONE;
    otRouterInfo parentInfo;
    void        *jsonList  = nullptr;
    void        *jsonArray = nullptr;

    SuccessOrExit(error = otThreadGetParentInfo(mHost.GetInstance(), &parentInfo));

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
    SendInvokeResponse(aRequest, &mBuf, error);
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

    while (otThreadGetNextNeighborInfo(mHost.GetInstance(), &iterator, &neighborInfo) == OT_ERROR_NONE)
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
    SendInvokeResponse(aRequest, &mBuf, error);
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

    SuccessOrExit(error = otDatasetGetActive(mHost.GetInstance(), &dataset));

    if (tb[NETWORKKEY] != nullptr)
    {
        dataset.mComponents.mIsNetworkKeyPresent = true;
        VerifyOrExit(
            blobmsg_get_hex_string_fixed(tb[NETWORKKEY], dataset.mNetworkKey.m8, sizeof(dataset.mNetworkKey.m8)) > 0,
            error = OT_ERROR_PARSE);
        length = 0;
    }
    if (tb[NETWORKNAME] != nullptr)
    {
        const char *networkname                   = blobmsg_get_string(tb[NETWORKNAME]);
        dataset.mComponents.mIsNetworkNamePresent = true;
        VerifyOrExit((length = static_cast<int>(strlen(networkname))) <= OT_NETWORK_NAME_MAX_SIZE,
                     error = OT_ERROR_PARSE);
        memset(&dataset.mNetworkName, 0, sizeof(dataset.mNetworkName));
        memcpy(dataset.mNetworkName.m8, networkname, static_cast<size_t>(length));
        length = 0;
    }
    if (tb[EXTPANID] != nullptr)
    {
        dataset.mComponents.mIsExtendedPanIdPresent = true;
        VerifyOrExit(blobmsg_get_hex_string_fixed(tb[EXTPANID], dataset.mExtendedPanId.m8,
                                                  sizeof(dataset.mExtendedPanId.m8)) > 0,
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
        VerifyOrExit(blobmsg_get_hex_string_fixed(tb[PSKC], dataset.mPskc.m8, sizeof(dataset.mPskc.m8)) > 0,
                     error = OT_ERROR_PARSE);
    }
    dataset.mActiveTimestamp.mSeconds++;
    if (otCommissionerGetState(mHost.GetInstance()) == OT_COMMISSIONER_STATE_DISABLED)
    {
        otCommissionerStop(mHost.GetInstance());
    }
    SuccessOrExit(error = otDatasetSendMgmtActiveSet(mHost.GetInstance(), &dataset, tlvs, static_cast<uint8_t>(length),
                                                     /* aCallback */ nullptr,
                                                     /* aContext */ nullptr));
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
    return 0;
}

int UbusServer::HandleCommissionerStart(ubus_request_data *aRequest)
{
    otError error = OT_ERROR_NONE;

    if (otCommissionerGetState(mHost.GetInstance()) == OT_COMMISSIONER_STATE_DISABLED)
    {
        error = otCommissionerStart(mHost.GetInstance(), &UbusServer::HandleStateChanged,
                                    &UbusServer::HandleJoinerEvent, this);
    }
    SendInvokeResponse(aRequest, &mBuf, error);
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
            VerifyOrExit(blobmsg_get_hex_string_fixed(tb[EUI64], addr.m8, sizeof(addr.m8)) > 0, error = OT_ERROR_PARSE);
            addrPtr = &addr;
        }
    }

    SuccessOrExit(error = otCommissionerAddJoiner(mHost.GetInstance(), addrPtr, pskd, kDefaultJoinerTimeout));

exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
            VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], addr.m8, sizeof(addr.m8)) > 0, error = OT_ERROR_PARSE);
            addrPtr = &addr;
        }
    }

    SuccessOrExit(error = otCommissionerRemoveJoiner(mHost.GetInstance(), addrPtr));

exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
    blobmsg_add_string(&mBuf, "NetworkName", otThreadGetNetworkName(mHost.GetInstance()));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleInterfaceName(ubus_request_data *aRequest)
{
    blobmsg_add_string(&mBuf, "InterfaceName", mHost.GetInterfaceName());
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleState(ubus_request_data *aRequest)
{
    blobmsg_add_string(&mBuf, "State", DeviceRoleToString(otThreadGetDeviceRole(mHost.GetInstance())));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleChannel(ubus_request_data *aRequest)
{
    blobmsg_add_u32(&mBuf, "Channel", otLinkGetChannel(mHost.GetInstance()));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandlePanId(ubus_request_data *aRequest)
{
    blobmsg_printf(&mBuf, "PanId", "0x%04x", otLinkGetPanId(mHost.GetInstance()));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleRloc16(ubus_request_data *aRequest)
{
    blobmsg_printf(&mBuf, "rloc16", "0x%04x", otThreadGetRloc16(mHost.GetInstance()));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleNetworkKey(ubus_request_data *aRequest)
{
    otNetworkKey key;

    otThreadGetNetworkKey(mHost.GetInstance(), &key);
    blobmsg_add_hex_string(&mBuf, "Networkkey", key.m8, OT_NETWORK_KEY_SIZE);
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandlePskc(ubus_request_data *aRequest)
{
    otPskc pskc;

    otThreadGetPskc(mHost.GetInstance(), &pskc);
    blobmsg_add_hex_string(&mBuf, "pskc", pskc.m8, OT_PSKC_MAX_SIZE);
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleExtPanId(ubus_request_data *aRequest)
{
    const uint8_t *extPanId = reinterpret_cast<const uint8_t *>(otThreadGetExtendedPanId(mHost.GetInstance()));
    blobmsg_add_hex_string(&mBuf, "ExtPanId", extPanId, OT_EXT_PAN_ID_SIZE);
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleMode(ubus_request_data *aRequest)
{
    otLinkModeConfig linkMode;
    char             mode[5] = "";

    memset(&linkMode, 0, sizeof(otLinkModeConfig));

    linkMode = otThreadGetLinkMode(mHost.GetInstance());

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
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandlePartitionId(ubus_request_data *aRequest)
{
    blobmsg_add_u32(&mBuf, "Partitionid", otThreadGetPartitionId(mHost.GetInstance()));
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleLeaderData(ubus_request_data *aRequest)
{
    otError      error = OT_ERROR_NONE;
    void        *jsonTable;
    otLeaderData leaderData;

    SuccessOrExit(error = otThreadGetLeaderData(mHost.GetInstance(), &leaderData));

    jsonTable = blobmsg_open_table(&mBuf, "leaderdata");

    blobmsg_add_u32(&mBuf, "PartitionId", leaderData.mPartitionId);
    blobmsg_add_u32(&mBuf, "Weighting", leaderData.mWeighting);
    blobmsg_add_u32(&mBuf, "DataVersion", leaderData.mDataVersion);
    blobmsg_add_u32(&mBuf, "StableDataVersion", leaderData.mStableDataVersion);
    blobmsg_add_u32(&mBuf, "LeaderRouterId", leaderData.mLeaderRouterId);

    blobmsg_close_table(&mBuf, jsonTable);

exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        IgnoreError(otThreadSendDiagnosticGet(mHost.GetInstance(), &address, tlvTypes, count,
                                              &UbusServer::HandleDiagnosticGetResponse, this));
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
    while (otCommissionerGetNextJoinerInfo(mHost.GetInstance(), &iterator, &joinerInfo) == OT_ERROR_NONE)
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
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleMacFilterState(ubus_request_data *aRequest)
{
    otMacFilterAddressMode mode = otLinkFilterGetAddressMode(mHost.GetInstance());

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

    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleMacFilterAddr(ubus_request_data *aRequest)
{
    otMacFilterEntry    entry;
    otMacFilterIterator iterator = OT_MAC_FILTER_ITERATOR_INIT;
    void               *jsonArray;

    jsonArray = blobmsg_open_array(&mBuf, "addrlist");

    while (otLinkFilterGetNextAddress(mHost.GetInstance(), &iterator, &entry) == OT_ERROR_NONE)
    {
        blobmsg_add_hex_string(&mBuf, "addr", entry.mExtAddress.m8, sizeof(entry.mExtAddress.m8));
    }

    blobmsg_close_array(&mBuf, jsonArray);
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
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
    snprintf(networkdata, sizeof(networkdata), "networkdata%d", mNetworkDataIndex++);
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
        SuccessOrExit(error = otThreadSetNetworkName(mHost.GetInstance(), newName));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        SuccessOrExit(error = otLinkSetChannel(mHost.GetInstance(), static_cast<uint8_t>(channel)));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        error = otLinkSetPanId(mHost.GetInstance(), static_cast<otPanId>(value));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], key.m8, sizeof(key.m8)) > 0, error = OT_ERROR_PARSE);
        SuccessOrExit(error = otThreadSetNetworkKey(mHost.GetInstance(), &key));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], pskc.m8, sizeof(pskc.m8)) > 0, error = OT_ERROR_PARSE);
        SuccessOrExit(error = otThreadSetPskc(mHost.GetInstance(), &pskc));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], extPanId.m8, sizeof(extPanId.m8)) > 0,
                     error = OT_ERROR_PARSE);
        error = otThreadSetExtendedPanId(mHost.GetInstance(), &extPanId);
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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

        SuccessOrExit(error = otThreadSetLinkMode(mHost.GetInstance(), linkMode));
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
        VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], extAddr.m8, sizeof(extAddr.m8)) > 0,
                     error = OT_ERROR_PARSE);

        error = otLinkFilterAddAddress(mHost.GetInstance(), &extAddr);

        VerifyOrExit(error == OT_ERROR_NONE || error == OT_ERROR_ALREADY);
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
    return 0;
}

int UbusServer::HandleMacFilterRemove(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError      error = OT_ERROR_INVALID_ARGS;
    otExtAddress extAddr;

    if (aArgs[0] != nullptr)
    {
        VerifyOrExit(blobmsg_get_hex_string_fixed(aArgs[0], extAddr.m8, sizeof(extAddr.m8)) > 0,
                     error = OT_ERROR_PARSE);

        otLinkFilterRemoveAddress(mHost.GetInstance(), &extAddr);
        error = OT_ERROR_NONE;
    }
exit:
    SendInvokeResponse(aRequest, &mBuf, error);
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
            otLinkFilterSetAddressMode(mHost.GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_DISABLED);
        }
        else if (strcmp(state, "allowlist") == 0)
        {
            otLinkFilterSetAddressMode(mHost.GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_ALLOWLIST);
        }
        else if (strcmp(state, "denylist") == 0)
        {
            otLinkFilterSetAddressMode(mHost.GetInstance(), OT_MAC_FILTER_ADDRESS_MODE_DENYLIST);
        }
    }
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleMacFilterClear(ubus_request_data *aRequest)
{
    otLinkFilterClearAddresses(mHost.GetInstance());
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleVersion(ubus_request_data *aRequest)
{
    blobmsg_add_string(&mBuf, "OtbrVersion", OTBR_PACKAGE_VERSION);
    blobmsg_add_string(&mBuf, "HostVersion", otGetVersionString());
    blobmsg_add_string(&mBuf, "RcpVersion", otPlatRadioGetVersionString(mHost.GetInstance()));
    blobmsg_add_string(&mBuf, "ThreadVersion", mHost.GetThreadVersion());
    blobmsg_add_u16(&mBuf, "ThreadVersionCode", otThreadGetVersion());
    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

int UbusServer::HandleStatus(ubus_request_data *aRequest)
{
    {
        otBorderAgentId id;
        if (otBorderAgentGetId(mHost.GetInstance(), &id) == OT_ERROR_NONE)
        {
            blobmsg_add_hex_string(&mBuf, "BorderAgentId", id.mId, sizeof(id.mId));
        }
    }

    {
        otDeviceRole role = otThreadGetDeviceRole(mHost.GetInstance());
        blobmsg_add_string(&mBuf, "DeviceRole", DeviceRoleToString(role));
        blobmsg_add_u8(&mBuf, "Attached", IsAttached(role));
    }

    {
        otOperationalDatasetTlvs dataset;
        mHost.GetDatasetActiveTlvs(dataset);
        blobmsg_add_hex_string(&mBuf, "ActiveDataset", dataset.mTlvs, dataset.mLength);
        mHost.GetDatasetPendingTlvs(dataset);
        blobmsg_add_hex_string(&mBuf, "PendingDataset", dataset.mTlvs, dataset.mLength);
    }

    SendInvokeResponse(aRequest, &mBuf, OT_ERROR_NONE);
    return 0;
}

static constexpr blobmsg_policy kProvisionPolicy[] = {
    [0] = {.name = "dataset", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleProvision(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError                  error = OT_ERROR_INVALID_ARGS;
    int                      datasetLength;
    otOperationalDatasetTlvs dataset;

    VerifyOrExit(aArgs[0] != nullptr);
    VerifyOrExit((datasetLength = blobmsg_get_hex_string(aArgs[0], dataset.mTlvs, sizeof(dataset.mTlvs))) > 0);
    dataset.mLength = datasetLength;

    // Only form a network when there is none, mirroring the Matter cluster,
    // which rejects SetActiveDatasetRequest once a dataset is configured.
    // GetDeviceRole() is used rather than Ip6IsEnabled() because the latter is
    // not implemented under NCP.
    VerifyOrExit(mHost.GetDeviceRole() == OT_DEVICE_ROLE_DISABLED, error = OT_ERROR_INVALID_STATE);

    {
        auto respond = DeferResponse(aRequest);

        // Going straight to otThreadSetEnabled() would start Thread while the
        // host still considers it disabled, and ScheduleMigration() would then
        // refuse to run. SetThreadEnabled() is what moves that state, and it is
        // idempotent.
        mHost.SetThreadEnabled(true, [this, dataset, respond](otError aError, const std::string &aInfo) {
            otError error = aError;

            // NCP does not implement SetThreadEnabled and does not need it: its
            // Join() has no enabled-state precondition.
            if (error == OT_ERROR_NOT_IMPLEMENTED)
            {
                error = OT_ERROR_NONE;
            }

            if (error == OT_ERROR_NONE)
            {
                // Commit the dataset here so a malformed one still fails the
                // call; Join() commits the same TLVs again.
                error = otDatasetSetActiveTlvs(mHost.GetInstance(), &dataset);
            }

            if (error != OT_ERROR_NONE)
            {
                respond(error, aInfo);
                return;
            }

            // Respond once the join is under way rather than when the device
            // attaches: attaching takes long enough that callers time out
            // waiting, and the Matter TBRM delegate has to answer its
            // controller well before then. The attach outcome is reported
            // through the device_role_changed notification.
            mHost.Join(dataset, [](otError aJoinError, const std::string &aJoinInfo) {
                if (aJoinError == OT_ERROR_NONE)
                {
                    otbrLogInfo("provision: join succeeded");
                }
                else
                {
                    otbrLogWarning("provision: join failed: %s (%s)", otThreadErrorToString(aJoinError),
                                   aJoinInfo.c_str());
                }
            });
            respond(OT_ERROR_NONE, "");
        });
    }

    return 0;

exit:
    SendInvokeResponse(aRequest, &mBuf, error);
    return 0;
}

int UbusServer::HandleDeprovision(ubus_request_data *aRequest)
{
    // Detach gracefully and erase the dataset, returning the device to the
    // unprovisioned state that provision expects. This is the counterpart of
    // provision, not of leave, which factory resets the whole instance.
    mHost.Leave(/* aEraseDataset */ true, DeferResponse(aRequest));
    return 0;
}

static constexpr blobmsg_policy kSetPendingPolicy[] = {
    [0] = {.name = "dataset", .type = BLOBMSG_TYPE_STRING},
};

int UbusServer::HandleSetPending(ubus_request_data *aRequest, blob_attr *(&aArgs)[1])
{
    otError                  error = OT_ERROR_INVALID_ARGS;
    int                      datasetLength;
    otOperationalDatasetTlvs dataset;

    VerifyOrExit(aArgs[0] != nullptr);
    VerifyOrExit((datasetLength = blobmsg_get_hex_string(aArgs[0], dataset.mTlvs, sizeof(dataset.mTlvs))) > 0);
    dataset.mLength = datasetLength;

    // ScheduleMigration() sends MGMT_PENDING_SET, so the network switches to the
    // new dataset when its delay timer expires instead of immediately. It
    // requires an attached device and reports its result asynchronously.
    mHost.ScheduleMigration(dataset, DeferResponse(aRequest));

    return 0;

exit:
    SendInvokeResponse(aRequest, &mBuf, error);
    return 0;
}

void UbusServer::HandleDeviceRoleChanged(otDeviceRole aRole)
{
    blob_buf_init(&mBuf, 0);
    blobmsg_add_string(&mBuf, "DeviceRole", DeviceRoleToString(aRole));
    blobmsg_add_u8(&mBuf, "Attached", IsAttached(aRole));
    ubus_notify(&mContext, &Object(), "device_role_changed", mBuf.head, -1);
}

void UbusServer::HandleActiveDatasetChanged(const otOperationalDatasetTlvs &aDataset)
{
    blob_buf_init(&mBuf, 0);
    blobmsg_add_hex_string(&mBuf, "ActiveDataset", aDataset.mTlvs, aDataset.mLength);
    ubus_notify(&mContext, &Object(), "active_dataset_changed", mBuf.head, -1);
}

void UbusServer::HandlePendingDatasetChanged(const otOperationalDatasetTlvs &aDataset)
{
    // An empty dataset is reported once a scheduled migration has completed,
    // which is how a subscriber learns that the switch has happened.
    blob_buf_init(&mBuf, 0);
    blobmsg_add_hex_string(&mBuf, "PendingDataset", aDataset.mTlvs, aDataset.mLength);
    ubus_notify(&mContext, &Object(), "pending_dataset_changed", mBuf.head, -1);
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

    OTBR_UBUS_METHOD_NOARG("version", &UbusServer::HandleVersion),
    OTBR_UBUS_METHOD_NOARG("status", &UbusServer::HandleStatus),
    OTBR_UBUS_METHOD("provision", &UbusServer::HandleProvision, kProvisionPolicy),
    OTBR_UBUS_METHOD("set_pending", &UbusServer::HandleSetPending, kSetPendingPolicy),
    OTBR_UBUS_METHOD_NOARG("deprovision", &UbusServer::HandleDeprovision),
};

ubus_object_type UbusServer::sObjectType = UBUS_OBJECT_TYPE("otbr", sMethods);

otError ParseLong(char *aString, long &aLong)
{
    char *endptr;
    aLong = strtol(aString, &endptr, 0);
    return (*endptr == '\0') ? OT_ERROR_NONE : OT_ERROR_PARSE;
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
