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

/**
 * @file
 * This file includes definitions for ubus API.
 */

#ifndef OTBR_AGENT_OTUBUS_HPP_
#define OTBR_AGENT_OTUBUS_HPP_

#include "openthread-br/config.h"

#include <functional>
#include <set>
#include <string>

#include <libubus.h>
#include <openthread/commissioner.h>
#include <openthread/thread.h>

#include "common/code_utils.hpp"
#include "common/mainloop.hpp"

namespace otbr {
namespace Host {
class RcpHost;
}

namespace ubus {

// Implementation note: Some of these classes use a pattern of inheriting
// privately from ubus_* structs that would commonly be members instead.
// This pattern allows static callback stubs that receive a pointer to the
// ubus structure to recover the owning object via a simple down cast from
// the base class pointer. In C, these functions would instead use
// container_of(), but its use is not generally safe in C++, especially when
// the container type is not standard-layout.

class UbusServer : private ubus_object
{
public:
    UbusServer(ubus_context &aContext, Host::RcpHost &aHost);
    ~UbusServer();

    /**
     * Publishes the server object on ubus.
     */
    void Init();

private:
    // === Ubus method handlers ===

    int HandleInterfaceName(ubus_request_data *aRequest);
    int HandleRloc16(ubus_request_data *aRequest);
    int HandleLeaderData(ubus_request_data *aRequest);
    int HandleNeighbor(ubus_request_data *aRequest);
    int HandleNetworkData(ubus_request_data *aRequest);
    int HandleParent(ubus_request_data *aRequest);
    int HandlePartitionId(ubus_request_data *aRequest);
    int HandleState(ubus_request_data *aRequest);

    int HandleChannel(ubus_request_data *aRequest);
    int HandleSetChannel(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleNetworkName(ubus_request_data *aRequest);
    int HandleSetNetworkName(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandlePanId(ubus_request_data *aRequest);
    int HandleSetPanId(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleExtPanId(ubus_request_data *aRequest);
    int HandleSetExtPanId(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleNetworkKey(ubus_request_data *aRequest);
    int HandleSetNetworkKey(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandlePskc(ubus_request_data *aRequest);
    int HandleSetPskc(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleMode(ubus_request_data *aRequest);
    int HandleSetMode(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);

    int HandleScan(ubus_request_data *aRequest);
    int HandleLeave(ubus_request_data *aRequest);
    int HandleMgmtSet(ubus_request_data *aRequest, blob_attr *(&aArgs)[6]);
    int HandleThreadStart(ubus_request_data *aRequest);
    int HandleThreadStop(ubus_request_data *aRequest);

    int HandleCommissionerStart(ubus_request_data *aRequest);
    int HandleJoinerAdd(ubus_request_data *aRequest, blob_attr *(&aArgs)[2]);
    int HandleJoinerNum(ubus_request_data *aRequest);
    int HandleJoinerRemove(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);

    int HandleMacFilterAdd(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleMacFilterRemove(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleMacFilterClear(ubus_request_data *aRequest);
    int HandleMacFilterState(ubus_request_data *aRequest);
    int HandleMacFilterSetState(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleMacFilterAddr(ubus_request_data *aRequest);

    int HandleVersion(ubus_request_data *aRequest);
    int HandleStatus(ubus_request_data *aRequest);
    int HandleProvision(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);
    int HandleSetPending(ubus_request_data *aRequest, blob_attr *(&aArgs)[1]);

    // === Callbacks ===

    // otThreadSendDiagnosticGet callback
    static void HandleDiagnosticGetResponse(otError              aError,
                                            otMessage           *aMessage,
                                            const otMessageInfo *aMessageInfo,
                                            void                *aContext);
    void        HandleDiagnosticGetResponse(otError aError, otMessage *aMessage, const otMessageInfo *aMessageInfo);

    // otLinkActiveScan callback
    static void HandleActiveScanResult(otActiveScanResult *aResult, void *aContext);
    void        HandleActiveScanResultDetail(otActiveScanResult *aResult);

    // otCommissionerStart callbacks
    static void HandleStateChanged(otCommissionerState aState, void *aContext);
    void        HandleStateChanged(otCommissionerState aState);
    static void HandleJoinerEvent(otCommissionerJoinerEvent aEvent,
                                  const otJoinerInfo       *aJoinerInfo,
                                  const otExtAddress       *aJoinerId,
                                  void                     *aContext);
    void        HandleJoinerEvent(otCommissionerJoinerEvent aEvent,
                                  const otJoinerInfo       *aJoinerInfo,
                                  const otExtAddress       *aJoinerId);

    // ThreadHelper callbacks
    void HandleDeviceRoleChanged(otDeviceRole role);
    void HandleActiveDatasetChanged(const otOperationalDatasetTlvs &dataset);
    void HandlePendingDatasetChanged(const otOperationalDatasetTlvs &dataset);

    // === Internal helpers ===

    static UbusServer &PrepareInvocation(ubus_object *aObj, ubus_request_data *aRequest, const char *aMethod);
    friend struct UbusMethodHandler; // calls PrepareInvocation

    // Adds the provided error code to the response and sends it.
    void SendInvokeResponse(ubus_request_data *aRequest, blob_buf *aBuf, otError aError);

    // Defers aRequest and returns a receiver that completes it with the async
    // result. Host operations such as Join() always report their result through
    // a callback, so the response cannot be sent from the handler itself.
    // Matches Host::ThreadHost::AsyncResultReceiver, spelled out so that this
    // header does not have to pull in the host definitions.
    std::function<void(otError, const std::string &)> DeferResponse(ubus_request_data *aRequest);

    static const ubus_method sMethods[];
    static ubus_object_type  sObjectType;

    ubus_object &Object() { return *this; }

    ubus_context  &mContext;
    Host::RcpHost &mHost;

    blob_buf mBuf{}; // default buffer for sync responses

    blob_buf mNetworkdataBuf{};
    int      mNetworkDataIndex;
    time_t   mLastNetworkDataTime = 0;

    ubus_request_data mScanRequest;
    blob_buf          mScanBuf{};
    void             *mScanArray;
};

/**
 * Integrates uloop with the OTBR main loop.
 */
class UloopProcessor : public MainloopProcessor
{
public:
    UloopProcessor() = default;
    ~UloopProcessor() override;

    /**
     * Initializes uloop.
     *
     * Only a single instance of UloopProcessor can be in an initialized state at any one time.
     *
     * Note: uloop will install signal handlers for SIGINT and SIGTERM if none have been installed yet.
     * These handlers only work correctly when using the uloop main loop via uloop_run(). This means
     * our otbr::Application handlers MUST be installed before calling this method.
     */
    void Init();

    void Update(MainloopContext &aMainloop) override;
    void Process(const MainloopContext &aMainloop) override;

private:
    static void ULoopFDHandler(uloop_fd *aFd, unsigned int aFlags);

    std::set<uloop_fd *> mFds;

    static UloopProcessor *sInstance;
};

class UBusAgent : public UloopProcessor, private ubus_context, private uloop_timeout
{
public:
    /**
     * The constructor to initialize the UBus agent.
     *
     * @param[in] aHost  A reference to the Thread controller.
     */
    UBusAgent(otbr::Host::RcpHost &aHost);
    ~UBusAgent() override;

    /**
     * Connects to ubus and publishes OTBR objects.
     */
    void Init(void);

private:
    ubus_context  &Context() { return *this; }
    uloop_timeout &ReconnectTimer() { return *this; }

    void OnConnectionLost();
    void OnReconnectTimer();

    void UbusConnected();

    UbusServer mServer;
};
} // namespace ubus
} // namespace otbr

#endif // OTBR_AGENT_OTUBUS_HPP_
