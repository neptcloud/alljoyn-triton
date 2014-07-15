/******************************************************************************
 * Copyright (c) 2013 - 2014, AllSeen Alliance. All rights reserved.
 *
 *    Permission to use, copy, modify, and/or distribute this software for any
 *    purpose with or without fee is hereby granted, provided that the above
 *    copyright notice and this permission notice appear in all copies.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 ******************************************************************************/

/**
 * Per-module definition of the current module for debug logging.  Must be defined
 * prior to first inclusion of aj_debug.h.
 * The corresponding flag dbgAJSERVICESHANDLERS is defined below statically.
 */
#define AJ_MODULE AJSERVICESHANDLERS
#include <aj_debug.h>

#include <alljoyn.h>
#include "Services_Handlers.h"

#include "Services_Common.h"
#include "PropertyStore.h"
#include "AboutSample.h"
#ifdef CONFIG_SERVICE
    #include "ConfigSample.h"
#endif
#ifdef ONBOARDING_SERVICE
    #include "OnboardingSample.h"
    #include <alljoyn/onboarding/OnboardingManager.h>
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER
    #include "NotificationProducerSample.h"
#endif
#ifdef CONTROLPANEL_SERVICE
    #include "ControlPanelSample.h"
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER
    #include "NotificationConsumerSample.h"
#endif

#include <aj_config.h>
#include <aj_link_timeout.h>

#ifndef NDEBUG
#ifndef ER_DEBUG_AJSERVICESHANDLERS
#define ER_DEBUG_AJSERVICESHANDLERS 0
#endif
static AJ_EXPORT uint8_t dbgAJSERVICESHANDLERS = ER_DEBUG_AJSERVICESHANDLERS;
#endif

/*
 * Define timeout/pause values. Values are in milli seconds.
 * The following may be tuned according to platform requirements such as battery usage.
 */
#define AJAPP_CONNECT_TIMEOUT     AJ_CONNECT_TIMEOUT
#define AJAPP_CONNECT_PAUSE       (1000 * 2) // Override AJ_CONNECT_PAUSE to be more responsive
#define AJAPP_SLEEP_TIME          (1000 * 2) // A little pause to let things settle

const char SESSIONLESS_MATCH[] = "sessionless='t',type='error'"; //AddMatch to allow sessionless messages coming in

#ifdef NOTIFICATION_SERVICE_CONSUMER
uint8_t addSessionLessMatch = TRUE;
#else
uint8_t addSessionLessMatch = FALSE;
#endif

typedef enum {
    INIT_START = 0,
    INIT_SERVICES_PORT = INIT_START,
    INIT_ADVERTISE_NAME,
    INIT_ADDSLMATCH,
    INIT_FINISHED
} enum_init_state_t;

static const uint8_t MAX_INIT_RETRIES = 5;
static uint8_t init_retries = 0;


static uint32_t PasswordCallback(uint8_t* buffer, uint32_t bufLen)
{
    AJ_Status status = AJ_OK;
    const char* hexPassword;
    size_t hexPasswordLen;
    uint32_t len = 0;

    hexPassword = AJSVC_PropertyStore_GetValue(AJSVC_PROPERTY_STORE_PASSCODE);
    if (hexPassword == NULL) {
        AJ_AlwaysPrintf(("Password is NULL!\n"));
        return len;
    }
    AJ_AlwaysPrintf(("Retrieved password=%s\n", hexPassword));
    hexPasswordLen = strlen(hexPassword);
    len = hexPasswordLen / 2;
    status = AJ_HexToRaw(hexPassword, hexPasswordLen, buffer, bufLen);
    if (status == AJ_ERR_RESOURCES) {
        len = 0;
    }

    return len;
}

AJ_Status AJServices_Init(AJ_Object* appObjects, AJ_Object* proxyObjects, AJ_Object* announceObjects, const char* deviceManufactureName, const char* deviceProductName)
{
    AJ_Status status = AJ_OK;

#ifdef CONFIG_SERVICE
    status = Config_Init();
    if (status != AJ_OK) {
        goto Exit;
    }
#endif
#ifdef ONBOARDING_SERVICE
    status = Onboarding_Init(deviceManufactureName, deviceProductName);
    if (status != AJ_OK) {
        goto Exit;
    }
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER
    status = NotificationProducer_Init();
    if (status != AJ_OK) {
        goto Exit;
    }
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER
    status = NotificationConsumer_Init(proxyObjects);
    if (status != AJ_OK) {
        goto Exit;
    }
#endif
#ifdef CONTROLPANEL_SERVICE
    status = Controlee_Init();
    if (status != AJ_OK) {
        goto Exit;
    }
#endif

Exit:

    return status;
}

uint8_t AJRouter_Connect(AJ_BusAttachment* busAttachment, const char* routerName)
{
    while (TRUE) {
        AJ_Status status = AJ_OK;
#ifdef ONBOARDING_SERVICE
        status = AJOBS_EstablishWiFi();
        if (status != AJ_OK) {
            AJ_AlwaysPrintf(("Failed to establish WiFi connectivity with status=%s\n", AJ_StatusText(status)));
            AJ_Sleep(AJAPP_CONNECT_PAUSE);
            return FALSE;
        }
#endif
        AJ_AlwaysPrintf(("Attempting to connect to bus '%s'\n", routerName));
        status = AJ_FindBusAndConnect(busAttachment, routerName, AJAPP_CONNECT_TIMEOUT);
        if (status != AJ_OK) {
            AJ_AlwaysPrintf(("Failed to connect to bus sleeping for %d seconds\n", AJAPP_CONNECT_PAUSE / 1000));
            AJ_Sleep(AJAPP_CONNECT_PAUSE);
#ifdef ONBOARDING_SERVICE
            if (status == AJ_ERR_DHCP) {
                AJOBS_SwitchToRetry();
            }
#endif
            continue;
        }
        const char* busUniqueName = AJ_GetUniqueName(busAttachment);
        if (busUniqueName == NULL) {
            AJ_AlwaysPrintf(("Failed to GetUniqueName() from newly connected bus, retrying\n"));
            continue;
        }
        AJ_AlwaysPrintf(("Connected to router with BusUniqueName=%s\n", busUniqueName));
        break;
    }
    return TRUE;
}

static enum_init_state_t currentServicesInitializationState = INIT_START;
static enum_init_state_t nextServicesInitializationState = INIT_START;

AJ_Status AJApp_ConnectedHandler(AJ_BusAttachment* busAttachment)
{
    AJ_Status status = AJ_OK;
    if (AJ_GetUniqueName(busAttachment)) {
        if (currentServicesInitializationState == nextServicesInitializationState) {
            switch (currentServicesInitializationState) {
            case INIT_SERVICES_PORT:
                status = AJ_BusBindSessionPort(busAttachment, AJ_ABOUT_SERVICE_PORT, NULL, 0);
                if (status != AJ_OK) {
                    goto Exit;
                }
                nextServicesInitializationState = INIT_ADVERTISE_NAME;
                break;

            case INIT_ADVERTISE_NAME:
                status = AJ_BusAdvertiseName(busAttachment, AJ_GetUniqueName(busAttachment), AJ_TRANSPORT_ANY, AJ_BUS_START_ADVERTISING, 0);
                if (status != AJ_OK) {
                    goto Exit;
                }
                if (addSessionLessMatch) {
                    nextServicesInitializationState = INIT_ADDSLMATCH;
                } else {
                    nextServicesInitializationState = INIT_FINISHED;
                }
                break;

            case INIT_ADDSLMATCH:
                status = AJ_BusSetSignalRule(busAttachment, SESSIONLESS_MATCH, AJ_BUS_SIGNAL_ALLOW);
                if (status != AJ_OK) {
                    goto Exit;
                }
                nextServicesInitializationState = INIT_FINISHED;
                break;

            case INIT_FINISHED:
                if (AJ_About_IsShouldAnnounce()) {
                    status = AJ_About_Announce(busAttachment);
                    if (status != AJ_OK) {
                        goto Exit;
                    }
                    AJ_About_SetShouldAnnounce(FALSE);
                }
#ifdef ONBOARDING_SERVICE
                if (!AJOBS_IsWiFiConnected()) {
                    status = AJ_ERR_RESTART;
                }
#endif
                break;
            }
        }
    }

Exit:

    if (status == AJ_ERR_RESOURCES) {
        init_retries++;
        if (init_retries > MAX_INIT_RETRIES) {
            status = AJ_ERR_READ; // Force disconnect
        } else {
            AJ_Sleep(AJAPP_SLEEP_TIME);
        }
    }
    return status;
}

AJ_Status AJServices_ConnectedHandler(AJ_BusAttachment* busAttachment)
{
    AJ_BusSetPasswordCallback(busAttachment, PasswordCallback);
    /* Configure timeout for the link to the Router bus */
    AJ_SetBusLinkTimeout(busAttachment, 60);     // 60 seconds

    AJ_Status status = AJ_OK;

    status = AJ_About_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#ifdef CONFIG_SERVICE

    status = AJCFG_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#endif
#ifdef ONBOARDING_SERVICE

    status = AJOBS_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER

    status = AJNS_Producer_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#endif
#ifdef CONTROLPANEL_SERVICE

    status = AJCPS_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER

    status = AJNS_Consumer_ConnectedHandler(busAttachment);
    if (status != AJ_OK) {
        goto ErrorExit;
    }
#endif
    return status;

ErrorExit:

    AJ_AlwaysPrintf(("Service ConnectedHandler returned an error %s\n", (AJ_StatusText(status))));
    return status;
}

static AJSVC_ServiceStatus AJApp_MessageProcessor(AJ_BusAttachment* bus, AJ_Message* msg, AJ_Status* status)
{
    AJSVC_ServiceStatus serviceStatus = AJSVC_SERVICE_STATUS_NOT_HANDLED;

    switch (currentServicesInitializationState) {
    case INIT_SERVICES_PORT:
        if (msg->msgId == AJ_REPLY_ID(AJ_METHOD_BIND_SESSION_PORT)) {
            currentServicesInitializationState = nextServicesInitializationState;
        }
        break;

    case INIT_ADVERTISE_NAME:
        if (msg->msgId == AJ_REPLY_ID(AJ_METHOD_ADVERTISE_NAME)) {
            currentServicesInitializationState = nextServicesInitializationState;
        }
        break;

    case INIT_ADDSLMATCH:
        if (msg->msgId == AJ_REPLY_ID(AJ_METHOD_ADD_MATCH)) {
            currentServicesInitializationState = nextServicesInitializationState;
        }
        break;

    default:
        break;
    }

    return serviceStatus;
}

static uint8_t AJServices_CheckSessionAccepted(uint16_t port, uint32_t sessionId, char* joiner)
{
    uint8_t session_accepted = FALSE;
    session_accepted |= (port == AJ_ABOUT_SERVICE_PORT);

#ifdef NOTIFICATION_SERVICE_PRODUCER
    session_accepted |= AJNS_Producer_CheckSessionAccepted(port, sessionId, joiner);
#endif

#ifdef CONTROLPANEL_SERVICE
    session_accepted |= AJCPS_CheckSessionAccepted(port, sessionId, joiner);
#endif

    return session_accepted;
}

static AJSVC_ServiceStatus AJServices_SessionJoinedHandler(AJ_BusAttachment* busAttachment, uint32_t sessionId, uint32_t replySerialNum)
{
    AJSVC_ServiceStatus serviceStatus = AJSVC_SERVICE_STATUS_NOT_HANDLED;

#ifdef NOTIFICATION_SERVICE_CONSUMER
    if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
        serviceStatus = AJNS_Consumer_SessionJoinedHandler(busAttachment, sessionId, replySerialNum);
    }
#endif

    return serviceStatus;
}

static AJSVC_ServiceStatus AJServices_SessionRejectedHandler(AJ_BusAttachment* busAttachment, uint32_t sessionId, uint32_t replySerialNum, uint32_t replyCode)
{
    AJSVC_ServiceStatus serviceStatus = AJSVC_SERVICE_STATUS_NOT_HANDLED;

#ifdef NOTIFICATION_SERVICE_CONSUMER
    if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
        serviceStatus = AJNS_Consumer_SessionRejectedHandler(busAttachment, replySerialNum, replyCode);
    }
#endif

    return serviceStatus;
}

static AJSVC_ServiceStatus AJServices_SessionLostHandler(AJ_BusAttachment* busAttachment, uint32_t sessionId, uint32_t reason)
{
    AJSVC_ServiceStatus serviceStatus = AJSVC_SERVICE_STATUS_NOT_HANDLED;

#ifdef NOTIFICATION_SERVICE_CONSUMER
    if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
        serviceStatus = AJNS_Consumer_SessionLostHandler(busAttachment, sessionId, reason);
    }
#endif

    return serviceStatus;
}

AJSVC_ServiceStatus AJServices_MessageProcessor(AJ_BusAttachment* busAttachment, AJ_Message* msg, AJ_Status* status)
{
    AJSVC_ServiceStatus serviceStatus = AJSVC_SERVICE_STATUS_NOT_HANDLED;
    if  (msg->msgId == AJ_METHOD_ACCEPT_SESSION) {    // Process all incoming request to join a session and pass request for acceptance by all services
        uint16_t port;
        char* joiner;
        uint32_t sessionId = 0;
        AJ_UnmarshalArgs(msg, "qus", &port, &sessionId, &joiner);
        uint8_t session_accepted = FALSE;

        session_accepted |= AJServices_CheckSessionAccepted(port, sessionId, joiner);

        *status = AJ_BusReplyAcceptSession(msg, session_accepted);
        AJ_AlwaysPrintf(("%s session session_id=%u joiner=%s for port %u\n", (session_accepted ? "Accepted" : "Rejected"), sessionId, joiner, port));
        serviceStatus = AJSVC_SERVICE_STATUS_HANDLED;
    } else if (msg->msgId == AJ_REPLY_ID(AJ_METHOD_JOIN_SESSION)) {     // Process all incoming replies to join a session and pass session state change to all services
        uint32_t replyCode = 0;
        uint32_t sessionId = 0;
        uint8_t sessionJoined = FALSE;
        uint32_t joinSessionReplySerialNum = msg->replySerial;
        if (msg->hdr->msgType == AJ_MSG_ERROR) {
            AJ_AlwaysPrintf(("JoinSessionReply: AJ_METHOD_JOIN_SESSION: AJ_ERR_FAILURE\n"));
            *status = AJ_ERR_FAILURE;
        } else {
            *status = AJ_UnmarshalArgs(msg, "uu", &replyCode, &sessionId);
            if (*status != AJ_OK) {
                AJ_AlwaysPrintf(("JoinSessionReply: failed to unmarshal\n"));
            } else {
                if (replyCode == AJ_JOINSESSION_REPLY_SUCCESS) {
                    AJ_AlwaysPrintf(("JoinSessionReply: AJ_JOINSESSION_REPLY_SUCCESS with sessionId=%u and replySerial=%u\n", sessionId, joinSessionReplySerialNum));
                    sessionJoined = TRUE;
                } else {
                    AJ_AlwaysPrintf(("JoinSessionReply: AJ_ERR_FAILURE\n"));
                    *status = AJ_ERR_FAILURE;
                }
            }
        }
        if (sessionJoined) {
            serviceStatus = AJServices_SessionJoinedHandler(busAttachment, sessionId, joinSessionReplySerialNum);
        } else {
            serviceStatus = AJServices_SessionRejectedHandler(busAttachment, sessionId, joinSessionReplySerialNum, replyCode);
        }
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            AJ_ResetArgs(msg);
        }
    } else if (msg->msgId == AJ_SIGNAL_SESSION_LOST || msg->msgId == AJ_SIGNAL_SESSION_LOST_WITH_REASON) {     // Process all incoming LeaveSession replies and lost session signals and pass session state change to all services
        uint32_t sessionId = 0;
        uint32_t reason = 0;
        if (msg->msgId == AJ_SIGNAL_SESSION_LOST_WITH_REASON) {
            *status = AJ_UnmarshalArgs(msg, "uu", &sessionId, &reason);
        } else {
            *status = AJ_UnmarshalArgs(msg, "u", &sessionId);
        }
        if (*status != AJ_OK) {
            AJ_AlwaysPrintf(("JoinSessionReply: failed to marshal\n"));
        } else {
            AJ_AlwaysPrintf(("Session lost: sessionId = %u, reason = %u\n", sessionId, reason));
            serviceStatus = AJServices_SessionLostHandler(busAttachment, sessionId, reason);
            if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
                AJ_ResetArgs(msg);
            }
        }
    } else {
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJApp_MessageProcessor(busAttachment, msg, status);
        }
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJ_About_MessageProcessor(busAttachment, msg, status);
        }
#ifdef CONFIG_SERVICE
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJCFG_MessageProcessor(busAttachment, msg, status);
        }
#endif
#ifdef ONBOARDING_SERVICE
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJOBS_MessageProcessor(busAttachment, msg, status);
        }
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJNS_Producer_MessageProcessor(busAttachment, msg, status);
        }
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJNS_Consumer_MessageProcessor(busAttachment, msg, status);
        }
#endif
#ifdef CONTROLPANEL_SERVICE
        if (serviceStatus == AJSVC_SERVICE_STATUS_NOT_HANDLED) {
            serviceStatus = AJCPS_MessageProcessor(busAttachment, msg, status);
        }
#endif
    }
    return serviceStatus;
}

void AJServices_DoWork(AJ_BusAttachment* busAttachment)
{
    About_DoWork(busAttachment);
#ifdef CONFIG_SERVICE
    Config_DoWork(busAttachment);
#endif
#ifdef ONBOARDING_SERVICE
    Onboarding_DoWork(busAttachment);
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER
    NotificationProducer_DoWork(busAttachment);
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER
    NotificationConsumer_DoWork(busAttachment);
#endif
#ifdef CONTROLPANEL_SERVICE
    Controlee_DoWork(busAttachment);
#endif
}

void AJServices_DisconnectHandler(AJ_BusAttachment* busAttachment)
{
    AJ_About_DisconnectHandler(busAttachment);
#ifdef CONFIG_SERVICE
    AJCFG_DisconnectHandler(busAttachment);
#endif
#ifdef ONBOARDING_SERVICE
    AJOBS_DisconnectHandler(busAttachment);
#endif
#ifdef NOTIFICATION_SERVICE_CONSUMER
    AJNS_Consumer_DisconnectHandler(busAttachment);
#endif
#ifdef NOTIFICATION_SERVICE_PRODUCER
    AJNS_Producer_DisconnectHandler(busAttachment);
#endif
#ifdef CONTROLPANEL_SERVICE
    AJCPS_DisconnectHandler(busAttachment);
#endif
}

AJ_Status AJApp_DisconnectHandler(AJ_BusAttachment* busAttachment, uint8_t restart)
{
    AJ_Status status = AJ_OK;

    if (restart) {
        AJ_BusAdvertiseName(busAttachment, AJ_GetUniqueName(busAttachment), AJ_TRANSPORT_ANY, AJ_BUS_STOP_ADVERTISING, 0);
        AJ_BusUnbindSession(busAttachment, AJ_ABOUT_SERVICE_PORT);
    }

    AJ_About_SetShouldAnnounce(TRUE);
    currentServicesInitializationState = nextServicesInitializationState = INIT_START;
    init_retries = 0;

    return status;
}

uint8_t AJRouter_Disconnect(AJ_BusAttachment* busAttachment, uint8_t disconnectWiFi)
{
    AJ_AlwaysPrintf(("AllJoyn disconnect\n"));
    AJ_Sleep(AJAPP_SLEEP_TIME); // Sleep a little to let any pending requests to router to be sent
    AJ_Disconnect(busAttachment);
#ifdef ONBOARDING_SERVICE
    if (disconnectWiFi) {
        AJOBS_DisconnectWiFi();
    }
#endif
    AJ_Sleep(AJAPP_SLEEP_TIME); // Sleep a little while before trying to reconnect

    return TRUE;
}
