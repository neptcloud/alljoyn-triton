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

#ifndef _SERVICES_HANDLERS_H_
#define _SERVICES_HANDLERS_H_
#define CONTROLPANEL_SERVICE 1

#include <alljoyn.h>
#include "Services_Common.h"

/**
 * Initialize the Services. called at the start of the application
 */
AJ_Status AJServices_Init(AJ_Object* appObjects, AJ_Object* proxyObjects, AJ_Object* announceObjects, const char* deviceManufactureName, const char* deviceProductName);

/**
 * Connect to Router
 * @param busAttachment
 * @param routerName - name of the Router to connect to
 * @return true/false if connected successfully
 */
uint8_t AJRouter_Connect(AJ_BusAttachment* busAttachment, const char* routerName);

/**
 * Run when the bus is connected to the Router
 * application is idle
 * @param busAttachment
 * @return aj_status for last request to router
 */
AJ_Status AJApp_ConnectedHandler(AJ_BusAttachment* busAttachment);

/**
 * Functions to call after the Router is Connected
 * @param busAttachment
 * @return ajStatus - status of last request to Router
 */
AJ_Status AJServices_ConnectedHandler(AJ_BusAttachment* busAttachment);

/**
 * Process an incoming message
 * @param busAttachment
 * @param msg
 * @param status
 * @return servicestatus - shows if the message was processed or not
 */
AJSVC_ServiceStatus AJServices_MessageProcessor(AJ_BusAttachment* busAttachment, AJ_Message* msg, AJ_Status* status);

/**
 * Run when there is a timeout reading off the bus
 * application is idle
 * @param busAttachment
 */
void AJServices_DoWork(AJ_BusAttachment* busAttachment);

/**
 * Shutdown services. Should be called on bus disconnect
 */
void AJServices_DisconnectHandler();

/**
 * Run when the bus is disconnecting from the Router
 * Connection to Router is either restarting or was stopped
 * @param busAttachment
 * @param restart
 * @return aj_status for last request to Router
 */
AJ_Status AJApp_DisconnectHandler(AJ_BusAttachment* busAttachment, uint8_t restart);

/**
 * Disconnect from the Router
 * @param busAttachment
 * @param disconnectWiFi
 * @return true/false if disconnected successfully
 */
uint8_t AJRouter_Disconnect(AJ_BusAttachment* busAttachment, uint8_t disconnectWiFi);

#endif /* _SERVICES_HANDLERS_H_ */
