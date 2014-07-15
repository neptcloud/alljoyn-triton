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

#ifndef _ABOUTICONSERVICE_H_
#define _ABOUTICONSERVICE_H_

/** @defgroup AboutIcon
 *
 *  @{
 */

#include "Services_Common.h"

/**
 * published About Icon interfaces that will be announced
 */
extern const AJ_InterfaceDescription AJ_AboutIconInterfaces[];

#define NUM_ABOUT_ICON_OBJECTS 1

#define ABOUT_ICON_APPOBJECTS \
    { "/About/DeviceIcon",   AJ_AboutIconInterfaces },

/*
 * AboutIcon API
 */

AJ_Status AJ_AboutIcon_Start(const char* aboutIconMimetype, const uint8_t* aboutIconContent, const size_t aboutIconContentSize, const char* aboutIconUrl);

AJSVC_ServiceStatus AJ_AboutIcon_MessageProcessor(AJ_BusAttachment* bus, AJ_Message* msg, AJ_Status* msgStatus);

/** @} */

 #endif // _ABOUTICONSERVICE_H_
