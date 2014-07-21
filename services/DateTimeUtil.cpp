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

#include <DateTimeUtil.h>
#include <ControlMarshalUtil.h>

AJ_Status marshalDatePropertyValue(DatePropertyValue* datePropertyValue, AJ_Message* reply)
{
    AJ_Status status;
    AJ_Arg outerStructArg, innerStructArg;

    status = AJ_MarshalVariant(reply, DATE_PROPERTY_SIG);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalContainer(reply, &outerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalArgs(reply, "q", DATE_PROPERTY);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalContainer(reply, &innerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_MarshalArgs(reply, "qqq", datePropertyValue->mDay, datePropertyValue->month, datePropertyValue->fullYear);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_MarshalCloseContainer(reply, &innerStructArg);
    if (status != AJ_OK) {
        return status;
    }
    return AJ_MarshalCloseContainer(reply, &outerStructArg);
}

AJ_Status marshalTimePropertyValue(TimePropertyValue* timePropertyValue, AJ_Message* reply)
{
    AJ_Status status;
    AJ_Arg outerStructArg, innerStructArg;

    status = AJ_MarshalVariant(reply, TIME_PROPERTY_SIG);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalContainer(reply, &outerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalArgs(reply, "q", TIME_PROPERTY);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_MarshalContainer(reply, &innerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_MarshalArgs(reply, "qqq", timePropertyValue->hour, timePropertyValue->minute, timePropertyValue->second);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_MarshalCloseContainer(reply, &innerStructArg);
    if (status != AJ_OK) {
        return status;
    }
    return AJ_MarshalCloseContainer(reply, &outerStructArg);
}

AJ_Status unmarshalDatePropertyValue(DatePropertyValue* datePropertyValue, AJ_Message* message)
{
    AJ_Status status;
    AJ_Arg outerStructArg, innerStructArg;
    uint16_t widgetType;

    status = AJ_UnmarshalContainer(message, &outerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_UnmarshalArgs(message, "q", &widgetType);
    if (status != AJ_OK) {
        return status;
    }

    if (widgetType != DATE_PROPERTY) {
        return AJ_ERR_UNEXPECTED;
    }
    status = AJ_UnmarshalContainer(message, &innerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_UnmarshalArgs(message, "qqq", &datePropertyValue->mDay, &datePropertyValue->month, &datePropertyValue->fullYear);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_UnmarshalCloseContainer(message, &innerStructArg);
    if (status != AJ_OK) {
        return status;
    }
    return AJ_UnmarshalCloseContainer(message, &outerStructArg);
}

AJ_Status unmarshalTimePropertyValue(TimePropertyValue* timePropertyValue, AJ_Message* message)
{
    AJ_Status status;
    AJ_Arg outerStructArg, innerStructArg;
    uint16_t widgetType;

    status = AJ_UnmarshalContainer(message, &outerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_UnmarshalArgs(message, "q", &widgetType);
    if (status != AJ_OK) {
        return status;
    }

    if (widgetType != TIME_PROPERTY) {
        return AJ_ERR_UNEXPECTED;
    }
    status = AJ_UnmarshalContainer(message, &innerStructArg, AJ_ARG_STRUCT);
    if (status != AJ_OK) {
        return status;
    }
    status = AJ_UnmarshalArgs(message, "qqq", &timePropertyValue->hour, &timePropertyValue->minute, &timePropertyValue->second);
    if (status != AJ_OK) {
        return status;
    }

    status = AJ_UnmarshalCloseContainer(message, &innerStructArg);
    if (status != AJ_OK) {
        return status;
    }
    return AJ_UnmarshalCloseContainer(message, &outerStructArg);
}

