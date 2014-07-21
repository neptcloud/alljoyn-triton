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

#ifndef _PROPERTY_STORE_H_
#define _PROPERTY_STORE_H_

#include <alljoyn.h>

/** @defgroup PropertyStore
 *
 *  @{
 */

/**
 * Field indices
 */
typedef enum _AJSVC_PropertyStoreFieldIndices {
    AJSVC_PROPERTY_STORE_ERROR_FIELD_INDEX = -1,
    //Start of keys
    AJSVC_PROPERTY_STORE_DEVICE_ID,
    AJSVC_PROPERTY_STORE_APP_ID,
    AJSVC_PROPERTY_STORE_DEVICE_NAME,
    AJSVC_PROPERTY_STORE_NUMBER_OF_PERSISTED_KEYS,
    //End of persisted keys that are not remotely configurable
#ifndef CONFIG_SERVICE
    AJSVC_PROPERTY_STORE_NUMBER_OF_CONFIG_KEYS = AJSVC_PROPERTY_STORE_NUMBER_OF_PERSISTED_KEYS,
    //End of configurable keys
#endif
    AJSVC_PROPERTY_STORE_DEFAULT_LANGUAGE = AJSVC_PROPERTY_STORE_NUMBER_OF_PERSISTED_KEYS,
    AJSVC_PROPERTY_STORE_PASSCODE,
    AJSVC_PROPERTY_STORE_REALM_NAME,
#ifdef CONFIG_SERVICE
    AJSVC_PROPERTY_STORE_NUMBER_OF_CONFIG_KEYS,
    //End of configurable keys
    AJSVC_PROPERTY_STORE_APP_NAME = AJSVC_PROPERTY_STORE_NUMBER_OF_CONFIG_KEYS,
#else
    AJSVC_PROPERTY_STORE_APP_NAME,
#endif
    AJSVC_PROPERTY_STORE_DESCRIPTION,
    AJSVC_PROPERTY_STORE_MANUFACTURER,
    AJSVC_PROPERTY_STORE_MODEL_NUMBER,
    AJSVC_PROPERTY_STORE_DATE_OF_MANUFACTURE,
    AJSVC_PROPERTY_STORE_SOFTWARE_VERSION,
    AJSVC_PROPERTY_STORE_AJ_SOFTWARE_VERSION,
#ifdef CONFIG_SERVICE
    AJSVC_PROPERTY_STORE_MAX_LENGTH,
#endif
    AJSVC_PROPERTY_STORE_NUMBER_OF_MANDATORY_KEYS,
    //End of mandatory keys
    AJSVC_PROPERTY_STORE_HARDWARE_VERSION = AJSVC_PROPERTY_STORE_NUMBER_OF_MANDATORY_KEYS,
    AJSVC_PROPERTY_STORE_SUPPORT_URL,
    AJSVC_PROPERTY_STORE_NUMBER_OF_KEYS,
    //End of About keys
} AJSVC_PropertyStoreFieldIndices;

/**
 * Get maximum value length for given key.
 * @param reply
 * @param fieldIndex
 * @param langIndex
 * @return aj_status
 */
uint8_t AJSVC_PropertyStore_GetMaxValueLength(AJSVC_PropertyStoreFieldIndices fieldIndex);

/**
 * Language indices
 */
#define AJSVC_PROPERTY_STORE_ERROR_LANGUAGE_INDEX   -1
#define AJSVC_PROPERTY_STORE_NO_LANGUAGE_INDEX       0

/**
 * Bitfield that defines the category to filter the properties
 */
typedef struct _AJSVC_PropertyStoreCategoryFilter {
    uint8_t bit0About : 1;
#ifdef CONFIG_SERVICE
    uint8_t bit1Config : 1;
#endif
    uint8_t bit2Announce : 1;
} AJSVC_PropertyStoreCategoryFilter;

/**
 * Read all properties that match filter for given a language.
 * @param reply
 * @param filter
 * @param langIndex
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_ReadAll(AJ_Message* reply, AJSVC_PropertyStoreCategoryFilter filter, int8_t langIndex);

/**
 * Update given property with value for given language.
 * @param key
 * @param langIndex
 * @param value
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_Update(const char* key, int8_t langIndex, const char* value);

/**
 * Reset given property back to default for given language.
 * @param key
 * @param langIndex
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_Reset(const char* key, int8_t langIndex);

/**
 * Reset all properties back to defaults.
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_ResetAll();

/**
 * get field name for given field index
 * @param fieldIndex
 * @return fieldName
 */
const char* AJSVC_PropertyStore_GetFieldName(AJSVC_PropertyStoreFieldIndices fieldIndex);

/**
 * Get field index for given field name.
 * @param fieldName
 * @return fieldIndex
 */
AJSVC_PropertyStoreFieldIndices AJSVC_PropertyStore_GetFieldIndex(const char* fieldName);

/**
 * Get value for given field index for given language index.
 * @param fieldIndex
 * @param langIndex
 * @return value
 */
const char* AJSVC_PropertyStore_GetValueForLang(AJSVC_PropertyStoreFieldIndices fieldIndex, int8_t langIndex);

/**
 * Get value for field index for default language.
 * @param fieldIndex
 * @return value
 */
const char* AJSVC_PropertyStore_GetValue(AJSVC_PropertyStoreFieldIndices fieldIndex);

/**
 * Set value for given field index for given language index.
 * @param fieldIndex
 * @param langIndex
 * @param value
 * @return success
 */
uint8_t AJSVC_PropertyStore_SetValueForLang(AJSVC_PropertyStoreFieldIndices fieldIndex, int8_t langIndex, const char* value);

/**
 * Set value for given field index for the default language.
 * @param fieldIndex
 * @param value
 * @return success
 */
uint8_t AJSVC_PropertyStore_SetValue(AJSVC_PropertyStoreFieldIndices fieldIndex, const char* value);

/**
 * Get default language index among all languages indexes.
 * @return langIndex
 */
int8_t AJSVC_PropertyStore_GetCurrentDefaultLanguageIndex();

/**
 * Get language name for given language index.
 * @param langIndex
 * @return languageName
 */
const char* AJSVC_PropertyStore_GetLanguageName(int8_t langIndex);

/**
 * Get the language index for the given language name.
 * @param language
 * @return langIndex
 */
int8_t AJSVC_PropertyStore_GetLanguageIndex(const char* const language);

/**
 * Load all persisted values.
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_LoadAll();

/**
 * Save all persisted values.
 * @return aj_status
 */
AJ_Status AJSVC_PropertyStore_SaveAll();

/** @} */
 #endif /* _PROPERTY_STORE_H_ */
