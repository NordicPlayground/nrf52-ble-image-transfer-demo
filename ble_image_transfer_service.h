/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**@file
 *
 * @defgroup ble_its Nordic UART Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service implementation.
 *
 * @details The Nordic UART Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate SoftDevice events to the Nordic UART Service module
 *       by calling the ble_its_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_IMAGE_TRANSFER_SERVICE_H__
#define BLE_IMAGE_TRANSFER_SERVICE_H__

#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_its instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _nus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_ITS_DEF(_name, _its_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_its_max_clients),                  \
                             sizeof(ble_its_client_context_t));   \
    static ble_its_t _name = {0, 0, {0,0,0,0},{0,0,0,0},{0,0,0,0}, \
                              0, 0, 0,                            \
                              &CONCAT_2(_name, _link_ctx_storage)}; \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_NUS_BLE_OBSERVER_PRIO,               \
                         ble_its_on_ble_evt,                      \
                         &_name)
    
#define BLE_UUID_ITS_SERVICE 0x0001                      /**< The UUID of the Nordic UART Service. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_ITS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#else
    #define BLE_ITS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
    #warning NRF_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define DBG_PIN_0 14
#define DBG_PIN_1 15
#define DBG_PIN_2 16
#define DBG_PIN_3 3
#define DBG_PIN_4 4


/* Forward declaration of the ble_its_t type. */
typedef struct ble_its_s ble_its_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_its_data_handler_t) (ble_its_t * p_its, uint8_t const * p_data, uint16_t length);

/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_its_client_context_t;

/**@brief Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_its_init
 *          function.
 */
typedef struct
{
    ble_its_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_its_init_t;

/**@brief Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_its_s
{
    uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t img_info_handles;      
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    bool                     is_info_char_notification_enabled;
    blcm_link_ctx_storage_t * const p_link_ctx_storage;
    ble_its_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

typedef struct
{
    uint32_t file_size_bytes;
    
}ble_its_img_info_t;

typedef PACKED_STRUCT
{
    uint16_t mtu;
    uint16_t con_interval;
    uint8_t  tx_phy;
    uint8_t  rx_phy;
}ble_its_ble_params_info_t;

/**@brief Function for initializing the Nordic UART Service.
 *
 * @param[out] p_its      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_its_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_its or p_its_init is NULL.
 */
uint32_t ble_its_init(ble_its_t * p_its, const ble_its_init_t * p_its_init);

/**@brief Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_its       Nordic UART Service structure.
 * @param[in] p_ble_evt   Event received from the SoftDevice.
 */
void ble_its_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for sending a string to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in] p_its       Pointer to the Nordic UART Service structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_its_string_send(ble_its_t * p_its, uint8_t * p_string, uint16_t length);

uint32_t ble_its_ble_params_info_send(ble_its_t * p_its, ble_its_ble_params_info_t * ble_params_info);

uint32_t ble_its_img_info_send(ble_its_t * p_its, ble_its_img_info_t * img_info);

uint32_t ble_its_send_file(ble_its_t * p_its, uint8_t * p_data, uint32_t data_length, uint32_t max_packet_length);

uint32_t ble_its_send_file_fragment(ble_its_t * p_its, uint8_t * p_data, uint32_t data_length);

bool ble_its_file_transfer_busy(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_H__

/** @} */
