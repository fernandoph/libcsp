#pragma once

/**
 * @file       sci_hercules.h
 * @brief      Hercules KISS Driver 
 * @details    Hercules KISS Driver (for Hercules MCU).
 * @author     Pablo Llull (PLL) <pllull@epic-aerospace.com>
 * @date       2025-01-06
 * @version    0.1
 * @pre        First initialize the KISS driver with _init()
 * @bug        No known bugs.
 */

#ifndef CSP_DRIVERS_SCI_HERCULES_H
#define CSP_DRIVERS_SCI_HERCULES_H

// SCI related
#include "HL_sci.h"
#include "HL_reg_het.h"
#include "HL_sys_core.h"

#include <csp/csp.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/csp_iflist.h>
#include <csp/csp_types.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Context definition for SCI driver 
 */
typedef struct {
    char name[CSP_IFLIST_NAME_MAX + 1];
    csp_iface_t iface;
    csp_kiss_interface_data_t ifdata;
    sciBASE_t *sci_base;    // SCI base to use
} sci_hercules_context_t;


/**
 * @brief Transmission function for SCI KISS interface
 * @param[in] driver_data Pointer to driver context
 * @param[in] data Data buffer to transmit
 * @param[in] data_length Length of data buffer
 * @return CSP_ERR_NONE on success, otherwise an error code
*/
int sci_hercules_tx(void *driver_data, const unsigned char * data, size_t data_length);

/**
 * @brief Reception callback for SCI KISS interface
 * @param[in] ctx Pointer to driver context
 * @param[in] data Received byte
 * @param[in] pxTaskWoken FreeRTOS task woken flag
 */
void sci_hercules_rx_callback(sci_hercules_context_t *ctx, uint8_t data, void *pxTaskWoken);

/**
 * @brief      Gets the sci context.
 * @param      sci   The sci
 * @return     The sci context.
 */
sci_hercules_context_t* get_sci_context(sciBASE_t *sci);


/**
 * @brief Initialize SCI interface for KISS
 * This is a convenience function for opening a SCI device and adding it
 * as a KISS interface with a given name.
 * 
 * @param[in] name Interface name (will be copied), or use NULL for default name
 * @param[in] sci_base SCI base to use (sciREG1, sciREG2, etc)
 * @param[out] return_iface The added interface
 * @return #CSP_ERR_NONE on success, otherwise an error code
 */
int csp_sci_hercules_init(const char *name, sciBASE_t *sci_base, csp_iface_t **return_iface);

#ifdef __cplusplus
}
#endif
#endif /* CSP_DRIVERS_SCI_HERCULES_H */