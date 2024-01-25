#pragma once

/**
 * @file       can_hercules.h
 * @brief      Hercules CAN Driver 
 * @details    Hercules CAN Driver (for Hercules MCU).
 * @author     Fernando Hauscarriaga (FH) <fhauscarriaga@epic-aerospace.com>
 * @date       2018-08-01
 * @version    0.1
 * @pre        First initialize the CAN driver with can_init()
 * @bug        No known bugs.
 */

// CAN related
#include "HL_can.h"
#include "HL_esm.h"
#include "HL_sys_core.h"
#include "HL_reg_can.h"

#define CSP_MYADDRESS 10

/*
    Initialize CAN driver
*/
int can_myinit(void);

/*
    Open Can Channel and add CSP Interface
*/
int csp_can_hercules_add_interface ( const char * ifname,
                                            canBASE_t  * canRegister, 
                                            uint8_t      canMessageBox);

/* 
    Transmit CAN frame
*/
static int csp_can_hercules_tx_frame (void * driver_data, uint32_t id, 
                                     const uint8_t * data, uint8_t dlc);

/* 
    Receive CAN task
*/
static void csp_hercules_can_rx_task(void * pvParameters);

/*
 * canRxData interrupt service routine
 */
uint32 canRxData(canBASE_t *node, uint32 messageBox, uint32 * header, uint8 * data, uint8 *dlc);

/*
* Epic_canInit
*/
void epic_can_init(void);




