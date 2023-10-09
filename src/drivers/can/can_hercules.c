/*
@file    can_hercules.c
@brief   CAN driver for Hercules TMS570LC43x in libcsp.
@version 20220822 v1.0.0   FH Initial release.
*/

// Includes
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/csp_iflist.h>
#include <csp/csp_types.h>
#include <csp/drivers/can_hercules.h>

#include <csptests.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "os_semphr.h"

// CAN related
#include "HL_can.h"
#include "HL_esm.h"
#include "HL_sys_core.h"
#include "HL_reg_can.h"

// libcsp
#include <csp/csp_rtable.h>

// CAN interface data, state, etc.
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
    TaskHandle_t rxTaskHandler;
	// pthread_t rx_thread; TODO: Implement this
} can_context_t;

#define CSP_CAN_RX_TASK_NAME "CSP_CAN_RX"
#define CSP_CAN_RX_TASK_STACK_SIZE 512

static TaskHandle_t rxTaskHandler = NULL;
static StaticTask_t rxTaskTCB;
static StackType_t rxTaskStackBuffer[CSP_CAN_RX_TASK_STACK_SIZE];

// Implementation of strtok_r, which is not implemented as default in CCS12
char *
strtok_r (char *s, const char *delim, char **save_ptr)
{
  char *end;
  if (s == NULL)
    s = *save_ptr;
  if (*s == '\0')
    {
      *save_ptr = s;
      return NULL;
    }
  /* Scan leading delimiters.  */
  s += strspn (s, delim);
  if (*s == '\0')
    {
      *save_ptr = s;
      return NULL;
    }
  /* Find the end of the token.  */
  end = s + strcspn (s, delim);
  if (*end == '\0')
    {
      *save_ptr = end;
      return s;
    }
  /* Terminate the token and make *SAVE_PTR point past it.  */
  *end = '\0';
  *save_ptr = end + 1;
  return s;
}

/*
    @brief   Open Can Channel and add CSP Interface
    @param[in] name Interface name
    @param[in] canRegister CAN register
    @param[in] canMessageBox CAN message box
    @return 0 on success, -1 on error
*/
int csp_can_hercules_add_interface ( const char * ifname,
                                            canBASE_t  * canRegister, 
                                            uint8_t      canMessageBox)
{
    can_context_t * ctx = calloc((size_t) 1, sizeof(*ctx));
    if (ctx == NULL) {
            return CSP_ERR_NOMEM;
        }
    int res = -1;

    strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);

    ctx->iface.name = ctx->name;
    ctx->iface.interface_data = &ctx->ifdata;
    ctx->iface.driver_data = ctx;
    ctx->ifdata.tx_func = csp_can_hercules_tx_frame;
//    ctx->ifdata.pbufs = NULL;


    // call csp_iflist_add(csp_iface_t * ifc) here

    /*
    * Create rx task
    */
    rxTaskHandler = xTaskCreateStatic(csp_hercules_can_rx_task,
                                        CSP_CAN_RX_TASK_NAME, 
                                        CSP_CAN_RX_TASK_STACK_SIZE,
                                        NULL, FREE_RTOS_TASK_PRIORITY_CANRX,
                                        rxTaskStackBuffer, &rxTaskTCB);

    if (rxTaskHandler != NULL)
    {
        ctx->rxTaskHandler = rxTaskHandler;
        // Para libscp for gomspace
        res = csp_can_add_interface(&ctx->iface);

        csp_rtable_set(CSP_DEFAULT_ROUTE, 0, &ctx->iface, CSP_NO_VIA_ADDRESS);
        //csp_rtable_set(0, 0, &ctx->iface, CSP_NO_VIA_ADDRESS);
        // Para libcsp de libcsp.org usamos esto
        // Set this as the default interface
      // csp_iflist_set_default(&ctx->iface);
    }

    return res;    
}

static void csp_hercules_can_rx_task(void * pvParameters)
{

    //canUpdateID(canREG1, canMESSAGE_BOX1, 0x30000004);
    while(1)
    {
        vTaskDelay(1000);
    }
}

/*
    @brief  Transmit CAN frame
    @param[in] driver_data Driver data
    @param[in] id CAN ID
    @param[in] data Data to transmit
    @param[in] dlc Data length code
    @return 0 on success, -1 on error
*/
static int csp_can_hercules_tx_frame (void * driver_data, uint32_t id,
                                      const uint8_t * data, uint8_t dlc)
{
    if (dlc > 8) {
		return CSP_ERR_INVAL;
	}
    uint8_t i;
    can_context_t * ctx = driver_data;
    const uint32 s_canByteOrder[8U] = {3U, 2U, 1U, 0U, 7U, 6U, 5U, 4U};

//    canTransmit(canREG1, canMESSAGE_BOX1, (uint8 * )&data); /* copy to RAM */
//    return CSP_ERR_NONE;
    while ((canREG1->IF1STAT & 0x80U) ==0x80U)
    {
    } /* Wait */

    // Configure mask register.
    //      MXtd = 1 Use Extended ID Mask
    //      MDir = 1 Use message direction mask
    //      Msk = 0x7FF Use bits 10:0 for filtering
    //canREG1->IF1MSK = 0xC0000000U | ((0x000007FFU & 0x1FFFFFFFU) << 0U);
    canREG1->IF1MSK = 0x0U; // Disable all filtering

    // Configure arbitration register.
    //      MsgVal = 1 Enable the message object
    //      Xtd = 1 Extended 25 bit identifier
    //      Dir = 1 Transmit mail box
    //      ID 1 Message ID 0x1
    //canREG1->IF1ARB = 0x80000000U | 0x40000000U | 0x20000000U | ((1U & 0x1FFFFFFFU) << 0U) ;
    //canREG1->IF1ARB = 0x80000000U | 0x40000000U | 0x20000000U | ((1U & 0x1FFFFFFFU) << 0U) ;
    canREG1->IF1ARB = 0x80000000U | // Message is valid
                      0x40000000U | // Message uses extended identifier
                      0x20000000U | // Message is set for transmission
                      id;



    // Configure message control register.
    //      UMask = 1 Use mask for filtering
    //      EoB = 1 Single message object
    //      TxIE = 1 Enable transmit interrupt
    //      RxIE = 1 Enable receive interrupt
    //      DLC = 8 Set data length as 8
    //canREG1->IF1MCTL = 0x00001080U | 0x00000C00U | 8U ;
    canREG1->IF1MCTL = 0x00001080U | 0x00000C00U | dlc ;

    // Configure command register.
    //      Message number = 0x20 Message number is 0x20
    //canREG1->IF1CMD = 0x20 ;
    //canREG1->IF1CMD = 0x87U;
    canREG1->IF1CMD = 0xB7U;

    for (i = 0U; i < dlc; i++)
    {
        canREG1->IF1DATx[s_canByteOrder[i]] = *data;
        //canREG1->IF1DATx[i] = *data;
        data++;
    }

    canREG1->IF1NO = 1 ;

/*
	struct can_frame frame = {.can_id = id | CAN_EFF_FLAG,
                                .can_dlc = dlc};
	memcpy(frame.data, data, dlc);

	uint32_t elapsed_ms = 0;
	can_context_t * ctx = driver_data;

	while (write(ctx->socket, &frame, sizeof(frame)) != sizeof(frame)) {
		if ((errno != ENOBUFS) || (elapsed_ms >= 1000)) {
			csp_print("%s[%s]: write() failed, errno %d: %s\n", __FUNCTION__, ctx->name, errno, strerror(errno));
			return CSP_ERR_TX;
		}
		usleep(5000);
		elapsed_ms += 5;
	}
*/

	return CSP_ERR_NONE;
}
