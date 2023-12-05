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

static const uint32 s_canByteOrder[8U] = {3U, 2U, 1U, 0U, 7U, 6U, 5U, 4U};

// CAN interface data, state, etc.
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
    TaskHandle_t rxTaskHandler;
	// pthread_t rx_thread; TODO: Implement this
} can_context_t;

static can_context_t * ctx = NULL;

#define CSP_CAN_RX_TASK_NAME "CSP_CAN_RX"
#define CSP_CAN_RX_TASK_STACK_SIZE 512

static TaskHandle_t rxTaskHandler = NULL;
static StaticTask_t rxTaskTCB;
static StackType_t rxTaskStackBuffer[CSP_CAN_RX_TASK_STACK_SIZE];

// Can related
#define D_COUNT  8

// CAN RX data
uint32 cnt=0, error =0, tx_done =0;
uint8 tx_data1[D_COUNT] = {1,2,3,4,5,6,7,8};
uint8 tx_data2[D_COUNT] = {11,12,13,14,15,16,17,18};
uint8 tx_data3[D_COUNT] = {21,22,23,24,25,26,27,28};
uint8 tx_data4[D_COUNT] = {31,32,33,34,35,36,37,38};

uint32 rx_header = 0;
uint8 rx_data1[D_COUNT] = {0};
uint8 rx_data2[D_COUNT] = {0};
uint8 rx_data3[D_COUNT] = {0};
uint8 rx_data4[D_COUNT] = {0};

uint8 *dptr=0;


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
    //can_context_t * ctx = calloc((size_t) 1, sizeof(*ctx));
    // Allocate memory for context using calloc
    ctx = calloc((size_t) 1, sizeof(*ctx));
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

/** @fn uint32 canGetData(canBASE_t *node, uint32 messageBox, uint8 * const data)
*   @brief Gets received a CAN message
*   @param[in] node Pointer to CAN node:
*              - canREG1: CAN1 node pointer
*              - canREG2: CAN2 node pointer
*              - canREG3: CAN3 node pointer
*              - canREG4: CAN4 node pointer
*   @param[in] messageBox Message box number of CAN node:
*              - canMESSAGE_BOX1: CAN message box 1
*              - canMESSAGE_BOXn: CAN message box n [n: 1-64]
*              - canMESSAGE_BOX64: CAN message box 64
*   @param[out] data Pointer to store CAN RX data
*   @return The function will return:
*           - 0: When RX message box hasn't received new data
*           - 1: When RX data are stored in the data buffer
*           - 3: When RX data are stored in the data buffer and a message was lost
*
*   This function writes a CAN message into a CAN message box.
*
*/


/* USER CODE BEGIN (9) */
/* USER CODE END */

/* SourceId : CAN_SourceId_003 */
/* DesignId : CAN_DesignId_003 */
/* Requirements : HL_CONQ_CAN_SR6 */
uint32 canRxData(canBASE_t *node, uint32 messageBox, uint32 * const header, uint8 * const data)
{
    uint32       i;
    uint32       size;
    uint8 * pData    = data;
    uint32       success  = 0U;
    uint32       regIndex = (messageBox - 1U) >> 5U;
    uint32       bitIndex = 1U << ((messageBox - 1U) & 0x1FU);

    /** - Check if new data have been arrived:
    *   - no new data, return 0
    *   - new data, get received message
    */
    if ((node->NWDATx[regIndex] & bitIndex) == 0U)
    {
        success = 0U;
    }

    else
    {
    /** - Wait until IF2 is ready for use */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((node->IF2STAT & 0x80U) ==0x80U)
    {
    } /* Wait */

    /** - Configure IF2 for
    *     - Message direction - Read
    *     - Data Read
    *     - Clears NewDat bit in the message object.
    */
    node->IF2CMD = 0x77U;

    /** - Copy data into IF2 */
    /*SAFETYMCUSW 93 S MR: 6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "LDRA Tool issue" */
    node->IF2NO = (uint8) messageBox;

    /** - Wait until data are copied into IF2 */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((node->IF2STAT & 0x80U) ==0x80U)
    {
    } /* Wait */

    /** - Get number of received bytes */
    size = node->IF2MCTL & 0xFU;
    if(size > 0x8U)
    {
        size = 0x8U;
    }
    /** - Copy RX data into destination buffer */
    for (i = 0U; i < size; i++)
    {
#if ((__little_endian__ == 1) || (__LITTLE_ENDIAN__ == 1))
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        *pData = node->IF2DATx[i];
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        pData++;
#else
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        *pData = node->IF2DATx[s_canByteOrder[i]];
        //*pData = node->IF2DATx[i];
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are only allowed in this driver" */
        pData++;
#endif
    }

    *header = (uint32) node->IF2ARB & 0x0FFFFFFFU;

    success = 1U;
    }
    /** - Check if data have been lost:
    *     - no data lost, return 1
    *     - data lost, return 3
    */
    if ((node->IF2MCTL & 0x4000U) == 0x4000U)
    {
        success = 3U;
    }

    return success;
}

void canMessageNotification(canBASE_t *node, uint32 messageBox)
{
    // Return value of canRxData
    uint32 rxSuccess = 0U;
    long task_woken = pdTRUE;

    if(node==canREG1)
    {
        rxSuccess = canRxData(canREG1, canMESSAGE_BOX2, (uint32 * ) &rx_header, (uint8 * )&rx_data1[0]); /* copy to RAM */
        if (rxSuccess == 1U)
        {
            cnt++;
            csp_can_rx(&ctx->iface, rx_header, rx_data1, 8, &task_woken);
        }
        else if (rxSuccess == 3U)
        {
            error++;
        }
        else
        {
            /* No data received */
        }
    }
}
