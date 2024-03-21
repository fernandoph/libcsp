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
#include "HL_hw_emac.h"

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

can_context_t * ctx = NULL;

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
char * strtok_r (char *s, const char *delim, char **save_ptr)
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
int csp_can_hercules_add_interface (const char * ifname,
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

    res = csp_can_add_interface(&ctx->iface);

    return res;    
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

    while (canIsTxMessagePending(canREG1, canMESSAGE_BOX1) != 0)
    {
    }

    while ((canREG1->IF1STAT & 0x80U) ==0x80U)
    {
    } /* Wait */

    // Configure mask register.
    //      MXtd = 1 Use Extended ID Mask
    //      MDir = 1 Use message direction mask
    //      Msk = 0x7FF Use bits 10:0 for filtering
    canREG1->IF1MSK = 0x0U; // Disable all filtering

    // Configure arbitration register.
    //      MsgVal = 1 Enable the message object
    //      Xtd = 1 Extended 25 bit identifier
    //      Dir = 1 Transmit mail box
    //      ID 1 Message ID 0x1
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
    canREG1->IF1MCTL = 0x00001080U | 0x00000C00U | dlc ;

    // Configure command register.
    // 0xB7 ==> 0b10110111
    //      Direction     1 = write
    //      Mask          0 = Mask bits will not be changed
    //      Arb           1 = The Arbitration bits (Identifier + Dir + Xtd + MsgVal)
    //                        will be transferred from the IF1/IF2 Register set to
    //                        the message object addressed by Message Number (Bits [7:0]).
    //      Control       1 = The Message Control bits will be transferred from the
    //                        IF1/IF2 Register set to the message object addressed
    //                        by Message Number (Bits [7:0]).
    //      ClrIntPnd     0 = Clear interrupt pending bit will not be changed
    //      TxRqst/NewDat 1 = Sets TxRqst/NewDat in the message object.
    //      Data A        1 = The Data Bytes 0-3 will be transferred from the IF1/IF2
    //                        Register set to the message object addressed by the
    //                        Message Number (Bits [7:0]).
    //      Data B        1 = The Data Bytes 4-7 will be transferred from the IF1/IF2
    //                        Register set to the message object addressed by the
    //                        Message Number (Bits [7:0]).
    canREG1->IF1CMD = 0xB7U;

    for (i = 0U; i < dlc; i++)
    {
        canREG1->IF1DATx[s_canByteOrder[i]] = *data;
        //canREG1->IF1DATx[i] = *data;
        data++;
    }

    canREG1->IF1NO = 1 ;

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
uint32 canRxData(canBASE_t *node, uint32 messageBox, uint32 * header, uint8 * data, uint8 *dlc)
{
    uint32       i;
    uint32       size;
    uint8 * pData    = data;
    uint32 pId;
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
    *dlc = size;
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

    *header = (node->IF2ARB & 0x3FFFFFFFU);

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

void hercules_can_init(void)
{
/* USER CODE BEGIN (4) */
/* USER CODE END */
    /** @b Initialize @b CAN1: */

    /** - Setup control register
    *     - Disable automatic wakeup on bus activity
    *     - Local power down mode disabled
    *     - Disable DMA request lines
    *     - Enable global Interrupt Line 0 and 1
    *     - Disable debug mode
    *     - Release from software reset
    *     - Enable/Disable parity or ECC
    *     - Enable/Disable auto bus on timer
    *     - Setup message completion before entering debug state
    *     - Setup normal operation mode
    *     - Request write access to the configuration registers
    *     - Setup automatic retransmission of messages
    *     - Disable error interrupts
    *     - Disable status interrupts
    *     - Enter initialization mode
    */
    canREG1->CTL = (uint32)0x00000000U
                 | (uint32)0x00000000U
                 | (uint32)((uint32)0x00000005U  << 10U)
                 | (uint32)0x00020043U;

    /** - Clear all pending error flags and reset current status */
    canREG1->ES |= 0xFFFFFFFFU;

    /** - Assign interrupt level for messages */
    canREG1->INTMUXx[0U] = (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U;

    canREG1->INTMUXx[1U] = (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U;

    /** - Setup auto bus on timer period */
    canREG1->ABOTR = (uint32)0U;

    /** - Initialize message 1
    *     - Wait until IF1 is ready for use
    *     - Set message mask
    *     - Set message control word
    *     - Set message arbitration
    *     - Set IF1 control byte
    *     - Set IF1 message number
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG1->IF1STAT & 0x80U) ==0x80U)
    {
    } /* Wait */


    canREG1->IF1MSK  = 0xC0000000U | (uint32)((uint32)((uint32)0x000007FFU & (uint32)0x1FFFFFFFU) << (uint32)0U);
    canREG1->IF1ARB  = (uint32)0x80000000U | (uint32)0x40000000U | (uint32)0x20000000U | (uint32)((uint32)((uint32)1U & (uint32)0x1FFFFFFFU) << (uint32)0U);
    canREG1->IF1MCTL = 0x00001000U | (uint32)0x00000000U | (uint32)0x00000000U | (uint32)0x00000000U | (uint32)8U;
    canREG1->IF1CMD  = (uint8) 0xF8U;
    canREG1->IF1NO   = 1U;

    /** - Initialize message 2
    *     - Wait until IF2 is ready for use
    *     - Set message mask
    *     - Set message control word
    *     - Set message arbitration
    *     - Set IF2 control byte
    *     - Set IF2 message number
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG1->IF2STAT & 0x80U) ==0x80U)
    {
    } /* Wait */

    //canREG1->IF2MSK  = 0xC0000000U | (uint32)((uint32)((uint32)0x000007FFU & (uint32)0x1FFFFFFFU) << (uint32)0U);
    canREG1->IF2MSK  = 0x0U;
    canREG1->IF2ARB  = (uint32)0x80000000U | (uint32)0x40000000U | (uint32)0x00000000U | (uint32)((uint32)((uint32)2U & (uint32)0x1FFFFFFFU) << (uint32)0U);
    canREG1->IF2MCTL = 0x00001000U | (uint32)0x00000400U | (uint32)0x00000000U | (uint32)0x00000000U | (uint32)8U;
    canREG1->IF2CMD  = (uint8) 0xF8U;
    canREG1->IF2NO   = 2U;

    /** - Setup IF1 for data transmission
    *     - Wait until IF1 is ready for use
    *     - Set IF1 control byte
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG1->IF1STAT & 0x80U) ==0x80U)
    {
    } /* Wait */
    canREG1->IF1CMD  = 0x87U;

    /** - Setup IF2 for reading data
    *     - Wait until IF1 is ready for use
    *     - Set IF1 control byte
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG1->IF2STAT & 0x80U) ==0x80U)
    {
    } /* Wait */
    canREG1->IF2CMD = 0x17U;

    /** - Setup bit timing
    *     - Setup baud rate prescaler extension
    *     - Setup TSeg2
    *     - Setup TSeg1
    *     - Setup sample jump width
    *     - Setup baud rate prescaler
    */
    canREG1->BTR = (uint32)((uint32)0U << 16U) |
                   (uint32)((uint32)(3U - 1U) << 12U) |
                   (uint32)((uint32)((8U + 3U) - 1U) << 8U) |
                   (uint32)((uint32)(3U - 1U) << 6U) |
                   (uint32)4U;



     /** - CAN1 Port output values */
    canREG1->TIOC =  (uint32)((uint32)1U  << 18U )
                   | (uint32)((uint32)0U  << 17U )
                   | (uint32)((uint32)0U  << 16U )
                   | (uint32)((uint32)1U  << 3U )
                   | (uint32)((uint32)0U  << 2U )
                   | (uint32)((uint32)0U << 1U );

    canREG1->RIOC =  (uint32)((uint32)1U  << 18U )
                   | (uint32)((uint32)0U  << 17U )
                   | (uint32)((uint32)0U  << 16U )
                   | (uint32)((uint32)1U  << 3U )
                   | (uint32)((uint32)0U  << 2U )
                   | (uint32)((uint32)0U <<1U );

    /** - Leave configuration and initialization mode  */
    canREG1->CTL &= ~(uint32)(0x00000041U);




    /** @b Initialize @b CAN1: */

    /** - Setup control register
    *     - Disable automatic wakeup on bus activity
    *     - Local power down mode disabled
    *     - Disable DMA request lines
    *     - Enable global Interrupt Line 0 and 1
    *     - Disable debug mode
    *     - Release from software reset
    *     - Enable/Disable parity or ECC
    *     - Enable/Disable auto bus on timer
    *     - Setup message completion before entering debug state
    *     - Setup normal operation mode
    *     - Request write access to the configuration registers
    *     - Setup automatic retransmission of messages
    *     - Disable error interrupts
    *     - Disable status interrupts
    *     - Enter initialization mode
    */
    canREG4->CTL = (uint32)0x00000000U
                 | (uint32)0x00000000U
                 | ((uint32)0x00000005U  << 10U)
                 | (uint32)0x00020043U;

    /** - Clear all pending error flags and reset current status */
    canREG4->ES |= 0xFFFFFFFFU;

    /** - Assign interrupt level for messages */
    canREG4->INTMUXx[0U] = (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U;

    canREG4->INTMUXx[1U] = (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U
                         | (uint32)0x00000000U;

    /** - Setup auto bus on timer period */
    canREG4->ABOTR = (uint32)0U;

    /** - Setup IF1 for data transmission
    *     - Wait until IF1 is ready for use
    *     - Set IF1 control byte
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG4->IF1STAT & 0x80U) ==0x80U)
    {
    } /* Wait */
    canREG4->IF1CMD  = 0x87U;

    /** - Setup IF2 for reading data
    *     - Wait until IF1 is ready for use
    *     - Set IF1 control byte
    */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Potentially infinite loop found - Hardware Status check for execution sequence" */
    while ((canREG4->IF2STAT & 0x80U) ==0x80U)
    {
    } /* Wait */
    canREG4->IF2CMD = 0x17U;

    /** - Setup bit timing
    *     - Setup baud rate prescaler extension
    *     - Setup TSeg2
    *     - Setup TSeg1
    *     - Setup sample jump width
    *     - Setup baud rate prescaler
    */
    canREG4->BTR = ((uint32)0U << 16U) |
                   (((uint32)3U - 1U) << 12U) |
                   ((((uint32)8U + (uint32)3U) - 1U) << 8U) |
                   (((uint32)3U - 1U) << 6U) |
                   (uint32)4U;




    /** - CAN4 Port output values */
    canREG4->TIOC =  (uint32)((uint32)1U  << 18U )
                   | (uint32)((uint32)0U  << 17U )
                   | (uint32)((uint32)0U  << 16U )
                   | (uint32)((uint32)1U  << 3U )
                   | (uint32)((uint32)0U  << 2U )
                   | (uint32)((uint32)0U << 1U );

    canREG4->RIOC =  (uint32)((uint32)1U  << 18U )
                   | (uint32)((uint32)0U  << 17U )
                   | (uint32)((uint32)0U  << 16U )
                   | (uint32)((uint32)1U  << 3U )
                   | (uint32)((uint32)0U  << 2U )
                   | (uint32)((uint32)0U << 1U );
    /** - Leave configuration and initialization mode  */
    canREG4->CTL &= ~(uint32)(0x00000041U);


    /**   @note This function has to be called before the driver can be used.\n
    *           This function has to be executed in privileged mode.\n
    */

/* USER CODE BEGIN (5) */
/* USER CODE END */
}

void canMessageNotification(canBASE_t *node, uint32 messageBox)
{
    // Return value of canRxData
    uint32 rxSuccess = 0U;
    long task_woken = pdTRUE;
    uint8 dlc;

    if(node==canREG1)
    {
        rxSuccess = canRxData(canREG1, canMESSAGE_BOX2, (uint32 * ) &rx_header, (uint8 * )&rx_data1[0], &dlc); /* copy to RAM */
        if (rxSuccess == 1U)
        {
            cnt++;
            csp_can_rx(&ctx->iface, rx_header, rx_data1, dlc, &task_woken);
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
