/*
@file    sci_hercules.c
@brief   SCI driver for Hercules TMS570LC43x in libcsp.
@version 20241210 v1.0.0   PLL Initial release.
*/

// Includes
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/csp_iflist.h>
#include <csp/csp_types.h>
#include <csp/drivers/sci_hercules.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "os_semphr.h"

// SCI related
#include "HL_sci.h"
#include "HL_reg_het.h"
#include "HL_sys_core.h"

// libcsp
#include <csp/csp_rtable.h>

// HALCoGen
#include "HL_sys_main.h"


// Array to store contexts for each SCI
static sci_hercules_context_t *sci_contexts[4] = {NULL, NULL, NULL, NULL};  // One for each SCIx

// SCIFLR flags used for polled transmit.
#define SCI_HERCULES_FLR_TXRDY    ((uint32)SCI_TX_INT)             // Bit 8: TD buffer ready for next byte.
#define SCI_HERCULES_FLR_TXEMPTY  ((uint32)((uint32)1U << 11U))    // Bit 11: transmit shift register drained.

// Driver Enable (DE) GPIO of the SCI1 RS-485 transceiver. Board-specific (CUB): mibspiPORT1[9].
#define SCI_HERCULES_DE_PORT      mibspiPORT1
#define SCI_HERCULES_DE_BIT       9U


// Per-byte writer used as the KISS tx_func. csp_kiss_tx() calls this once per byte / escape
// sequence, so it must be a blocking, polled write: an interrupt-driven sciSend() must not be
// re-armed until its previous transfer completes, and successive calls would clobber the in-flight
// transfer. The DE line is driven at frame scope by sci_hercules_kiss_tx(), so it is not touched here.
int sci_hercules_tx(void *driver_data, const unsigned char * data, size_t data_length)
{
    sci_hercules_context_t *ctx = driver_data;
    sciBASE_t *sci = ctx->sci_base;
    size_t i;

    for (i = 0U; i < data_length; i++)
    {
        while ((sci->FLR & SCI_HERCULES_FLR_TXRDY) == 0U) { /* wait for TD buffer ready */ }
        sci->TD = (uint32)data[i];
    }

    return CSP_ERR_NONE;
}


// Frame-level nexthop wrapper. Asserts the RS-485 driver enable for the whole KISS frame, lets
// csp_kiss_tx() stream the framed bytes through sci_hercules_tx(), then waits for the shift
// register to drain (TX EMPTY) before releasing the driver, so the last byte is not truncated and
// the half-duplex bus is returned to receive.
static int sci_hercules_kiss_tx(const csp_route_t * ifroute, csp_packet_t * packet)
{
    sci_hercules_context_t *ctx = ifroute->iface->driver_data;
    int res;

    gioSetBit(SCI_HERCULES_DE_PORT, SCI_HERCULES_DE_BIT, 1);   // DE on (transmit).
    res = csp_kiss_tx(ifroute, packet);
    while ((ctx->sci_base->FLR & SCI_HERCULES_FLR_TXEMPTY) == 0U) { /* wait TX EMPTY */ }
    gioSetBit(SCI_HERCULES_DE_PORT, SCI_HERCULES_DE_BIT, 0);   // DE off (receive).

    return res;
}


void sci_hercules_rx_callback(sci_hercules_context_t *ctx, uint8_t data, void *pxTaskWoken)
{
    // Pass the received byte to the KISS processor
    csp_kiss_rx(&ctx->iface, &data, 1, pxTaskWoken);
}

sci_hercules_context_t* get_sci_context(sciBASE_t *sci) 
{
    if(sci == sciREG1) return sci_contexts[0];
    else if(sci == sciREG2) return sci_contexts[1];
    else if(sci == sciREG3) return sci_contexts[2];
    else if(sci == sciREG4) return sci_contexts[3];
    return NULL;
}

int csp_sci_hercules_init(const char *name, sciBASE_t *sci_base, csp_iface_t **return_iface)
{
    sci_hercules_context_t *ctx = csp_calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        return CSP_ERR_NOMEM;
    }

    // Configure context
    strncpy(ctx->name, name, sizeof(ctx->name) - 1);
    ctx->sci_base = sci_base;

    // Set up CSP interface
    ctx->iface.name = ctx->name;
    ctx->iface.driver_data = ctx;
    ctx->iface.interface_data = &ctx->ifdata;
    ctx->ifdata.tx_func = sci_hercules_tx;

    // Register the context so the RX ISR can retrieve it via get_sci_context().
    if      (sci_base == sciREG1) sci_contexts[0] = ctx;
    else if (sci_base == sciREG2) sci_contexts[1] = ctx;
    else if (sci_base == sciREG3) sci_contexts[2] = ctx;
    else if (sci_base == sciREG4) sci_contexts[3] = ctx;
    else
    {
        csp_free(ctx);
        return CSP_ERR_INVAL;
    }

    // Add the KISS interface
    int res = csp_kiss_add_interface(&ctx->iface);
    if (res != CSP_ERR_NONE)
    {
        sci_contexts[(sci_base == sciREG1) ? 0 : (sci_base == sciREG2) ? 1 : (sci_base == sciREG3) ? 2 : 3] = NULL;
        csp_free(ctx);
        return res;
    }

    // csp_kiss_add_interface() sets nexthop = csp_kiss_tx. Override it with the frame-level wrapper
    // that drives the RS-485 DE line around the whole KISS frame.
    ctx->iface.nexthop = sci_hercules_kiss_tx;

    if (return_iface)
    {
        *return_iface = &ctx->iface;
    }

    return CSP_ERR_NONE;
}
