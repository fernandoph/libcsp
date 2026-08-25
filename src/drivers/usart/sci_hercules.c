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

// SCIFLR flag used for the polled transmit.
#define SCI_HERCULES_FLR_TXRDY    ((uint32)SCI_TX_INT)             // Bit 8: TD buffer ready for next byte.


// Per-byte writer used as the KISS tx_func. csp_kiss_tx() calls this once per byte / escape
// sequence, so it must be a blocking, polled write: an interrupt-driven sciSend() must not be
// re-armed until its previous transfer completes, and successive calls would clobber the in-flight
// transfer. RS-422 full-duplex: the transceiver driver is permanently enabled in hardware, so there
// is no DE line to manage here.
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

    if (return_iface)
    {
        *return_iface = &ctx->iface;
    }

    return CSP_ERR_NONE;
}
