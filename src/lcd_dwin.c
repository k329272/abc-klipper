// Commands for sending messages to a DWIN T5UIC1 lcd over an mcu usart
// (the stock display on the Creality Ender 3 V2 / V2 Neo)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "basecmd.h" // oid_alloc
#include "board/usart_bus.h" // usart_bus_setup
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK

// The T5UIC1 serial protocol frames every message with a one byte
// header and a four byte tail.  Frames are queued to the usart
// atomically - either an entire frame is queued for transmit or (if
// the transmit buffer is too full) the frame is dropped and the drop
// is reported to the host.  Since the host periodically redraws the
// screen contents, a dropped (or corrupted) frame heals on a
// subsequent update instead of leaving the display stream
// desynchronized.

#define DWIN_FRAME_HEAD 0xAA
static const uint8_t dwin_frame_tail[4] = { 0xCC, 0x33, 0xC3, 0x3C };

#define DWIN_MAX_DATA 58
#define DWIN_MAX_RX 48

struct dwin {
    struct usart_bus_config usart;
    uint32_t tx_drops, reported_tx_drops;
    uint8_t rx_buf[DWIN_MAX_RX + 5];
    uint8_t rx_pos;
    uint8_t flags;
};

enum { DF_ACTIVE = 1 };

static struct task_wake dwin_wake;

void
command_config_dwin(uint32_t *args)
{
    struct dwin *d = oid_alloc(args[0], command_config_dwin, sizeof(*d));
    d->usart = usart_bus_setup(args[1], args[2], &dwin_wake);
    d->flags = DF_ACTIVE;
}
DECL_COMMAND(command_config_dwin, "config_dwin oid=%c uart_bus=%u baud=%u");

void
command_dwin_send(uint32_t *args)
{
    struct dwin *d = oid_lookup(args[0], command_config_dwin);
    uint_fast8_t len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    if (len > DWIN_MAX_DATA)
        shutdown("dwin data too large");
    uint8_t frame[DWIN_MAX_DATA + 5];
    frame[0] = DWIN_FRAME_HEAD;
    memcpy(&frame[1], data, len);
    memcpy(&frame[1 + len], dwin_frame_tail, sizeof(dwin_frame_tail));
    int ret = usart_bus_write(d->usart, frame, len + 5);
    if (ret) {
        // Transmit buffer full - drop the frame and let the host know
        // so it can schedule a full redraw.
        d->tx_drops++;
        sched_wake_task(&dwin_wake);
    }
}
DECL_COMMAND(command_dwin_send, "dwin_send oid=%c data=%*s");

// Check if the receive buffer holds a complete frame from the display
static int
dwin_check_rx_frame(struct dwin *d)
{
    return (d->rx_pos >= 5
            && memcmp(&d->rx_buf[d->rx_pos - sizeof(dwin_frame_tail)]
                      , dwin_frame_tail, sizeof(dwin_frame_tail)) == 0);
}

void
dwin_task(void)
{
    if (!sched_check_wake(&dwin_wake))
        return;
    uint8_t oid;
    struct dwin *d;
    foreach_oid(oid, d, command_config_dwin) {
        if (!(d->flags & DF_ACTIVE))
            continue;
        // Report dropped transmit frames
        if (d->tx_drops != d->reported_tx_drops) {
            d->reported_tx_drops = d->tx_drops;
            sendf("dwin_tx_drops oid=%c count=%u", oid, d->tx_drops);
        }
        // Forward complete frames received from the display
        uint8_t data;
        while (usart_bus_read(d->usart, &data, 1)) {
            if (!d->rx_pos && data != DWIN_FRAME_HEAD)
                // Garbage between frames - discard
                continue;
            d->rx_buf[d->rx_pos++] = data;
            if (dwin_check_rx_frame(d)) {
                sendf("dwin_rx oid=%c data=%*s", oid, d->rx_pos - 5
                      , &d->rx_buf[1]);
                d->rx_pos = 0;
            } else if (d->rx_pos >= sizeof(d->rx_buf)) {
                // Oversized response - discard
                d->rx_pos = 0;
            }
        }
    }
}
DECL_TASK(dwin_task);
