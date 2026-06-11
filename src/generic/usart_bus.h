#ifndef __GENERIC_USART_BUS_H
#define __GENERIC_USART_BUS_H

#include <stdint.h> // uint8_t

struct task_wake;

struct usart_bus_config {
    void *state;
};

struct usart_bus_config usart_bus_setup(uint32_t bus, uint32_t baud
                                        , struct task_wake *rx_wake);
// Queue len bytes for transmit.  The write is all-or-nothing - returns
// zero on success, -1 if there is not enough buffer space (in which
// case no bytes are queued).
int usart_bus_write(struct usart_bus_config config
                    , const uint8_t *data, uint32_t len);
// Read up to maxlen received bytes.  Returns the number of bytes read.
uint32_t usart_bus_read(struct usart_bus_config config
                        , uint8_t *data, uint32_t maxlen);

#endif // usart_bus.h
