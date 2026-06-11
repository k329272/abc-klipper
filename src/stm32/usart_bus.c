// Auxiliary USART support on STM32 (for serially connected peripherals)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_STM32_SERIAL_USART1
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_save
#include "board/usart_bus.h" // usart_bus_setup
#include "command.h" // DECL_ENUMERATION, shutdown
#include "internal.h" // enable_pclock
#include "sched.h" // sched_wake_task

DECL_ENUMERATION("uart_bus", "usart1_PA10_PA9", 0);
DECL_CONSTANT_STR("BUS_PINS_usart1_PA10_PA9", "PA10,PA9");
DECL_ENUMERATION("uart_bus", "usart2_PA3_PA2", 1);
DECL_CONSTANT_STR("BUS_PINS_usart2_PA3_PA2", "PA3,PA2");
// Convenience aliases
DECL_ENUMERATION("uart_bus", "usart1", 0);
DECL_CONSTANT_STR("BUS_PINS_usart1", "PA10,PA9");
DECL_ENUMERATION("uart_bus", "usart2", 1);
DECL_CONSTANT_STR("BUS_PINS_usart2", "PA3,PA2");
#ifdef USART3
DECL_ENUMERATION("uart_bus", "usart3_PB11_PB10", 2);
DECL_CONSTANT_STR("BUS_PINS_usart3_PB11_PB10", "PB11,PB10");
DECL_ENUMERATION("uart_bus", "usart3", 2);
DECL_CONSTANT_STR("BUS_PINS_usart3", "PB11,PB10");
#endif

struct usart_bus_info {
    USART_TypeDef *usart;
    uint8_t rx_pin, tx_pin;
    IRQn_Type irqn;
};

static const struct usart_bus_info usart_bus[] = {
    { USART1, GPIO('A', 10), GPIO('A', 9), USART1_IRQn },
    { USART2, GPIO('A', 3), GPIO('A', 2), USART2_IRQn },
#ifdef USART3
    { USART3, GPIO('B', 11), GPIO('B', 10), USART3_IRQn },
#endif
};

// The usart (if any) reserved for the main Klipper console
#if CONFIG_STM32_SERIAL_USART1 || CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6
  #define CONSOLE_USART USART1
#elif CONFIG_STM32_SERIAL_USART2 || CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5
  #define CONSOLE_USART USART2
#elif CONFIG_STM32_SERIAL_USART3 || CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8
  #define CONSOLE_USART USART3
#else
  #define CONSOLE_USART 0
#endif

#define TX_BUFFER_SIZE 2048 // Must be power of 2
#define RX_BUFFER_SIZE 128 // Must be power of 2

struct usart_bus_state {
    USART_TypeDef *usart;
    struct task_wake *rx_wake;
    uint32_t tx_push, tx_pop, rx_push, rx_pop;
    uint8_t tx_buf[TX_BUFFER_SIZE], rx_buf[RX_BUFFER_SIZE];
};

// Only one auxiliary usart may be active
static struct usart_bus_state bus_state;

#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

static void
usart_bus_irq_handler(void)
{
    struct usart_bus_state *s = &bus_state;
    USART_TypeDef *usart = s->usart;
    uint32_t sr = usart->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        uint8_t data = usart->DR;
        if (s->rx_push - s->rx_pop < RX_BUFFER_SIZE) {
            s->rx_buf[s->rx_push % RX_BUFFER_SIZE] = data;
            s->rx_push++;
        }
        sched_wake_task(s->rx_wake);
    }
    if (sr & USART_SR_TXE && usart->CR1 & USART_CR1_TXEIE) {
        if (s->tx_pop == s->tx_push)
            usart->CR1 = CR1_FLAGS;
        else
            usart->DR = s->tx_buf[s->tx_pop++ % TX_BUFFER_SIZE];
    }
}

struct usart_bus_config
usart_bus_setup(uint32_t bus, uint32_t baud, struct task_wake *rx_wake)
{
    if (bus >= ARRAY_SIZE(usart_bus))
        shutdown("Invalid uart bus");
    const struct usart_bus_info *info = &usart_bus[bus];
    USART_TypeDef *usart = info->usart;
    if (usart == CONSOLE_USART)
        shutdown("uart bus is in use as the Klipper console");
    struct usart_bus_state *s = &bus_state;
    if (s->usart) {
        if (s->usart != usart)
            shutdown("Auxiliary usart already in use");
        return (struct usart_bus_config){ .state = s };
    }
    s->usart = usart;
    s->rx_wake = rx_wake;

    enable_pclock((uint32_t)usart);
    uint32_t pclk = get_pclock_frequency((uint32_t)usart);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
    usart->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                  | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    usart->CR1 = CR1_FLAGS;
    armcm_enable_irq(usart_bus_irq_handler, info->irqn, 0);

    gpio_peripheral(info->rx_pin, GPIO_FUNCTION(7), 1);
    gpio_peripheral(info->tx_pin, GPIO_FUNCTION(7), 0);

    return (struct usart_bus_config){ .state = s };
}

int
usart_bus_write(struct usart_bus_config config
                , const uint8_t *data, uint32_t len)
{
    struct usart_bus_state *s = config.state;
    irqstatus_t flag = irq_save();
    uint32_t used = s->tx_push - s->tx_pop;
    if (len > TX_BUFFER_SIZE - used) {
        // Not enough space - drop the entire write
        irq_restore(flag);
        return -1;
    }
    while (len--) {
        s->tx_buf[s->tx_push % TX_BUFFER_SIZE] = *data++;
        s->tx_push++;
    }
    s->usart->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
    irq_restore(flag);
    return 0;
}

uint32_t
usart_bus_read(struct usart_bus_config config
               , uint8_t *data, uint32_t maxlen)
{
    struct usart_bus_state *s = config.state;
    uint32_t count = 0;
    while (count < maxlen) {
        irqstatus_t flag = irq_save();
        if (s->rx_pop == s->rx_push) {
            irq_restore(flag);
            break;
        }
        data[count++] = s->rx_buf[s->rx_pop % RX_BUFFER_SIZE];
        s->rx_pop++;
        irq_restore(flag);
    }
    return count;
}
