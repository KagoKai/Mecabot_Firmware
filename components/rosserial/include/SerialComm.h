#ifndef ROSSERIAL_CLIENT_STM32_SERIAL_H_
#define ROSSERIAL_CLIENT_STM32_SERIAL_H_

#define STM32F1xx  // Change for your device

#ifdef STM32F1xx
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#endif /* STM32F1xx */
#ifdef STM32F3xx
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#endif /* STM32F3xx */
#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif /* STM32F4xx */
#ifdef STM32F7xx
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#endif /* STM32F7xx */

#include "stdint.h"
#include "cstring"

extern UART_HandleTypeDef huart_rosserial;

static constexpr uint16_t buffer_size = 512;

class SerialComm
{
private:
    UART_HandleTypeDef &huart;
    // Data buffers are implemented as Ring Buffers (FIFO)
    uint8_t tx_buf[buffer_size];
    uint8_t rx_buf[buffer_size];
    bool is_tx_cplt = true;
    uint16_t tx_head = 0;
    uint16_t tx_tail = 0;
    uint16_t rx_tail = 0;
    static constexpr uint16_t buf_mask = buffer_size - 1;

public:
    SerialComm(UART_HandleTypeDef &huart) : huart(huart)
    {
        // Other initialization code
    }

    void init(void)
    {
        reset_rbuf();
    }

    UART_HandleTypeDef* const get_handle()
    {
        return &huart;
    }

    /** @brief Read a character from the Rx buffer using the FIFO method.
     * 
     * @return The read character.
    */
    int read(void)
    {
       uint16_t rx_head = (buffer_size - huart.hdmarx->Instance->CNDTR) & buf_mask;

        // Check for overlapping (which is bad, data will be overwritten)
        if (rx_tail == rx_head)
        {
            return -1;
        }

        // Read a value and increment the read pointer (TAIL).
        int c = (int) rx_buf[rx_tail++];
        rx_tail &= buf_mask;

        return c;
    }

    /** @brief Write a string of data to the Tx buffer using the FIFO method.
     * 
     * @param data The pointer to the data string.
     * @param length The length of the data string.
     * 
     * @return Void. 
    */
    void write(const uint8_t* const data, const int length)
    {
        // Faulty data guard.
        if (length > buffer_size || length < 1)
        {
            return;
        }

        // Wait for the completion of the previous Tx transfer.
        while (!is_tx_cplt);

        // Check if the data is bigger than the remaining storage
        int w_len = (length <= (buffer_size - tx_head)) ? length : (buffer_size - tx_head);

        // Write the data and increment the write pointer (TX_HEAD)
        memcpy(&tx_buf[tx_head], data, w_len);
        tx_head = (tx_head + length) & buf_mask;

        // Reset the TX_HEAD pointer and write the remaining data
        // (If data is bigger than the previous remaining data)
        if (length != w_len)
        {
            memcpy(tx_buf, &data[w_len], length - w_len);
        }

        // TODO: Start transfering
        flush();
    }

    /** @brief Transmit the just written data in the Tx buffer
     * 
     * @return void.
    */
    void flush()
    {
        if (is_tx_cplt)
        {
            // if HEAD == TAIL => The buffer is empty => Nothing to send.
            if (tx_head != tx_tail)
            {
                uint16_t len = 0;

                if (tx_tail < tx_head)
                {
                    len = tx_head - tx_tail;
                    HAL_UART_Transmit_DMA(&huart, &tx_buf[tx_tail], len);
                }
                else
                {
                    len = buffer_size - tx_tail;
                    HAL_UART_Transmit_DMA(&huart, &tx_buf[tx_tail], len);
                    HAL_UART_Transmit_DMA(&huart, tx_buf, tx_head);
                }
                
                tx_tail = tx_head;
            }
            
            is_tx_cplt = false;
        }
    }

    void set_tx_cplt(void)
    {
        is_tx_cplt = true;
    }

    void reset_rbuf(void)
    {
        HAL_UART_Receive_DMA(&huart, (uint8_t *)rx_buf, buffer_size);
    }
};

SerialComm comm_rosserial(huart_rosserial);

#endif
