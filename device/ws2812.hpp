/**
 * @file ws2812.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-20
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef WS2812_H
#define WS2812_H

#include "stdint.h"

#define DM_CM02_H723_WS2812_SPI_PORT hspi6

template<
    typename SPI_HANDLE,
    typename HAL_StatusTypeDef,
    SPI_HANDLE* HSPI,
    HAL_StatusTypeDef (*HAL_SPI_Transmit)(SPI_HANDLE*, const uint8_t*, uint16_t, uint32_t)>
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b) {
    constexpr uint8_t WS2812_LowLevel = 0xC0;  // 0Ты
    constexpr uint8_t WS2812_HighLevel = 0xF0; // 1Ты

    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
        txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&HSPI, &res, 0, 0xFFFF);
    while (HSPI.State != HAL_SPI_STATE_READY)
        ;
    HAL_SPI_Transmit(&HSPI, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++) {
        HAL_SPI_Transmit(&HSPI, &res, 1, 0xFFFF);
    }
}

#endif
