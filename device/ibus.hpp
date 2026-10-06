/**
 * @file ibus.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-10-04
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * @note
 * 这是 ibus 协议的解析代码，根据以下的官网信息编写。
 * [富斯AFHDS3 FSI6 协议通道数据格式公布公告](https://www.flyskytech.com/info_detail/18.html)
 *
 * 数据格式如下：
 * 串口格式：Baud Rate 115200 (8, N, 1)
 * 帧长：32 bytes
*/

#include <cstdint>

namespace vlib {
namespace comm {

class Ibus {
#define IBUS_CH_LEN 18
#define IBUS_FRAME_LEN 32
#define IBUS_FRAME_HEAD_0 0x5A
#define IBUS_FRAME_HEAD_1 0x40

public:
    uint16_t channel[IBUS_CH_LEN];

    bool Parse(const uint8_t* buf, uint8_t len) {
        uint8_t i = 0, t = 0;

        // check
        uint16_t frame_checksum = (uint16_t)(buf[30] | (buf[31] << 8));
        if (len != IBUS_FRAME_LEN || buf[0] != IBUS_FRAME_HEAD_0 || buf[1] != IBUS_FRAME_HEAD_1
            || Checksum16(buf, IBUS_FRAME_LEN - 2) != frame_checksum)
        {
            return false;
        }

        // ch 1 -- 14
        for (; i < 14; i++) {
            t = 2 * i;
            channel[i] = (uint16_t)((buf[t + 2] | (buf[t + 3] << 8)) & 0x07FF);
        }

        // ch 15 -- 18
        for (; i < 18; i++) {
            t = (i - 14) * 6;
            channel[i] = (uint16_t)(((buf[t + 3] >> 4) | buf[t + 5] | (buf[t + 7] << 4)) & 0x07FF);
        }

        return true;
    }

    inline uint16_t Checksum16(const uint8_t* data, uint8_t len) {
        uint16_t checksum = 0;
        while (len--)
            checksum += data[len];
        return ~checksum;
    }
};

} // namespace comm
} // namespace vlib
