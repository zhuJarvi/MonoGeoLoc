#pragma once
#include "vector"
#include <unistd.h>
#include <string>

uint8_t addData(std::vector<uint8_t> &buf){
    uint8_t checksum = 0;
    for (const auto &byte : buf)
    {
        checksum += byte;
    }
    return checksum;
}

const std::vector<uint8_t> CMD_START = {0x59, 0x74};
const std::vector<uint8_t> CMD_END = {0xED, 0xED};

const std::vector<uint8_t> CMD_INIT = {0x01, 0x01};
const std::vector<uint8_t> CMD_INIT = {0x03, 0x03};