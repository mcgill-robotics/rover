#pragma once
#include <Arduino.h>

namespace SerialAPI
{
    void init(char sys_id, unsigned long baudrate);

    // Updates the serial interface. Returns true if a new packet is available to be read.
    bool update(void);

    // Read a packet from the serial stream. Returns the incoming packet id or -1 if the operation failed.
    uint8_t read_data(void* buff, uint32_t max_length);

    // Send a stream of bytes through the serial interface
    bool send_bytes(uint8_t packet_id, const void* data, size_t length);


    //CHANGED FROM STATIC
    void send_retransmit(void);

}