#include "Rover_SerialAPI.h"
#include <Arduino.h>
#include <utils/crc.h>


/* ################### PACKET LAYOUT #######################
---------------------------------------------------------
| 1 byte |   1 byte   |   1 byte   | Variable |  1 byte |
---------------------------------------------------------
| Start  | Frame type |   Length   |  PAYLOAD  |   CRC  |
---------------------------------------------------------
########################################################## */

typedef enum ReadState
{
    RS_WAITING,
    RS_READING,
    RS_PACKET_READY,
} ReadState;
// Controls which serial interface the device should use.
static HardwareSerial& SerialInterface = Serial;

static uint8_t sys_id = 0;

#define MAX_PACKET_SIZE 255
// Incoming data buffer
static char data_buffer[MAX_PACKET_SIZE];
static size_t data_offset = 0;
static size_t payload_length = 0;


static ReadState read_state = RS_WAITING; //Changed this one or otherwise it never receives


#define START_OF_PACKET (char)'~'
#define DATA_SEGMENT_LENGTH(packet_len) (packet_len - 4)

#define FRAMEID_SEGMENT_OFFSET 1
#define FRAME_LENGTH_SEGMENT_OFFSET 2
#define DATA_SEGMENT_OFFSET 3

static bool crc_validate_packet(void* data, size_t len, uint8_t checksum)
{
    const uint8_t computed_checksum = crc8ccitt(data, len);
    return computed_checksum == checksum;
}

namespace SerialAPI
{
    void init(char _sys_id, unsigned long baudrate)
    {
        sys_id = _sys_id;
        SerialInterface.begin(baudrate);
        SerialInterface.setTimeout(10);

        //This function doesn't exist for arduino
        //SerialInterface.setBufferSize(MAX_PACKET_SIZE, MAX_PACKET_SIZE);


    }

    bool update(void)
    {
        int num_avail = SerialInterface.available();
        // Sanity check that the number of available bytes won't overflow our data buffer

        if ((size_t)num_avail + data_offset < MAX_PACKET_SIZE)
        {
            // #todo [roey]: somehow this should warn or indicate error of some kind.

            // For now we will just clamp the value down to prevent overflow
            num_avail = MAX_PACKET_SIZE - data_offset;
        }

        if (num_avail > 0)
        {
            // Read the available bytes into the data buffer
            const size_t bytes_read = SerialInterface.readBytes(data_buffer + data_offset, num_avail);
            data_offset += bytes_read;
            
            // If the old offset was 0 this means we're still waiting for a new packet to arrive so search the new data for a start byte
            if (read_state == RS_WAITING)
            {
                size_t start_offset = 0;
                bool found_start = false;
                for (size_t i = 0; i < data_offset; ++i)
                {
                    if (data_buffer[i] == START_OF_PACKET)
                    {
                        start_offset = i;
                        found_start = true;
                        break;
                    }
                }
                if (found_start)
                {
                    read_state = RS_READING;
                    // Shift the array over so that it starts at the correct start byte
                    memmove(data_buffer + start_offset, data_buffer, MAX_PACKET_SIZE - start_offset);
                    // the start offset basically tells us how many bytes extra were read so we need to move it back since we shift the whole data buffer over
                    data_offset -= start_offset; 
                }
            }

            if (read_state == RS_WAITING)
            {
                memset(data_buffer, 0, MAX_PACKET_SIZE);
                data_offset = 0;
                return false;
            }

            if (data_offset > FRAME_LENGTH_SEGMENT_OFFSET)
            {
                payload_length = data_buffer[FRAME_LENGTH_SEGMENT_OFFSET];
            }

            // Check if we read an end-of-packet character so we can signal that packet processing should happen:
            // Since we are using Stop-and-wait, we will never have more than one queue'd incoming message and
            // thus the last character we read will always be the EOP if the packet is finished.
            if (data_offset >= payload_length + 4)
            {
                read_state = RS_PACKET_READY;
            }
        }
        return read_state == RS_PACKET_READY;
    }
    
    void send_retransmit(void)
    {
        char output_buffer[3];
        output_buffer[0] = START_OF_PACKET;
        output_buffer[1] = 'R';

        // Must be null terminated for Serial.write to process it correctly.
        output_buffer[2] = 0;

        SerialInterface.write(output_buffer);
    }

    static void send_acknowledgement(void)
    {
        char output_buffer[3];
        output_buffer[0] = START_OF_PACKET;
        output_buffer[1] = 'A';

        // Must be null terminated for Serial.write to process it correctly.
        output_buffer[2] = 0;

        SerialInterface.write(output_buffer);
    }

    static void reset_buffer(void)
    {
        memset(data_buffer, 0, MAX_PACKET_SIZE);
        data_offset = 0;
        payload_length = 0;
        read_state = RS_WAITING;
    }

    uint8_t read_data(void* buff, uint32_t max_length)
    {
        if (read_state != RS_PACKET_READY)
        {
            return -1;
        }

        if (max_length < data_offset)
        {
            // #todo: this is kind of a silent error that should probably be addressed in a better way
            return -1;
        }
        
        
        // Couldn't manage the crc validate for now
        const size_t packet_length = data_offset;

/*
        //if (!crc_validate_packet(data_buffer + DATA_SEGMENT_OFFSET, DATA_SEGMENT_LENGTH(packet_length), data_buffer[packet_length - 2]))
        //Small correction:  data_buffer[packet_length - 1]
        if (!crc_validate_packet(data_buffer + DATA_SEGMENT_OFFSET, DATA_SEGMENT_LENGTH(packet_length), data_buffer[packet_length - 1]))
        {
            send_retransmit();
            reset_buffer();
            return -1;
        }
        */
        
        // By now the packet is valid so we should reply with an ack
        send_acknowledgement();

        // copy the output data to the out buffer
        memcpy(buff, data_buffer + DATA_SEGMENT_OFFSET, payload_length);
        const uint8_t packet_id = data_buffer[FRAMEID_SEGMENT_OFFSET];
        reset_buffer();
        return packet_id;
    }

    bool send_bytes(uint8_t packet_id, const void* buff, size_t len)
    {
        const size_t packet_size = len + 4;

        if (packet_size >= MAX_PACKET_SIZE)
        {
            return false;
        }

        uint8_t output_buffer[MAX_PACKET_SIZE];
        output_buffer[0] = START_OF_PACKET;
        output_buffer[1] = packet_id;
        output_buffer[2] = len;

        memcpy(output_buffer + DATA_SEGMENT_OFFSET, buff, len);
        output_buffer[packet_size - 1] = crc8ccitt(output_buffer + DATA_SEGMENT_OFFSET, len);         

        SerialInterface.write(output_buffer, packet_size);
        return true;
    }
}