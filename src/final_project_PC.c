/*
* Author: 
* Day: 30/11/2023
* This code only simple module tranfer data between MCU and PC bu UART protocol
*/

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdint.h>
#include <windows.h>

#define BUFFER_SIZE 256


#define RET_FAIL -1
#define RET_NONE 0
#define RET_SUCCESS 1

#define OFFSET_HEADER_1 0
#define OFFSET_HEADER_2 1
#define OFFSET_CMD 2
#define OFFSET_DATALENGTH 3
#define OFFSET_DATA 4
#define OFFSET_TRAILER_1 5
#define OFFSET_TRAILER_2 6

#define HEADER_1 0xAB
#define HEADER_2 0xCD
#define TRAILER_1 0xE1
#define TRAILER_2 0xE2

//Define for Light
#define LIGHT_CONTROL 00
#define LIGHT_A_ID 00
#define LIGHT_B_ID 01
#define LIGHT_ON 00
#define LIGHT_OFF 01
//Define for Get door status
#define READ_STATUS_LIGHT   01
#define READ_STATUS_LIGHT_ON  00
#define READ_STATUS_LIGHT_OFF  01

//Define for Led
#define LED_ID 02
//Define for Accelerometer
#define ACCELEROMETER 03
//Define for flash
#define FLASH_ID 04
//Define for check uuid
#define CHECK_UUID 05


//Define for program
#define WAITING 01
#define ON_PROGRAM 02
#define END_PROGRAM 03


int flush_buffer(HANDLE port);
int input_data(unsigned char* writebuf);
HANDLE open_port(const char* device, unsigned long baud_rate, unsigned char bit_size, unsigned char parity);
int compose_packet(unsigned char rw, unsigned char address, unsigned int data_length, unsigned char* data, unsigned char* packet);
int uart_transmit(HANDLE port, unsigned char* writebuf, unsigned int inputLength);
int uart_receive(HANDLE port, unsigned char* rx_buf, unsigned char* data_length, unsigned char* cmd);

int input_data(unsigned char* writebuf) {
    unsigned int inputLength = 0;

    printf("Enter the input length: ");
    scanf("%d", &inputLength);


    printf("Enter input data (HEX):\n");
    for (int i = 0; i < inputLength; i++) {
        printf("Byte %d: ", i);
        scanf("%x", &writebuf[i]);
    }
    printf("Entered input data: ");
    for (int i = 0; i < inputLength; i++)
        printf("%x", writebuf[i] & 0xFF);
    return inputLength;
}

int flush_buffer(HANDLE port) {
    BOOL success = FlushFileBuffers(port);
    if (!success)
    {
        printf("Failed to flush serial port");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    return RET_SUCCESS;
}

HANDLE open_port(const char* device, unsigned long baud_rate, unsigned char bit_size, unsigned char parity) {
    HANDLE port
        = CreateFileA(device, GENERIC_READ | GENERIC_WRITE, 0, NULL,
            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (






        port == INVALID_HANDLE_VALUE)
    {
        return INVALID_HANDLE_VALUE;
    }

    // Flush away any bytes previously read or written.
    BOOL success = FlushFileBuffers(port);
    if (!success)
    {
        printf("Failed to flush serial port");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }

    // Configure read and write operations to time out after 100 ms.
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = 5000;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 5000;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    success = SetCommTimeouts(port, &timeouts);
    if (!success)
    {
        printf("Failed to set serial timeouts");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }

    // Set the baud rate and other options.
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = baud_rate;
    dcb.ByteSize = bit_size;
    dcb.Parity = parity;
    dcb.StopBits = ONESTOPBIT;
    success = SetCommState(port, &dcb);
    if (!success)
    {
        printf("Failed to set serial settings");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }

    // display information
    printf("----------------------------------\n");
    printf("baud rate = %d\n", dcb.BaudRate);
    printf("Parity = %d\n", dcb.Parity);
    printf("Byte Size = %d\n", dcb.ByteSize);
    printf("Stop Bit = %d\n", dcb.StopBits);
    printf("----------------------------------\n");
    return port;
}

int uart_transmit(HANDLE port, unsigned char* writebuf, unsigned int inputLength) {
    int writtenbytes = 0; // Number of bytes successfully written by WriteFile().    
    if (WriteFile(port, writebuf, inputLength, (LPDWORD)&writtenbytes, NULL)) {
        if (writtenbytes == 0) {
            printf("\nWriteFile() timed out\n");
            return RET_FAIL;
        }
    }
    else {
        printf("\nWriteFile() failed\n");
        return RET_FAIL;
    }
    printf("\nTransmited Packet: ");
    for (int i = 0; i < inputLength; i++) {
        printf("\t%x", writebuf[i]);
    }
    return RET_SUCCESS;
}

int uart_receive(HANDLE port, unsigned char* rx_buf, unsigned char* data_length, unsigned char* cmd) {
    int i = 0;
    int state = OFFSET_HEADER_1;
    unsigned char readbuf;
    DWORD nbbytes;

    do {
        if (ReadFile(port, &readbuf, 1, &nbbytes, NULL)) {
            if (nbbytes == 0) {
                printf("\nReadFile() timed out\n");
                return RET_FAIL;
            }
        }
        else {
            printf("\nReadFile() failed\n");
            return RET_FAIL;
        }
        switch (state) {
        case OFFSET_HEADER_1:
            if (readbuf == HEADER_1)
                state = OFFSET_HEADER_2;
            else {
                printf("\n[ERROR]: Wrong header 1");
                return RET_FAIL;
            }
            break;
        case OFFSET_HEADER_2:
            if (readbuf == HEADER_2)
                state = OFFSET_CMD;
            else {
                printf("\n[ERROR]: Wrong header 2");
                return RET_FAIL;
            }
            break;
        case OFFSET_CMD:
            *cmd = readbuf;
            state = OFFSET_DATALENGTH;
            break;
        case OFFSET_DATALENGTH:
            *data_length = readbuf;
            state = OFFSET_DATA;
            break;
        case OFFSET_DATA:
            if (i < *data_length - 1)
                rx_buf[i++] = readbuf;
            else {
                rx_buf[i++] = readbuf;
                state = OFFSET_TRAILER_1;
            }
            break;
        case OFFSET_TRAILER_1:
            if (readbuf == TRAILER_1)
                state = OFFSET_TRAILER_2;
            else {
                printf("\n[ERROR]: Wrong trailer 1");
                return RET_FAIL;
            }
            break;
        case OFFSET_TRAILER_2:
            if (readbuf == TRAILER_2)
                return RET_SUCCESS;
            else {
                printf("\n[ERROR]: Wrong trailer 2");
                return RET_FAIL;
            }
            break;
        default:
            break;
        }
    } while (1);
    return RET_FAIL;
}

int compose_packet(unsigned int CMD, unsigned int data_length, unsigned char* data, unsigned char* packet) {
    unsigned char frame_length = 6;
    int i = 4;

    packet[0] = HEADER_1;
    packet[1] = HEADER_2;
    packet[2] = CMD;
    packet[3] = data_length;
    for (i; i < data_length + 4; ++i)
        packet[i] = data[i - 4];
    packet[i] = TRAILER_1;
    packet[i + 1] = TRAILER_2;

    frame_length += data_length;

    printf("\Packet after compose: ");
    for (int a = 0; a < (i + 2); a++) {
        printf("\t%x", packet[a]);

    }

    return frame_length;
}


int main()
{
    // configuration parameters
    const char* device = "\\\\.\\COM20";
    unsigned long baud_rate = 9600;
    unsigned char bit_size = 8;
    unsigned char parity = 0;
    // code
    int loop = 0;
    unsigned char TX_buf[BUFFER_SIZE], RX_buf[BUFFER_SIZE], TX_packet[BUFFER_SIZE], RX_packet[BUFFER_SIZE];
    int status = RET_FAIL;
    char sel = 0, c;
    unsigned int TX_buf_len = 0, RX_buf_len = 0;
    unsigned int CMD = 0;
    unsigned int TX_packet_len = 0;
    unsigned char cmd;
    unsigned int address;
    int status_run = WAITING;
    int program = 0;
    // open serial port
    HANDLE port = open_port(device, baud_rate, bit_size, parity);
    if (port == INVALID_HANDLE_VALUE) {
        return -1;
    }

    do {
        switch (status_run)
        {
        case WAITING:
            //Call to MCU for request the UUID
            //We will check UUID at MCU then return true or false to PC 
            CMD = CHECK_UUID;
            //Init data
            TX_buf_len = 1;
            //TX_buf[0] = 0x00;
            //Compose packet
            TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
            //Start transmit
            status = uart_transmit(port, TX_packet, TX_packet_len);
            printf("\nCompleted Request UUID data");
            //After send request UUID, Take feedback from MCU
            if (status == RET_FAIL)
                loop = 1;
            //wait response
            status = uart_receive(port, RX_buf, &RX_buf_len, &cmd);
            if (status == RET_SUCCESS) {
                printf("\nReceiving data successed! \nReceived data after request read status :\n");
                printf("\n Data receive: ");
                for (int i = 0; i < RX_buf_len; i++) {
                    printf("\t%x", RX_buf[i]);
                }
                if (RX_buf[0] == 1)
                {
                    printf("\n True UUID \n");
                    status_run = ON_PROGRAM;
                }
                else
                {
                    printf("\n Wrong UUID ");
                    printf("\n Please take u card again ");
                }
            }
            break;

        case ON_PROGRAM:
            do {
                printf("\n==================================\n");
                printf("\nChoose Option u need");
                printf("\nPress '1': Control light");
                printf("\nPress '2': Read light status");
                printf("\nPress '3': Control LED");
                printf("\nPress '4': Read Accelerometer");
                printf("\nPress '5': Read Temperature");
                printf("\nPress '0': exit");
                printf("\nOption:  ");
                while ((sel = getchar()) == '\n');
                switch (sel) {
                    //Case 1 for control light
                case '1':
                    //Choice light
                    printf("\nEnter light id u wana control ");
                    printf("\nPress '1': Control light A ");
                    printf("\nPress '2': Control light B ");
                    printf("\nLight ID: ");
                    int light_id;
                    scanf("%d", &light_id);
                    //Choice Status
                    printf("\nEnter status light ");
                    printf("\nPress '1': Turn on light");
                    printf("\nPress '2': Turn off light");
                    printf("\nStatus: ");
                    int light_status;
                    scanf("%d", &light_status);
                    //Init data
                    CMD = LIGHT_CONTROL;

                    if (light_id == 1)
                        TX_buf[0] = LIGHT_A_ID;
                    if (light_id == 2)
                        TX_buf[0] = LIGHT_B_ID;

                    if (light_status == 1)
                        TX_buf[1] = LIGHT_ON;
                    if (light_status == 2)
                        TX_buf[1] = LIGHT_OFF;
                    //define length for this option
                    TX_buf_len = 2;
                    //Compose packet
                    TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
                    //Start transmit
                    status = uart_transmit(port, TX_packet, TX_packet_len);
                    printf("\nCompleted control light");
                    break;
                    //Case 2 for Read light door
                case '2':
                    //Choice light wana read status
                    printf("\nEnter light id u wana read status ");
                    printf("\nPress '1': Read status light A ");
                    printf("\nPress '2': Read status light B ");
                    printf("\Light ID: ");
                    int light_id_status;
                    scanf("%d", &light_id_status);

                    CMD = READ_STATUS_LIGHT;

                    if (light_id_status == 1)
                        TX_buf[0] = LIGHT_A_ID;
                    if (light_id_status == 2)
                        TX_buf[0] = LIGHT_B_ID;

                    //define length for this option
                    TX_buf_len = 1;
                    //Compose packet
                    TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
                    status = uart_transmit(port, TX_packet, TX_packet_len);

                    //After send request read status, Take feedback from MCU
                    if (status == RET_FAIL)
                        loop = 1;
                    //wait response
                    status = uart_receive(port, RX_buf, &RX_buf_len, &cmd);
                    if (status == RET_SUCCESS) {
                        printf("\nReceiving data successed! \nReceived data after request read status :\n");
                        printf("\n Data receive: /t  ");
                        for (int i = 0; i < RX_buf_len; i++) {
                            printf("\t%x", RX_buf[i]);
                        }
                        if (RX_buf[0] == LIGHT_A_ID)
                        {
                            if (RX_buf[1] == READ_STATUS_LIGHT_ON)
                                printf("\nLight A: Turn on");
                            else
                                printf("\nLight A: Turn off");
                        }
                        if (RX_buf[0] == LIGHT_B_ID)
                        {
                            if (RX_buf[1] == READ_STATUS_LIGHT_ON)
                                printf("\nLight B: Turn on");
                            else
                                printf("\nLight B: Turn off");
                        }
                    }
                    break;
                    //Case 3 for control led
                case '3':
                    //Enter time which setup for led hour_minute_second
                    //Struct of data : 
                    // TX_buf[0] - TX_buf[1] - TX_buf[2]
                    //   Hour   -   Minute  -  Second
                    printf("\nEnter time for led ");
                    int led_hour, led_min, led_sec;
                    printf("\nEnter time: Hour = ");
                    scanf("%d", &led_hour);
                    TX_buf[0] = led_hour;
                    printf("\nEnter time: Minute = ");
                    scanf("%d", &led_min);
                    TX_buf[1] = led_min;
                    //Init data
                    CMD = LED_ID;

                    TX_buf_len = 2;
                    printf("\nWarning: Now, Data is transmit with HEX data");
                    printf("\nThe receiver will be change HEX data to DEC data\n");
                    //Compose packet
                    TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
                    //Start transmit
                    status = uart_transmit(port, TX_packet, TX_packet_len);
                    printf("\nCompleted setup led");
                    break;

                case '4':
                    CMD = ACCELEROMETER;
                    //define length for this option
                    TX_buf_len = 1;
                    //Compose packet
                    TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
                    status = uart_transmit(port, TX_packet, TX_packet_len);
                    printf("\nCompleted request read Accelerometer");
                    //After send request read Accelerometer, Take feedback from MCU
                    if (status == RET_FAIL)
                        loop = 1;
                    //wait response
                    status = uart_receive(port, RX_buf, &RX_buf_len, &cmd);
                    if (status == RET_SUCCESS) {
                        printf("\nReceiving data successed! \nReceived data after request read status :\n");
                        printf("\n Data receive: ");
                        for (int i = 0; i < RX_buf_len; i++) {
                            printf("\t%x", RX_buf[i]);
                        }
                    }
                    break;
                    //Case 5 request to MCU that it need read temperature
                case '5':
                    //This case only request to MCU that it need read temperature

                    //Init data
                    CMD = FLASH_ID;
                    TX_buf_len = 1;
                    //TX_buf[0] = 0x00;
                    //Compose packet
                    TX_packet_len = compose_packet(CMD, TX_buf_len, TX_buf, TX_packet);
                    //Start transmit
                    status = uart_transmit(port, TX_packet, TX_packet_len);
                    printf("\nCompleted Request temperature data");
                    //After send request read Accelerometer, Take feedback from MCU
                    if (status == RET_FAIL)
                        loop = 1;
                    //wait response
                    status = uart_receive(port, RX_buf, &RX_buf_len, &cmd);
                    if (status == RET_SUCCESS) {
                        printf("\nReceiving data successed! \nReceived data after request read status :\n");
                        printf("\n Data receive: ");
                        for (int i = 0; i < RX_buf_len; i++) {
                            printf("\t%x", RX_buf[i]);
                        }
                    }

                    break;
                case '0':
                    loop = 1;
                    status_run = WAITING;
                    break;
                }
            } while (loop != 1);
            break;
            //end case on_program
        case END_PROGRAM:
            program = 1;
            break;

        default:
            printf("\n Wrong status_run ");
            break;
        }
    } while (program != 1);
    // Close the serial port.
    if (!CloseHandle(port))
    {
        printf("CloseHandle() failed\n");
        return RET_FAIL;
    }
    return RET_NONE;
}
