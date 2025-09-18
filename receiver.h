
#include <chrono>
#include <bitset>
#include <iostream>
#include <ctime>
#include <pigpio.h>
using namespace std;

//define registers and configs
#define READ_MASK 0x7F // & 01111111
#define WRITE_MASK 0x80 // | 10000000

#define SET_SLEEP_MODE 0x00
#define SET_LORA_MODE 0x80
#define SET_MODE_IDLE 0x01
#define SET_MODE_FSTX 0x02
#define SET_MODE_TX 0x03
#define SET_MODE_FSRX 0x04
#define SET_MODE_RXCONT 0x05
#define SET_MODE_RXSINGLE 0x06
#define SET_MODE_CAD 0x07
//registers
#define CONFIG_REG 0x01
#define VERSION_REG 0x42
#define FIFO_TX_BASE_REG 0x0E
#define FIFO_RX_BASE_REG 0x0F
#define FRF_MSB_REG 0x06
#define FRF_MID_REG 0x07
#define FRF_LSB_REG 0x08
#define PREAMBLE_MSB_REG 0x20
#define PREAMBLE_LSB_REG 0x21
#define MODEM_CONFIG1_REG 0x1D
#define MODEM_CONFIG2_REG 0x1E
#define MODEM_CONFIG3_REG 0x26
#define PADAC_REG 0x4D
#define PA_CONFIG_REG 0x09
#define IRQ_FLAG_REG 0x12
#define IRQ_FLAG_MASK_REG 0x11
#define DIO_MAPPINGS_REG 0x40
#define RX_BYTES_REG 0x13
#define FIFO_RX_START_ADDR 0x10
#define FIFO_ADDR_PTR 0x0D
#define FIFO 0x00
#define PAYLOADLEN_REG 0x22

//define config
#define RF_FREQUENCY 433.30
#define RF_CS_PIN 8 //gpio 8, aka spi0 ce0
#define RF_IRQ_PIN 4 //gpio 4- NOT USED
#define RF_RST_PIN 25 //gpio 25
#define MAX_BUFF_LEN 251 //max bytes

int spi_write(int handle, char* buf, int num);
int spi_read(int handle, char* tx, char* rx, int txnum, int rxnum);
bool checkRxFlag(int handle);
bool checkTxFlag(int handle);
bool checkCRCErrorFlag(int handle);
string recv(int handle);
void send(int handle);
bool setupReceiver(int handle);

