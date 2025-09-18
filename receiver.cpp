#include "receiver.h"

int spi_write(int handle, char* buf, int num){
    gpioWrite(RF_CS_PIN, 0);
    int sent = spiWrite(handle, buf, num);
    gpioWrite(RF_CS_PIN, 1);
//    cout << "sent " << sent << " bytes" << endl;
    return sent;
}

int spi_read(int handle, char* tx, char* rx, int txnum, int rxnum){
    gpioWrite(RF_CS_PIN, 0);
    int sent = spiWrite(handle, tx, txnum);
    int recv = spiRead(handle, rx, rxnum);
    gpioWrite(RF_CS_PIN, 1);
//    cout << "sent " << sent << " bytes" << endl;
//    cout << "rcvd " << recv << " bytes" << endl;
    return recv;
}
bool checkRxFlag(int handle){
    char rx = 0;
    char tx = IRQ_FLAG_REG & READ_MASK;
    spi_read(handle, &tx, &rx, 1, 1);
    if((rx & 0x40) == 0x40){
        return true;
    }
    return false;
}
bool checkTxFlag(int handle){
    char rx = 0;
    char tx = IRQ_FLAG_REG & READ_MASK;
    spi_read(handle, &tx, &rx, 1, 1);
    if((rx & 0x08) == 0x08){
        return true;
    }
    return false;
}
bool checkCRCErrorFlag(int handle){
    char rx = 0;
    char tx = IRQ_FLAG_REG & READ_MASK;
    spi_read(handle, &tx, &rx, 1, 1);
    if((rx & 0x20) == 0x20){ //had payload crc error :/
        return true;
    }
    return false;
}

string recv(int handle){
    //check if available, get the message / remove the header, clear the buffer
    char* buf = new char[2];
    char tx = 0;
    char rx = 0;
    double timeout = 0.5;
    bool timedOut = false;
    //set to rx mode:
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_RXCONT;
    spi_write(handle, buf, 2);
    //set di0 mode to 00 (should be default)
    buf[0] = DIO_MAPPINGS_REG | WRITE_MASK;
    buf[1] = 0x00;
    spi_write(handle, buf, 2);

    auto t = std::chrono::steady_clock::now();
    while(!timedOut && !checkRxFlag(handle)){
        auto e = std::chrono::steady_clock::now();
        auto diff = chrono::duration_cast<chrono::microseconds>(e - t);
        if(diff.count() >= timeout*1000000){ //bc in microseconds
            timedOut = true;
        }
    }
    if(timedOut){
        return "timeout";
    }

    //skip rssi reading
    //skip snr reading

    //enter idle mode
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_IDLE;
    spi_write(handle, buf, 2);

    bool gotPacket = false;
    string msg = "";

    if(!timedOut){
        //that means we received a package
        if(checkCRCErrorFlag(handle)){ //crc error
            cout << "crc error detected :/" << endl;
            //clear the flag
            buf[0] = IRQ_FLAG_REG | WRITE_MASK;
            buf[1] = 0xFF;
            spi_write(handle, buf, 2);
            //TODO: do i return false or true in this case?
        }
        else{
            //read data from fifo

            //read len of fifo
            rx = 0;
            tx = RX_BYTES_REG & READ_MASK;
            spi_read(handle, &tx, &rx, 1, 1);
            int fifoLen = rx;
            cout << "fifolen: " << rx << endl;
            char* fifoBuf = new char[fifoLen];
            if(fifoLen > 0){ //have something
                gotPacket = true;
                //get start addr of where packet is
                rx = 0;
                tx = FIFO_RX_START_ADDR & READ_MASK;
                spi_read(handle, &tx, &rx, 1, 1);
                char startAddr = rx;
                //write that addr to fifo pointer
                buf[0] = FIFO_ADDR_PTR | WRITE_MASK;
                buf[1] = startAddr;
                spi_write(handle, buf, 2);

                //now we start reading from fifo buffer
                tx = FIFO & READ_MASK;

                int readBytes = spi_read(handle, &tx, fifoBuf, 1, fifoLen);
                cout << "read " << readBytes << " from fifo buffer" << endl;
            }
            //clear interrupt
            buf[0] = IRQ_FLAG_REG | WRITE_MASK;
            buf[1] = 0xFF;
            spi_write(handle, buf, 2);
            if(fifoLen < 5){
                //missing header + payload, since header is 4 bytes
                gotPacket = false;
            }
            else{
                //check if broadcast
                if(fifoBuf[0] != 0xFF){
                    gotPacket = false;
                }
            }
            if(gotPacket){
//                cout << "fifoBuf: " << fifoBuf << endl;
//                msg = fifoBuf[4,fifoLen]; //remove header (4 bytes)
		for(int i=4; i<fifoLen; i++){
			msg += fifoBuf[i];
		}

                cout << "msg: " << msg << endl;
            }
            //clear interrupts
            buf[0] = IRQ_FLAG_REG | WRITE_MASK;
            buf[1] = 0xFF;
            spi_write(handle, buf, 2);
            delete[] fifoBuf;
        }
    }
    delete[] buf;
    return msg;
}


void send(int handle){
    char* buf = new char[2];
    //put in idle mode
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_IDLE;
    spi_write(handle, buf, 2);

    //fill fifo with packet to send
    buf[0] = FIFO | WRITE_MASK;
    buf[1] = 0x00;
    spi_write(handle, buf, 2);

    //hope this sends correctly to the
    char* sendbuf = new char[256];
    sendbuf[0] = FIFO | WRITE_MASK;
    sendbuf[1] = 0xFF;
    sendbuf[2] = 0xFF;
    sendbuf[3] = 0x00;
    sendbuf[4] = 0x00;
    sendbuf[5] = 'h';
    sendbuf[6] = 'a';
    sendbuf[7] = 'l';
    sendbuf[8] = 'l';
    sendbuf[9] = 'o';
    sendbuf[10] = '\0';
    int size = 11;
    spi_write(handle, sendbuf, size);


    //set size bc implicit header
    buf[0] = PAYLOADLEN_REG | WRITE_MASK;
    buf[1] = 0x0B;
    spi_write(handle, buf, 2);

    //turn on transmit mode
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_TX;
    spi_write(handle, buf, 2);
    buf[0] = DIO_MAPPINGS_REG | WRITE_MASK;
    buf[1] = 0x40;
    spi_write(handle, buf, 2);
    //now wait for transmitting to be done
    bool timedOut = false;
    double timeout = 0.5;
    auto t = std::chrono::steady_clock::now();
    while(!timedOut && !checkTxFlag(handle)){
        auto e = std::chrono::steady_clock::now();
        auto diff = chrono::duration_cast<chrono::microseconds>(e - t);
        if(diff.count() >= timeout*1000000){ //bc in microseconds
            timedOut = true;
        }
    }
    //return to idle
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_IDLE;
    spi_write(handle, buf, 2);
    //clear interrupts
    buf[0] = IRQ_FLAG_REG | WRITE_MASK;
    buf[1] = 0xFF;
    spi_write(handle, buf, 2);

    delete[] sendbuf;
    delete[] buf;
}

bool setupReceiver(int handle){
    //setup reset pin and do a reset
    gpioSetMode(RF_RST_PIN, PI_OUTPUT);
    gpioWrite(RF_RST_PIN, 0); //0 is low
    gpioDelay(150); //in microseconds
    gpioWrite(RF_RST_PIN, 1); //1 is high
    gpioDelay(150);

    //check if spi reads version:
    char rx = 0;
    char tx = 0x42; // & 0x7f makes it a read, | 0x80 would be write
    /*gpioWrite(8,0); //set the cs pin low to indicate start of data transfer
    spiWrite(handle, &tx, 1); //write 1 byte: read from register 0x42
    spiRead(handle, &rx, 1); //read 1 byte from reg 42
    gpioWrite(8,1); //set cs pin high to indicate end of data transfer
    */
    spi_read(handle, &tx, &rx, 1,1);
    if(rx == 0x12){
        cout << "success, version number 0x12" << endl;
    }
    else{
        cout << "no success, version number " << hex << rx << endl;
        return false;
    }

    //turn on sleep mode and loramode
    char* buf = new char[2];
    buf[0] = (CONFIG_REG | WRITE_MASK); //register 0x01 is a config reg
    buf[1] = SET_LORA_MODE | SET_SLEEP_MODE;
    spi_write(handle, buf, 2);
    gpioDelay(150);

    //check that it's in sleep / lora?
    tx = CONFIG_REG & READ_MASK;
    spi_read(handle, &tx, &rx, 1,1);
    /*gpioWrite(8,0);
    spiWrite(handle, &tx, 1);
    spiRead(handle, &rx, 1);
    gpioWrite(8,1);*/
//    cout << "rx: "  << rx << endl;
    if(rx == 0x80){
        cout << "rx is 0x80" << endl;
    }
    else{
        cout << "not in sleep / lora mode, rx not equal to 0x80" << endl;
        return false;
    }

    //interrupt pin?
    //setup interrupt pin input / pull down
    /*gpioSetMode(RF_RST, PI_INPUT);
    gpioSetPullUpDown(RF_IRQ, PI_PUD_DOWN);*/

    //configure rx/tx buffers to use 256 bytes
    buf[0] = FIFO_TX_BASE_REG | WRITE_MASK;
    buf[1] = 0;
    spi_write(handle, buf, 2);
    buf[0] = FIFO_RX_BASE_REG | WRITE_MASK;
    spi_write(handle, buf, 2);

    //change to idle mode
    buf[0] = CONFIG_REG | WRITE_MASK;
    buf[1] = SET_MODE_IDLE;
    spi_write(handle, buf, 2);
    //check idle mode
    tx = CONFIG_REG & READ_MASK;
    rx = 0;
    spi_read(handle, &tx, &rx, 1, 1);
    if(rx == 0x81){
        cout << "rx is 0x81, in lora idle mode" << endl;
    }
    else{
        cout << "rx is not 0x81" << endl;
        return false;
    }

    //set frequency
    cout << "setting freq" << endl;
    //check that this ends up correctly sending the last byte...
    uint32_t frf = (RF_FREQUENCY * 1000000.0) / ( 32000000.0/524288);
//    cout << bitset<32>(frf) << endl;
    buf[0] = FRF_MSB_REG | WRITE_MASK;
    buf[1] = (frf >> 16) & 0xff;
  //  cout << bitset<32>(buf[1]) << endl;
    spi_write(handle, buf, 2);
    buf[0] = FRF_MID_REG | WRITE_MASK;
    buf[1] = (frf >> 8) & 0xff;
 //   cout << bitset<32>(buf[1]) << endl;
    spi_write(handle, buf, 2);
    buf[0] = FRF_LSB_REG | WRITE_MASK;
    buf[1] = frf & 0xff;
  //  cout << bitset<32>(buf[1]) << endl;
    spi_write(handle, buf, 2);

    //set preamble to 8 to match arduino radiohead
    buf[0] = PREAMBLE_MSB_REG | WRITE_MASK;
    buf[1] = 0x08 >> 8;
    spi_write(handle, buf, 2);
    buf[0] = PREAMBLE_LSB_REG | WRITE_MASK;
    buf[1] = 0x08 & 0xff;
    spi_write(handle, buf, 2);

    //modem config settings Bw125Cr45Sf128 (should be default)
    buf[0] = MODEM_CONFIG1_REG | WRITE_MASK;
    buf[1] = 0x72; //125 kHz, 4/5 error coding rate, explicit header mode
    spi_write(handle, buf, 2);
    buf[0] = MODEM_CONFIG2_REG | WRITE_MASK;
    buf[1] = 0x74; //128 chips, normal mode, crc on, 0
    spi_write(handle, buf, 2);
    buf[0] = MODEM_CONFIG3_REG | WRITE_MASK;
    buf[1] = 0x00; //disabled low data rate optimization, LNA gain set by LnaGain reg
    spi_write(handle, buf, 2);

    //set tx power- use PA_BOOST
    //set pa dac enable for extra 3 dbm when using power 20
    int power = 20;
    if(power == 20){
        tx = PADAC_REG & READ_MASK;
        rx = 0;
        spi_read(handle, &tx, &rx, 1, 1);
  //      cout << "padac before: " << bitset<8>(rx) << endl;
        buf[0] = PADAC_REG | WRITE_MASK;
        buf[1] = 0x07; //enable padac... should this be 0x87?? to preserve the reserved bit? it will change that bit...
        spi_write(handle, buf, 2);
        tx = PADAC_REG & READ_MASK;
        rx = 0;
        spi_read(handle, &tx, &rx, 1, 1);
    //    cout << "padac after: " << bitset<8>(rx) << endl;
    }
    else{
        //disable padac,,, should be default value
        buf[0] = PADAC_REG | WRITE_MASK;
        buf[1] = 0x04;
        spi_write(handle, buf, 2);
    }
    //to pa config
    buf[0] = PA_CONFIG_REG | WRITE_MASK;
    buf[1] = 0x80 | (power-5); //0x8F when power = 20 (-5=15=F), paboost mode, output power 17
    spi_write(handle, buf, 2);

    delete[] buf;

    return true;
}
