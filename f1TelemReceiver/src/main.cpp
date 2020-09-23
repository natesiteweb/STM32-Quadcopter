#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#define MY_ADDRESS 0x04

#define CE_pin PB8
#define CSN_pin PB12
#define MOSI_pin PB15
#define MISO_pin PB14
#define SCK_pin PB13

void I2C_Receive(int howMany);

void NRF_Init(void);
void NRFwrite_bit_write(byte, byte, byte);
void transmit(byte, byte, byte);
void NRF_set_RX_payload(byte, byte);
void NRF_get_address(byte, byte);
void NRF_ClearInterrupts(void);
void get_data(void);

SPIClass rfspi(MOSI_pin, MISO_pin, SCK_pin);
byte data_in[32], data2, data3;

HardwareSerial Serial3(PB11, PB10);

uint8_t received_num;
volatile char wire_buf[40];
volatile bool i2c_receive_flag = false;

void setup()
{
  //pinMode(PA6, OUTPUT);
  pinMode(PC13, OUTPUT);

  Serial.begin(115200);

  delay(5000);
  NRF_Init();
  NRF_set_RX_payload(0, 3);
  NRF_get_address(7, 1);
  NRFwrite_bit_write(0, 0, 1); //register#, bit#, and value 0 or 1, ::  0,0,1 RX Mode
  NRFwrite_bit_write(0, 1, 1); //register, bit, and value 0,1,1 PowerUP
  NRFwrite_bit_write(0, 4, 1); //RT Mask turns off the RT interrupt
  NRFwrite_bit_write(0, 5, 1); //TX Mask turns off the TX interrupt
  NRFwrite_bit_write(0, 6, 1);
  //Serial3.begin(9600);

  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B11100010); //flush RX
  digitalWrite(CSN_pin, HIGH);
  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B11100001); //flush TX
  digitalWrite(CSN_pin, HIGH);

  NRF_ClearInterrupts(); //clears any interrupts
  delay(100);

  digitalWrite(PC13, HIGH);

  //Wire.setSDA(PB7);
  //Wire.setSCL(PB6);
  //Wire.begin(MY_ADDRESS);

  //Wire.onReceive(I2C_Receive);
}

byte testState = 0;

void loop()
{
  //transmit(0x02, 0x51, 0x23);

  NRF_get_address(7, 0);

  if (bitRead(data_in[1], 6))
  {
    get_data();

    if(data_in[1] == 0x34 && data_in[2] == 0x51 && data_in[3] == 0x23)
    {
      testState = (~testState) & 0x01;

      digitalWrite(PC13, testState);
    }

    Serial.print(data_in[1], HEX);
    Serial.print(" ");
    Serial.print(data_in[2], HEX);
    Serial.print(" ");
    Serial.println(data_in[3], HEX);

    NRFwrite_bit_write(7, 6, 1); //clear the RX interrupt flag
  }

  delay(5);

  /*if(i2c_receive_flag)
  {
    i2c_receive_flag = false;
    Serial.print("Received: ");
    Serial.println((char*)wire_buf);
  }*/
}

void NRF_Init()
{
  pinMode(CE_pin, OUTPUT);
  pinMode(CSN_pin, OUTPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(SCK_pin, OUTPUT);
  rfspi.setBitOrder(MSBFIRST);
  rfspi.setDataMode(SPI_MODE0);
  rfspi.setClockDivider(SPI_CLOCK_DIV8);
  digitalWrite(CE_pin, HIGH);
  digitalWrite(CSN_pin, HIGH);
  rfspi.begin();
  Serial.println("rfspi Started");
}

void NRF_set_RX_payload(byte pipe, byte bytes) //Set RF payload size
{
  byte address = pipe + 32 + 16 + 1;
  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(address); //write register 11
  data_in[1] = rfspi.transfer(bytes);
  digitalWrite(CSN_pin, HIGH);
  Serial.println("Setup RX Payload");
}

void get_data()
{ // get data start get data start get data start get data start get data start
  // this routine is called when the IRQ pin is pulled LOW by the NRF

  int i;
  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B01100001); //read the payload
  data_in[1] = rfspi.transfer(B00000000);
  data_in[2] = rfspi.transfer(B00000000);
  data_in[3] = rfspi.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B11100010); //flush RX
  digitalWrite(CSN_pin, HIGH);

  //NRFwrite_bit_write(7, 6, 1); //clear the RX interrupt flag

} //     END   get  data     END   get  data     END   get  data     END   get  data     END   get  data

void NRFwrite_bit_write(byte address, byte bit_add, byte val)
{ //   start bit write   start bit write   start bit write
  //This routine writes single bits of a register, without affecting the rest of the register

  NRF_get_address(address, 0); //first read out the register
  if (val == 1)                //if we want to write a one to the bit then set the bit in the register we read
    bitSet(data_in[1], bit_add);
  else
    bitClear(data_in[1], bit_add); //clear it if not

  digitalWrite(CSN_pin, LOW);                //now we'll write the modified data back in
  data_in[0] = rfspi.transfer(32 + address); //a write to a register adds 32
  data_in[1] = rfspi.transfer(data_in[1]);   //write the modified register
  digitalWrite(CSN_pin, HIGH);
} //    END bit write    END bit write    END bit write    END bit write    END bit write    END bit write

void NRF_get_address(byte address, byte info)
{ // START Get Address   START Get Address   START Get Address
  //send the address and either a 1 or 0 if you want to do a serial print of the address
  //after a call to this routine, data_in[1] will equal the address you called

  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(address);
  data_in[1] = rfspi.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);
  if (info == 1)
  { // if the user wanted it, you will get a print out of the register - good fo debugging
    Serial.print("R");
    Serial.print(address);
    switch (address)
    {
    case 0:
      Serial.print(" CONFIG REGISTER =");
      Serial.println(data_in[1]);
      Serial.print("PRIM_RX = ");
      if (bitRead(data_in[1], 0))
        Serial.println("PRX");
      else
        Serial.println("PTX");

      Serial.print("PWR_UP = ");
      if (bitRead(data_in[1], 1))
        Serial.println("POWER UP");
      else
        Serial.println("POWER DOWN");

      Serial.print("CRCO = ");
      if (bitRead(data_in[1], 2))
        Serial.println("2Bytes");
      else
        Serial.println("1Byte");

      Serial.print("EN_CRC = ");
      if (bitRead(data_in[1], 3))
        Serial.println("Enabled");
      else
        Serial.println("Disabled");

      Serial.print("MASK_MAX_RT = ");
      if (bitRead(data_in[1], 4))
        Serial.println("Interrupt not reflected on the IRQ pin");
      else
        Serial.println("Reflect MAX_RT as active low interrupt on the IRQ pin");

      Serial.print("MASK_TX_DS = ");
      if (bitRead(data_in[1], 5))
        Serial.println("Interrupt not reflected on the IRQ pin");
      else
        Serial.println("Reflect TX_DS as active low interrupt on the IRQ pin");

      Serial.print("MASK_RX_DR = ");
      if (bitRead(data_in[1], 6))
        Serial.println("Interrupt not reflected on the IRQ pin");
      else
        Serial.println("Reflect RX_DR as active low interrupt on the IRQ pin");
      break; //0
    case 1:
      Serial.print(" EN_AA REGISTER Enhanced ShockBurst =");
      Serial.println(data_in[1]);
      break; //1
    case 2:
      Serial.print(" EN_RXADDR REGISTER Enabled RX Addresses =");
      Serial.println(data_in[1]);
      break; //2
    case 3:
      Serial.print(" SETUP_AW REGISTER Setup of Address Widths =");
      Serial.println(data_in[1]);
      break; //3
    case 4:
      Serial.print(" SETUP_RETR REGISTER Setup of Automatic Retransmission =");
      Serial.println(data_in[1]);
      break; //4
    case 5:
      Serial.print(" RF_CH REGISTER RF Channel =");
      Serial.println(data_in[1]);
      break; //5
    case 6:
      Serial.print(" RF_SETUP REGISTER RF Setup Register =");
      Serial.println(data_in[1]);
      Serial.print("RF Power = ");
      Serial.print(bitRead(data_in[1], 2));
      Serial.println(bitRead(data_in[1], 1));
      Serial.print("RF_DR_HIGH = ");
      Serial.println(bitRead(data_in[1], 3));
      Serial.print("PLL_LOCK = ");
      Serial.println(bitRead(data_in[1], 4));
      Serial.print("RF_DR_LOW = ");
      Serial.println(bitRead(data_in[1], 5));
      Serial.print("CONT_WAVE = ");
      Serial.println(bitRead(data_in[1], 7));
      break; //6
    case 7:
      Serial.print(" STATUS REGISTER  =");
      Serial.println(data_in[1]);
      Serial.print("TX_FULL = ");
      if (bitRead(data_in[1], 0))
        Serial.println("TX FIFO full");
      else
        Serial.println("TX FIFO Not full");

      Serial.print("RX_P_NO = ");
      if (bitRead(data_in[1], 1) && bitRead(data_in[1], 2) && bitRead(data_in[1], 3))
        Serial.println("RX FIFO Empty");
      else
        Serial.println(bitRead(data_in[1], 1) + (bitRead(data_in[1], 2) << 1) + (bitRead(data_in[1], 2) << 2));
      Serial.print("MAX_RT Interrupt = ");
      Serial.println(bitRead(data_in[1], 4));
      Serial.print("TX_DS Interrupt = ");
      Serial.println(bitRead(data_in[1], 5));
      Serial.print("RX_DR Interrupt = ");
      Serial.println(bitRead(data_in[1], 6));
      break; //7
    case 8:
      Serial.print(" OBSERVE_TX REGISTER Transmit observe register  =");
      Serial.println(data_in[1]);
      Serial.print("ARC_CNT = ");
      Serial.println(bitRead(data_in[1], 0) + (bitRead(data_in[1], 1) << 1) + (bitRead(data_in[1], 2) << 2) + (bitRead(data_in[1], 3) << 3));
      Serial.print("PLOS_CNT = ");
      Serial.println(bitRead(data_in[1], 4) + (bitRead(data_in[1], 5) << 1) + (bitRead(data_in[1], 6) << 2) + (bitRead(data_in[1], 7) << 3));
      break; //8
    case 9:
      Serial.print(" RPD REGISTER Received Power Detector =");
      Serial.println(bitRead(data_in[1], 0));
      break; //9
    case 10:
      Serial.print(" RX_ADDR_P0 LSB =");
      Serial.println(data_in[1]);
      break; //10
    case 11:
      Serial.print(" RX_ADDR_P1 LSB =");
      Serial.println(data_in[1]);
      break; //11
    case 12:
      Serial.print(" RX_ADDR_P2 LSB =");
      Serial.println(data_in[1]);
      break; //12
    case 13:
      Serial.print(" RX_ADDR_P3 LSB =");
      Serial.println(data_in[1]);
      break; //13
    case 14:
      Serial.print(" RX_ADDR_P4 LSB =");
      Serial.println(data_in[1]);
      break; //14
    case 15:
      Serial.print(" RX_ADDR_P5 LSB =");
      Serial.println(data_in[1]);
      break; //15
    case 16:
      Serial.print(" TX_ADDR LSB =");
      Serial.println(data_in[1]);
      break; //16
    case 17:
      Serial.print(" RX_PW_P0 RX payload =");
      Serial.println(data_in[1]);
      break; //17
    case 18:
      Serial.print(" RX_PW_P1 RX payload =");
      Serial.println(data_in[1]);
      break; //18
    case 19:
      Serial.print(" RX_PW_P2 RX payload =");
      Serial.println(data_in[1]);
      break; //19
    case 20:
      Serial.print(" RX_PW_P3 RX payload =");
      Serial.println(data_in[1]);
      break; //20
    case 21:
      Serial.print(" RX_PW_P4 RX payload =");
      Serial.println(data_in[1]);
      break; //21
    case 22:
      Serial.print(" RX_PW_P5 RX payload =");
      Serial.println(data_in[1]);
      break; //22

    case 23:
      Serial.print(" FIFO_STATUS Register =");
      Serial.println(data_in[1]);
      Serial.print("RX_EMPTY = ");
      if (bitRead(data_in[1], 0))
        Serial.println("RX FIFO empty");
      else
        Serial.println("Data in RX FIFO");

      Serial.print("RX_EMPTY = ");
      if (bitRead(data_in[1], 1))
        Serial.println("RX FIFO full");
      else
        Serial.println("Available locations in RX FIFO");

      Serial.print("TX_EMPTY = ");
      if (bitRead(data_in[1], 4))
        Serial.println("TX FIFO empty");
      else
        Serial.println("Data in TX FIFO");

      Serial.print("TX_FULL = ");
      if (bitRead(data_in[1], 5))
        Serial.println("TX FIFO full");
      else
        Serial.println("Available locations in TX FIFO");
      Serial.print("TX_REUSE = ");
      Serial.println(bitRead(data_in[1], 6));
      break; //23
    }        //switch
  }          //if 1
} // END get_address END get_address END get_address END get_address END get_address END get_address END get_address

void NRF_ClearInterrupts()
{ //    start clear interrupts      start clear interrupts      start clear interrupts
  //there are three interrupt flags in the NRF.  Thsi routine checks them, and if set, it will clear them

  NRF_get_address(7, 0); //RT interrupt
  if (bitRead(data_in[1], 4))
    NRFwrite_bit_write(7, 4, 1);

  NRF_get_address(7, 0); //TX interrupt
  if (bitRead(data_in[1], 5))
    NRFwrite_bit_write(7, 5, 1);

  NRF_get_address(7, 0); //RX interrupt
  if (bitRead(data_in[1], 6))
    NRFwrite_bit_write(7, 6, 1);

} //   END clear interrupts   END clear interrupts   END clear interrupts   END clear interrupts

void transmit(byte mode, byte pin, byte value)
{ //     transmit start     transmit start     transmit start
  //mode pin and value don't mena anything yet, but they will be

  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B11100001); //flush TX, get rid of anything that might be in there
  digitalWrite(CSN_pin, HIGH);

  digitalWrite(CSN_pin, LOW);
  data_in[0] = rfspi.transfer(B10100000); //load TX payload
  data_in[1] = rfspi.transfer(mode);      //action digital Read
  data_in[2] = rfspi.transfer(pin);       //pin number
  data_in[3] = rfspi.transfer(value);
  digitalWrite(CSN_pin, HIGH);

  digitalWrite(CE_pin, LOW);   //pull CE pin LOW
  delay(1);                    //small delay
  NRFwrite_bit_write(0, 0, 0); //go into TX mode
  delay(1);                    //small delay
  digitalWrite(CE_pin, HIGH);
  delay(1); //this is the time CE pin must be HIGH for before going back into RX mode
  //delay(1) seems to work best for this.  any longer or shorter doesn't work as well
  NRFwrite_bit_write(0, 0, 1); //go back into RX mode

} //    end  transmit    end  transmit    end  transmit    end  transmit    end  transmit

void I2C_Receive(int howMany)
{
  Wire.readBytes((char *)wire_buf, howMany);

  i2c_receive_flag = true;
}