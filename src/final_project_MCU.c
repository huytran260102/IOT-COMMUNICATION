#include <MPU6050_tockn.h>
#include <Wire.h>
#include "SPI.h" // SPI library
#include "MFRC522.h" //RFID library
#include <DIYables_4Digit7Segment_74HC595.h> // DIYables_4Digit7Segment_74HC595 library
#include <avr/interrupt.h> 
#define BUF_SIZE 256

//Define for program
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

#define MAX_TABLE 4


//Define for Light
#define LIGHT_CONTROL 0x00
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
#define SCLK  30  // The Arduino pin connected to SCLK
#define RCLK  31  // The Arduino pin connected to RCLK
#define DIO   32  // The Arduino pin connected to DIO
//Define for ACCELEROMETER  
#define ACCELEROMETER 03
//Define for flash
#define FLASH_ID 04
////Define for Check_UUID 
#define CHECK_UUID 05
#define take_UUID 1
#define check_UUID 2
#define true_UUID 3
#define false_UUID 4


uint8_t status;
uint8_t rx_buf [BUF_SIZE], rx_pack[BUF_SIZE];
uint8_t tx_buf [BUF_SIZE], tx_pack[BUF_SIZE];
uint8_t rx_buf_len;
uint8_t tx_buf_len;
uint8_t cmd;
uint8_t address;
uint8_t rw = 0;
uint8_t state_receive = OFFSET_HEADER_1;
uint16_t data_receive[10];
//variable for RFID
static uint16_t customer_UUID[4];  
bool access_uuid = false;
int status_run = take_UUID;
uint16_t main_data[10];

const int pinRST = 5;     
const int pinSDA = 53;

MPU6050 mpu6050(Wire);
MFRC522 mfrc522(pinSDA, pinRST); // Set up mfrc522 on the Mega
DIYables_4Digit7Segment_74HC595 display(SCLK, RCLK, DIO);
/* * Connect device
 * LED:(Completed)
 * D2  -  Write LED 1
 * D3  -  Write LED 2
 * 
 * D6  - Read led 1
 * D7  - Read led 2
 *     
 * LM35 (Temperature Sensor)         Arduino
 *    VCC                              5V
 *    OUT                              A0
 *    GND                              GND
 * 
 * Led 7 Segment       Arduino
 *    Vcc                5V
 *    D30                SCLK   
 *    D31                RCLK    
 *    D32                DIO           
 *    GND                GND
 * MPU 6050(Completed)           Arduino
 *    VCC                         5V
 *    GND                         GND
 *    SCL                         SCL
 *    SDA                         SDA
 *    
 * RFID-RC522             Arduino(Mega)
 *    SDA                     53
 *    SCK                     52
 *    MOSI                    51
 *    MISO                    50
 *    IRQ                   Not use
 *    GND                     GND
 *    RST                     5
 *    3.3V                    3.3V
 */
//IRQ function
ISR (TIMER1_OVF_vect) 
{
    TCNT1 = 64536;
    display.loop(); //For LED 7 Segment
}
int uart_transmit() {
    for (int i = 0; i < tx_buf_len + 6; i++){
      Serial1.write(tx_pack[i]);
    }

    Serial.print("\nTransmited packet:\t");
    for (int j = 0; j < tx_buf_len + 6; j++) {
        Serial.print(tx_pack[j], HEX);
        Serial.print("\t");
    }
    return RET_SUCCESS;
}

int uart_receive(unsigned int* data_temp) {
  int incomingByte;
  uint8_t readbuf; 
  int readbytes = 0; 
  int i;
  i = 0;
  int count = 0;
  while (Serial1.available() > 0){
    incomingByte = Serial1.read();
    data_temp[count] = incomingByte;
    count ++;
    readbuf = incomingByte & 0xFF;
    if (state_receive == OFFSET_HEADER_1){
      Serial.print("\nReceived packet:\n");
    }
    Serial.print(readbuf, HEX);
    Serial.print("\t");
    switch (state_receive) {
      case OFFSET_HEADER_1:
          if (readbuf == HEADER_1) {
            state_receive = OFFSET_HEADER_2;
            Serial.println("HEADER 1");
          }
          else {
              Serial.println("[ERROR]: Wrong header 1");
              return RET_FAIL;
          }
          break;
      case OFFSET_HEADER_2:
          if (readbuf == HEADER_2) {
            state_receive = OFFSET_CMD;
            Serial.println("HEADER 2");
          }  
          else {
              Serial.println("[ERROR]: Wrong header 2");
              state_receive = OFFSET_HEADER_1;
              return RET_FAIL;
          }
          break;
      case OFFSET_CMD:
          cmd = readbuf;
          state_receive = OFFSET_DATALENGTH;
          Serial.println("CMD");
          break;
      case OFFSET_DATALENGTH:
          rx_buf_len = readbuf;
          state_receive = OFFSET_DATA;
          Serial.println("DATALENGTH");
          break;
      case OFFSET_DATA:
          if (i < rx_buf_len-1){
            rx_buf[i++] = readbuf;
          }
          else{
            rx_buf[i++] = readbuf;
            state_receive = OFFSET_TRAILER_1;
          }
          Serial.println("DATA");
          break;
      case OFFSET_TRAILER_1:
          if (readbuf == TRAILER_1){
            state_receive = OFFSET_TRAILER_2;
            Serial.println("TRAILER_1");
          }
          else {
              Serial.println("[ERROR]: Wrong trailer 1");
              state_receive = OFFSET_HEADER_1;
              return RET_FAIL;
          }
          break;
      case OFFSET_TRAILER_2:
          state_receive = OFFSET_HEADER_1;
          if (readbuf == TRAILER_2){
            Serial.println("TRAILER_2");
            return RET_SUCCESS;
          }
          else {
              Serial.println("[ERROR]: Wrong trailer 2");
              return RET_FAIL;
          }
          break;
      default:
          break;
      }
  }
  return RET_FAIL;
}

 void compose_packet() {
  int i = 4;
  tx_pack[0] = HEADER_1;
  tx_pack[1] = HEADER_2;
  tx_pack[2] = cmd;
  tx_pack[3] = tx_buf_len;
  for (i; i < tx_buf_len+4; ++i)
      tx_pack[i] = tx_buf[i-4];
  tx_pack[i] = TRAILER_1;
  tx_pack[i + 1] = TRAILER_2;
}

//lIGHT CONTROL ----------------------------
void lightcontrol_function(int data[])
{
    Serial.print("\n Doing: Control Light ");
  if(data[0] == LIGHT_A_ID)
  {
    if(data[1] == LIGHT_ON)
    {
      //LIGHT A
      Serial.print("\nTURN ON LIGHT A");
      digitalWrite(2,HIGH);
      //code to turn on light A
    }
    else
    {
      Serial.print("\nTURN OFF LIGHT A");
      digitalWrite(2,LOW);
      //code to turn off light A
    }
  }
  else
  {
    //LIGHT B
      if(data[1] == LIGHT_ON)
    {
      Serial.print("\nTURN ON LIGHT B");
      digitalWrite(3,HIGH);
      //code to turn on light B
    }
    else
    {
      Serial.print("\nTURN OFF LIGHT B");
      digitalWrite(3,LOW);
      //code to turn off light B
    }
  }
}
//Read Status Light ---------------------------
void readstatuslight_function(int data[])
{
  //Call function read status of light
  //Describe this function: This function is requested feedback data which about status light 
  //It receive data about id light then check status that light, final it resend data to PC
  
  tx_buf_len =  2; //It include LightID and LightStatus
  //Example: Read status light is Turn off now.
  cmd = READ_STATUS_LIGHT; 
  tx_buf[0] = data[0]; //data[4] is LightID which PC need check status
 
  int light = data[0]; // light A or B
  if(light == LIGHT_A_ID)
  {
    if( digitalRead(6) == LOW) 
    {
      tx_buf[1] = READ_STATUS_LIGHT_OFF;
      Serial.print("\n Light A is turned off ");

    }
    else
    {
      tx_buf[1] = READ_STATUS_LIGHT_ON;
      Serial.print("\n Light A is turned on ");
    }
  }
    else
    {
      if( digitalRead(7) == LOW) 
      {
        tx_buf[1] = READ_STATUS_LIGHT_OFF;
        Serial.print("\n Light B is turned off ");
      }
      else
      {
        tx_buf[1] = READ_STATUS_LIGHT_ON;
        Serial.print("\n Light B is turned on ");
      }
    }
  
//  tx_buf[1] = READ_STATUS_LIGHT_OFF;
  compose_packet();
  uart_transmit();
  Serial.print("\n Reading status of light ");
}
//Setup time for LED --------------------------
void led_control_function(int data[])
{
  //Call function to setup time to led
    

  Serial.print("\n setup time to led ");
  Serial.print("\nHour: ");Serial.println(data[0]);
  Serial.print("\nMin: ");Serial.println(data[1]);
  int data_print = data[0] * 100 + data[1];
  Serial.println(data_print);
  display.printInt(data_print, false); // you can display a value from -999 to 9999
  
}
//Read Temperature -----------------------------
void read_temperature_function(int data[])
{
  //Call function read sensor then return data to PC
  Serial.print("\n Read Temperature from sensor ");
  tx_buf_len =  1; //Only return data of temperature
  //Example: Temperature is 32oC now.
  uint8_t data_temperature = 32;
  tx_buf[0] = data_temperature;
 
// int reading = analogRead(A0);  
// float voltage = reading * 5.0 / 1024.0;
// float temp = voltage * 100.0;
// Serial.print("Temperature: ");
// Serial.println(temp);
// tx_buf[0] = temp;

  cmd = FLASH_ID;        //data[2] is CMD from PC send to MCU.
  delay(5);
  compose_packet();
  uart_transmit();
  Serial.print("\nReturn temperature data to PC ");
}

void read_accelerometer_function(int data[])
{
  //Call function read sensor then return data to PC
  Serial.print("\n Read accel from sensor ");
  tx_buf_len =  3;
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  
  tx_buf[0] = mpu6050.getAngleX();
  tx_buf[1] = mpu6050.getAngleY();
  tx_buf[2] = mpu6050.getAngleZ();
  cmd = ACCELEROMETER;        //data[2] is CMD from PC send to MCU.
  compose_packet();
  uart_transmit();
  Serial.print("\nReturn accel data to PC ");
}

void Check_UUID_function(int data[])
{
  int count_uuid = 0;
  do{
  // take UUID and return true or false UUID to PC
  switch(status_run)
    {
      case take_UUID:
//      Serial.print(" \n case take_UUID\n");
//      Serial.print(" \n Wating for card \n");
      count_uuid ++;
      //delay(1000);
      if (mfrc522.PICC_IsNewCardPresent()) { // (true, if RFID tag/card is present ) PICC = Proximity Integrated Circuit Card
        if(mfrc522.PICC_ReadCardSerial()) { // true, if RFID tag/card was read
          Serial.print("RFID TAG ID:");
          for (byte i = 0; i < mfrc522.uid.size; ++i) { // read id (in parts)
            Serial.print(mfrc522.uid.uidByte[i], HEX); // print id as hex values
            customer_UUID[i] = (mfrc522.uid.uidByte[i]);
            Serial.print(" "); // add space between hex blocks to increase readability
          }
          
          Serial.println(); // Print out of id is complete.
        }
        //Serial.print("\nHello dont run me\n");
        status_run = check_UUID;
      }
      break;
  
      case check_UUID:
      {
        //Serial.print(" \n case check_UUID\n");
          Check_UUID();
          if(access_uuid == true)
          {
            status_run = true_UUID;
          }
          else 
          {
             status_run = false_UUID;
          }
      }
      break;
      
      case true_UUID:
          //Serial.print(" \n Your UUID true\n Going to program"); 
          status_run = take_UUID;
          tx_buf_len =  1;
            tx_buf[0] = 1; //Return data = 1 is true UUID
            cmd = CHECK_UUID;
            compose_packet();
            uart_transmit();
      break;
      
      case false_UUID:
          //Serial.print(" \n Wrong UUID \n Please take your card again"); 
          status_run = take_UUID;
          tx_buf_len =  1;
          tx_buf[0] = 0; //Return data = 1 is wrong UUID
          cmd = CHECK_UUID;
          compose_packet();
          uart_transmit();
          delay(1000);
      break;
    
      default:
          Serial.print(" \n Wrong status_run\n");
          status_run = take_UUID; 
          break;
      } 
  }while(count_uuid < 2);
}
void Check_UUID()
{
  Serial.print(" \n Im at check_uuid check user \n");
  uint16_t user1_UUID[3][4] = {{ 250 , 176 , 246 , 218 }, { 163 , 19 , 140 , 16 }, { 227 , 100 , 129 , 16 }};
              for(int x = 0; x < 3; x++){
              for(int i = 0; i < 4; i++ ){
                  if(customer_UUID[i] != user1_UUID[x][i]) {
                      access_uuid = false;
                      break;
                  } else {
                      access_uuid = true;
                  }
              }
              if(access_uuid) break;
            }
}


void setup() {
  Serial.begin(9600);                       
  Serial1.begin(9600); 
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT);     
  pinMode(6, INPUT); 
  pinMode(7, INPUT);  
  SPI.begin(); // open SPI connection
  mfrc522.PCD_Init(); // Initialize Proximity Coupling Device (PCD)   
  //Setup for interupt
        cli();                                  // tắt ngắt toàn cục
    
    /* Reset Timer/Counter1 */
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    
    /* Setup Timer/Counter1 */
    TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64
    TCNT1 = 64536;
    TIMSK1 = (1 << TOIE1);                  // Overflow interrupt enable 
    sei();    
  display.printInt(0000, false);// init fisrt data for LED 7 Segment
}
void loop() {  
    status = RET_FAIL;
    status = uart_receive(data_receive);
    if(status ==  RET_SUCCESS)
    {
      for(int i=0; i< rx_buf_len ; i++) 
      {
        main_data[i] = rx_buf[i];
      }
      int process = cmd; //CMD here. It will be decide on the operation of device
      switch(process)
      {
          Serial.print(" \n What CMD ??? \n");
          Serial.print("\n CMD:  ");
          Serial.println(process);
        case LIGHT_CONTROL:
          lightcontrol_function(data_receive);
        break;

        case READ_STATUS_LIGHT:
          readstatuslight_function(main_data);
        break;

        
        case ACCELEROMETER:
          read_accelerometer_function(main_data);
        break;
        
        case LED_ID:
          led_control_function(main_data);
        break;

        case FLASH_ID:
          read_temperature_function(main_data);
        break;
      
        case CHECK_UUID:
          Check_UUID_function(main_data);
        break;
        default:
          Serial.print(" \n Wrong CMD ??? \n");
          Serial.print("\n CMD:  ");
          Serial.println(process);
          break;
      }
      
    }

  
}
