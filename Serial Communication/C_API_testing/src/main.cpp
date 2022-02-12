#include <Arduino.h>
#include <Rover_SerialAPI.h>
#include <stdio.h>


#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 


//https://www.google.com/search?q=upload+code+to+arduino+with+platform+io&rlz=1C1SQJL_frCA922CA922&oq=upload+code+to+arduino+with+plat&aqs=chrome.1.69i57j33i22i29i30l2.7350j1j7&sourceid=chrome&ie=UTF-8#kpvalbx=_f9vpYdOvCf-ZptQP14KwgAI14

  /*
  To read/send data from the serial you can:
  
  Use CMD:

    Windows
    >powershell
    >[System.IO.Ports.SerialPort]::getportnames()
    >$port= new-Object System.IO.Ports.SerialPort COM#,Baudrate,None,8,one //This creates the port object

    Port object commands:
    >$port.open()
    >$port.WriteLine("some string")
    >$port.ReadLine()
    >$port.Close()

    >exit //this exits powershell


  Use the platform IO monitor

  Use PUTTY
  */



#define ob 13

#define ID 'a'


//Will make a lib for that
typedef struct {
    float b[SERIAL_RX_BUFFER_SIZE/sizeof(float)];
    size_t count;
} FloatBuffer;
FloatBuffer fbuf;


void AddFloatToBuffer(FloatBuffer fb, float val);
void print_byte_array(byte* byte_array, size_t size);
void char_to_float(char* str_byte, float* f);


static char buffer[SERIAL_RX_BUFFER_SIZE];

void setup() {
  pinMode(ob, OUTPUT);
  SerialAPI::init(ID, 9600);
}

void loop() {



   if(Serial.available() > 0)
  {
      memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
      
      
      while(!SerialAPI::update()) delay(10);
      int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));
      const size_t payload_size = strlen(buffer);

      SerialAPI::send_bytes('1', buffer, payload_size);

      delay(100);
   } 

     delay(10);
}



/*
void serialEvent()
{
   while(Serial.available()) 
   {
      char ch = Serial.read();
      //Serial.write(ch);
      Serial.print(ch);
    }  
   Serial.println();
}
*/




//Will make a lib for that
void AddFloatToBuffer(FloatBuffer fb,float val)
{
    if(fb.count < SERIAL_RX_BUFFER_SIZE/sizeof(float))
    {
        fb.b[fb.count] = val;
        fb.count++;
    }
}





void decode_msg(char* buffer){

}



void sync(void){
  //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(!SerialAPI::update()) delay(1000);

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  while(!(buffer[1] == 'Y')){
    memset(buffer,0,SERIAL_RX_BUFFER_SIZE);

    //Ask for a retransmit of wrong ID
    SerialAPI::send_retransmit();

    //Wait for answer
    while(!SerialAPI::update()) delay(1000);

    //int tmp = Serial.available();
    //while(Serial.available()==tmp) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  

  //External validation
  for(int i=0;i<5;i++){
    digitalWrite(ob,LOW);
    delay(200);
    digitalWrite(ob,HIGH);
    delay(200);
  }
}



void print_byte_array(byte* byte_array, size_t size){
  char buffer[1];
  memset(buffer,0,1);
  for (int i = 0; i<size;i++){
    sprintf(buffer, BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(byte(byte_array[i])));
    Serial.print(buffer);
  }
}


void char_to_float(char* str_byte, float* f){
  char array[4];

  #ifdef BIGENDIAN
  array[0] = byte(str_byte[3]);
  array[1] = byte(str_byte[2]);
  array[2] = byte(str_byte[1]);
  array[3] = byte(str_byte[0]);
  #else 
  array[0] = byte(str_byte[0]);
  array[1] = byte(str_byte[1]);
  array[2] = byte(str_byte[2]);
  array[3] = byte(str_byte[3]);
  #endif

  memcpy(f,array,4);
}