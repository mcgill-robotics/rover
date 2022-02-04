#include <Arduino.h>
#include <Rover_SerialAPI.h>
#include <stdio.h>

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


static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuff[SERIAL_RX_BUFFER_SIZE];
static int ibuff[SERIAL_RX_BUFFER_SIZE];


void setup() {
  pinMode(ob, OUTPUT);
  SerialAPI::init(ID, 9600);
}

void loop() {

  /*
  //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(Serial.available()==tmp){delay(6000);}
  //while(!SerialAPI::update()) delay(1000); //Doesn't take into account the bytes that we sent but don't want to clear buffer

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  while(!(buffer[1] == 'Y')){
    //Ask for a retransmit of wrong ID
    SerialAPI::send_retransmit();

    //Wait for answer
    while(!SerialAPI::update()) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  

  for(int i=0;i<5;i++){
    SerialAPI::send_bytes('0',"YOO C MALADE",0);
    digitalWrite(ob,LOW);
    delay(1500);
    digitalWrite(ob,HIGH);
    delay(1500);
  }

  */

      char* s = "0x40a75c29";
      uint32_t num;
      float f;
      sscanf(s, "%x", &num);  // assuming you checked input
      f = *((float*)&num);

      char* buf;
      sprintf(buf, "%.3f", f);
      Serial.println(buf);
      Serial.println(num);
      Serial.println(f);
      delay(1000);


   if(Serial.available() > 0)
  {
      memset(fbuff, 0,SERIAL_RX_BUFFER_SIZE);
      memset(fbuf.b,0,SERIAL_RX_BUFFER_SIZE);

      memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
      //while(!SerialAPI::update()) delay(10);
      //SerialAPI::read_data(buffer,sizeof(buffer));

      const size_t bytes_read = Serial.readBytes(buffer, Serial.available());

      /*
      CONVERT THE VALUE RECEIVED TO HEX / DECODE WHAT WAS SENT BY PYTHON
      OR
      MAKE PYTHON SEND HEX FORMATTED DATA
      */  

      //char* msg;
      //memcpy(msg, buffer+2, bytes_read-3);

      fbuff[0]=939.6958;
      fbuff[1]=410;
      fbuff[2]=50;
      //SerialAPI::send_bytes('0',fbuf.b,fbuf.count*sizeof(float));
      //SerialAPI::send_bytes('4',fbuff,3*sizeof(float));

      SerialAPI::send_bytes('1', buffer, bytes_read);

      for (int i=0; i<buffer[2]-3; i++){
        
      }

      delay(100);

      //if (bytes_read>0)
      if (buffer[0]=='~')
      {
        for (int i=0;i<5;i++)
        {
        digitalWrite(ob,1);           
        delay(100);                                      
        digitalWrite(ob,0);          
        delay(100);
        }
      }
      

      delay(100);
   } 
   delay(3000);
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



bool syn2master(HardwareSerial Serial){
   //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(Serial.available()==tmp) delay(1000);
  //while(!SerialAPI::update()) delay(1000); //Doesn't take into account the bytes that we sent but don't want to clear buffer

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  while(!(buffer[1] == 'Y')){
    char buffer[SERIAL_RX_BUFFER_SIZE];

    //Ask for a retransmit of wrong ID
    SerialAPI::send_retransmit();

    //Wait for answer
    int tmp = Serial.available();
    while(Serial.available()==tmp) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  return true;

}


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