/*
 *  * Structure:
 *  
 *  
 * Important variables:
 *  - Server IP
 *  - Server TCP port
 *  - Baudrates
 * 
 * Preamble:
 *  - Initialize SIM800L
 *  - Define ID of the device by getting IMEI serial number (6 digit)
 *  - Initialize buffer freezing first 6 characters to serial number
 *  - Configure APN
 * 
 * Loop:
 *  - Get data from BT block by reading software serial
 *  - Prepare packet by type
 *  - Connect to server and send packet
 *  - Restart buffer
 *  
 *  Multiple SoftwareSerial ports: https://www.arduino.cc/en/Tutorial/TwoPortReceive
 */



/***********************
 ****** Libraries ******
 ***********************/

#include <SoftwareSerial.h>

/*******************************
 ****** Runtime Variables ******
 ******************************/

//Time [ms]:
unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 30000; //total time elapsed on each iteration in
//int           time_response       = 300;  //time to wait for response

#define DEFAULT_TIMEOUT 5

//Serial Communications:
/*
#define baud_SIM_serial     9600 
#define baud_reader_serial  9600 
#define baud_debug_serial   9600
*/
SoftwareSerial SIM_serial(8,7);     //SIM800L

SoftwareSerial readerSerial(3,2);   //Pro mini reader RX TX

//TCP communications:
char server[] = "52.13.248.109"; //"162.248.55.95";     //Server IP address
int port = 2000; //9002; //REPLACE WHEN LOADING TO ARDUINO        //TCP port
char data_to_send[512];                                 //Packet buffer
uint32_t _ip;                                           //Local IP address
char ip_string[20];

/*****************************************
 *****************************************
 ****************  SETUP  ****************
 *****************************************
 *****************************************/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  
  SIM_serial.begin(9600);
  readerSerial.begin(9600);
  Serial.begin(9600);
  
  SIM_serial.listen();
  SIM_debug();  
  
 /********************************
  ****** Initialize SIM800L ******
  ********************************/
  
  //+CFUN=1, +CPIN?
  while(0 != initialize()){delay(1000);}  delay(1000);
  //sendCmd("AT+CIPSTATUS\r\n"); delay(2000);
  
 /********************************
  ***** Replace APN Settings ***** 
  ********************************/

  //+CSTT
  //sendCmd("AT+CSTT=\"WAP.TMOVIL.CL\",\"WAP\",\"WAP\"\r\n");                  //Movistar (Chile)
  sendCmd("AT+CSTT=\"WEB.TMOVIL.CL\",\"WEB\",\"WEB\"\r\n");                    //Movistar (Chile)
  //sendCmd("AT+CSTT=\"IMOVIL.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");   //entel 1 (Chile)
  //sendCmd("AT+CSTT=\"BAM.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");      //entel 2 (Chile)
  delay(2000);

  //+CGATT, +CREG
  //while(0 != sendCmdAndWaitForResp("AT+CGATT?\r\n","OK",DEFAULT_TIMEOUT)){delay(1000);}
  while(0 != networkCheck()){delay(1000);}
  Serial.println("Attached to GPRS network");

  //+CIICR, +CIFSR
  sendCmdAndWaitForResp("AT+CIICR\r\n","OK",DEFAULT_TIMEOUT);
  delay(1000);
  sendCmd("AT+CIFSR\r\n");
  delay(1000);

  Serial.println("2G Initialized.");

  /*******************************
   ****** Get Serial Number ****** 
   *******************************/
  while(SIM_serial.available()) {   // display the other thing..
      SIM_serial.read();
  }

  //+CGSN: Get serial number
  //Expected response: 865691035634435\n\nOK
  sendCmd("AT+CGSN\r\n");
  delay(2000);

  int i=0;
  while(1) {
    if(i>23)break;
    if(SIM_serial.available()) {
        char c = SIM_serial.read();
        if(i>=18){ 
          data_to_send[i-18] = c ;
        }
    }
    i++;
  }
  //Dismiss rest of response after 14th character
  while(SIM_serial.available()) {
      SIM_serial.read();
  }

  Serial.print("\n");
  Serial.print("Serial Number = ");
  Serial.println(data_to_send);
  Serial.print("\n");
  // buffer:  char array
  // See communication protocol: https://github.com/simple-auto/OBD-monitor/blob/master/Protocol.md
   
   
}

void loop(){
  timestamp = millis();
  
  /*************************
   ****** Get Payload ******
   *************************/

  readerSerial.listen();
  Serial.println("Waiting for payload... ");

  int i = 6;
  char c = '\0';
  //while (readerSerial.available()>0){
  while (true){ // -> delete / comment 
    if(readerSerial.available()>0){
      c = readerSerial.read();
      delay(1);
      data_to_send[i]=c;
      i++;
    }
    if(c == '&'){ 
      data_to_send[i-1]=' ';
      break;
    }
    if(i>=510) break;
  }
 
  /*************************
   ****** Send Packet ****** 
   *************************/

  
  SIM_serial.listen();  
  //Connect to gprs service

  //get time
  sendCmd("AT+CCLK?\r\n");
  delay(1000);
  int j = 0;
  while(1) {
    if(j>35)break;
    if(SIM_serial.available()) {
        char c = SIM_serial.read();
        if(j>=19){ 
          data_to_send[i] = c ;
          i++;
        }
    }
    j++;
  }

  Serial.println(data_to_send); // -> delete / comment  
  
  send_data_to_server(data_to_send);
  

  /**************************
   ****** Reset Buffer ******
   **************************/

  if(true){
    //Buffer. erase trailer/tail of packet, preserve header/head
    Serial.println("Erasing buffer... ");
    for(int i=6; i<510; i++){
      data_to_send[i] = '\0';
    }
    Serial.println(data_to_send); // -> delete / comment  
  }
  else{
    Serial.println("Failed to send. Retrying... ");
  }
  

  /********************
   ***** End loop *****
   ********************/

  time_elapsed=millis()-timestamp;
  if(time_between_loops>time_elapsed){delay(time_between_loops-time_elapsed);}


}


/*****************************************
 *****************************************
 **************  FUNCTIONS  **************
 *****************************************
 *****************************************/


void send_data_to_server(char* message_to_server){
//void send_data_to_server(String message_to_server){
  /*
   * 

  message_to_server.replace("\n","");
  message_to_server.replace("\t","");
  message_to_server.replace("\r","");
  char* tcp_snd = const_cast<char*>(message_to_server.c_str()); //Parse payload to char array
  Serial.print("msg: ");
  Serial.println(tcp_snd);

  */
  char* tcp_snd = message_to_server;
  if(0 == connectTCP(server, port)){
    Serial.print("\nConnect successfuly to: ");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);
  }
  else{
    Serial.println("Connect error");
    closeTCP(); 
    return;
  }
  if(0 == sendTCPData(tcp_snd)){
    Serial.println("sent");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for 200 ms
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);   
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for 200 ms
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(200);   
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for 200 ms
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  else{Serial.println("Not sent");}
  closeTCP();
  
}



/*
 * GPRS
 * A library for SeeedStudio seeeduino GPRS shield 
 *
 * Copyright (c) 2013 seeed technology inc.
 * Author        :   lawliet zou
 * Create Time   :   Dec 2013
 * Change Log    :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

  int waitForResp(const char *resp, unsigned int timeout)
{
    int len = strlen(resp);
    int sum=0;
    unsigned long timerStart,timerEnd;
    timerStart = millis();
    
    while(1) {
        if(SIM_serial.available()) {
            char c = SIM_serial.read();
            sum = (c==resp[sum]) ? sum+1 : 0;
            if(sum == len)break;
        }
        timerEnd = millis();
        if(timerEnd - timerStart > 1000 * timeout) {
            return -1;
        }
    }

    while(SIM_serial.available()) {
        SIM_serial.read();
    }

    return 0;
}

int readBuffer(char *buffer,int count, unsigned int timeOut)
{
    int i = 0;
    unsigned long timerStart,timerEnd;
    timerStart = millis();
    while(1) {
        while (SIM_serial.available()) {
            char c = SIM_serial.read();
            if (c == '\r' || c == '\n') c = '$';                            
            buffer[i++] = c;
            if(i > count-1)break;
        }
        if(i > count-1)break;
        timerEnd = millis();
        if(timerEnd - timerStart > 1000 * timeOut) {
            break;
        }
    }
    delay(500);
    while(SIM_serial.available()) {   // display the other thing..
        SIM_serial.read();
    }
    return 0;
}

void cleanBuffer(char *buffer, int count)
{
    for(int i=0; i < count; i++) {
        buffer[i] = '\0';
    }
}

void sendCmd(const char* cmd)
{
  Serial.println(cmd);
  SIM_serial.write(cmd);
}

int sendCmdAndWaitForResp(const char* cmd, const char *resp, unsigned timeout)
{
    sendCmd(cmd);
    return waitForResp(resp,timeout);
}

void SIM_debug(void)
{
    while(1) {
        if(SIM_serial.available()){
            Serial.write(SIM_serial.read());
        }
        if(Serial.available()){     
            SIM_serial.write(Serial.read()); 
        }
    }
}

void sendEndMark(void)
{
    SIM_serial.println((char)26);
}

int initialize(void){
    if(sendCmdAndWaitForResp("AT\r\n","OK\r\n",DEFAULT_TIMEOUT*3)){return -1;}
    if(sendCmdAndWaitForResp("AT+CFUN=1\r\n","OK\r\n",DEFAULT_TIMEOUT*3)){return -1;}
    if(checkSIMStatus()) {return -1;}
    return 0;
}

int checkSIMStatus(void){
    char gprsBuffer[32];
    int count = 0;
    cleanBuffer(gprsBuffer,32);
    while(count < 3) {
        sendCmd("AT+CPIN?\r\n");
        readBuffer(gprsBuffer,32,DEFAULT_TIMEOUT);
        if((NULL != strstr(gprsBuffer,"+CPIN: READY"))) {
            break;
        }
        count++;
        delay(300);
    }
    if(count == 3) {return -1;}
    return 0;
}

int networkCheck(void)
{
    delay(1000);
    if(0 != sendCmdAndWaitForResp("AT+CGREG?\r\n","+CGREG: 0,1",DEFAULT_TIMEOUT*3)) {
        //ERROR("ERROR:CGREG");
        return -1;
    }
    delay(1000);
    if(0 != sendCmdAndWaitForResp("AT+CGATT?\r\n","+CGATT: 1",DEFAULT_TIMEOUT)) {
        //ERROR("ERROR:CGATT");
        return -1;
    }
    return 0;
}

bool join(const char *apn, const char *userName, const char *passWord)
{
    char cmd[64];
    char ipAddr[32];
    char gprsBuffer[32];
   
    //Select multiple connection
    //sim900_check_with_cmd("AT+CIPMUX=1\r\n","OK",DEFAULT_TIMEOUT,CMD);
      
    cleanBuffer(ipAddr,32);
    sendCmd("AT+CIFSR\r\n");    
    readBuffer(ipAddr,32,2);
    
    // If no IP address feedback than bring up wireless 
    if( NULL != strstr(ipAddr, "ERROR") )
    {
        if( 0 != sendCmdAndWaitForResp("AT+CSTT?\r\n", apn, DEFAULT_TIMEOUT) )
        {
            sendCmd("AT+CSTT=\"");
            sendCmd(apn);
            sendCmd("\",\"");
            sendCmd(userName);
            sendCmd("\",\"");
            sendCmd(passWord);        
            sendCmdAndWaitForResp("\"\r\n","OK\r\n",DEFAULT_TIMEOUT*3);
        }
        
        //Brings up wireless connection
        sendCmd("AT+CIICR\r\n");
         
        //Get local IP address
        cleanBuffer(ipAddr,32);
        sendCmd("AT+CIFSR\r\n");
        readBuffer(ipAddr,32,2);        
    }          
#if 0    
    Serial.print("ipAddr: ");
    Serial.println(ipAddr);
#endif

    if(NULL != strstr(ipAddr,"AT+CIFSR")) {        
        _ip = str_to_ip(ipAddr+11);
        if(_ip != 0) {
            return true;
        }
    }
    return false;
}
uint32_t str_to_ip(const char* str)
{
    uint32_t ip = 0;
    char *p = (char*)str;
    
    for(int i = 0; i < 4; i++) {
        ip |= atoi(p);
        p = strchr(p, '.');
        if (p == NULL) {
            break;
        }
        if(i < 3) ip <<= 8;
        p++;
    }
    return ip;
}

char* getIPAddress()
{
    uint8_t a = (_ip>>24)&0xff;
    uint8_t b = (_ip>>16)&0xff;
    uint8_t c = (_ip>>8)&0xff;
    uint8_t d = _ip&0xff;

    snprintf(ip_string, sizeof(ip_string), "%d.%d.%d.%d", a,b,c,d);
    return ip_string;
}



int connectTCP(const char *ip, int port)
{
    char cipstart[50];
    sprintf(cipstart, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", ip, port);
    if(0 != sendCmdAndWaitForResp(cipstart, "CONNECT OK", 2*DEFAULT_TIMEOUT)) {// connect tcp
        //ERROR("ERROR:CIPSTART");
        return -1;
    }

    return 0;
}
int sendTCPData(char *data)
{
    char cmd[32];
    int len = strlen(data); 
    snprintf(cmd,sizeof(cmd),"AT+CIPSEND=%d\r\n",len);
    if(0 != sendCmdAndWaitForResp(cmd,">",2*DEFAULT_TIMEOUT)) {
        //ERROR("ERROR:CIPSEND");
        return -1;
    }
        
    if(0 != sendCmdAndWaitForResp(data,"SEND OK",2*DEFAULT_TIMEOUT)) {
        //ERROR("ERROR:SendTCPData");
        return -1;
    }     
    return 0;
}

int closeTCP(void)
{
    sendCmd("AT+CIPCLOSE\r\n");
    return 0;
}

