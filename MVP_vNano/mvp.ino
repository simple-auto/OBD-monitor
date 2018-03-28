/***********************
 ****** Libraries ******
 ***********************/

#include <SoftwareSerial.h>
#include <gprs_lite.h> 

/*******************************
 ****** Runtime Variables ******
 ******************************/
//(All time variables in [ms])
unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 1000; //total time elapsed on each iteration in
int           time_response       = 300;  //time to wait for response
uint8_t       lines               = 10;

#define baud_serial_to_HC05 38400
#define baud_serial_to_SIM800L 9600 

/***********************************
 ****** Serial Communications ******
 ***********************************/
 
//SIM_serial.begin(baud_serial_to_SIM800L); //Software serial for SIM800L, wired to pins 7,8, handled by SoftwareSerial library
GPRS gprs;                                //Software serial for SIM800L, wired to pins 7,8, handled by gprs_lite library
//SoftwareSerial SIM_serial(8,7);     //SIM800L
//SIM800L's serial can be handled by gprs_lite library
//SoftwareSerial BTOBD_serial(2,3); //BT-OBD
//Serial for BT-OBD changed to "hardware" serial (pins 0,1)

/**********************************
 ****** OBD Reading Variables *****
 **********************************/
String  raw_ELM327_response = ""; //to save OBD's response after sending commands
byte    inData;                   //to parse data received from OBD
char    inChar;                   //to read OBD's response char-wise
String  WorkingString="";         //to cut substrings from raw_ELM327_response
uint8_t     A;                    //to save numeric data gotten from control unit's responses
uint8_t     B;
uint8_t      j;              
const String pid2[] PROGMEM={"0105", "at rv"};


/****************************
 ***** Server variables *****
 ****************************/
char server[] =   "REPLACE_WHEN_LOADING_TO_ARDUINO";    
int port = 0;    //REPLACE_WHEN_LOADING_TO_ARDUINO

String data_to_send="";

/*****************************************
 *****************************************
 ****************  SETUP  ****************
 *****************************************
 *****************************************/
 
void setup() {  

  Serial.begin(baud_serial_to_HC05);        //"Hardware" serial for HC05, wired to pins 0,1
  /********************************
   ****** Initialize SIM800L ******
   ******  & connect to APN  ******
   ********************************/
  while(0 != gprs.init()) delay(1000);
  delay(5000);
  if(0==gprs.sendCmdAndWaitForResp("AT+CFUN=1\r\n","OK",DEFAULT_TIMEOUT)){
    Serial.println("CFUN=1");
  }
  delay(2000);
  if(0==gprs.sendCmdAndWaitForResp("AT+CGATT?\r\n","OK",DEFAULT_TIMEOUT)){
    Serial.println("CGATT OK");
  }
  gprs.sendCmd("AT+CIPSTATUS\r\n");
  delay(2000);
  //REPLACE APN, USERNAME, AND PASSWORD
  //gprs.sendCmd("AT+CSTT=\"WAP.TMOVIL.CL\",\"WAP\",\"WAP\"\r\n");                    //Movistar (Chile)
  gprs.sendCmd("AT+CSTT=\"WEB.TMOVIL.CL\",\"WEB\",\"WEB\"\r\n");                    //Movistar (Chile)
  //gprs.sendCmd("AT+CSTT=\"IMOVIL.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");   //entel 1 (Chile)
  //gprs.sendCmd("AT+CSTT=\"BAM.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");      //entel 2 (Chile)
  delay(3000);
  gprs.sendCmdAndWaitForResp("AT+CIICR\r\n","OK",DEFAULT_TIMEOUT);
  delay(1000);
  gprs.sendCmd("AT+CIFSR\r\n");
  delay(4000);
  Serial.println("2G Initialized.");
  
  /********************************************
  ****** Initialize ELM327 through HC 05 ******
  *********************************************/ 
  
  while(true){
    Serial.println("atz");
    delay(2000);
    read_elm327_response();
      if(raw_ELM327_response.substring(4,5)!="?"){
      break;
      }
  }
  while(true){
    Serial.println("at sp 0"); //Protocol n°6: ISO 15765-4 CAN (11/500)
    delay(2000);
    read_elm327_response();
    if(raw_ELM327_response.substring(8,9)!="?"){
      break;
    }
  }
  /**********************
   ***** Check DTCs *****
   **********************/
  while(true){
    Serial.println("0101");
    delay(1000); 
    read_elm327_response(); 
    WorkingString = raw_ELM327_response.substring(11,13);       //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);                  //Convert to integer
    if(A==0 && raw_ELM327_response.substring(11,13)!="IT"){     //CHECK IN TERMINAL FOR ISO 15765-4
    //if(A==0 && raw_ELM327_response.substring(11,13)=="00"){   //CHECK IN TERMINAL
      data_to_send += "0 DTC";
      break;
    }
    else if(A>0){
      //A=A-128;
      Serial.println("03");                            //Request Mode 03 (List of DTCs)
      delay(5000); 
      read_elm327_response(); 
      data_to_send += raw_ELM327_response;
      
      break;
    }
  }
  
  /*********************************
   ***** Report DTCs to Server *****
   *********************************/
  send_data_to_server(data_to_send);
  
}//setup

/************************************
 ************************************
 **************  LOOP  **************
 ************************************
 ************************************/

void loop() {
  data_to_send = "";    
  for(uint8_t i=0; i<lines; i++){
     timestamp = millis(); //Register initial timestamp
    if(i==0){
      data_to_send+=String(timestamp) + ",";
    }
    /***************************************
    ******** Get data from sensors ********
    ***************************************
    Structure for each sensor:  
    - Send sensor PID
    - Wait for the ELM327 to acquire
    - Read ELM327's response
    Post-process (at server):
    - Cut A,B,C,... Bytes values
    - Convert to bytes
    - Apply formula
    - Display value and unit (tabulated)
    ***************************************/ 
    
    /***************
     **** Speed **** 
     ***************/
    Serial.println("010D");                         //Send sensor PID for speed 
    delay(time_response);                                 //Wait for the ELM327 to acquire
    read_elm327_response();                               //Read ELM327's response              
    WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
    data_to_send += String(A) + ",";                  //Concat to data to send
    
    /***************
     ***** RPM ***** 
     ***************/
    Serial.println("010C");                             //Send sensor PID for RPM 
    delay(time_response);                                     //Wait for the ELM327 to acquire
    read_elm327_response();                                   //Read ELM327's response                      
    WorkingString = raw_ELM327_response.substring(11,13);     //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);                //Convert to integer
    WorkingString = raw_ELM327_response.substring(14,16);     //Cut B Byte value  
    B = strtol(WorkingString.c_str(),NULL,16);                //Convert to integer    
    data_to_send += String((256*A+B)/4) + ",";                      //Concat to data to send
    
    /****************************
     **** Coolant or Battery **** 
     ****************************/
    if(i>=lines-10){
      Serial.println(pid2[j]);                           //Send sensor PID for Coolant temp or battery voltage
      delay(time_response);                                    //Wait for the ELM327 to acquire
      read_elm327_response();                                  //Read ELM327's response
      if (j==0){
        WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
        A = strtol(WorkingString.c_str(),NULL,16);                //Convert to integer
        data_to_send += String(A-40) + ",";
      }
      else{
        WorkingString = raw_ELM327_response.substring(6,10);
        data_to_send += WorkingString + ",";
      }
      j++;
      if(j==2){
        j=0;
      }
      
    } 
      
    time_elapsed=millis()-timestamp;
    if(time_between_loops>time_elapsed){
      delay(time_between_loops-time_elapsed);
    }
  }//for
  /*********************************
   ****** Send data to server ******
   *********************************/
  send_data_to_server(data_to_send);
  
  /********************
   ***** End loop *****
   ********************/
}

     

/*****************************************
 *****************************************
 **************  FUNCTIONS  **************
 *****************************************
 *****************************************/

void read_elm327_response(){
  raw_ELM327_response = "";
  while (Serial.available()>0){
    inData=0;
    inChar=0;
    inData = Serial.read();
    inChar=char(inData);
    raw_ELM327_response = raw_ELM327_response + inChar;
  } 
  //read_elm327_response()
  //Serial.println(raw_ELM327_response);
}

void send_data_to_server(String message_to_server){
  //gprs.closeTCP();
  message_to_server.replace("\n","");
  message_to_server.replace("\t","");
  message_to_server.replace("\r","");
  char* tcp_snd = const_cast<char*>(message_to_server.c_str()); //Parse payload to char array
  Serial.print("msg: ");
  Serial.println(tcp_snd);
  if(0 == gprs.connectTCP(server, port)){
    Serial.print("connect successfuly to: ");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);
  }
  else{
    Serial.println("connect error");
    gprs.closeTCP(); 
    return;
  }
  if(0 == gprs.sendTCPData(tcp_snd)){Serial.println("sent");}
  else{Serial.println("not sent");}
  gprs.closeTCP();
  
}
