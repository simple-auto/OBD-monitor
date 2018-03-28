/***********************
 ****** Libraries ******
 ***********************/

#include <SoftwareSerial.h>
#include <gprs_lite.h> 

/*******************************
 ****** Runtime Variables ******
 ******************************/

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


/****************************
 ***** Server variables *****
 ****************************/
 
char server[] = "REPLACE_WHEN_LOADING_TO_ARDUINO";     //Server's address
int port = 00; //REPLACE WHEN LOADING TO ARDUINO
//String data_to_send="";
char data_to_send[300];

/*****************************************
 *****************************************
 ****************  SETUP  ****************
 *****************************************
 *****************************************/
 
void setup() {  
  
  /********************************
   ****** Initialize SIM800L ******
   ******  & connect to APN  ******
   ********************************/
   
  while(0 != gprs.init()) delay(5000);
  gprs.sendCmdAndWaitForResp("AT+CGATT?\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIPSTATUS\r\n");
  delay(1000);
  //REPLACE APN, USERNAME, AND PASSWORD
  //gprs.sendCmd("AT+CSTT=\"WEB.TMOVIL.CL\",\"WEB\",\"WEB\"\r\n");                    //Movistar (Chile)
  //gprs.sendCmd("AT+CSTT=\"IMOVIL.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");   //entel 1 (Chile)
  gprs.sendCmd("AT+CSTT=\"BAM.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");      //entel 2 (Chile)
  delay(1000);
  gprs.sendCmdAndWaitForResp("AT+CIICR\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIFSR\r\n");
  delay(1000);
  
}//setup

/************************************
 ************************************
 **************  LOOP  **************
 ************************************
 ************************************/ 
void loop() {
  uint8_t i=0;
  while (Serial.available()>0){ //Listen if promini_reader sends something
		
    //Serial.readBytes(data_to_send,300);
    data_to_send[i]=Serial.read();
		i++;
    if(data_to_send[i]=='#'){
      break;
    }
  }
   /*********************************
   ****** Send data to server ******
   *********************************/
  send_data_to_server(data_to_send);
	Serial.println(data_to_send);
  /********************
   ***** End loop *****
   ********************/
}

     

/*****************************************
 *****************************************
 **************  FUNCTIONS  **************
 *****************************************
 *****************************************/


int send_data_to_server(String message_to_server){
  gprs.connectTCP(server, port);/*
  while(0 != gprs.connectTCP(server, port)){
      Serial.println(message_to_server);
      delay(1000);
  }*/
  char* tcp_snd = const_cast<char*>(message_to_server.c_str()); //Parse payload to char array
  message_to_server.replace("\n","");
  message_to_server.replace("\t","");
  message_to_server.replace("\r","");
  //gprs.shutTCP();
  gprs.closeTCP();
  return gprs.sendTCPData(tcp_snd);
}
