// Agregar Serial number con comandos AT+CGSN o AT+CSN

/***********************
 ****** Libraries ******
 ***********************/

#include <SoftwareSerial.h>
#include <gprs_lite.h> 

/*******************************
 ****** Runtime Variables ******
 ******************************/

//#define baud_serial_to_SIM800L 9600 
#define baud_serial0 9600 

/***********************************
 ****** Serial Communications ******
 ***********************************/
SoftwareSerial readerSerial(4,3); // Pro mini reader RX TX
 
    //SIM_serial.begin(baud_serial_to_SIM800L); //Software serial for SIM800L, wired to pins 7,8, handled by SoftwareSerial library
GPRS gprs;                                //Software serial for SIM800L, wired to pins 7,8, handled by gprs_lite library
    //SoftwareSerial SIM_serial(8,7);     //SIM800L
    //SIM800L's serial can be handled by gprs_lite library
    //Serial for BT-OBD changed to "hardware" serial (pins 0,1)


/****************************
 ***** Server variables *****
 ****************************/
 
char server[] = "REPLACE_WHEN_LOADING_TO_ARDUINO";     //Server's address
int port = 00; //REPLACE WHEN LOADING TO ARDUINO
//String data_to_send="";

char data_to_send[512];

/*****************************************
 *****************************************
 ****************  SETUP  ****************
 *****************************************
 *****************************************/
 
void setup() {  

  Serial.begin(baud_serial0);
  readerSerial.begin(baud_serial0);

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
 
  
}//setup

/************************************
 ************************************
 **************  LOOP  **************
 ************************************
 ************************************/ 
void loop() {

  while (readerSerial.available()>0){
    //delay(100);
    readerSerial.readBytes(data_to_send,512);
    delay(500);
    Serial.println(data_to_send); // -> delete / comment
    //send_data_to_server(data_to_send);
    /*
    if(true){ //si se manda exitosamente a la nube
      ;//vaciar el buffer
    }
    */
  }

  /********************
   ***** End loop *****
   ********************/

}

     

/*****************************************
 *****************************************
 **************  FUNCTIONS  **************
 *****************************************
 *****************************************/


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

