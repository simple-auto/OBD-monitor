/**
 * 
 * Terralink - Revo Project
 * 
 * CODE FOR KIA OBD Prototype 1
 * 
 * Collaborators:
 * Vicente Matus      vicente.matus@terralink.cl
 * Andres Astudillo   andres.astudillo@terralink.cl
 * 
 * 
 * Vehicle monitor through OBD-II Bluetooth adapter
 * and Arduino Nano based circuit
 * 
 * 
 * 
 * OBD-II  // ELM327 // BT--BT // NANO // SIM800L // CLOUD (NOT IMPLEMENTED YET)
 * 
 * 
 * 
*/

//#include <gprs.h>
#include <SoftwareSerial.h>
//

//SoftwareSerial SIM_serial(8,7);  // SIM
SoftwareSerial BTOBD_serial(2,3);  // BT-OBD

#define baud_serial0 9600       //Serial Monitor
//#define baud_serial1 9600       //SIM
#define baud_serial2 38400      //BT-OBD


/**
 * OBD Variables
 */
//boolean ECU_on = false;         //Engine Control Unit's state
String  raw_OBD_response = "";  //to save OBD's response after sending commands
byte    inData;                 //to parse data received from OBD
char    inChar;                 //to read OBD's response char-wise
String  WorkingString="";       //to cut substrings from raw_OBD_response
long    A;                      //to save numeric data gotten from control unit's responses
float   rpm;                    //to save rpm data after applying formula
float   veloc;                  //to save velocity data after applying formula

/**
 * SIM and Cloud Variables
*/

/*
#define DEFAULT_TIMEOUT     5
char server[] = "api.thingspeak.com";     //Server's address
String WriteAPIKey = "AEYR0MF2O3Y512QQ";  //Thingspeak channel key to write data
//String channel_ID = "369437";             //Thingspeak channel ID
String thingspeak_command = "";           //GET command with fields data (defined after getting OBD data)
//char buffer[512];   
GPRS gprs;                                //SIM808 object
//boolean connectivity = false;             //to attempt connection to the cloud or skip and work ofline

*/
unsigned long start_millis = 0;              //to measure time elapsed getting OBD data
unsigned long stop_millis = 0;               //to measure time elapsed getting OBD data

/**
 * Runtime Variables
 */


void setup() {
  /**
  * Begin serial:
  */
  Serial.begin(baud_serial0);       
  Serial.println("Initializing OBD Monitor System");
  //delay(2000);

  /*
  * Initialize 2G SIM800L
  *
  *

  //SIM_serial.begin(baud_serial1);
  //gprs.serialDebug();
  gprs.preInit();
  while(0 != gprs.init()) {
     delay(1000);
     Serial.println("2G initialization error. Check SIM card.");
  }
  
    /*Configure GPRS with local APN data (entel Chile)
     * 
     * Command                                          Expected response
   * AT...                                              OK
   * AT+CPIN?...                                        READY
   * AT+CFUN=1...                                       OK
   * AT+CGATT?...                                       OK
   * AT+CIPSTATUS...                                    STATE: IP INITIAL
   * AT+CSTT?...                                        "CMNET","","" OK   
   * AT+CSTT="BAM.ENTELPCS.CL","ENTELPCS","ENTELPCS"... OK
   * AT+CIICR...                                        OK
   * AT+CIFSR                                           10.158.17.235 (or similar)
   */
/*
  gprs.sendCmdAndWaitForResp("AT+CFUN=1\r\n","OK",DEFAULT_TIMEOUT);
  delay(1000);
  gprs.sendCmdAndWaitForResp("AT+CGATT?\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIPSTATUS\r\n");
  delay(1000);
  gprs.sendCmd("AT+CSTT=\"BAM.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");
  delay(1000);
  gprs.sendCmdAndWaitForResp("AT+CIICR\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIFSR\r\n");
  delay(1000);
  //char* IP = gprs.getIPAddress();


  Serial.println("2G Initialized.");
  //Serial.println(IP);
  
  
  /**
  * Initialize BT HC05
  */
  
  BTOBD_serial.begin(baud_serial2);

  /**
   * Bluetooth HC-05 was already permanently configured 
   * to be binded to ELM327 Bluetooth's address by the
   * following commands:
   * 
   * (Press ENABLE button in HC-05 board)
   */
  
  //Serial.println("BT Initialized");
  
  /**
  * Initialize ELM327
  */
  
  //Ask version (ELM327 v2.1):
  BTOBD_serial.println("atz");
  delay(500);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  }
  delay(500);
  //Serial.println("\n");
  
  
  //Automatically choose OBD protocol:
  BTOBD_serial.println("at sp 0");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  }
  
  BTOBD_serial.println("at dp");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  }
  Serial.println("\n"); 

  BTOBD_serial.println("at dpn");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  }
  Serial.println("\n"); 

  //Start sensors:
  
  Serial.println("\n");
  BTOBD_serial.println("01 00");
  delay(3000);
  while (BTOBD_serial.available()){
    //Serial.write(BTOBD_serial.read());   //Debug mode
    BTOBD_serial.read();                 //Silent mode.
  } 
  Serial.println("ELM327 Initialized.");

/*
  Serial.println("\n");
  BTOBD_serial.println("01 05");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  } 
  Serial.println("Temp. Sensor initialized.");
*/

  //Serial.println("\n");
  BTOBD_serial.println("01 0C");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  } 
  Serial.println("Engine RPM sensor initialized.");

  
  BTOBD_serial.println("01 0D");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  } 
  Serial.println("Vehicle Speed sensor initialized");
  Serial.println("\n");
  
  
}


/**
 * 
 * 
 * 
 *  LOOP
 * 
 * 
 */


void loop(){
  
  /*
  * Comment to run loop. Un-comment to run terminal mode:
  */ 
  
  //debug_serial2(); //OBD-BT   
  //debug_serial1();
  //gprs.serialDebug();

  start_millis = millis();

  Serial.print("\n");
  Serial.print(start_millis);
  Serial.print("\t[ms]\t");
  
/**
 * Get data from sensors
 */



  //Get Temperature:
  BTOBD_serial.println("0105");  //Send Temp PID
  //BTOBD_serial.flush();
  delay(380);
  read_serial2();             //Receive response
  //Recover data
  WorkingString = raw_OBD_response.substring(11,13);   
  //Serial.println(raw_OBD_response);
  //Serial.println(WorkingString);
  A = strtol(WorkingString.c_str(),NULL,16)-40;  //convert hex to decimnal
  Serial.print(A);
  Serial.print(" ºC\t");
  
  /**
   * Get RPM
   */
  
  BTOBD_serial.println("010C");                     //Send RPM PID (0C from group 01)
  delay(400);
  read_serial2();
  WorkingString = raw_OBD_response.substring(11,13)+raw_OBD_response.substring(14,16);   
  //Serial.println(raw_OBD_response);
  //Serial.println(WorkingString);
  A = strtol(WorkingString.c_str(),NULL,16);        //convert hex to decimal
  rpm = A/4;                                        //Apply OBD formula (see OBD PIDs wiki)
  Serial.print(rpm);
  Serial.print("\tRPM\t");
  
  /**
   * Get Velocity
   */

  BTOBD_serial.println("010D");                       //Send Velocity PID
  delay(330);
  read_serial2(); //Receive response
  WorkingString = raw_OBD_response.substring(11,13);  //Recover data
  //Serial.println(raw_OBD_response);
  //Serial.println(WorkingString);
  A = strtol(WorkingString.c_str(),NULL,16);          //convert hex to decimal and apply formula
  veloc=A; //Apply OBD formula (see OBD PIDs wiki)
  Serial.print(veloc);
  Serial.print("\t[km/h]\t");

  //BTOBD_serial.end();
  /**
   * Send data to the cloud
   * 
   * 
   *
  GPRS gprs;
  delay(1000);
  if(0 == gprs.connectTCP(server, 80)){
      //Serial.print("Connect successfuly to ");
      //Serial.println(server);
  }
  else{
      //Serial.println("connect error");
  }


  thingspeak_command = ("GET /update?api_key="+WriteAPIKey+"&field1="+rpm+"&field2="+veloc+"    HTTP/1.0\r\n\r\n");
  //Serial.println("command="+thingspeak_command);
  char* http_cmd = const_cast<char*>(thingspeak_command.c_str()); //Parse command to char array

  if(0 == gprs.sendTCPData(http_cmd)){
    //gprs.serialDebug();
    char fin_get[] = "CLOSED\n";
    gprs.waitForResp(fin_get,5);
    Serial.println("Sent");
  }
  else{
    Serial.println("Not sent");
  }

  gprs.serialSIM800.end();
  delay(1000);
  BTOBD_serial.begin(baud_serial2);
  */
  stop_millis=millis();
  //Serial.print(stop_millis);
  delay(5000-(stop_millis-start_millis));
   

}

void read_serial2(){
  raw_OBD_response = "";
  while (BTOBD_serial.available()>0){
    inData=0;
    inChar=0;
    inData = BTOBD_serial.read();
    inChar=char(inData);
    raw_OBD_response = raw_OBD_response + inChar;
  } 
  //Serial.println(raw_OBD_response);
}



void debug_serial2(){
  /**Debug HC05 Bluetooth and ELM327 OBD
   * *  Captures serial2 data and writes input to it.
  */
  while(true){
    if (BTOBD_serial.available()){
      Serial.write(BTOBD_serial.read());
    }
    if (Serial.available()){
      BTOBD_serial.write(Serial.read());
    } //if
  }//while
}//debug_serial2()



/*
void debug_serial1(){
  /**Debug (serial 1)
   *  Captures serial2 data and writes input to it.
  * (add / after * if function uncommented)
  while(true){
    if (SIM_serial.available()){
      Serial.write(SIM_serial.read());
    }
    if (Serial.available()){
      SIM_serial.write(Serial.read());
    }
  }

}
*/
