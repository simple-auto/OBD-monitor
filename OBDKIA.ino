/**
 * 
 * Cloud Car Monitor
 *
 * Revo Project revoproject@terralink.cl
 * Terralink SpA, Santiago, Chile
 * 
 * CODE FOR KIA OBD Prototype 1
 * 
 * Collaborators:
 * Vicente Matus
 * Andres Astudillo
 *
 * 
 * 
 * OBD-II -- ELM327 -- BT -- BT -- NANO -- SIM800L
 * 
 * Online Utilities:
 * HEX to Binary: https://www.binaryhexconverter.com/hex-to-binary-converter
*/

/***********************
 ****** Libraries ******
 ***********************/

//#include <gprs.h>
#include <SoftwareSerial.h>

/*******************************
 ****** Runtime Variables ******
 ******************************/

unsigned long start_millis = 0;           //to measure time elapsed getting OBD data
unsigned long stop_millis = 0;            //to measure time elapsed getting OBD data
int           time_between_loops = 5000;  //total time elapsed on each iteration in [ms]

#define baud_serial0 9600           //Serial inncluded in arduino
//#define baud_serial1 9600           //SIM
#define baud_serial2 38400          //BT-OBD

//SoftwareSerial SIM_serial(8,7);     //SIM
SoftwareSerial BTOBD_serial(2,3);   //BT-OBD

/**************************
 ****** OBD Variables *****
 **************************/

//boolean ECU_on = false;         //Engine Control Unit's state
String  raw_OBD_response = "";  //to save OBD's response after sending commands
byte    inData;                 //to parse data received from OBD
char    inChar;                 //to read OBD's response char-wise
String  WorkingString="";       //to cut substrings from raw_OBD_response
long    A;                      //to save numeric data gotten from control unit's responses
//long    B;
//long    C;

float   temp;              
float   rpm;               
float   veloc;             


/*****************************************
 ******** SIM and Cloud Variables ********
 *****************************************/
/*
#define DEFAULT_TIMEOUT     5
char server[] = "api.thingspeak.com";     //Server's address
String WriteAPIKey = "AEYR0MF2O3Y512QQ";  //Thingspeak channel key to write data
//String channel_ID = "369437";             //Thingspeak channel ID
String thingspeak_command = "";           //GET command with fields data (defined after getting OBD data)
//char buffer[512];   
GPRS gprs;                                //SIM808 object
//boolean connectivity = false;             //to attempt connection to the cloud or skip and work oflin
*/

/********************************
 *********** OBD PIDs ***********
 ********************************/

#define pids_rpm "010C"

void setup() {
  /*****************************************
   *****************************************
   ****************  SETUP  ****************
   *****************************************
   *****************************************/
  
  /*** Begin serial: */
  Serial.begin(baud_serial0);       
  Serial.println("Initializing Cloud Car Monitor System");
  //SIM_serial.begin(baud_serial1);
  //gprs.serialDebug();
  /*
  gprs.preInit();
  while(0 != gprs.init()) {
     delay(1000);
     Serial.println("2G initialization error. Check SIM card.");
  }
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
  
 /******************************
 *** Initialize Bluetooth HC 05
 ******************************/  
  BTOBD_serial.begin(baud_serial2);
  Serial.println("Bluetooth communication to ELM327 Initialized");
  
 /*********************
 *** Initialize ELM327
 *********************/  
 
  BTOBD_serial.println("atz");          //Reset
  delay(500);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
  BTOBD_serial.println("at sp 0");      //Automatically choose OBD protocol
  delay(300);
  read_elm327_response()
  Serial.println(raw_OBD_response);
 
  BTOBD_serial.println("at dp");        //Describe Protocol
  delay(100);
  read_elm327_response()
  Serial.println(raw_OBD_response);

  BTOBD_serial.println("at dpn");       //Describe Protocol by Number (check this description please)
  delay(100);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
  BTOBD_serial.println("01 00");        //Receive Mode-01 Available Sensors List
  delay(1000);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
  BTOBD_serial.println("01 05");        //Request PID 05 (Coolant Temperature)
  delay(300);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
  BTOBD_serial.println("01 0C");        //Request PID 0C (Engine RPM)
  delay(300);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
  BTOBD_serial.println("01 0D");        //Request PID 0D (Vehicle Speed)
  delay(300);
  read_elm327_response()
  Serial.println(raw_OBD_response);
  
}

void loop(){
/****************************************
 ****************************************
 ****************  LOOP  ****************
 ****************************************
 ****************************************/
  
  //debug_serial2();      //Debugger for OBD-BT serial interface
  //debug_serial1();      //Debugger for SIM800L serial interface
  //gprs.serialDebug();   //Library-native debugger for SIM800L serial interface

  start_millis = millis();    //Register initial timestamp

  Serial.print("\n");         //Start new line (do not use println in any print of the loop)
  Serial.print(start_millis); 
  Serial.print("\t[ms]\t");
  
/***************************************
 ******** Get data from sensors ********
 ***************************************
 Structure for each sensor:
  - Send sensor PID
  - Wait for the ELM327 to acquire
  - Read ELM327's response
  - Cut A,B,C,... Bytes values
  - Convert to integers
  - Apply formula
  - Display value and unit (tabulated)
 ***************************************/
  
  /*
   * Get Temperature
   */
  
  BTOBD_serial.println("0105");                       //Send temperature sensor PID
  delay(310);                                         //Wait for the ELM327 to acquire
  read_elm327_response();                             //Read ELM327's response
  WorkingString = raw_OBD_response.substring(11,13);  //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16)-40;       //Convert to integer
  temp = A;                                           //Apply formula
  Serial.print(A); Serial.print("\tºC\t");            //Display value and unit (tabulated)
  
  /*
   * Get RPM
   */
  
  BTOBD_serial.println("010C");                    
  delay(310);
  read_elm327_response();
  WorkingString = raw_OBD_response.substring(11,13)+raw_OBD_response.substring(14,16);   
  A = strtol(WorkingString.c_str(),NULL,16);        
  rpm = A/4;
  Serial.print(rpm); Serial.print("\tRPM\t");
  
  /**
   * Get Velocity
   */

  BTOBD_serial.println("010D");                      
  delay(310);
  read_elm327_response();
  WorkingString = raw_OBD_response.substring(11,13); 
  A = strtol(WorkingString.c_str(),NULL,16);        
  veloc=A; 
  Serial.print(veloc); Serial.print("\t[km/h]\t");

  //BTOBD_serial.end();
  
  
  /************************************
   ****** Send data to the cloud ******
   ************************************/
  
  /*
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
  delay(time_between_loops-(stop_millis-start_millis));
   

}

void read_elm327_response(){
  raw_OBD_response = "";
  while (BTOBD_serial.available()>0){
    inData=0;
    inChar=0;
    inData = BTOBD_serial.read();
    inChar=char(inData);
    raw_OBD_response = raw_OBD_response + inChar;
  } 
  //read_elm327_response()
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
