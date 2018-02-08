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

unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 5000; //total time elapsed on each iteration in [ms]

#define baud_serial0 9600           //Serial inncluded in arduino
//#define baud_serial1 9600           //SIM
#define baud_serial2 38400          //BT-OBD

//SoftwareSerial SIM_serial(8,7);     //SIM
SoftwareSerial BTOBD_serial(2,3);   //BT-OBD

/********************************
 ****** Temporary Variables *****
 ********************************/

//boolean ECU_on = false;         //Engine Control Unit's state
String  raw_ELM327_response = ""; //to save OBD's response after sending commands
byte    inData;                   //to parse data received from OBD
char    inChar;                   //to read OBD's response char-wise
String  WorkingString="";         //to cut substrings from raw_ELM327_response
long    A;                        //to save numeric data gotten from control unit's responses
long    B;
//long    C;

/**************************
 ****** Car Variables *****
 **************************/

float   temp;              
float   rpm;               
float   veloc;             
float   ndtc;
float   var;

String pidstart[]={"atz", "at sp 0", "at dp", "at dpn","0902", "0A", "07", "0100", "0120", "0140", "0160"};
String %start[]={"Reset: ", "Auto protocol: ", "Protocol: ", "Protocol number: ", "VIN: ", "Cleared DTCs: ", "Last cycle DTCs: ", "Supported PIDs 00: ", "Supported PIDs 20:", "Supported PIDs 40: ", "Supported PIDs 60: "};
int l1=10;

String pid1s[]={"010D", "0133", "0111", "010C", "0163", "015E", "011F"};
String s1[]={"Speed: ", "RPM: ", "Ref Torque: ", "Pressure: ", "Fuel rate: ", "Run time: ", "Throttle: "};
String u1[]={" [km/h]", " [RPM]", " [Nm]", " [kPa]", " [L/h]", " [s]", " [%]"};
int l2=6;

String pid30s[]={"015C", "0105", "0146", "015B"};
String s30[]={"Oil temp: ", "Coolant temp: ", "Ambient temp: ", "Battery life: "};
String u30[]={ " [°C]", " [°C]", " [°C]", " [%]"};
int l3=3;

//(A*a+B)/c
float c1[]={1, 1, 2.55, 4, 1, 20, 1};
int b30[]={40, 40, 40, 0};
float c30[]={1, 1, 1, 2.55};

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
   //Start 
  for(int i=0; i<=l1; i++){             
  BTOBD_serial.println(pidstart[i]);          
  delay(500); read_elm327_response();   
  Serial.print(start[i]); Serial.println(raw_ELM327_response);  
  }
  //1s
  for(int i=0; i<=l2; i++){             
  BTOBD_serial.println(pid1s[i]);          
  delay(500); read_elm327_response(); Serial.println(raw_ELM327_response);  
  }
  //30s
  for(int i=0; i<=l1; i++){             
  BTOBD_serial.println(pid30s[i]);          
  delay(500); read_elm327_response(); Serial.println(raw_ELM327_response);  
  }  
  //Number of DTC codes 
  BTOBD_serial.println("0101");
  delay(500); read_elm327_response(); Serial.println(raw_ELM327_response);
  WorkingString = raw_ELM327_response.substring(11,13);   //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);              //Convert to integer
  ndtc = A-128;                                           //Apply formula
  Serial.print("Number of DTCs: ");Serial.println(ndtc);  
  
  //Get DTC codes  
  if(ndtc!=0){
    BTOBD_serial.println("03");           //Request Mode 03 (List of DTCs)
    delay(5000); read_elm327_response();  //Check delay
    Serial.print("DTC: ");Serial.println(raw_ELM327_response);
  }
  
  //Get supported PIDs
 
}

void loop(){
/****************************************
 ****************************************
 ****************  LOOP  ****************
 ****************************************
 ****************************************/
  
  //(do not use println in the loop)
  
  //ELM327_enter_terminal_mode();       //Un-comment to access terminal mode for ELM327
  //gprs.serialDebug();                 //Un-comment to access terminal mode for SIM800L

  timestamp = millis();   //Register initial timestamp

  Serial.print("\n");     //Start new line (do not use println in any print of the loop)
  Serial.print(timestamp); 
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
  
  //1 sec A byte
  for(int i=0; i<=2; i++){
  BTOBD_serial.println(pid1s[i]);                       //Send sensor PID
  delay(310);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = A/c1[i];                                         //Apply formula
  Serial.print(s1[i]); Serial.print(var); Serial.print(u1[i]); Serial.print("\t);  //Display value and unit (tabulated)
  }
  //1 sec A and B bytes
  for(int i=3; i<=l2; i++){
  BTOBD_serial.println(pid1s[i]);                         //Send sensor PID
  delay(310);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
  var = (256*A+B)/c1[i];                                //Apply formula
  Serial.print(s1[i]); Serial.print(var); Serial.print(u1[i]); Serial.print("\t);  //Display value and unit (tabulated)
  }
  //30 sec
  for(int i=0; i<=l3; i++){
  BTOBD_serial.println(pid30s[i]);                       //Send sensor PID
  delay(310);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = (A-b30[i])/c30[i];                              //Apply formula
  Serial.print(s30[i]); Serial.print(var); Serial.print(u30[i]); Serial.print("\t);  //Display value and unit (tabulated)
  }
  

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
  
  
  time_elapsed=millis()-timestamp;
  if(time_between_loops>time_elapsed){
    delay(time_between_loops-time_elapsed);
  }

}//loop end


void read_elm327_response(){
  raw_ELM327_response = "";
  while (BTOBD_serial.available()>0){
    inData=0;
    inChar=0;
    inData = BTOBD_serial.read();
    inChar=char(inData);
    raw_ELM327_response = raw_ELM327_response + inChar;
  } 
  //read_elm327_response()
  //Serial.println(raw_ELM327_response);
}


void ELM327_enter_terminal_mode(){
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
}//ELM327_enter_terminal_mode()
