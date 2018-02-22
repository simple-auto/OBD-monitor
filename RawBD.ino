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
int           time_between_loops  = 1000; //total time elapsed on each iteration in [ms]

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
String  fdig;                     //to cut DTC code first digit
long    A;                        //to save numeric data gotten from control unit's responses
long    B;
//long    C;

/**************************
 ****** Car Variables *****
 **************************/
        
float   ndtc;
float   var;
int     j = 0; // 30 sec variable selector
int     k = 0; // Array element
int     l = 0; // Array element
const int     lines = 10; //Quantity of data to store in array

String var1[lines*3]; //Array for 1 sec variables
String var30[lines*4]; //Array for 30 sec variables

String pidstart[]={"atz", "at sp 6"}; //Protocol n°6: ISO 15765-4 CAN (11/500)
String start[]={"Reset: ", "Set protocol: "};

String pid1s[]={"010D", "0133", "010C"};
String s1[]={"Speed: ", "Pressure: ", "RPM: "};
String u1[]={" [km/h]", " [kPa]", " [RPM]"};
//String p1[]={13, 51, 12}; //bit position for support check

String pid30s[]={"0105", "0146", "0131", "at rv"};
String s30[]={"Coolant temp: ", "Ambient temp: ", "Distance DTCs: ", "Battery voltage: "};
String u30[]={ " [°C]", " [°C]", " [km]", "[V]"};
//String p30[]={5, 70, 49}; //bit position for support check

//((A-p)*q+B)*r/s not needed

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
  
  //ELM327_enter_terminal_mode();
  
 /*********************
 *** Initialize ELM327
 *********************/  

  //Start 
  for(int i=0; i<=1; i++){             
  BTOBD_serial.println(pidstart[i]);          
  delay(1000); 
  read_elm327_response();
  Serial.print(start[i]); Serial.println(raw_ELM327_response);  
  }

  //Number of DTC codes 
  getnDTC();
  Serial.print("Number of DTCs: "); Serial.println(ndtc); 
  
  //Get DTC codes  
  if(ndtc>0){
    BTOBD_serial.println("03");           //Request Mode 03 (List of DTCs)
    delay(7000); 
    read_elm327_response();  //Check delay
    delay(7000); 
    //Serial.print("DTC: ");Serial.println(raw_ELM327_response); //Raw response
    //Get code from look up table
      for(int j = 9; j <= (ndtc*6+3); j = j+6){                  //Go through every first code digit
      WorkingString = raw_ELM327_response.substring(j,(j+1));    //Cut first digit  
      if(WorkingString == "0"){
          fdig = "P0";        
        }
        else if(WorkingString == "1"){
          fdig = "P1";
        }
        else if(WorkingString == "2"){
          fdig = "P2";
        }
        else if(WorkingString == "3"){
          fdig = "P3";
        }
        else if(WorkingString == "4"){
          fdig = "C0";
        }
        else if(WorkingString == "5"){
          fdig = "C1";
        }
        else if(WorkingString == "6"){
          fdig = "C2";
        }
        else if(WorkingString == "7"){
          fdig = "C3";
        }
        else if(WorkingString == "8"){
          fdig = "B0";
        }
        else if(WorkingString == "9"){
          fdig = "B1";
        }
        else if(WorkingString == "A"){         
          fdig = "B2";
        }
        else if(WorkingString == "B"){
          fdig = "B3";
        }
        else if(WorkingString == "C"){
          fdig = "U0";
        }
        else if(WorkingString == "D"){
          fdig = "U1";
        }
        else if(WorkingString == "E"){
          fdig = "U2";
        }
        else if(WorkingString == "F"){
          fdig = "U3";
        }
      WorkingString = fdig + raw_ELM327_response.substring((j+1),(j+2)) + raw_ELM327_response.substring((j+3),(j+5));
      int n = (j-3)/6;
      Serial.print("DTC #"); Serial.print(n); Serial.print(": "); Serial.println(WorkingString);
      }//for
  }//if
  

} // end set up

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
  
  //30 sec variables requested every 4 seconds (for filter in case of glitches)         
  
  //1 sec A byte (Speed and Pressure)
  for(int i=0; i<=1; i++){
  BTOBD_serial.println(pid1s[i]);                       //Send sensor PID
  delay(240);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response              
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = A;                                              //Apply formula
  Serial.print(s1[i]); Serial.print(var); Serial.print(u1[i]); Serial.print("\t");  //Display value and unit (tabulated)
  var1[k+i]=var;
  }
                                                                            
  //1 sec A and B bytes (RPM)
  BTOBD_serial.println(pid1s[2]);                       //Send sensor PID
  delay(240);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response                              
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
  var = (256*A+B)/4;                                //Apply formula
  Serial.print(s1[2]); Serial.print(var); Serial.print(u1[2]); Serial.print("\t");  //Display value and unit (tabulated)
  var1[k+2]=var;
        
  //30 sec
  BTOBD_serial.println(pid30s[j]);                      //Send sensor PID
  delay(240);                                           //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
  if(j==0 || j==1){
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = A-40;                                           //Apply formula  
  }
  else if(j==2){
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer  
  var = A*256+B;                                        //Apply formula
  }
  else{
  var = raw_ELM327_response.toFloat();
  }
  Serial.print(s30[j]); Serial.print(var); Serial.print(u30[j]); Serial.print("\t");
  var30[l]=var;
        
  j++;
  if(j==4){
  j=0;
  }
  
  k=k+3;
  if(k==lines*3){
  k=0;
  }
        
  l++;
  if(l==lines*4){
  l=0;
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

void getnDTC(){
  while(true){
  BTOBD_serial.println("0101");
  delay(2000); 
  read_elm327_response(); 
  WorkingString = raw_ELM327_response.substring(11,13);   //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);              //Convert to integer
  ndtc = A-128;                                           //Apply formula
    if(ndtc==-128){
      ndtc=0; 
      break;
    }
    else if(ndtc>0){
      break;
    }
  }//while
} //Consider abort (break) if it takes too much time
