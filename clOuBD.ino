/**
 * 
 * Cloud Car Monitor
 *
 * Revo Project revoproject@terralink.cl
 * Terralink SpA,
 * El Director 6000, San Pascual 300, Santiago, Chile
 * 
 * 
 * 
 * Collaborators:
 * Vicente Matus
 * Andres Astudillo
 * Felipe Silva
 * 
 * 
 * OBD-II -- ELM327 -- BT -- BT(HC05) -- NANO -- SIM800L -- SERVER
 * 
 * 
 * Plateau filter, spikes filter and exponential filter (to solve ELM327 glitches)
*/

/***********************
 ****** Libraries ******
 ***********************/

#include <gprs.h>
#include <SoftwareSerial.h>

/*******************************
 ****** Runtime Variables ******
 ******************************/

unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 1000; //total time elapsed on each iteration in [ms]
int           time_response       = 300;  //time to wait for response

#define baud_serial0 9600           //Serial inncluded in arduino
//#define baud_serial1 9600           //SIM
#define baud_serial2 38400          //BT-OBD

//SoftwareSerial SIM_serial(8,7);     //SIM
SoftwareSerial BTOBD_serial(2,3);   //BT-OBD


/********************************
 ****** Temporary Variables *****
 ********************************/

String  raw_ELM327_response = ""; //to save OBD's response after sending commands
byte    inData;                   //to parse data received from OBD
char    inChar;                   //to read OBD's response char-wise
String  WorkingString="";         //to cut substrings from raw_ELM327_response
String  fdig;                     //to cut DTC code first digit
//String  dummy;
long    A;                        //to save numeric data gotten from control unit's responses
long    B;

/**************************
 ****** Car Variables *****
 **************************/
        
int     ndtc;                   // Number of DTCs
float   var;                    // elm327 response after equation
int     j = 0;                  // 2 sec variable selector
int     k = 0;                  // Temporary Array element (Speed, RPM) for 10 var
int     l = 0;                  // Temporary Array element (Coolant temperature, Battery voltage) for 5 var
int     m = 0;                  // to fill arrays to send
int     p = 0;                  // 10 element temporary array usage counter
const int     lines = 30;       // For array size, (lines/second)*(time betwen sendings)
int     dv = 5;                 // for speed spike filter
int     dn = 3000;               // for rpm spike filter
int     dc = 10;                // for coolant temperature spike filter

float   a  = 0.3;               // for exponential filter
float   b  = 1-a;

int     av = 0;                 
int     bv = 0;
int     an = 0;
int     bn = 0;                 // to save last two elements in temporary array, also defined as start point
int     ac = 30;
int     bc = 30;
int     bb = 12;

//float var1[lines*4];          //Array for all variables
int vtemp[10];                // temporary array for speed 
int ntemp[10];                // temporary array for rpm
int ctemp[5];                 // temporary array for temperature
int btemp[5];                 // temporary array for voltage
//float btemp[5];               // temporary array for voltage
unsigned long tsnd[lines];    // time array to send
//int tsnd[lines];            // time array to send
int vsnd[lines];              // speed array to send
int nsnd[lines];              // rpm array to send
int vraw[lines];              // raw speed array 
int nraw[lines];              // raw rpm array 
int csnd;                     // coolant temperature to send
int bsnd;                     // battery voltage to send
//float bsnd;                   // battery voltage to send
unsigned long dsnd = 0;       // trip distance to send in cm

String pid2[]={"0105", "at rv"};
String u2[]={"\t[Celsius]\t", "\t[V]\t"};

/*****************************************
 ******** SIM and Cloud Variables ********
 *****************************************/

String payload = "";
String DTCnd = "";
String V = "";
String N = "";
String T = "";
String D = "";
String C = "";
String Bat = "";

#define DEFAULT_TIMEOUT     5
char server[] = "REPLACE_WHEN_LOADING_TO_ARDUINO";     //Server's address
int port = 00; //REPLACE WHEN LOADING TO ARDUINO
GPRS gprs;                                //SIM808 object
//boolean connectivity = false;             //to attempt connection to the cloud or skip and work oflin
//char buffer[512];

/*******Thingspeak Mode*******
char server[] = "api.thingspeak.com";     //Server's address
String WriteAPIKey = "REPLACE WHEN LOADING TO ARDUINO";  //Thingspeak channel key to write data
String channel_ID = "REPLACE WHEN LOADING TO ARDUINO";             //Thingspeak channel ID
String thingspeak_command = "";           //GET command with fields data (defined after getting OBD data)
*/

/*******Amazon JSON Mode*******
char message_to_server[] = "";           //POST command with fields data (defined after getting OBD data)
char headers[] = "Host: REPLACE WHEN LOADING TO ARDUINO \n Content-Type: application/json \n X-Amz-Date: 20180226T230303Z \n Authorization: AWS4-HMAC-SHA256 Credential=AKIAIZ3AHYRVVR4WPTZA/20180226/us-west-2/execute-api/aws4_request, SignedHeaders=content-length;content-type;host;x-amz-date, Signature=dbbc3cee46af480843b1655cd6c77d081cf8d6e2770f727050f3b0279852633";
char jprueba[] = "{\"arreglo1\": {\"TableName\": \"DataOBD\",\"Item\": {\"dataId\" : \"4\",\"Vel\ocidad\": \"90\",\"RPM\": \"90\",\"Voltaje\": \"14\"}},\"arreglo2\": {\"TableName\": \"DataOBD\",\"Item\": {\"dataId\" : \"5\",\"Velocidad\": \"90\",\"RPM\": \"90\",\"Voltaje\": \"14\"}}}";
*/


void setup() {
  /*****************************************
   *****************************************
   ****************  SETUP  ****************
   *****************************************
   *****************************************/
  
  /*** Begin serial: */
  Serial.begin(baud_serial0);       
  //Serial.println("Initializing Cloud Car Monitor System");
  //SIM_serial.begin(baud_serial1);
  //gprs.serialDebug();
  
  gprs.preInit();
  while(0 != gprs.init()) {
     delay(1000);
     //Serial.println("2G initialization error. Check SIM card.");
  }
  
  
  gprs.sendCmdAndWaitForResp("AT+CFUN=1\r\n","OK",DEFAULT_TIMEOUT);
  delay(1000);
  gprs.sendCmdAndWaitForResp("AT+CGATT?\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIPSTATUS\r\n");
  delay(1000);
  //REPLACE APN, USERNAME, AND PASSWORD
  gprs.sendCmd("AT+CSTT=\"WEB.TMOVIL.CL\",\"WEB\",\"WEB\"\r\n");
  //gprs.sendCmd("AT+CSTT=\"BAM.ENTELPCS.CL\",\"ENTELPCS\",\"ENTELPCS\"\r\n");
  delay(1000);
  gprs.sendCmdAndWaitForResp("AT+CIICR\r\n","OK",DEFAULT_TIMEOUT);
  gprs.sendCmd("AT+CIFSR\r\n");
  delay(1000);
  //char* IP = gprs.getIPAddress();

  Serial.println("2G Initialized.");
  gprs.serialSIM800.end();
  //Serial.println(IP);
  
 /******************************
 *** Initialize Bluetooth HC 05
 ******************************/  
  BTOBD_serial.begin(baud_serial2);
  //Serial.println("Bluetooth communication to ELM327 Initialized");
  
  //ELM327_enter_terminal_mode();
  
 /*********************
 *** Initialize ELM327
 *********************/  
 
  //Start 
  
  while(true){
  BTOBD_serial.println("atz");
  delay(2000);
  read_elm327_response();
  if(raw_ELM327_response.substring(4,5)!="?"){
  break;
  }
  }
  //Serial.print("Reset: "); Serial.println(raw_ELM327_response); 

  while(true){
  BTOBD_serial.println("at sp 6"); //Protocol n°6: ISO 15765-4 CAN (11/500)
  delay(2000);
  read_elm327_response();
  if(raw_ELM327_response.substring(8,9)!="?"){
  break;
  }
  }
  Serial.print("Set protocol: "); Serial.println(raw_ELM327_response); 

  //Number of DTC codes                                     Try 0101 in terminal and see (if) different responses

  while(true){
  BTOBD_serial.println("0101");
  delay(1000); 
  read_elm327_response(); 
    WorkingString = raw_ELM327_response.substring(11,13);   //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);              //Convert to integer
  ndtc = A-128;                                           //Apply formula
    if(ndtc==-128 && raw_ELM327_response.substring(11,13)!="IT"){ //CHECK IN TERMINAL FOR ISO 15765-4
      ndtc=0; 
      break;
    }
    else if(ndtc>0){
      break;
    }
  }//while
  //ndtc=3; //Test
  Serial.print("Number of DTCs: "); Serial.println(ndtc); 
  DTCnd+="Num DTCs: "+ndtc;

  //Get DTC codes  
  if(ndtc>0){
    //String DTC[ndtc];
    BTOBD_serial.println("03");           //Request Mode 03 (List of DTCs)
    delay(5000); 
    read_elm327_response();  //Check delay
    //delay(7000); 
    
    //Serial.print("DTC: ");Serial.println(raw_ELM327_response); //Raw response
    //Get code from look up table

      Serial.print("\n");
      for(int j = 9; j <= (ndtc*6+3); j = j+6){                  //Go through every first code digit in ISO 15765-4 CAN (11/500)
      //for(int j = 6; j <= (ndtc*6); j = j+6){
      WorkingString = raw_ELM327_response.substring(j,(j+1));    //Cut first digit 
      
      //String resp = ">0343 xx 01 33 C0 00 50 00 01 33 C0 00 50 00"; //Test response Add xx in first byte position for ISO 15765-4
      //WorkingString = resp.substring(j,(j+1));                   //Test
      
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
      //WorkingString = fdig + resp.substring((j+1),(j+2)) + resp.substring((j+3),(j+5));         //Test
      int n = (j-3)/6; //ISO 15765-4 CAN
      //int n = (j)/6; //KWP Fast
      //Serial.print("DTC #"); Serial.print(n); Serial.print(": "); Serial.println(WorkingString);
      DTCnd+="\t"+WorkingString;
      //DTC[n-1]=WorkingString;
      }//for
  /*
  //SEND DTCs
  BTOBD_serial.end();  
  
  GPRS gprs;
  delay(1000);
  if(0 == gprs.connectTCP(server, port)){
      //Serial.print("Connect successfuly to ");
      //Serial.println(server);
  }
  else{
      //Serial.println("connect error");
  }
  payload = DTCnd;
  char* tcp_payload = const_cast<char*>(payload.c_str()); //Parse payload to char array
  Serial.println("Message to server=\n"+payload);

  if(0 == gprs.sendTCPData(tcp_payload)){
    //gprs.serialDebug();
    //char fin_get[] = "CLOSED\n";
    //gprs.waitForResp(fin_get,5);
    Serial.println("\nSent");
  }
  else{
    Serial.println("\nNot sent");
  }

  gprs.serialSIM800.end();
  delay(1000);
  BTOBD_serial.begin(baud_serial2);
  */
          
  //Show DTCs stored in array -> Json -> Send
  /*
  //Create json object
  StaticJsonBuffer<100> jsonBuffer; //Check size
  JsonObject& root = jsonBuffer.createObject();
  //JsonArray& codes = root.createNestedArray("codes");
  
  for(int i=0; i<ndtc; i++){
  /*Serial.print(DTC[i]); Serial.print(";");
  delay(1);
  codes.add(DTC[i]);

  dummy="code "+String(i+1);
  JsonObject& code = root.createNestedObject(dummy);
  //code["TN"]= "DataOBD";
  code["DTC"]=DTC[i];
  }
  
  Serial.print("\n");
  root.prettyPrintTo(Serial); //****(TO SEND)****
  Serial.print("\n");
  */
  }//if

} // end set up

void loop(){
/****************************************
 ****************************************
 ****************  LOOP  ****************
 ****************************************
 ****************************************/
  
  //ELM327_enter_terminal_mode();       //Un-comment to access terminal mode for ELM327
  //gprs.serialDebug();                 //Un-comment to access terminal mode for SIM800L

  timestamp = millis();   //Register initial timestamp
  Serial.print("\n");     //Start new line (do not use println in any print of the loop)
  Serial.print(timestamp); 
  //Serial.print("\t[ms]\t");

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
  //1 sec
  
  //Speed
  BTOBD_serial.println("010D");                         //Send sensor PID for speed 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response              
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = A;                                              //Apply formula
  //Serial.print(var); Serial.print("\t[km/h]\t");        //Display value and unit (tabulated)
  vtemp[k]=var;
  
  //RPM    
  BTOBD_serial.println("010C");                         //Send sensor PID for RPM 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response                      
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
  var = (256*A+B)/4;                                    //Apply formula
  //Serial.print(var); Serial.print("\t[RPM]\t");               //Display value and unit (tabulated)
  ntemp[k]=var;

  //2 sec
  BTOBD_serial.println(pid2[j]);                           //Send sensor PID for Coolant temp or battery voltage
  delay(time_response);                                    //Wait for the ELM327 to acquire
  read_elm327_response();                                  //Read ELM327's response
  
  if(j==0){

  //Coolant temperature
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  var = A-40;
  ctemp[l] = var;
  }
  else if(j==1){
    
  //Voltage
  WorkingString = raw_ELM327_response.substring(6,10); //Cut voltage value 
  var = WorkingString.toFloat();
  btemp[l]= var*10;
  l++;
  }
  //Serial.print(var); Serial.print(u2[j]);
  
  tsnd[m]=int(timestamp/1000)-1; //get timestamp every 1 sec    
  m++;
       
  k++;

  j++;
  if(j==2){
  j=0;
  }  
  
  if(k==10){ //Filled temporary array, filter, transfer to array to send and reset
    
    for(int i=0; i<10; i++){
    vraw[(10*p+i)]=vtemp[i];
    nraw[(10*p+i)]=ntemp[i];
    }
  
    //SPIKES
    //SPEED Analizing bv (last element in last array)
    if(((av-bv)>dv && (vtemp[0]-bv)>dv) || ((bv-av)>dv && (bv-vtemp[0])>dv)){
    bv=(vtemp[0]+av)/2;
    }
    //Analizing first element
    if(((bv-vtemp[0])>dv && (vtemp[1]-vtemp[0])>dv) || ((vtemp[0]-bv)>dv && (vtemp[0]-vtemp[1])>dv)){
    vtemp[0]=(vtemp[1]+bv)/2;  
    }
    
    //RPM Analizing bn (last element in last array)
    if(((an-bn)>dn && (ntemp[0]-bn)>dn) || ((bn-an)>dn && (bn-ntemp[0])>dn)){
    bn=(ntemp[0]+an)/2;
    }
    //Analizing first element
    if(((bn-ntemp[0])>dn && (ntemp[1]-ntemp[0])>dn) || ((ntemp[0]-bn)>dn && (ntemp[0]-ntemp[1])>dn)){
    ntemp[0]=(ntemp[1]+bn)/2;  
    }

    //Spike Filter rest of the SPEED and RPM array
    for(int i=1; i<=8; i++){
      
    //First filter for 10 SPEED data stored
    if(((vtemp[i-1]-vtemp[i])>dv && (vtemp[i+1]-vtemp[i])>dv) || ((vtemp[i]-vtemp[i-1])>dv && (vtemp[i]-vtemp[i+1])>dv)){
    vtemp[i]=(vtemp[i+1]+vtemp[i-1])/2; 
    }//if

    //First filter for 10 RPM data stored
    if(((ntemp[i-1]-ntemp[i])>dn && (ntemp[i+1]-ntemp[i])>dn) || ((ntemp[i]-ntemp[i-1])>dn && (ntemp[i]-ntemp[i+1])>dn)){
    ntemp[i]=(ntemp[i+1]+ntemp[i-1])/2;  
    }//if
    
    }//for

    //PLATEAU
    //SPEED Analizing bv (last element in last array)
    if(((av-bv)>dv && (vtemp[0]==bv) && (vtemp[0]!=0)) || ((bv-av)>dv && (bv==vtemp[0]))){ //added-> && (vtemp[0]!=0)
    bv=av;
    }
    //Analizing first element
    if(((bv-vtemp[0])>dv && (vtemp[1]==vtemp[0]) && (vtemp[1]!=0)) || ((vtemp[0]-bv)>dv && (vtemp[0]==vtemp[1]))){ //added -> && (vtemp[1]!=0)
    vtemp[0]=bv;  
    }

    //Filter Plateau in rest of the SPEED array 
    for(int i=1; i<=8; i++){
      
    //filter for 10 SPEED data stored
    if(((vtemp[i-1]-vtemp[i])>dv && (vtemp[i+1]==vtemp[i]) && (vtemp[i+1]!=0)) || ((vtemp[i]-vtemp[i-1])>dv && (vtemp[i]==vtemp[i+1]))){
    vtemp[i]=vtemp[i-1]; 
    }//if
    }//for
    
    //Fill array to SEND

    vsnd[10*p]=bv;
    nsnd[10*p]=bn;
    for(int i=1; i<10; i++){
    vsnd[(10*p+i)]=a*vsnd[(10*p+i-1)]+b*vtemp[i-1];  // Exponential filter
    //nsnd[(10*p+i)]=a*nsnd[(10*p+i-1)]+b*ntemp[i-1];  //DO NOT USE EXPONENTIAL FILTER IN RPM
    nsnd[(10*p+i)]=ntemp[i-1];
    }
    
    av=vtemp[8]; 
    bv=vtemp[9];
    an=ntemp[8]; 
    bn=ntemp[9]; 
       
    k=0;
  
    p++;
    if(p==3){ //=lines/(temporary array size)
    p=0;
    }   
  
  }//if

  if(l==5){ //Filled temporary array, filter, transfer to send array and reset

    if(ctemp[0]<0){
    ctemp[0]=bc;
    }
    if(btemp[0]<10){ //before: btemp[0]==0
    btemp[0]=bb;
    }
    for(int i=1; i<5; i++){
    if(ctemp[i]<0){
      ctemp[i]=ctemp[i-1];
    }//if
    if(btemp[i]<10){ //before: btemp[i]==0
      btemp[i]=btemp[i-1];
    }//if
    }//for

    //SPIKE Filter (Coolant temperature)
    //COOLANT temperature Analizing bc (last element in last array)
    if(((ac-bc)>dc && (ctemp[0]-bc)>dc) || ((bc-ac)>dc && (bc-ctemp[0])>dc)){
    bc=(ctemp[0]+ac)/2;
    }
    //Analizing first element
    if(((bc-ctemp[0])>dc && (ctemp[1]-ctemp[0])>dc) || ((ctemp[0]-bc)>dc && (ctemp[0]-ctemp[1])>dc)){
    ctemp[0]=(ctemp[1]+bc)/2;  
    }
    
    //Filter rest of the array
    for(int i=1; i<=3; i++){
    //First filter for 5 coolant temperature data stored
    if(((ctemp[i-1]-ctemp[i])>dc && (ctemp[i+1]-ctemp[i])>dc) || ((ctemp[i]-ctemp[i-1])>dc && (ctemp[i]-ctemp[i+1])>dc)){
    ctemp[i]=(ctemp[i+1]+ctemp[i-1])/2;   
    }//if

    }//for

    //Variables to send -> AVERAGE (ONLY TAKES FIVE LAST READINGS!!)
    csnd=(bc+ctemp[0]+ctemp[1]+ctemp[2]+ctemp[3])/5;
    bsnd=(bb+btemp[0]+btemp[1]+btemp[2]+btemp[3])/5;

    ac=ctemp[3]; 
    bc=ctemp[4];
    bb=btemp[4];
    
    l=0;
    
  }//if
  
  if(m==(lines)){       //Filled array in all positions 0 to lines-1 -> write Json -> Send
  T = "\tT\t";
  for(int i=0; i<(lines); i++){
      T += tsnd[i] + " "; 
  }
  V = "\tV\t";
  for(int i=0; i<(lines); i++){
      V += vsnd[i] + " ";
  }
  N = "\tN\t";
  for(int i=0; i<(lines); i++){
      N += nsnd[i] + " "; 
  }
  for(int i=1; i<(lines); i++){
      dsnd=dsnd+(((float(vsnd[i])+float(vsnd[i-1]))/7200)*100000); //multplied by 100000 (sent in cm)
  }
  D = "\tD\t"+dsnd;
  C = "\tC\t"+csnd;        
  Bat = "\tB\t"+bsnd; 
          
  m=0;            //Reset position for re-fill time array
   
    
  /************************************
   ****** Send data to the cloud ******
   ************************************/
  BTOBD_serial.end();  
  
  GPRS gprs;
  delay(1000);
  if(0 == gprs.connectTCP(server, port)){
      //Serial.print("Connect successfuly to ");
      //Serial.println(server);
  }
  else{
      //Serial.println("connect error");
  }
  payload = V+N+T+C+Bat;
  char* tcp_payload = const_cast<char*>(payload.c_str()); //Parse payload to char array
  Serial.println("Message to server=\n"+payload);
          
  //thingspeak_command = ("GET /update?api_key="+WriteAPIKey+"&field1="+rpm+"&field2="+veloc+"    HTTP/1.0\r\n\r\n");
  //Serial.println("command="+thingspeak_command);
  //char* http_cmd = const_cast<char*>(thingspeak_command.c_str()); //Parse command to char array

  if(0 == gprs.sendTCPData(tcp_payload)){
    //gprs.serialDebug();
    //char fin_get[] = "CLOSED\n";
    //gprs.waitForResp(fin_get,5);
    Serial.println("\nSent");
  }
  else{
    Serial.println("\nNot sent");
  }

  gprs.serialSIM800.end();
  delay(1000);
  BTOBD_serial.begin(baud_serial2);
  }//if
        
  else{
  time_elapsed=millis()-timestamp;
  if(time_between_loops>time_elapsed){
    delay(time_between_loops-time_elapsed);
  }
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

/*
void ELM327_enter_terminal_mode(){
  /**Debug HC05 Bluetooth and ELM327 OBD
   * *  Captures serial2 data and writes input to it.
  *
  while(true){
    if (BTOBD_serial.available()){
      Serial.write(BTOBD_serial.read());
    }
    if (Serial.available()){
      BTOBD_serial.write(Serial.read());
    } //if
  }//while
}//ELM327_enter_terminal_mode()
*/
