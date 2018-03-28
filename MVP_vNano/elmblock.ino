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

#include <SoftwareSerial.h>

/*******************************
 ****** Runtime Variables ******
 ******************************/

unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 1000; //total time elapsed on each iteration in [ms]
int           time_response       = 300;  //time to wait for response

#define baud_serial0 9600           //Serial inncluded in arduino
#define baud_serial2 38400          //BT-OBD

SoftwareSerial BTOBD_serial(2,3);   //BT-OBD


/********************************
 ****** Temporary Variables *****
 ********************************/

String  raw_ELM327_response = ""; //to save OBD's response after sending commands
byte    inData;                   //to parse data received from OBD
char    inChar;                   //to read OBD's response char-wise
String  WorkingString="";         //to cut substrings from raw_ELM327_response
String  fdig;                     //to cut DTC code first digit
long    A;                        //to save numeric data gotten from control unit's responses
long    B;

/**************************
 ****** Car Variables *****
 **************************/
        
//int     ndtc;                   // Number of DTCs
float   var;                    // elm327 response after equation
uint8_t    i;                      // Used in for
uint8_t    j = 0;                  // 2 sec variable selector
uint8_t    k = 0;                  // Temporary Array element (Speed, RPM) for 10 var
uint8_t    l = 0;                  // Temporary Array element (Coolant temperature, Battery voltage) for 5 var
uint8_t    m = 0;                  // to fill arrays to send
uint8_t    p = 0;                  // 10 element temporary array usage counter
uint8_t		 num = 1;								 // firstdata package
const uint8_t lines PROGMEM= 30;       // For array size, (lines/second)*(time between sendings)
const uint8_t dv PROGMEM= 5;                 // for speed spike filter
const uint16_t dn PROGMEM= 3000;               // for rpm spike filter
const uint8_t dc PROGMEM= 10;                // for coolant temperature spike filter

const float a PROGMEM  = 0.3;               // for exponential filter
const float b PROGMEM = 1-a;

uint8_t     av;                 
uint8_t     bv;
uint16_t     an;
uint16_t     bn;                 // to save last two elements in temporary array, also defined as start point
uint8_t      ac = 30;
uint8_t      bc = 30;
uint8_t      bb = 12;

uint8_t vtemp[10];                // temporary array for speed 
uint16_t ntemp[10];                // temporary array for rpm
int16_t ctemp[5];                 // temporary array for temperature
uint8_t btemp[5];                 // temporary array for voltage
unsigned long tsnd;    // time array to send
uint8_t vsnd[lines];              // speed array to send
uint16_t nsnd[lines];              // rpm array to send
int16_t csnd;                     // coolant temperature to send
uint8_t bsnd;                     // battery voltage to send
unsigned long dsnd = 0;       // trip distance to send in cm

const String pid2[] PROGMEM={"0105", "at rv"};

/*****************************************
 ******** SIM and Cloud Variables ********
 *****************************************/

String snd = "";

void setup() {
  /*****************************************
   *****************************************
   ****************  SETUP  ****************
   *****************************************
   *****************************************/
  
  /*** Begin serial: */
  Serial.begin(baud_serial0);       
  //Serial.println("Initializing Cloud Car Monitor System");
  
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
  //Serial.print("Set protocol: "); Serial.println(raw_ELM327_response); 

  //Number of DTC codes                                     Try 0101 in terminal and see (if) different responses

  while(true){
    BTOBD_serial.println("0101");
    delay(1000); 
    read_elm327_response(); 
    WorkingString = raw_ELM327_response.substring(11,13);   //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);              //Convert to integer
    var = A-128;                                           //Apply formula
    if(var==-128 && raw_ELM327_response.substring(11,13)!="IT"){ //CHECK IN TERMINAL FOR ISO 15765-4
    //if(var==-128 && raw_ELM327_response.substring(11,13)=="00"){ //CHECK IN TERMINAL
      var=0; 
      break;
    }
    else if(var>0){
      break;
    }
  }//while
  //ndtc=3; //Test
  //Serial.print("Number of DTCs: "); Serial.println(ndtc); 
  snd+=String(var)+" DTC";

  //Get DTC codes  
  if(var>0){
    BTOBD_serial.println("03");           //Request Mode 03 (List of DTCs)
    delay(5000); 
    read_elm327_response();  //Check delay
    //delay(7000); 
    
    //Serial.print("DTC: ");Serial.println(raw_ELM327_response); //Raw response
    //Get code from look up table

      //Serial.print("\n");
      for(i = 9; i <= (var*6+3); i = i+6){                  //Go through every first code digit in ISO 15765-4 CAN (11/500)
      //for(i = 6; i <= (var*6); i = i+6){
      WorkingString = raw_ELM327_response.substring(i,(i+1));    //Cut first digit 
      
      //String resp = ">0343 xx 01 33 C0 00 50 00 01 33 C0 00 50 00"; //Test response Add xx in first byte position for ISO 15765-4
      //WorkingString = resp.substring(i,(i+1));                   //Test
      
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
      WorkingString = fdig + raw_ELM327_response.substring((i+1),(i+2)) + raw_ELM327_response.substring((i+3),(i+5));
      //WorkingString = fdig + resp.substring((i+1),(i+2)) + resp.substring((i+3),(i+5));         //Test
      snd+=" "+WorkingString;
      }//for
 
  }//if DTCs>0


//************Make string and give it to promini_sender   
//Serial.println(snd); //(->DELETE!) 
  
char* data = const_cast<char*>(snd.c_str()); //Parse payload to char array
Serial.println(data); //(->DELETE!) 

  
} // end set up

void loop(){
/****************************************
 ****************************************
 ****************  LOOP  ****************
 ****************************************
 ****************************************/
  
  snd="";
        
  timestamp = millis();   //Register initial timestamp
  //Serial.print("\n");     //Start new line (do not use println in any print of the loop)
  //Serial.print(timestamp); 
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
  //vtemp[k]=var;
  vtemp[k]=120;
        
  //RPM    
  BTOBD_serial.println("010C");                         //Send sensor PID for RPM 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response                      
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
  var = (256*A+B)/4;                                    //Apply formula
  //ntemp[k]=var;
  ntemp[k]=5000;
        
  //2 sec
  BTOBD_serial.println(pid2[j]);                           //Send sensor PID for Coolant temp or battery voltage
  delay(time_response);                                    //Wait for the ELM327 to acquire
  read_elm327_response();                                  //Read ELM327's response
  
  if(m>=lines-10){
          
    if(j==0){

      //Coolant temperature
      WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
      A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
      var = A-40;
      ctemp[l] = var;
      }
      else{
    
      //Voltage
      WorkingString = raw_ELM327_response.substring(6,10); //Cut voltage value 
      var = WorkingString.toFloat();
      btemp[l]= var*10;
      l++;
      }
  }//if
  if(m==0){      
    tsnd=int(timestamp/1000)-1; //get timestamp every 1 sec    
  }
  
  m++;  
  k++;
  j++;
  
  if(j==2){
    j=0;
  }  
  
  if(k==10){ //Filled temporary array, filter, transfer to array to send and reset
     
    //SPIKES
    if(num == 0){
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
    }//if num = 0 -> NOT first data package
    
    //Spike Filter rest of the SPEED and RPM array
    for(i=1; i<=8; i++){
      
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
    if(num == 0){
      //SPEED Analizing bv (last element in last array)
      if(((av-bv)>dv && (vtemp[0]==bv) && (vtemp[0]!=0)) || ((bv-av)>dv && (bv==vtemp[0]))){ //added-> && (vtemp[0]!=0)
        bv=av;
      }
      //Analizing first element
      if(((bv-vtemp[0])>dv && (vtemp[1]==vtemp[0]) && (vtemp[1]!=0)) || ((vtemp[0]-bv)>dv && (vtemp[0]==vtemp[1]))){ //added -> && (vtemp[1]!=0)
        vtemp[0]=bv;  
      }
    }//if num = 0 -> NOT first data package
    
    //Filter Plateau in rest of the SPEED array 
    for(i=1; i<=8; i++){
      
      //filter for 10 SPEED data stored
      if(((vtemp[i-1]-vtemp[i])>dv && (vtemp[i+1]==vtemp[i]) && (vtemp[i+1]!=0)) || ((vtemp[i]-vtemp[i-1])>dv && (vtemp[i]==vtemp[i+1]))){
        vtemp[i]=vtemp[i-1]; 
      }//if
    }//for
    
    //Fill array to SEND
		if(num == 0){ // -> not first data package
      vsnd[10*p]=bv;
    	nsnd[10*p]=bn;
    }
    else{
      vsnd[10*p]=vtemp[0];
    	nsnd[10*p]=ntemp[0];      
    }
    
    for(i=1; i<10; i++){
      vsnd[(10*p+i)]=a*float(vsnd[(10*p+i-1)])+b*float(vtemp[i-1]);  // Exponential filter
      //nsnd[(10*p+i)]=a*nsnd[(10*p+i-1)]+b*ntemp[i-1];  //DO NOT USE EXPONENTIAL FILTER IN RPM
      nsnd[(10*p+i)]=ntemp[i-1];
    }
    
  	av=vtemp[8]; 
    bv=vtemp[9];
    an=ntemp[8]; 
    bn=ntemp[9]; 
    
    num = 0;
    
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
    for(i=1; i<5; i++){
      if(ctemp[i]<0){
        ctemp[i]=ctemp[i-1];
      }
      if(btemp[i]<10){ //before: btemp[i]==0
        btemp[i]=btemp[i-1];
      }
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
    for(i=1; i<=3; i++){
      //First filter for 5 coolant temperature data stored
      if(((ctemp[i-1]-ctemp[i])>dc && (ctemp[i+1]-ctemp[i])>dc) || ((ctemp[i]-ctemp[i-1])>dc && (ctemp[i]-ctemp[i+1])>dc)){
        ctemp[i]=(ctemp[i+1]+ctemp[i-1])/2;   
      }

    }//for

    //Variables to send -> AVERAGE (ONLY TAKES FIVE LAST READINGS!!)
    csnd=(bc+ctemp[0]+ctemp[1]+ctemp[2]+ctemp[3])/5;
    bsnd=(bb+btemp[0]+btemp[1]+btemp[2]+btemp[3])/5;

    ac=ctemp[3]; 
    bc=ctemp[4];
    bb=btemp[4];
    
    l=0;
    
  }//if
  
  if(m==lines){       //Filled array in all positions 0 to lines-1 -> write Json -> Send

    snd += String(tsnd);

    snd += ",V";
    for(i=0; i<(lines); i++){
        snd += "," + String(vsnd[i]);
    }
    snd += ",N";
    for(i=0; i<(lines); i++){
        snd += "," + String(nsnd[i]); 
    }
    for(i=1; i<(lines); i++){
        dsnd=dsnd+(((float(vsnd[i])+float(vsnd[i-1]))/7200)*100000); //multplied by 100000 (sent in cm)
    }
    snd += ",D," + String(dsnd);
    snd += ",C," + String(csnd);        
    snd += ",B," + String(bsnd); 
    snd += "#";

    m=0;            //Reset position for re-fill time array


    /************************************
     ****** Send data to Pro mini sender ******
     ************************************/

    char* data = const_cast<char*>(snd.c_str()); //Parse payload to char array
    Serial.println(data); //(->DELETE!) 

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

