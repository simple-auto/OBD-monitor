/**
 * 
 * Updated ELMBlock 02/07/18
 * Arduino connected to Bluetooth and GPS
 * 
 * Always check:
 * Protocol number AT SP [number] 
 * Bluetooth baud rate 
 * SenderSerial(rx,tx) pins
 * 
 * Cloud Car Monitor
 *
 * Revo Project revoproject@terralink.cl
 * Terralink SpA,
 * El Director 6000, San Pascual 397, Santiago, Chile
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
#include <TinyGPS.h>
#include <AltSoftSerial.h>


/*******************************
 ****** Runtime Variables ******
 ******************************/

unsigned long timestamp           = 0;    //to measure time elapsed getting OBD data
unsigned long time_elapsed        = 0;    //to measure time elapsed getting OBD data
int           time_between_loops  = 1000; //total time elapsed on each iteration in [ms]
int           time_response       = 300;  //time to wait for response

long lat,lon; // create variable for latitude and longitude object
int alt;      //altitude
    
String pos;   //store position

#define baud_serial0 9600                 //Sender Serial
//#define baud_serial2 38400              //BT-OBD
#define baud_serial2 9600                 //BT-OBD

//SoftwareSerial BTOBD_serial(2,3);       //BT-OBD
SoftwareSerial senderSerial(2,3);         //Pro mini sender RX TX
AltSoftSerial gpsSerial;
TinyGPS gps;


/********************************
 ****** Temporary Variables *****
 ********************************/

String  raw_ELM327_response = "";         //to save OBD's response after sending commands
byte    inData;                           //to parse data received from OBD
char    inChar;                           //to read OBD's response char-wise
String  WorkingString="";                 //to cut substrings from raw_ELM327_response
String  fdig;                             //to cut DTC code first digit
long    A;                             //to save numeric data gotten from control unit's responses
long    B;                             //to save numeric data gotten from control unit's response

/**************************
 ****** Car Variables *****
 **************************/
        
float      var;                           // elm327 response after equation
uint8_t    i;                             // Used in for
uint8_t    j = 0;                         // 2 sec variable selector
uint8_t    k = 0;                         // Temporary Array element (Speed, RPM) for 10 var
uint8_t    l = 0;                         // Temporary Array element (Coolant temperature, Battery voltage) for 5 var
uint16_t   m = 0;                         // 
uint8_t    p = 0;                         // 
//uint8_t    num = 1;                       // first data package
const uint8_t lines PROGMEM = 30;         // For array size, (lines/second)*(time between sendings)
//const uint8_t dv PROGMEM = 8;             // for speed spike filter
//const uint16_t dn PROGMEM = 400;          // for rpm spike filter
const uint8_t dc PROGMEM = 10;            // for coolant temperature spike filter

//const float a PROGMEM  = 0.3;             // for exponential filter
//const float b PROGMEM = 1-a;

//uint8_t      av;                 
//uint8_t      bv;
//uint16_t     an;
//uint16_t     bn;                          // to save last two elements in temporary array, also defined as start point
uint8_t      ac = 30;
uint8_t      bc = 30;
uint8_t      bb = 120;

int16_t ctemp[5];                         // temporary array for temperature
uint8_t btemp[5];                         // temporary array for voltage
unsigned long tsnd;                       // time array to send
uint16_t vsnd[lines];                      // speed array to send
uint16_t nsnd[lines];                     // rpm array to send
int16_t csnd;                             // coolant temperature to send
uint8_t bsnd;                             // battery voltage to send
unsigned long dsnd = 0;                   // trip distance to send in m

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
  Serial.begin(baud_serial2);               // BLUETOOTH when connected to hardware serial    
  //Serial.println("Initializing Cloud Car Monitor System");
  senderSerial.begin(baud_serial0);         // Serial to the other Pro mini
  
 /******************************
 *** Initialize Bluetooth HC 05
 ******************************/  
  //BTOBD_serial.begin(baud_serial2);
  //Serial.println("Bluetooth communication to ELM327 Initialized");
  
  //ELM327_enter_terminal_mode();

  /*******
   * GPS *
   *******/
  gpsSerial.begin(9600);
  
 /*********************
 *** Initialize ELM327
 *********************/  
  
  //Start 
  
  while(true){
    
    //BTOBD_serial.println("atz");
    Serial.println("atz");
    delay(1000);
    read_elm327_response();
    //Serial.println("ATZ: "+raw_ELM327_response);
    if(raw_ELM327_response.substring(4,5)!="?"){
      break;
    }
  }
  //Serial.print("Reset: "); Serial.println(raw_ELM327_response); 
  //snd+=raw_ELM327_response;


  while(true){
    //BTOBD_serial.println("at sp 0");          //3: ISO 9141-2; 5: ISO 14230-4 KWP fast; 6: ISO 15765-4
    Serial.println("at sp 0");              //Protocol n°6: ISO 15765-4 CAN (11/500)
    delay(1000);
    read_elm327_response();
    //Serial.println("SP: "+raw_ELM327_response);
    if(raw_ELM327_response.substring(8,10)=="OK"){
      break;
    }
  }
  //Serial.print("Set protocol: "); Serial.println(raw_ELM327_response); 

/*
//Try almost all protocols option 1: at sp n and check response
  while(true){
    for(i=1; i<=6; i++){
      WorkingString="at sp "+String(i);
      Serial.println(WorkingString);              //Protocol n°6: ISO 15765-4 CAN (11/500)
      //BTOBD_serial.println("WorkingString");          //3: ISO 9141-2; 5: ISO 14230-4 KWP fast; 6: ISO 15765-4
      delay(1000);
      print 0101
      read response
      if(response length<1){
        //
      }
      read_elm327_response();
      //Serial.println("SP: "+raw_ELM327_response);
      if(raw_ELM327_response.substring(8,10)=="OK"){
        break;
      }  
    }
  }
//finish trying all protocols
*/

/*
//Try almost all protocols option 2: at sp 0 and at dpn gets the protocol number
  while(true){
      Serial.println("at sp 0);              //Protocol n°6: ISO 15765-4 CAN (11/500)
      //BTOBD_serial.println("WorkingString");          //3: ISO 9141-2; 5: ISO 14230-4 KWP fast; 6: ISO 15765-4
      delay(1000);
      read_elm327_response();
      //Serial.println("SP: "+raw_ELM327_response);
      if(raw_ELM327_response.substring(8,10)=="OK"){
        break;
      }  
  }
//finish trying all protocols
*/

  //Number of DTC codes      
  while(true){
    //BTOBD_serial.println("0101");
    Serial.println("0101");
    delay(2000); 
    read_elm327_response(); 
    //Serial.println("NDTC: "+raw_ELM327_response);
    for(i=0; i<raw_ELM327_response.length(); i++){
        delay(1);
        if(raw_ELM327_response.substring(i,i+1)=="N" || raw_ELM327_response.substring(i,i+1)=="?" || raw_ELM327_response.substring(i,i+1)=="R"){
          j=1;
        }//if invalid data 
    }//for Checking all response characters    
        
    //here exit loop
    if(j==0){
      B = raw_ELM327_response.length();
      i = 11;
      while(i < B){
        WorkingString = raw_ELM327_response.substring(i,i+2);        //Cut A Byte value
        if(B>22 && WorkingString!="00"){
          break;
        }
        i = i+18;       
      }//while looking for real ndtc      
      
      //WorkingString = raw_ELM327_response.substring(11,13);           //Cut A Byte value
      //WorkingString = resp.substring(11,13);                        //Cut A Byte value
      
      A = strtol(WorkingString.c_str(),NULL,16);                      //Convert to integer
      var = A-128;                                                    //Apply formula
      //if(var==-128 && WorkingString!="IT")
      if(var==-128){    //CHECK IN TERMINAL FOR ISO 15765-4
      //if(var==-128 && raw_ELM327_response.substring(11,13)=="00"){  //CHECK IN TERMINAL
        var=0; 
        break;
      }
      else if(var>0){
        break;
      }
    }//if j=0 -> valid data

    j=0;               
  }//while true
  
  B=var;
  //B=1; //TEST
  /*
  if(B<10){
    snd+= "00" + String(B); //ANN-> A: type of packet, NN: Number of DTCs
  }
  else{
    snd+= "0" + String(B); //ANN
  }
  */
  if(B==0){
    snd+="000.&";                    //ANN -> A: type 0; NN = 00 DTCs "." end of message
  }
  else{                             //Get DTC codes  
    //BTOBD_serial.println("03");
    Serial.println("03");           //Request Mode 03 (List of DTCs)
    delay(3000); 
    read_elm327_response();         //Check delay
    
    //Serial.print("DTC: ");Serial.println(raw_ELM327_response); //Raw response
    
    if(B>1){
      snd += "3 " + raw_ELM327_response + ".&";                //A type 3 -> raw data
    }  
    else{ //B = 1
          //for(i = 6; i <= (B*6); i = i+6){                  //KWP Fast
          
          //raw_ELM327_response =">0343 00 43 01 20 BD 43 00";  //TEST
          /*if(atdp=6){
            TODO lo a continuación
          }
          */
          i = 6;
          while(i <= raw_ELM327_response.length()){
            if(raw_ELM327_response.substring(i,i+2)=="01"){
              WorkingString = raw_ELM327_response.substring(i+3,i+4);        //Cut DTC first digit
              break;
            }
                  
            i = i+6;       
          }//while
          
          //Data Types ISO 15765-4
          //String resp =">0343 01 20 BD 43 00 43 00";                //Test response ISO 15765-4
          //String resp =">0343 00 43 01 20 BD 43 00";                //Test
          //String resp =">0343 00 43 00 43 01 20 BD";                //Test
          //WorkingString = resp.substring(i,(i+1));                  //Test
          /*
           * if(nuber of DTCs>1 -> Send raw data)
           * else [ndtc=1] -> recorrer string hasta encontrar el 01 y leer próximos 2 bytes)
           */
    
          //WorkingString = raw_ELM327_response.substring(i,(i+1));    //Cut first digit 
          //WorkingString = raw_ELM327_response.substring(9,10);    //Cut first digit 
          
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
        WorkingString = fdig + raw_ELM327_response.substring((i+4),(i+5)) + raw_ELM327_response.substring((i+6),(i+8));
        //WorkingString = fdig + resp.substring((i+1),(i+2)) + resp.substring((i+3),(i+5));         //Test
        snd+="001" + WorkingString + ".&";
      //}//for
    }//else
 
  }//else DTCs>0

delay(20000); //wait for sim800l to initialize
//************Make string and give it to promini_sender   
  
char* data = const_cast<char*>(snd.c_str()); //Parse payload to char array
senderSerial.println(data); 
delay(1000);
//Serial.println(data);
//senderSerial.flush();

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
  
  //BTOBD_serial.println("010D");
  Serial.println("010D");                               //Send sensor PID for speed 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
    
  if(raw_ELM327_response.substring(0,4)!="010D"){
    vsnd[k]=999;    
  }
  else{
    WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
    var = A;                                              //Apply formula
    vsnd[k]=var;

      for(i=0; i<raw_ELM327_response.length(); i++){
        //Serial.println(raw_ELM327_response.substring(i,i+1));
        if(raw_ELM327_response.substring(i,i+1)=="N" || raw_ELM327_response.substring(i,i+1)=="?"){   //Just in case for black dongle
        //if(raw_ELM327_response.substring(i,i+1)=="N" || raw_ELM327_response.substring(i,i+1)=="?" || raw_ELM327_response.substring(i,i+1)=="R"){
         delay(1);
         vsnd[k]=999;
        }
        delay(1);
      }

  }//else
    /*
    Serial.print(raw_ELM327_response);
    Serial.print("\t");
    Serial.print(vsnd[k]);    
    Serial.print("\t");  
    //raw+=String(vsnd[k]);            
    */  
  //RPM    
  
  //BTOBD_serial.println("010C");
  Serial.println("010C");                               //Send sensor PID for RPM 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response 
    
  if(raw_ELM327_response.substring(0,4)!="010C"){
    nsnd[k]=999;
  }         
  else{
    WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
    A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
    WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
    B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
    var = (256*float(A)+float(B))/4;                                    //Apply formula
    nsnd[k]=var/10;
    
      for(i=0; i<raw_ELM327_response.length(); i++){
        //Serial.println(raw_ELM327_response.substring(i,i+1));
        //if(raw_ELM327_response.substring(i,i+1)=="N" || raw_ELM327_response.substring(i,i+1)=="?" || raw_ELM327_response.substring(i,i+1)=="R"){
        if(raw_ELM327_response.substring(i,i+1)=="N" || raw_ELM327_response.substring(i,i+1)=="?"){
          delay(1);
          nsnd[k]=999;    
        }
        delay(1);
      }

  }
    //Serial.print(raw_ELM327_response);    
    //Serial.print("\t");
    /*
    if(k>=lines-10){
    Serial.print(nsnd[k]); 
    Serial.print("\t");
    }
    else{
    Serial.println(nsnd[k]);
    }  
    //raw+=","+String(nsnd[k])+",";
    */
  //MAF
  /*
  Serial.println("0110");                               //Send sensor PID for MAF 
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response                      
  WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
  A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
  WorkingString = raw_ELM327_response.substring(14,16); //Cut B Byte value  
  B = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer    
  var = (256*A+B)/100;                                    //Apply formula
  ftemp[k]=var;
  */
        
  //2 sec

  if(k>=lines-10){

  //BTOBD_serial.println(pid2[j]);
  Serial.println(pid2[j]);                              //Send sensor PID for Coolant temp or battery voltage
  delay(time_response);                                 //Wait for the ELM327 to acquire
  read_elm327_response();                               //Read ELM327's response
            
    if(j==0){

      //Coolant temperature
      WorkingString = raw_ELM327_response.substring(11,13); //Cut A Byte value
      A = strtol(WorkingString.c_str(),NULL,16);            //Convert to integer
      var = A-40;
      ctemp[l] = var;
      /*
      //Serial.print("l: "+String(l));
      Serial.print(raw_ELM327_response);
      Serial.print("\t");
      Serial.println(ctemp[l]);
      */
    }
    else{
    
      //Voltage
      WorkingString = raw_ELM327_response.substring(6,10); //Cut voltage value 
      var = WorkingString.toFloat();
      btemp[l]= var*10;
      /*
      Serial.print(raw_ELM327_response);
      Serial.print("\t");
      Serial.println(btemp[l]);
      */
      l++;
    }
  }//if

/*  
  else{
    Serial.println("");
  }
*/

  if(k==0){      
    tsnd=int(timestamp/1000)-1; //get timestamp every 1 sec    
  }
  
  k++;
  j++;
  
  if(j==2){
    j=0;
  }  
  
  if(l==5){ //Filled temporary array, filter, transfer to send array and reset

    if(ctemp[0]<0){
      ctemp[0] = bc;
    }
    if(btemp[0]<10){ //before: btemp[0]==0
      btemp[0] = bb;
    }
    for(i=1; i<5; i++){
      if(ctemp[i]<0){
        ctemp[i] = ctemp[i-1];
      }
      if(btemp[i]<10){ //before: btemp[i]==0
        btemp[i] = btemp[i-1];
      }
    }//for

    //SPIKE Filter (Coolant temperature)
    //COOLANT temperature Analizing bc (last element in last array)
    if(((ac-bc)>dc && (ctemp[0]-bc)>dc) || ((bc-ac)>dc && (bc-ctemp[0])>dc)){
      bc = (ctemp[0]+ac)/2;
    }
    //Analizing first element
    if(((bc-ctemp[0])>dc && (ctemp[1]-ctemp[0])>dc) || ((ctemp[0]-bc)>dc && (ctemp[0]-ctemp[1])>dc)){
      ctemp[0] = (ctemp[1]+bc)/2;  
    }
    
    //Filter rest of the array
    for(i=1; i<=3; i++){
      //First filter for 5 coolant temperature data stored
      if(((ctemp[i-1]-ctemp[i])>dc && (ctemp[i+1]-ctemp[i])>dc) || ((ctemp[i]-ctemp[i-1])>dc && (ctemp[i]-ctemp[i+1])>dc)){
        ctemp[i] = (ctemp[i+1]+ctemp[i-1])/2;   
      }

    }//for

    //Variables to send -> AVERAGE (ONLY TAKES FIVE LAST READINGS!!)
    csnd = (bc+ctemp[0]+ctemp[1]+ctemp[2]+ctemp[3])/5;
    bsnd = (bb+btemp[0]+btemp[1]+btemp[2]+btemp[3])/5;

    ac = ctemp[3]; 
    bc = ctemp[4];
    bb = btemp[4];
    
    l=0;
    
  }//if l=5
  
  if(k==lines){       //Filled array in all positions 0 to lines-1 -> write Json -> Send

    snd = "1"; //type processed data
        
    for(i=0; i<(lines); i++){
      if(vsnd[i]<10){
        snd += "00" + String(vsnd[i]);
      }
      else if(vsnd[i]<100){
        snd += "0" + String(vsnd[i]);
      }
      else{
        
        //Filter 999s
        if(vsnd[i]==999){
            if(i==0){
              //look forward
              //999|000|...
              // ^
              if(vsnd[i+1]!=999){
                vsnd[i]=vsnd[i+1];
              }
              //999|999|000|...
              // ^
              else if(vsnd[i+1]==999 && vsnd[i+2]!=999){
                vsnd[i]=vsnd[i+2];
              }
              //999|999|999|...
              // ^
              else if(vsnd[i+1]==999 && vsnd[i+2]==999){
                vsnd[i]=666;
              }
            }
            else if(i==28){
              //...|999|000
              //     ^
              if(vsnd[i+1]!=999 && vsnd[i-1]!=666){ //vsnd[i-1]!=666 redundante
                vsnd[i]=(vsnd[i-1]+vsnd[i+1])/2;  
              }
              //...|999|999
              //     ^
              else if(vsnd[i+1]==999){ 
                vsnd[i]=666;
              }    
            }          
            else if(i==29){
              //look backwards
              //...|999
              //     ^
                vsnd[i]=vsnd[i-1];        
            }
            //from 1 to 27
            //look back and forward
            //...|000|999|000|...
            //         ^
            else if(vsnd[i+1]!=999 && vsnd[i-1]!=666){ //vsnd[i-1]!=666 redundante
              vsnd[i]=(vsnd[i-1]+vsnd[i+1])/2;
            }
            //...|000|999|999|000|...
            //         ^
            else if(vsnd[i+1]==999 && vsnd[i+2]!=999 && vsnd[i-1]!=666){ //vsnd[i-1]!=666 redundante
              vsnd[i]=(vsnd[i-1]+vsnd[i+2])/2;
            }
            //...|000|999|999|999|...
            //         ^
            else if(vsnd[i+1]==999 && vsnd[i+2]==999){ //vsnd[i-1]!=666 redundante
              vsnd[i]=666;
            }
            //...|666|999|999|000|...
            //         ^
            else if(vsnd[i-1]==666 && vsnd[i+2]!=999){ 
              vsnd[i]=vsnd[i+2];
            }
            //...|666|999|999|999|...
            //         ^
            else if(vsnd[i-1]==666 && vsnd[i+2]==999){ 
              vsnd[i]=666;
            }
    
        //End Filter 999s
        }
        
        if(vsnd[i]==666){
          snd += "NaN";  
        }
        else{
          snd += String(vsnd[i]);  
        }
      }//else
    }// for

    for(i=0; i<(lines); i++){
      if(nsnd[i]<10){
        snd += "00" + String(nsnd[i]); 
      }
      else if(nsnd[i]<100){
        snd += "0" + String(nsnd[i]);
      }
      else{
        //Filter 999s
        if(nsnd[i]==999){
            if(i==0){
              //look forward
              //999|000|...
              // ^
              if(nsnd[i+1]!=999){
                nsnd[i]=nsnd[i+1];
              }
              //999|999|000|...
              // ^
              else if(nsnd[i+1]==999 && nsnd[i+2]!=999){
                nsnd[i]=nsnd[i+2];
              }
              //999|999|999|...
              // ^
              else if(nsnd[i+1]==999 && nsnd[i+2]==999){
                nsnd[i]=666;
              }
            }
            else if(i==28){
              //...|999|000
              //     ^
              if(nsnd[i+1]!=999 && nsnd[i-1]!=666){ //nsnd[i-1]!=666 redundante
                nsnd[i]=(nsnd[i-1]+nsnd[i+1])/2;  
              }
              //...|999|999
              //     ^
              else if(nsnd[i+1]==999){ 
                nsnd[i]=666;
              }    
            }          
            else if(i==29){
              //look backwards
              //...|999
              //     ^
                nsnd[i]=nsnd[i-1];        
            }
            //from 1 to 27
            //look back and forward
            //...|000|999|000|...
            //         ^
            else if(nsnd[i+1]!=999 && nsnd[i-1]!=666){ //nsnd[i-1]!=666 redundante
              nsnd[i]=(nsnd[i-1]+nsnd[i+1])/2;
            }
            //...|000|999|999|000|...
            //         ^
            else if(nsnd[i+1]==999 && nsnd[i+2]!=999 && nsnd[i-1]!=666){ //nsnd[i-1]!=666 redundante
              nsnd[i]=(nsnd[i-1]+nsnd[i+2])/2;
            }
            //...|000|999|999|999|...
            //         ^
            else if(nsnd[i+1]==999 && nsnd[i+2]==999){ //nsnd[i-1]!=666 redundante
              nsnd[i]=666;
            }
            //...|666|999|999|000|...
            //         ^
            else if(nsnd[i-1]==666 && nsnd[i+2]!=999){ 
              nsnd[i]=nsnd[i+2];
            }
            //...|666|999|999|999|...
            //         ^
            else if(nsnd[i-1]==666 && nsnd[i+2]==999){ 
              nsnd[i]=666;
            }
    
        //End Filter 999s
        }
        
        if(nsnd[i]==666){
          snd += "NaN";  
        }
        else{
          snd += String(nsnd[i]);  
        }
      }//else
    }//for
//Coolant temperature
    if(csnd<10){
      snd += "00" + String(csnd);
    }
    else if(csnd<100){
      snd += "0" + String(csnd);
    }
    else{
      snd += String(csnd);
    }
//Battery voltage
    if(bsnd<100){
      snd += "0" + String(bsnd);         
    }
    else{
      snd += String(bsnd);
    }
//Time
    snd += String(tsnd) + ",";
    
    for(i=0; i<(lines); i++){
      if(vsnd[i]==666){
        dsnd+=0;
      }
      else{
        dsnd+=float(vsnd[i])/3.6; //multplied by 1000 (sent in m)
      }
    }
//Distance [m] (delta)
    snd += String(dsnd) + ".";   
    
    k=0;            //Reset position for re-fill time array


/**************************************************
 ****** Add latitude, longitude and altitude ******
 **************************************************/
  pos="noGPS&";
  //long tiempo=millis();
  while(p<100){
   while(gpsSerial.available()){                    // check for gps data
     if(m<1000){     
       if(gps.encode(gpsSerial.read())){            // encode gps data
        //Serial.println("GPS Encode");
        gps.get_position(&lat,&lon);                // get latitude and longitude
        alt=gps.f_altitude();
        pos=String((float(lat)*0.000001),7)+","+String((float(lon)*0.000001),7)+","+String(alt)+"&";
        //Serial.println(pos);
        delay(1);   
       }//if gps
     }
     
     else{
        break;
     } 
     
     m++;    
   }// while gps
   p++;
   delay(10);
  }//while p<100
  /*
  Serial.println("milis:" + String(millis()-tiempo));
  Serial.println("m:" + String(m));
  Serial.println("p:" + String(p));
  */
  m=0;
  p=0;
  snd+=pos;

    /************************************
     ****** Send data to Pro mini sender ******
     ************************************/

    char* data = const_cast<char*>(snd.c_str()); //Parse payload to char array
    
    senderSerial.println(data); 
    //Serial.println(snd);
    //Serial.println(data); //(->DELETE!) 
    //senderSerial.flush();
    //Serial.flush();

    //raw="";

    dsnd=0;
    
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
  //while (BTOBD_serial.available()>0){
  while (Serial.available()>0){
    inData=0;
    inChar=0;
    //inData = BTOBD_serial.read();
    inData = Serial.read();
    inChar=char(inData);
    raw_ELM327_response = raw_ELM327_response + inChar;
  } 
  //read_elm327_response()
  //Serial.println(raw_ELM327_response);
}

/*
void ELM327_enter_terminal_mode(){
  //*Debug HC05 Bluetooth and ELM327 OBD
   // *  Captures serial2 data and writes input to it.
  
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
