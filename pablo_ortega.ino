/**
 * Vehicle monitor through OBD-II Bluetooth adapter
 * and Arduino Nano based circuit
 * 
 * OBD-II  // ELM327 // BT--BT // NANO // SIM800L // Th-Spk
 * 
 * 
 * See OBD ṔIDs wiki (Documentation, control unit IDs, formulas):
 * https://en.wikipedia.org/wiki/OBD-II_PIDs
 * 
 * 
*/


#include <SoftwareSerial.h>


//SoftwareSerial SIM_serial(8,7);  // SIM
SoftwareSerial BTOBD_serial(2,3);  // BT-OBD

#define baud_serial0 9600       //Serial Monitor
#define baud_serial1 9600       //SIM
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
float    rpm;                    //to save rpm data after applying formula
float    veloc;                  //to save velocity data after applying formula

int delay_entre_peticiones = 1000;


//boolean connectivity = false;             //to attempt connection to the cloud or skip and work ofline
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
  */

  //SIM_serial.begin(baud_serial1);
  //gprs.serialDebug();

  
  
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
  
  Serial.println("BT Initialized");
  
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
  //Serial.println("\n"); 

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

/*
  //Serial.println("\n");
  BTOBD_serial.println("01 0C");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  } 
  //Serial.println("Engine RPM sensor initialized.");
*/
  
  BTOBD_serial.println("01 0D");
  delay(300);
  while (BTOBD_serial.available()){
    Serial.write(BTOBD_serial.read());
  } 
  //Serial.println("Vehicle Speed sensor initialized");
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
  //debug_serial2(); //Entrar en modo debug
  
  Serial.print("\n");
  start_millis = millis();
  Serial.print(start_millis);
  Serial.print("\t [ms]\t");
  

  /**
   * Get Velocity
   */

  BTOBD_serial.println("010D");                       //Send Velocity PID
  delay(delay_entre_peticiones);
  read_serial2(); //Receive response
  
  Serial.print("\t"+raw_OBD_response+"\t"); //Saber respuesta del OBD
  
  
  WorkingString = raw_OBD_response.substring(11,13);  //Recover data
  //Serial.println(raw_OBD_response);
  //Serial.println(WorkingString);
  A = strtol(WorkingString.c_str(),NULL,16);          //convert hex to decimal and apply formula
  veloc=A*0.27; //Apply OBD formula (see OBD PIDs wiki)
  Serial.print(veloc);
  Serial.print("\t[m/s]\t");

  stop_millis=millis();
  //delay(13000-(stop_millis-start_millis));
   
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
