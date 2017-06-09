#include <math.h>
#include <dht.h>
#include <ESP8266WiFi.h>
#include "MQ135.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
 
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


#define dht_dpin A0 //no ; here. Set equal to channel sensor is on
dht DHT;



#define ANALOGPIN A0    //  Define Analog PIN on Arduino Board
#define RZERO 206.85    //  Define RZERO Calibration Value
MQ135 gasSensor = MQ135(ANALOGPIN);


#define WLAN_SSID       "AndroidG2"
#define WLAN_PASS       "apapapap"


#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "iotair"
#define AIO_KEY         "58d1d1b9c88542719337785d0870d534"


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");
Adafruit_MQTT_Publish temperaturem = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish smokem = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Smoke");
Adafruit_MQTT_Publish LPGm = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/LPG");
Adafruit_MQTT_Publish COm = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CO");
Adafruit_MQTT_Publish COx = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CO2");
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

void MQTT_connect();



//Variables for the sound sensor LM386
int time_frame=50;
double soundDetectedVal; // This is where we record our Sound Measurement
int counter=1;
double sum=0;

double LPG;
double CO;
double smoke;
double temperature;
double humidity;
double dBvalue;
double CO2;

void setup() {
  
  Serial.begin(9600);
  delay(200);//Wait rest of 1000ms recommended delay before

  Serial.println(F("IOT Enviornmental Pollution Monitoring System"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoffbutton);


         
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    



  pinMode (A0, INPUT) ; // input from the Sound Detection Module
  pinMode (D0, OUTPUT) ;
  pinMode (D1, OUTPUT) ;
  pinMode (D2, OUTPUT) ;
  pinMode (D3, OUTPUT) ;
  
  delay(300);//Let system settle
  delay(700);//Wait rest of 1000ms recommended delay before
  float rzero = gasSensor.getRZero();
  
}

void loop() {
  
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
    }
  }

  //Calculating and publishing the dB value
  digitalWrite(D0, LOW);
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  for(counter=0;counter<20;counter++){
            soundDetectedVal = analogRead (A0) ; // read the sound value
            soundDetectedVal =10*log10((1023-soundDetectedVal)*(1023-soundDetectedVal)/0.05);  //caliberation
    
            sum=sum+soundDetectedVal;
            delay (time_frame);
   }
      
      dBvalue=(sum/20)+10;
      sum=0;
  Serial.print(F("\nSending dBValue val "));
  Serial.print(dBvalue);
  Serial.print("...");
  if (! photocell.publish(dBvalue)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }



  //Calculating and publishing the MQ-2 gas sensor readings
  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  
    LPG=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
    CO=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
    smoke=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
    
   Serial.print("LPG:");      
   Serial.print(LPG);
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:"); 
   Serial.print(CO);
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("SMOKE:"); 
   Serial.print(smoke);
   Serial.print( "ppm" );
   Serial.print("\n");

  //publish LPG PPM in server
  if (! photocell.publish(LPG)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  //publish CO PPM in server
  if (! photocell.publish(CO)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  //publish smoke PPM in server
  if (! photocell.publish(smoke)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }


    //Output the temperature and humidity sensor readings
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);
  
    humidity=DHT.humidity;
    temperature=DHT.temperature;
    
    DHT.read11(dht_dpin);
    Serial.print("Current humidity = ");   
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");

  //publish temperature in server
  if (! photocell.publish(temperature)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  //publish humidity in server
  if (! photocell.publish(humidity)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }


  //Output the MQ 135 sensor readings
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);
  
  float ppm = gasSensor.getPPM();
  CO2=ppm;
  Serial.print("CO2 ppm value : ");
  Serial.println(ppm);
  
  //publish CO2 ppm in server
  if (! photocell.publish(CO2)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Serial.println("");

  if (LPG>2 && CO>2 && smoke>2 && temperature>29 && humidity<50 && dBvalue>85 && CO2>2)
  {
    digitalWrite(D4, HIGH);
  }
}



float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}


float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 


int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(100);           // basically die and wait for WDT to reset me
// wait 5 seconds
       retries--;
       if (retries == 0) {
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
