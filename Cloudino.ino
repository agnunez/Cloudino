#include <SPI.h>
#include <i2cmaster.h>
#include <TimerOne.h>
//#include "Streaming.h"
#include "Ethernet.h"
#include "WebServer.h"
// no-cost stream operator as described at 
// http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }

double Tobj,Tamb;


// CHANGE THIS TO YOUR OWN UNIQUE VALUE
// static uint8_t mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x55, 0x68 };
static uint8_t mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x24, 0x58 };

// CHANGE THIS TO MATCH YOUR HOST NETWORK
static uint8_t ip[] = { 161,72,123,174 };
//byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x24, 0x58};
// assign an IP address for the controller:
//IPAddress ip(161,72,123,174);
//IPAddress gateway(161,72,123,129);	
//IPAddress subnet(255,255,255,128);

#define PREFIX ""

WebServer webserver(PREFIX, 80);


/* Cloudino.pde -- Cloud sensor with Webduino server library */
/* Agustin Nunez 2011 */



// commands are functions that get called by the webserver framework
// they can read any posted data from client, and they output to server

void cloudCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  P(htmlHead) =
    "<html>"
    "<head>"
    "<title>Cloudino Web Server</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; text-decoration: underline }"
    "P  { font-size: 10pt; }"
    "</style>"
    "<meta http-equiv='refresh' content='1' >"
    "</head>"
    "<body>";

  int i;
  server.httpSuccess();
  server.printP(htmlHead);

  server << "</p><h1>MXL90614 Temperature Sensor</h1><p>";
  server << "Tobj: " << Tobj << "<br/>";
  server << "Tamb: " << Tamb << "<br/>";
  server << "</body></html>";

}


void jsonCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  
  
  if (type == WebServer::POST)
  {
    server.httpFail();
    return;
  }

  server.httpSuccess(false, "application/json");
  
  if (type == WebServer::HEAD)
    return;

  int i;    
  server << "{ ";
  for (i = 0; i <= 9; ++i)
  {
    // ignore the pins we use to talk to the Ethernet chip
    int val = digitalRead(i);
    server << "\"d" << i << "\": " << val << ", ";
  }

  for (i = 0; i <= 5; ++i)
  {
    int val = analogRead(i);
    server << "\"a" << i << "\": " << val;
    if (i != 5)
      server << ", ";
  }
  
  server << " }";

}

void outputPins(WebServer &server, WebServer::ConnectionType type, bool addControls = false)
{
  P(htmlHead) =
    "<html>"
    "<head>"
    "<title>Arduino Web Server</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; text-decoration: underline }"
    "P  { font-size: 10pt; }"
    "</style>"
    "</head>"
    "<body>";

  int i;
  server.httpSuccess();
  server.printP(htmlHead);

  if (addControls)
    server << "<form action='" PREFIX "/form' method='post'>";

  server << "</p><h1>MXL Temperature Sensor</h1><p>";
  server << "Tobj: " << Tobj << "<br/>";
  server << "Tamb: " << Tamb << "<br/>";

  server << "<h1>Digital Pins</h1><p>";

  for (i = 0; i <= 9; ++i)
  {
    // ignore the pins we use to talk to the Ethernet chip
    int val = digitalRead(i);
    server << "Digital " << i << ": ";
    if (addControls)
    {
      char pinName[4];
      pinName[0] = 'd';
      itoa(i, pinName + 1, 10);
      server.radioButton(pinName, "1", "On", val);
      server << " ";
      server.radioButton(pinName, "0", "Off", !val);
    }
    else
      server << (val ? "HIGH" : "LOW");

    server << "<br/>";
  }

  server << "</p><h1>Analog Pins</h1><p>";
  for (i = 0; i <= 5; ++i)
  {
    int val = analogRead(i);
    server << "Analog " << i << ": " << val << "<br/>";
  }

  server << "</p>";

  if (addControls)
    server << "<input type='submit' value='Submit'/></form>";
  server << "</body></html>";

}

void formCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (type == WebServer::POST)
  {
    bool repeat;
    char name[16], value[16];
    do
    {
      repeat = server.readPOSTparam(name, 16, value, 16);
      if (name[0] == 'd')
      {
        int pin = strtoul(name + 1, NULL, 10);
        int val = strtoul(value, NULL, 10);
        digitalWrite(pin, val);
      }
    } while (repeat);

    server.httpSeeOther(PREFIX "/form");
  }
  else
    outputPins(server, type, true);
}

void defaultCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  outputPins(server, type, false);  
}

double readMLXtemp(int TaTo){
    int dev = 0x5A<<1;
//    int dev = 0x55<<1;  //device address modified
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    if (TaTo) i2c_write(0x06); else i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    return(tempData);
}

void setup()
{
  // set pins 0-8 for digital input
  for (int i = 0; i <= 9; ++i)
    pinMode(i, INPUT);
  pinMode(9, OUTPUT);

  Ethernet.begin(mac, ip);
  webserver.begin();

  webserver.setDefaultCommand(&defaultCmd);
  webserver.addCommand("cloud", &cloudCmd);
  webserver.addCommand("json", &jsonCmd);
  webserver.addCommand("form", &formCmd);
//debug
  Serial.begin(115200);
  Serial.println("MXL90614 Setup...");
	
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(readSensors);  // attaches callback() as  
}

void readSensors(){
  Tobj = readMLXtemp(0) - 273.15;
  Tamb = readMLXtemp(1) - 273.15;  
}

void loop()
{
  // process incoming connections one at a time forever
  webserver.processConnection();
  
  // if you wanted to do other work based on a connecton, it would go here
}
