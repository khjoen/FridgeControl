#include "SPI.h"
#include "Ethernet.h"
#include "WebServer.h"

#include <DHT.h>

//#include <LiquidCrystal.h>
//#include <PID_v1.h>

#include <SD.h>

#define __CS 4
#define __SS 10

/*******************************************
*  NOTE: 
*
*  ## Don't know when Pascal
*
*     The Ethernet Shield comes with an SD card slot onboard. 
*     The shield fits on top of your Arduino. Because the Ethernet module
*     uses pin 10, the CS pin for the SD card has been moved to pin 4. 
*     Make sure you use SD.begin(4) to use the SD card functionality or
*     do not use pin 4 if both SD and Ethernet are needed.
*
*     This code uses pin 4 for LCD display.  
*  
*  2016-08-24 Pascal
*
*     No use of LCD anymore.
*     I suspect the pin to screw up the LCD.
*     LCD may have better success if handled by another arduino 
*     connected to serial or other communication means.
*     The arduino mega might be good for the job.
*
*  2016-09-09 Pascal
*    
*     - PIN 11, 12, 13 used by SPI.
*     - Removed all code to retrieve config from a distant server
*     - Added SD card support for persistent configuration 
*
*
*   2016-09-13 Pascal
*   
*     - float_to_string ~400b compared to dtostrf ~1500b   
*/


// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(9, 8, 5, 4, 3, 2);

static uint8_t mac[] = { 0x90, 0xA2, 0xda, 0x00, 0x2d, 0xc8 };
#define MAC_STRLEN ((sizeof(mac) << 1) + 5)


#define PREFIX ""
WebServer webserver(PREFIX, 80);

#define PROBES 2
#define DHT1_PIN     6
#define RELAY1_PIN   7
#define DHT2_PIN     8
#define RELAY2_PIN   9
#define DHTTYPE    DHT22
DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

unsigned long iterationTime;
unsigned long lastIterationTime = 0;

char _mybuffer[18];
char varname[4];
unsigned short idx;

const char str_45[] PROGMEM = "-";
const char str_47[] PROGMEM = "/";
const char str_60[] PROGMEM = "<";
const char str_62[] PROGMEM = ">";
const char str_cfg[] PROGMEM = "cfg";
const char str_commit[] PROGMEM = "commit";
const char str_cs[] PROGMEM = "cs";
const char str_d[] PROGMEM = "d";
const char str_er[] PROGMEM = "er";
const char str_ees[] PROGMEM = "ees";
const char str_h[] PROGMEM = "h";
const char str_live[] PROGMEM = "live";
const char str_o[] PROGMEM = "o";
const char str_p[] PROGMEM = "p";
const char str_s[] PROGMEM = "s";
const char str_SD[] PROGMEM = "SD";
const char str_t[] PROGMEM = "t";
const char str_da[] PROGMEM = "da";
const char str_db[] PROGMEM = "db";
const char str_mac[] PROGMEM = "mac";
const char str_on[] PROGMEM = "on";
const char str_off[] PROGMEM = "off";
const char str_pp[] PROGMEM = "pp";
const char str_sp[] PROGMEM = "sp";
const char str_contentType[] PROGMEM = "text/xml";
const char str_xmlheader[] PROGMEM = "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>";
const char str_Host[] PROGMEM = "Host: ";
const char str_connectionclose[] PROGMEM = "Connection: close";
const char str_config[] PROGMEM = "controler.cfg";


//Define Variables we'll be connecting to
//double Output, _kp = 200.0, _ki = 5.0, _kd = 10.0;

//Specify the links and initial tuning parameters
//PID myPID(&t, &Output, &Setpoint, 200.0, 5.0, 10.0, REVERSE);
//unsigned long PIDWindowSize = 900000;
//unsigned long PIDwindowStartTime;
//unsigned int  PIDSampleTime = 30000;


enum e_chill_state {
  ON,
  OFF
};

enum e_SD_state {
  READY,
  FAILED
};
e_SD_state sd_state = FAILED;

enum e_config_state {
  CONFIG_COMMIT_PENDING = 'P',
  CONFIG_SD_INIT_FAILED = 'F',
  CONFIG_OPEN_WRITE_FAILED = 'W',
  CONFIG_OPEN_READ_FAILED = 'R',
  CONFIG_FILE_INEXISTENT = 'E',
  CONFIG_NORMAL = 'N'
};
e_config_state config_state = CONFIG_NORMAL;

typedef struct 
{
  int DHT_pin;
  int relay_pin;
  e_chill_state chill_state;
  double set_point;
  double delta_below;
  double delta_above;
  unsigned long probe_read_period;
  unsigned long next_read_time;
  bool          next_read_time_after_flip;
  double humidity;  // humidity read
  double temperature;  // temperature read
  double temperature_range_low;  // set point - delta_below
  double temperature_range_high;  // set point + delta_above
  double liquid_temperature;  // temperature read to use when we add the thermowell in the system
  double liquid_set_point;  // temperature read to use when we add the thermowell in the system
  DHT *dht;
} t_temperature_control;

t_temperature_control control[PROBES] = {
  {DHT1_PIN, RELAY1_PIN, OFF, 17.0, 1.0, 1.0, 30000, 30000, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, &dht1},
  {DHT2_PIN, RELAY2_PIN, OFF, 17.0, 1.0, 1.0, 30000, 30000, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, &dht2}
};


struct 
{
  char set_point[8];
  char delta_below[8];
  char delta_above[8];
  char probe_read_period[10];
} config_rec;

File config_file;

struct {
  bool commit_requested;
  bool will_update_config;
  unsigned long next_commit_time;
  bool next_commit_time_after_flip;
} commit_control = {false, false, 0, false};






void read_DHT_info(int probe)
{
  control[probe].humidity = control[probe].dht->readHumidity();
  control[probe].temperature = control[probe].dht->readTemperature();
}


/*
*
* test fonction for the relay.
**//*
void set_on_off_cmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;

  server.httpSuccess();

  rc = server.nextURLparam(&url_tail, varname, sizeof(varname), _mybuffer, sizeof(_mybuffer));
  if (rc != URLPARAM_OK)
    return;

  //Serial.print("Got ");
  //Serial.println(value);
  if (strlen(_mybuffer) == 3) {
    if (isdigit(_mybuffer[2])) {
      short i = short(varname[2] - '1');
      if (!memcmp_P(_mybuffer, (char*)pgm_read_word(str_on), 2)) {
        digitalWrite(control[i].relay_pin, HIGH);
        control[i].chill_state = ON;
      } else if (!memcmp_P(_mybuffer, (char*)pgm_read_word(str_off), 2)) {
        digitalWrite(control[i].relay_pin, LOW);
        control[i].chill_state = OFF;
      }
    }
  }

  // wait a bit to have time to see the effect.
  delay(4000);
}
*/

void recalc_temperature_range(short idx)
{
  control[idx].temperature_range_low = control[idx].set_point - control[idx].delta_below;
  control[idx].temperature_range_high = control[idx].set_point + control[idx].delta_above;
}

char char_to_nibble(char in)
{
  char c = 0;

  if (in >= '0' && in <= '9') {
    c = in - '0';
  }
  else if (in >= 'a' && in <= 'f') {
    c = in - 'a' + 10; 
  }

  return c;
}

void load_mac(char* value)
{
  unsigned short i;
  for (idx=0, i=0; idx < sizeof(_mybuffer); idx++, i++) {
    char c1 = char_to_nibble(value[idx++]);
    char c2 = char_to_nibble(value[idx++]);
    mac[i] = (c1 << 4) | c2; 
  }
}


void trigger_config_file_update()
{
  commit_control.will_update_config = true;
  commit_control.next_commit_time = iterationTime + 300000;  // in 5 minutes.
  if (commit_control.next_commit_time == 0) 
    commit_control.next_commit_time++;
  if (commit_control.next_commit_time < iterationTime)
    commit_control.next_commit_time_after_flip = true;
}

void check_set_param(char *name, char *value) 
{
//  if (!strcmp(name, "kp")) {
//    _kp = tmp;
//  } else if (!strcmp(name, "ki")) {
//    _ki = tmp;
//  } else if (!strcmp(name, "kd")) {
//    _kd = tmp;
//  } else 
  if (strlen(name) == 3) {

    if (!memcmp(name, (char*)pgm_read_word(str_cfg), 3)) {
      if (!memcmp(value, (char*)pgm_read_word(str_commit), 6)) {
        if (commit_control.will_update_config == true) {
          commit_control.commit_requested = true;
        }
      }      
    }
    else if (!memcmp(name, (char*)pgm_read_word(str_mac), 3)) {

      if (!memcmp(&value[2], (char*)pgm_read_word(str_45), 1) &&
          !memcmp(&value[5], (char*)pgm_read_word(str_45), 1) &&
          !memcmp(&value[8], (char*)pgm_read_word(str_45), 1)) {

        load_mac(value);

        trigger_config_file_update();
      }

    } else {

      double tmp = (double)atof(value);

      if (isdigit(name[2])) {

        idx = name[2] - '1';   
        if (idx < PROBES) {  

          if (!memcmp(name, (char*)pgm_read_word(str_sp), 2)) {
            control[idx].set_point = tmp;
      //  } else if (!strcmp(name, "pw")) {
      //    PIDWindowSize = tmp;
      //  } else if (!strcmp(name, "ps")) {
      //    PIDSampleTime = tmp;
      //    myPID.SetSampleTime(PIDSampleTime);
          } else if (!memcmp(name, (char*)pgm_read_word(str_pp), 2)) {
              control[idx].probe_read_period = tmp;
          } else if (!memcmp(name, (char*)pgm_read_word(str_db), 2)) {
              control[idx].delta_below = tmp;
          } else if (!memcmp(name, (char*)pgm_read_word(str_da), 2)) {
              control[idx].delta_above = tmp;
          }  

          recalc_temperature_range(idx);
          trigger_config_file_update();

        }
      }
    }
  }
//  myPID.SetTunings(_kp, _ki, _kd);
}

void set_configs_cmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;

  server.httpSuccess();
  
  while(true) {
    rc = server.nextURLparam(&url_tail, varname, sizeof(varname), _mybuffer, sizeof(_mybuffer));
    if (rc != URLPARAM_OK)
      break;

    check_set_param(varname, _mybuffer);
  }
}



void mac_to_string(char *buf)
{
  short buf_idx = 0;
  for (idx = 0; idx < sizeof(mac); idx++) {
    char c = (mac[idx] & 0xF0) >> 4;

    if (c < 10) 
      c = c + '0';
    else
      c = (c - 10) + 'a';
    buf[buf_idx++] = c;

    c = mac[idx] & 0x0F;
    if (c < 10) 
      c = c + '0';
    else
      c = (c - 10) + 'a';
    buf[buf_idx++] = c;

    if (buf_idx == 17)
      break;

    buf[buf_idx++] = '-';
  }
  
  buf[buf_idx] = 0;
}

void output_xmlnode_double(WebServer &server, char *nodename, double nodevalue)
{
  server.print((char*)pgm_read_word(str_60));
  server.print(nodename);  
  server.print((char*)pgm_read_word(str_62));
  server.print(nodevalue);
  server.print((char*)pgm_read_word(str_60));
  server.print((char*)pgm_read_word(str_47));
  server.print(nodename);  
  server.print((char*)pgm_read_word(str_62));
}

void output_xmltag(WebServer &server, char id, char *name, bool closing)
{
  server.print((char*)pgm_read_word(str_60));
  server.print(name);
  if (id != -1) {
    server.print(id);
  }
  if (closing == true) {
    server.print((char*)pgm_read_word(str_47));
  }
  server.print((char*)pgm_read_word(str_62));
}

bool open_config_read() 
{
  if (false == switch_to_SD()) {
    switch_to_Ethernet();
    return false;
  }

  if (false == SD.exists((char*)pgm_read_word(str_config))) {
    config_state = CONFIG_FILE_INEXISTENT;
    switch_to_Ethernet();
    return false;
  }

  if (false == open_config(FILE_READ)) {
    config_state = CONFIG_OPEN_READ_FAILED;
    switch_to_Ethernet();
    return false;
  } 

  return true;
}

/* commands are functions that get called by the webserver framework
 * they can read any posted data from client, and they output to the
 * server to send data back to the web browser. */
void dash_cmd(WebServer &server, WebServer::ConnectionType type, char *, bool)
{
  bool config_opened = true;

  server.httpSuccess((char*)pgm_read_word(str_contentType));

  if (type != WebServer::HEAD)  {

    server.print((char*)pgm_read_word(str_xmlheader));

    output_xmltag(server, -1, (char*)pgm_read_word(str_d), false);

    output_xmltag(server, -1, (char*)pgm_read_word(str_er), false);
    
    output_xmltag(server, -1, (char*)pgm_read_word(str_live), false);

    output_xmltag(server, -1, (char*)pgm_read_word(str_mac), false);
    mac_to_string(_mybuffer);
    server.print(_mybuffer);
    output_xmltag(server, -1, (char*)pgm_read_word(str_mac), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_live), true);


    output_xmltag(server, -1, (char*)pgm_read_word(str_SD), false);

    config_opened = open_config_read();

    output_xmltag(server, -1, (char*)pgm_read_word(str_mac), false);
    if (true == config_opened) {
      config_file.seek(0);
      config_file.read((uint8_t*)_mybuffer, MAC_STRLEN);
      server.print(_mybuffer);
    }
    output_xmltag(server, -1, (char*)pgm_read_word(str_mac), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_SD), true);
    output_xmltag(server, -1, (char*)pgm_read_word(str_er), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_ees), false);
    for(idx = 0; idx < PROBES; idx++) {

      output_xmltag(server, idx, (char*)pgm_read_word(str_p), false);
      output_xmltag(server, -1, (char*)pgm_read_word(str_live), false);

      output_xmlnode_double(server, (char*)pgm_read_word(str_t), control[idx].temperature);
      output_xmlnode_double(server, (char*)pgm_read_word(str_h), control[idx].humidity);
      output_xmlnode_double(server, (char*)pgm_read_word(str_sp), control[idx].set_point);

      output_xmltag(server, -1, (char*)pgm_read_word(str_o), false);
      if (control[idx].chill_state == ON)
        server.print((char*)pgm_read_word(str_on));
      else
        server.print((char*)pgm_read_word(str_off));
      output_xmltag(server, -1, (char*)pgm_read_word(str_o), true);

      output_xmlnode_double(server, (char*)pgm_read_word(str_pp), (double)control[idx].probe_read_period);
      output_xmlnode_double(server, (char*)pgm_read_word(str_db), control[idx].delta_below);
      output_xmlnode_double(server, (char*)pgm_read_word(str_da), control[idx].delta_above);

      output_xmltag(server, -1, (char*)pgm_read_word(str_live), true);

      output_xmltag(server, -1, (char*)pgm_read_word(str_SD), false);

      if (true == config_opened) {

        read_next_control_config();

        output_xmltag(server, -1, (char*)pgm_read_word(str_sp), false);
        server.print(config_rec.set_point);
        output_xmltag(server, -1, (char*)pgm_read_word(str_sp), true);

        output_xmltag(server, -1, (char*)pgm_read_word(str_db), false);
        server.print(config_rec.delta_below);
        output_xmltag(server, -1, (char*)pgm_read_word(str_db), true);

        output_xmltag(server, -1, (char*)pgm_read_word(str_da), false);
        server.print(config_rec.delta_above);
        output_xmltag(server, -1, (char*)pgm_read_word(str_da), true);

        output_xmltag(server, -1, (char*)pgm_read_word(str_pp), false);
        server.print(config_rec.probe_read_period);
        output_xmltag(server, -1, (char*)pgm_read_word(str_pp), true);

      }

      output_xmltag(server, -1, (char*)pgm_read_word(str_SD), true);

      output_xmltag(server, idx, (char*)pgm_read_word(str_p), true);
    }

    output_xmltag(server, -1, (char*)pgm_read_word(str_ees), true);

    //dump_config(server);

    output_xmltag(server, -1, (char*)pgm_read_word(str_d), true);

    if (true == config_opened) {
      close_config();
    }
  }
}

bool switch_to_SD()
{
  digitalWrite(__SS, HIGH);
  digitalWrite(__CS, LOW);

  if (sd_state != READY) {
    if (SD.begin(4)) {
      sd_state = READY;
    }
    else {
      sd_state = FAILED;
      config_state = CONFIG_SD_INIT_FAILED;
      return false;
    }
  }

  return true;
}

void switch_to_Ethernet()
{
  digitalWrite(__CS, HIGH);
  digitalWrite(__SS, LOW);
}


void load_config()
{

  if (false == open_config_read())
    return;

  // read & load mac
  config_file.seek(0);
  config_file.read((uint8_t*)_mybuffer, MAC_STRLEN);
  load_mac(_mybuffer);

  // read and load array of temperature control.
  for (idx = 0; idx < PROBES; idx++) {

    read_next_control_config();

    control[idx].set_point = (double)atof(config_rec.set_point);
    control[idx].delta_below = (double)atof(config_rec.delta_below);
    control[idx].delta_above = (double)atof(config_rec.delta_above);
    control[idx].probe_read_period = (unsigned long)atoi(config_rec.probe_read_period);

    recalc_temperature_range(idx);

  }

  close_config();

  switch_to_Ethernet();
}


/*
void dump_config(WebServer &server) 
{

  if (false == open_config_read())
    return;

  output_xmltag(server, -1, (char*)pgm_read_word(str_SD), false);
  output_xmltag(server, -1, (char*)pgm_read_word(str_mac), false);
  // read & write mac
  config_file.seek(0);
  config_file.read((uint8_t*)_mybuffer, MAC_STRLEN);
  server.print(_mybuffer);
  output_xmltag(server, -1, (char*)pgm_read_word(str_mac), true);

  output_xmltag(server, -1, (char*)pgm_read_word(str_cs), false);
  server.print((char)config_state);
  output_xmltag(server, -1, (char*)pgm_read_word(str_cs), true);

  output_xmltag(server, -1, (char*)pgm_read_word(str_p), false);

  // write array of temperature control.
  for (idx = 0; idx < PROBES; idx++) {

    read_next_control_config();

    output_xmltag(server, idx+1, (char*)pgm_read_word(str_p), false);

    output_xmltag(server, -1, (char*)pgm_read_word(str_sp), false);
    server.print(config_rec.set_point);
    output_xmltag(server, -1, (char*)pgm_read_word(str_sp), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_db), false);
    server.print(config_rec.delta_below);
    output_xmltag(server, -1, (char*)pgm_read_word(str_db), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_da), false);
    server.print(config_rec.delta_above);
    output_xmltag(server, -1, (char*)pgm_read_word(str_da), true);

    output_xmltag(server, -1, (char*)pgm_read_word(str_pp), false);
    server.print(config_rec.probe_read_period);
    output_xmltag(server, -1, (char*)pgm_read_word(str_pp), true);

    output_xmltag(server, idx+1, (char*)pgm_read_word(str_p), true);

  }

  output_xmltag(server, -1, (char*)pgm_read_word(str_p), true);

  output_xmltag(server, -1, (char*)pgm_read_word(str_SD), true);

  close_config();

  switch_to_Ethernet();
}
*/

int int_to_string(unsigned long natural, char* str, size_t str_size) {

  int i = 1, r;
  unsigned long n;

  for (n = natural; n>0 && idx < str_size; n/=10) {
    str[idx++] = (char)(n - (unsigned long)(n/10)*10 + '0');
  }

  if (0 == idx) {
    str[0] = '0';
    idx++;
  }

  for(i = 0, r = idx - 1; i < r; i++, r--) {
    char c;
    c = str[r];
    str[r] = str[i];
    str[i] = c;    
  }

  if (idx==str_size)
    idx--;

  str[idx] = 0;

  return idx;
}

int float_to_string(float f, char* str, size_t str_size) 
{
  float frem;
  int i = 1, r, n;
  int pow10;
  
  unsigned long intpart = (unsigned long)f;
  idx = 0;

  idx = int_to_string(intpart, str, str_size);
  if (idx == str_size-1) {
    return (int)idx;
  }

  str[idx++] = '.';

  frem = f - (float)intpart;
  for (pow10 = 10, i = 1; i > 0 && idx < str_size; pow10*=10) {
    i = (int)(frem*pow10);
    str[idx++] = (char)(i + '0');
    r = r*10+i;
    frem = frem - (float)i/pow10;
    i = (int) (frem*1000000);
  }

  if (idx==str_size)
    idx--;

  str[idx] = 0;

  return (int)idx;
}


void format_config_record(short index)
{
  float_to_string(control[index].set_point, config_rec.set_point, sizeof(config_rec.set_point)); 
  float_to_string(control[index].delta_below, config_rec.delta_below, sizeof(config_rec.delta_below)); 
  float_to_string(control[index].delta_above, config_rec.delta_above, sizeof(config_rec.delta_above)); 
  int_to_string(control[index].probe_read_period, config_rec.probe_read_period, sizeof(config_rec.probe_read_period));
}

int read_next_control_config()
{
  return config_file.read((uint8_t*)&config_rec, sizeof(config_rec));  
}

bool open_config(char mode)
{
  config_file = SD.open((char*)pgm_read_word(str_config), mode);
  if (config_file == false)
    return false;
  return true;
}

void close_config() 
{
  config_file.close();  
}




void write_all_configs() 
{
  if (false == switch_to_SD()) {
    switch_to_Ethernet();
    return;
  }

  if (false == open_config(FILE_WRITE)) {
    config_state = CONFIG_OPEN_WRITE_FAILED;
    switch_to_Ethernet();
    return;
  }

  // write mac
  mac_to_string(_mybuffer);
  config_file.seek(0);
  config_file.write((uint8_t*)_mybuffer, MAC_STRLEN);

  // write array of temperature control.
  for (idx = 0; idx < PROBES; idx++) {
    format_config_record(idx);
    config_file.write((uint8_t*)&config_rec, sizeof(config_rec));
  }

  close_config();

  switch_to_Ethernet();
}


void setup()
{
  // set the pin modes
  pinMode(__SS, OUTPUT);
  pinMode(__CS, OUTPUT);

  load_config();

  /* initialize the Ethernet adapter */
  //Serial.begin(9600);  
  
  Ethernet.begin(mac);

//  lcd.begin(16, 2);


  // setup the probes and relays  
  for(idx = 1; idx < PROBES; idx++) {
    control[idx].dht->begin();
    control[idx].next_read_time = control[idx].probe_read_period;
    pinMode(control[idx].relay_pin, OUTPUT);
    digitalWrite(control[idx].relay_pin, LOW);
    control[idx].chill_state = OFF;
    read_DHT_info(idx);
  }

/*  
  lcd.setCursor(0, 0);
  lcd.print("IP:");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    lcd.print(Ethernet.localIP()[thisByte], DEC);
    lcd.print(".");
    //Serial.print(Ethernet.localIP()[thisByte], DEC);
    //Serial.print("."); 
  }
  lcd.print("    ");
  //Serial.println();
*/


  // setup the web server.
  webserver.setDefaultCommand(&dash_cmd);
  //webserver.addCommand("index.html", &dash_cmd); 
  //webserver.addCommand("ssp", &set_set_point_cmd);
  //webserver.addCommand("onoff", &set_on_off_cmd);
  webserver.addCommand("cfg", &set_configs_cmd);    
  webserver.begin();

//  PIDwindowStartTime = 30000;

//  myPID.SetOutputLimits(0, PIDWindowSize);
//  myPID.SetMode(AUTOMATIC);
//  myPID.SetControllerDirection(REVERSE);
//  myPID.SetSampleTime(PIDSampleTime);

}

void loop()
{
  char buff[64];
  int len = sizeof(buff);

  Ethernet.maintain();

  iterationTime = millis();

  // we flipped
  if (iterationTime < lastIterationTime) {
    // all clock set to next_read_time_after_flip completed.
    for(idx = 0; idx < PROBES; idx++) {
      control[idx].next_read_time_after_flip = false;
    }
    commit_control.next_commit_time_after_flip = false;
  }

   /*
    *  automatic commit of config changes in memory:
    *
    *  check if we shoud auto-commit or if commit was requested 
    *  to apply changes persistently   
    */
  if (true == commit_control.will_update_config && commit_control.next_commit_time > 0) {
    if ((true == commit_control.commit_requested) || 
          (iterationTime > commit_control.next_commit_time &&
           false == commit_control.next_commit_time_after_flip)) {
      write_all_configs();
      commit_control.next_commit_time = 0;
      commit_control.will_update_config = false;
      commit_control.commit_requested = false;
      config_state = CONFIG_NORMAL;
    }
  }

   /*
    *  check if it is time to read the probe for controlled object.
    *  apply relay logic if a reading is taken.
    */
  for(idx = 0; idx < PROBES; idx++) {

    //if ((iterationTime - control[idx].DHT_window_start_time) > control[idx].probe_read_period) { 
    if ((iterationTime > control[idx].next_read_time) && 
        (false == control[idx].next_read_time_after_flip)) {         // skip reading probe having an expiry after flipping time.

      // read probe
      read_DHT_info(idx);

      unsigned long last_expiry = control[idx].next_read_time;
      control[idx].next_read_time += control[idx].probe_read_period;
      if (last_expiry > control[idx].next_read_time)
        control[idx].next_read_time_after_flip = true;

      if (control[idx].temperature <= control[idx].temperature_range_low) {
        digitalWrite(control[idx].relay_pin, LOW);
        control[idx].chill_state = OFF;
      } else if (control[idx].temperature >= control[idx].temperature_range_high) {
        digitalWrite(control[idx].relay_pin, HIGH);
        control[idx].chill_state = ON;
      }

  //    if (control[idx].temperature <= (control[idx].set_point - control[idx].delta_below)) {
  //      digitalWrite(control[idx].relay_pin, LOW);
  //      control[idx].chill_state = OFF;
  //    } else if (control[idx].temperature >= (control[idx].set_point + control[idx].delta_above)) {
  //      digitalWrite(control[idx].relay_pin, HIGH);
  //      control[idx].chill_state = ON;
  //    }
    }
  }
/*
  else {
    boolean computed = myPID.Compute();
    // ************************************************
    // * turn the output pin on/off based on pid output
    // ************************************************
    if ((iterationTime - PIDwindowStartTime) > PIDWindowSize) { 
      //time to shift the Relay Window
      PIDwindowStartTime += PIDWindowSize;
    }

    if (Output < (iterationTime - PIDwindowStartTime)) {
      // Normally opened setting.
      digitalWrite(RELAY_PIN, LOW);
      chillState = OFF;
    }
    else {
      digitalWrite(RELAY_PIN, HIGH);
      chillState = ON;
    }
  }
*/

  if (webserver.available() == true)
    webserver.processConnection(buff, &len);

  lastIterationTime = iterationTime;

}
