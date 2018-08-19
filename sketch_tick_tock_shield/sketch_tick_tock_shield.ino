/*
 * Seeed Studio Starter (Tick-Tock) Shield
 * http://wiki.seeedstudio.com/Starter_Shield_EN/
 * 
 * Creator: Shawon Shahryiar
 * https://libstock.mikroe.com/projects/view/1437/seeedstudio-arduino-tick-tock-shield-demo
 * 
 * Modified by David Chao (2018-08-19):
 * Use TimerOne ISR to toggle the 1 sec LED blinking. Old method 
 * uses a delay() in the main loop to control the timing, this 
 * causes the key response to be sluggish (at 500ms). The keys can 
 * now be scanned at a higher frequence to improve key response.
 * A variable key repeat rate is also added to make menu and clock 
 * adjustments easier. Plus general cleaning up.
 */
 
#include <Wire.h>
#include <TimerOne.h>

//#define DEBUG

//Shield I/O connection//

#define LED1                                 2
#define LED2                                 3
#define LED3                                 4
#define LED4                                 5  //pwm

#define Buzzer                               6

#define TM1636_CLK                           7
#define TM1636_SDIO                          8

#define Button1                              9
#define Button2                             10
#define Button3                             11

#define Thermistor                          A0
#define LDR                                 A1
#define V_batt                              A2

// Button Keys

#define Menu_BTN                          0x01
#define Inc_BTN                           0x02
#define Dec_BTN                           0x04

#define key_click                           40  //ms
#define key_debounce                        10  //ms
#define key_delay_short                     50  //ms
#define key_delay_med                      200  //ms
#define key_delay_long                     450  //ms
#define key_repeat_delay                     4  //4 * 500ms = 2sec

enum KeyState {
  unarmed, armed, trigger };
  
enum DisplayMode {
  time_disp, date_disp, 
  year_disp, week_disp, 
  temp_disp, lux_disp, 
  BV_disp, no_disp };

enum MenuMode {
  set_hour, set_minute,
  set_month, set_date,
  set_year, set_week,
  set_end };
  
//TM1636 Defintions//

#define Write_Display_Data_Cmd            0x40
#define Read_Key_Data_Cmd                 0x42
#define Auto_Address_Increment            0x40
#define Fixed_Addressing                  0x44
#define Normal_Mode                       0x40
#define Test_Mode                         0x48

#define Display_Address                   0xC0

#define Display_Off                       0x80
#define Display_On                        0x88

#define Display_colon                     0x80

//Thermistor Definitions//

#define B                               3975.0
#define _25C_K                          298.15
#define _0C_K                           273.15
#define VDD_max                           1023
#define R0                             10000.0
#define R_fixed                        10000.0                     

//LDR Definitions//

#define LDR_constant                  900000.0
#define R_fixed                        10000.0

//DS1307 RTC Definitions//

#define DS1307_sec_reg                    0x00
#define DS1307_min_reg                    0x01
#define DS1307_hour_reg                   0x02
#define DS1307_day_reg                    0x03
#define DS1307_date_reg                   0x04
#define DS1307_mnth_reg                   0x05
#define DS1307_year_reg                   0x06
#define DS1307_ctrl_reg                   0x07
#define DS1307_addr                       0x68
#define _12_hr                            false
#define _24_hr                            true
#define pm                                false
#define am                                true


//Shared variables with ISR, make sure compiler always use a fresh copy//

volatile boolean sec_dp_blink = false;   //seconds indicator blink flag   
volatile boolean display_blink = false;  //LED display blink flag  

//Global variables//

DisplayMode mode = time_disp;
MenuMode menu = set_hour;
boolean set_time_state = false;
signed char led_level = 2;
unsigned char led_states = 0;

unsigned long loop_delay = key_delay_short;
unsigned char slow_repeat = 0;
KeyState but1_state = unarmed;
KeyState but2_state = unarmed;
KeyState but3_state = unarmed;

unsigned char m = 0;
unsigned char s = 0;
unsigned char h = 0;
unsigned char wk = 1;
unsigned char dt = 1;
unsigned char mt = 1;
unsigned char yr = 18;

float R_inf = 0.0;

//0~9,A,b,C,d,E,F,"-"," "
//a(18) c(19) d(20) e(21) f(22) h(23) i(24) m(25) 
//n(26) o(27) p(28) r(29) s(30) t(31) u(32) 
const unsigned char segmap[33] = 
{
  0x3F, 0x06, 0x5B, 
  0x4F, 0x66, 0x6D, 
  0x7D, 0x07, 0x7F, 
  0x6F, 0x77, 0x7C, 
  0x39, 0x5E, 0x79, 
  0x71, 0x40, 0x00, 
  0x77, 0x58, 0x5E, 
  0x79, 0x71, 0x74, 
  0x10, 0x37, 0x54, 
  0x5C, 0x73, 0x50, 
  0x6D, 0x78, 0x3E
};


void setup() 
{
  unsigned char z = 0;

  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  calculate_R_inf();
  
  for(z = LED1; z <= TM1636_SDIO; z++)
  {
      pinMode(z, OUTPUT);
      digitalWrite(z, LOW);
  }
  
  for(z = Button1; z <= Button3; z++)
  {
      pinMode(z, INPUT_PULLUP);
  }
  
  Wire.begin();
  DS1307_init();
  clear_display();

  Timer1.initialize(500000);     //500ms
  Timer1.attachInterrupt( timerIsr ); 
}


void loop() 
{
  get_time();
  display_values();
  
  change_display_brightness();
  change_display_mode();
  adjust_time();

  delay(loop_delay); 

  //Modify the button repeat rate after it's pressed for 1 sec
  if ( loop_delay == key_delay_long ) {
    slow_repeat--;
    if (slow_repeat == 0)
      loop_delay = key_delay_med;
  }
}


void timerIsr()
{
  //Toggle LED once every 500ms
  display_blink ^= 1;   
  sec_dp_blink ^= 1;
}


void start_variable_repeat_rate()
{
  loop_delay = key_delay_long;    //start with a long repeat rate
  slow_repeat = key_repeat_delay; //trigger the variable repeat rate counter
}


void keyboard_click()
{
  digitalWrite(Buzzer, HIGH);
  delay(key_click);
  digitalWrite(Buzzer, LOW);
}


unsigned char get_keypad()
{
  boolean but1_pressed = (digitalRead(Button1) == false);
  boolean but2_pressed = (digitalRead(Button2) == false);
  boolean but3_pressed = (digitalRead(Button3) == false);
  
  if ( (but3_pressed) && (but3_state == unarmed) )
    but3_state = armed;  
  if ( (but2_pressed) && (but2_state == unarmed) )
    but2_state = armed;  
  if ( (but1_pressed) && (but1_state == unarmed) )
    but1_state = armed;

  if (but3_state == armed)
  {
    delay(key_debounce);
    if (digitalRead(Button3) == false) {
      but3_state = trigger;
      start_variable_repeat_rate();
      return Menu_BTN;
    } else {
      but3_state = unarmed;
      return 0;
    }
  }
  if (but2_state == armed)
  {
    delay(key_debounce);
    if (digitalRead(Button2) == false) {
      but2_state = trigger;
      start_variable_repeat_rate();
      return Inc_BTN;
    } else{
      but2_state = unarmed;
      return 0;
    }
  }
  if (but1_state == armed)
  {
    delay(key_debounce);
    if (digitalRead(Button1) == false) {
      but1_state = trigger;
      start_variable_repeat_rate();
      return Dec_BTN;
    } else{
      but1_state = unarmed;
      return 0;
    }
  }
  
  if (but3_state == trigger) 
  {
    if (but3_pressed) 
      return Menu_BTN;
    else {
      but3_state =unarmed;
      loop_delay = key_delay_short;
      return 0;
    } 
  }
  if (but2_state == trigger) 
  {
    if (but2_pressed) 
      return Inc_BTN;
    else {
      but2_state =unarmed;
      loop_delay = key_delay_short;
      return 0;
    } 
  }
  if (but1_state == trigger) 
  {
    if (but1_pressed)
      return Dec_BTN;
    else {
      but1_state =unarmed;
      loop_delay = key_delay_short;
      return 0;
    } 
  }
}


#ifdef DEBUG
void print_time()
{
  Serial.print("Time: "+String(h)+":");
  if (m<10)
    Serial.print("0");
  Serial.print(String(m)+":");
  if (s<10)
    Serial.print("0");
  Serial.println(String(s));
}

void print_date()
{
  Serial.println("Date: 20"+String(yr)+"/"+String(mt)+"/"+String(dt));  
}
#endif


void show_LED_states()
{
  digitalWrite(LED1, (led_states & 0x01));
  digitalWrite(LED2, ((led_states & 0x02) >> 1));
  digitalWrite(LED3, ((led_states & 0x04) >> 2));
//digitalWrite(LED4, ((led_states & 0x08) >> 3));
  if ( ((led_states & 0x08) >> 3) && (led_level != (-1)) )
    analogWrite(LED4, 10+led_level*30);
  else
    analogWrite(LED4, 0);
}


void display_values()
{ 
  if ( (display_blink == false) && (set_time_state == true) )
  {
    clear_display();
    return;
  }
  
  switch(mode)
  {
    case time_disp:
      led_states = 8;
      show_time();
      #ifdef DEBUG
      print_time();
      #endif
      break;
      
    case date_disp:
      led_states = 8;
      show_date();
      #ifdef DEBUG
      print_date();
      #endif
      break; 
      
    case year_disp:
      led_states = 8;
      show_year();
      break; 
      
    case week_disp:
      led_states = 8;
      show_day();
      break; 
      
    case temp_disp:
      led_states = 1;
      show_temperature();
      break; 
      
    case lux_disp:
      led_states = 2;
      display_light_intensity();
      break; 
      
    case BV_disp:
      led_states = 4;
      show_battery_voltage();
      break; 
      
    default:
      led_states = 0;
      clear_display();
      break;
  }
  show_LED_states();
}


void change_display_mode()
{
  if ( (set_time_state == false) && (get_keypad() == Dec_BTN) )
  {
    keyboard_click();
    clear_display();
    mode = mode + 1;
    if (mode > BV_disp)
      mode = time_disp;  //mode wrap around
  }
}


//TM1636 PWM duty cycles (8 levels + OFF)
//   -1 : OFF     010:  4/16    101: 12/16
//   000: 1/16    011: 10/16    110: 13/16
//   001: 2/16    100: 11/16    111: 14/16
void change_display_brightness()
{   
  if ( (set_time_state == false) && (get_keypad() == Inc_BTN) )
  {
    keyboard_click();
    led_level++;   

    //Above level 3, values too close to each other, and show no 
    //differernce in brightness , we increase two steps instead
    if (led_level>=4) 
      led_level++;
      
    if (led_level >= 0x08)
      led_level = -1;  //LED is off
  }
  
  if (led_level == (-1))
    dislay_status(led_level, Display_Off);
  else
    dislay_status(led_level, Display_On);

  show_LED_states();
}


void TM1636_start()
{
  digitalWrite(TM1636_CLK, HIGH);
  digitalWrite(TM1636_SDIO, HIGH); 
  digitalWrite(TM1636_SDIO, LOW); 
} 


void TM1636_stop()
{
  digitalWrite(TM1636_CLK, LOW);
  digitalWrite(TM1636_SDIO, LOW);
  digitalWrite(TM1636_CLK, HIGH);
  digitalWrite(TM1636_SDIO, HIGH); 
}


void TM1636_ack()
{
  digitalWrite(TM1636_CLK, LOW);
  pinMode(TM1636_SDIO, INPUT_PULLUP);
  while(digitalRead(TM1636_SDIO) == HIGH);
  digitalWrite(TM1636_CLK, HIGH);
  digitalWrite(TM1636_CLK, LOW); 
  pinMode(TM1636_SDIO, OUTPUT);
}


void TM1636_write_byte(unsigned char value)
{
  unsigned char z = 0;

  for(z = 0; z < 8; z++)
  {
    digitalWrite(TM1636_CLK, LOW);
    if ((value & 0x01) != 0)
    {
      digitalWrite(TM1636_SDIO, HIGH);
    }
    else
    {
      digitalWrite(TM1636_SDIO, LOW);
    }
    value >>= 1;
    digitalWrite(TM1636_CLK, HIGH); 
  }
}

/*
unsigned char TM1636_scan_keys()
{
  unsigned char key = 0;
  unsigned char z = 0;

  TM1636_start();
  TM1636_write_byte(Read_Key_Data_Cmd); 
  TM1636_ack();
  digitalWrite(TM1636_SDIO, HIGH);
  
  for(z = 0; z < 8; z++)
  {
    digitalWrite(TM1636_CLK, LOW);
    key >>= 1;
    delayMicroseconds(30);
    digitalWrite(TM1636_CLK, HIGH);
    pinMode(TM1636_SDIO, INPUT_PULLUP);

    if (digitalRead(TM1636_SDIO) == HIGH)
    {
      key |= 0x80;
    }
    delayMicroseconds(30);
  }
  
  TM1636_ack();
  TM1636_stop();

  return key;
}
*/

void display_data(unsigned char value, unsigned char location)
{
  TM1636_start();
  TM1636_write_byte(Fixed_Addressing);
  TM1636_ack();
  TM1636_stop();
  
  TM1636_start();
  TM1636_write_byte(Display_Address | (location & 0x03));
  TM1636_ack();

  if ( (sec_dp_blink == true) && (mode == time_disp) )
  {
    TM1636_write_byte( segmap[value]  | Display_colon );
  }
  else
  {
    TM1636_write_byte( segmap[value] );
  }
  
  TM1636_ack();
  TM1636_stop();  
}


void dislay_status(unsigned char brightness, unsigned char display_cmd)
{
  TM1636_start();
  TM1636_write_byte(display_cmd | (brightness & 0x07));
  TM1636_ack();
  TM1636_stop();
}


void clear_display()
{
  display_data(0x11, 0);
  display_data(0x11, 1);
  display_data(0x11, 2);
  display_data(0x11, 3);
}


unsigned int adc_avg(unsigned char channel)
{
  unsigned char samples = 64;
  unsigned int avg = 0;

  while(samples > 0)
  {
    avg += analogRead(channel);
    samples--;
    delayMicroseconds(10);
  }
  avg >>= 6;

  return avg;
}


void calculate_R_inf()
{
  R_inf = (-1.0 * (B / _25C_K));
  R_inf = exp(R_inf); 
  R_inf *= R0;
}


signed int measure_temperature()
{
  float T_therm = 0;

  T_therm = adc_avg(Thermistor);
  if (T_therm > 0)
  {
    T_therm = ((VDD_max / T_therm) - 1.0);
    T_therm *= R_fixed;

    T_therm = (B / (log (T_therm / R_inf)));
    T_therm = (T_therm - _0C_K);

    return T_therm;
  }
  else
  {
    return 126;
  }
}


void show_temperature()
{
  signed char T = 0;

  T = measure_temperature();

  if (T <= -40)
  {
    T = -40;
  }
  if (T > 125)
  {
    T = 126;
  }

  if ((T >= -40) && (T <= 99))
  {
    if (T > 0)
    {
      display_data(0x11, 0);
    }
    else
    {
      display_data(0x10, 0);
      T *= -1;
    }
    display_data((T / 10), 1);
    display_data((T % 10), 2);
  }
  else if ((T > 99) && (T < 126))
  {
    display_data((T / 100), 0);
    display_data(((T / 10) % 10), 1);
    display_data((T % 10), 2);
  }
  else
  {
    display_data(0x10, 0);
    display_data(0x10, 1);
    display_data(0x10, 2);
  }
  
  display_data(0x0C, 3);
}


signed int measure_light_intensity()
{
  float lux = 0;

  lux = adc_avg(LDR);
  if (lux > 0)
  {
    lux = ((VDD_max / lux) - 1.0);
    lux *= R_fixed;
    lux = (LDR_constant / lux);
  }

  if ((lux >= 0) && (lux <= 9999))
  {
    return lux;
  }
  else
  {
    return -1;
  }
}


void display_light_intensity()
{
  signed int lx = 0;
  
  lx = measure_light_intensity();
  
  if (lx < 0)
  {
    display_data(0x10, 0);
    display_data(0x10, 1);
    display_data(0x10, 2);
    display_data(0x10, 3);
    
  }
  else
  {
    display_data((lx / 1000), 0);
    display_data(((lx / 100) % 10), 1);
    display_data(((lx / 10) % 10), 2);
    display_data((lx % 10), 3);
  }
}


void show_battery_voltage()
{
  unsigned int mV = 0;

  mV = adc_avg(V_batt);
  mV = ((mV * 5000.0) / VDD_max);
  display_data((mV / 1000), 0);
  display_data(((mV / 100) % 10), 1);
  display_data(((mV / 10) % 10), 2);
  display_data((mV % 10), 3);
}


unsigned char bcd_to_decimal(unsigned char value)                
{                                                                                          
  return ((value & 0x0F) + (((value & 0xF0) >> 0x04) * 0x0A));
}                                
                                                             

unsigned char decimal_to_bcd(unsigned char value)
{
  return (((value / 0x0A) << 0x04) & 0xF0) | ((value % 0x0A) & 0x0F);
} 


void DS1307_init()
{
//DS1307_write(DS1307_sec_reg, 0x00);
}


void DS1307_block_read()
{                                     
  unsigned char value = 0; 
  
  Wire.beginTransmission(DS1307_addr);   
  Wire.write(DS1307_sec_reg);  
  Wire.endTransmission();             
  Wire.requestFrom(DS1307_addr, 7);  
  s = Wire.read();
  m = Wire.read();
  h = Wire.read();
  wk = Wire.read();
  dt = Wire.read();
  mt = Wire.read();
  yr = Wire.read();
}                     


void DS1307_write(unsigned char address, unsigned char value)    
{  
  Wire.beginTransmission(DS1307_addr);   
  Wire.write(address);
  Wire.write(value);                   
  Wire.endTransmission();
}  


void DS1307_block_write()    
{  
  Wire.beginTransmission(DS1307_addr);   
  Wire.write(DS1307_sec_reg);
  Wire.write(0x80);                    //Stop RTC
  Wire.write(decimal_to_bcd(m));
  Wire.write(decimal_to_bcd(h));
  Wire.write(decimal_to_bcd(wk));
  Wire.write(decimal_to_bcd(dt));
  Wire.write(decimal_to_bcd(mt));
  Wire.write(decimal_to_bcd(yr));
  Wire.endTransmission();
}  


void get_time()
{
  if (set_time_state == false)
  {    
    DS1307_block_read();
  
    s &= 0x7F;
    s = bcd_to_decimal(s);
  
    m &= 0x7F;
    m = bcd_to_decimal(m);
  
    h &= 0x3F;
    h = bcd_to_decimal(h);
  
    dt &= 0x3F;
    dt = bcd_to_decimal(dt);
  
    mt &= 0x1F; 
    mt = bcd_to_decimal(mt);
    
    yr = bcd_to_decimal(yr);
  
    wk &= 0x07;
    wk = bcd_to_decimal(wk);
  }
}


void show_time()
{
  if (h<10)
    display_data(17, 0);
  else
    display_data((h / 10), 0);
  display_data((h % 10), 1);
  display_data((m / 10), 2);
  display_data((m % 10), 3);
}


void show_date()
{
  display_data((mt / 10), 0);
  display_data((mt % 10), 1);
  display_data((dt / 10), 2);
  display_data((dt % 10), 3);
}


void show_year()
{
  display_data(2, 0);
  display_data(0, 1);
  display_data((yr / 10), 2);
  display_data((yr % 10), 3);
}


void show_day()
{
  unsigned char n = 0;
  unsigned char buf[4];
  
  switch(wk)
  {
    case 1:  //Monday
      buf[0] = 25;
      buf[1] = 25;
      buf[2] = 27;
      buf[3] = 26;
      break;
      
    case 2:  //Tuesday
      buf[0] = 31;
      buf[1] = 32;
      buf[2] = 21;
      buf[3] = 17;
      break;
      
    case 3:  //Wednesday
      buf[0] = 32;
      buf[1] = 32;
      buf[2] = 21;
      buf[3] = 20;
      break;
      
    case 4:  //Thursday
      buf[0] = 31;
      buf[1] = 23;
      buf[2] = 29;
      buf[3] = 17;
      break;
      
    case 5:  //Friday
      buf[0] = 22;
      buf[1] = 29;
      buf[2] = 24;
      buf[3] = 17;
      break;
      
    case 6:  //Saturday
      buf[0] = 30;
      buf[1] = 18;
      buf[2] = 31;
      buf[3] = 17;
      break;
    case 7:  //Sunday
      buf[0] = 30;
      buf[1] = 32;
      buf[2] = 26;
      buf[3] = 17;
      break;
  }

  for(n = 0; n < 4; n++)
  {
    display_data(buf[n], n);
  }
}


void set_time()
{
  DS1307_block_write();
  DS1307_write(DS1307_sec_reg, 0x00);  //Start RTC
}


unsigned char inc_dec(signed char value, signed char max_value, signed char min_value)
{
  if (get_keypad() == Inc_BTN)
  {
    keyboard_click();
    value++;
  }
  if (value > max_value)
  {
    value = min_value;
  }
  
  if (get_keypad() == Dec_BTN)
  {
    keyboard_click();
    value--;
  }
  if (value < min_value)
  {
    value = max_value;
  }
  
  return value;
}


void adjust_time()
{ 
  if ((get_keypad() == Menu_BTN) && (set_time_state == false))
  {
    keyboard_click();
    while( get_keypad() == Menu_BTN ); 

    set_time_state = true;
    DS1307_write(DS1307_sec_reg, 0x80);  //Stop RTC
    mode = time_disp;
    menu = set_hour;
    delay(500);
  }

  if (set_time_state == true)
  {
    if (get_keypad() == Menu_BTN)
    {
      keyboard_click();
      while( get_keypad() == Menu_BTN ); 
      menu = menu + 1;

      switch(menu) 
      {
        case set_month:
          mode = date_disp;
          break;
        case set_year:
          mode = year_disp;
          break;
        case set_week:
          mode = week_disp;
          break;
        case set_end:
          menu = 0;
          mode = time_disp;
          set_time_state = false;
          set_time();
          return;
      }
    }
    
    switch(menu) 
    {
      case set_hour:
        h = inc_dec(h, 23, 0);
        break;
      case set_minute:
        m = inc_dec(m, 59, 0);
        break;
      case set_month:
        mt = inc_dec(mt, 12, 1);
        break;
      case set_date:
        dt = inc_dec(dt, 31, 1);
        break;
      case set_year:
        yr = inc_dec(yr, 99, 0);
        break;
      case set_week:
        wk = inc_dec(wk, 7, 1);
        break;
    }
  }
}

