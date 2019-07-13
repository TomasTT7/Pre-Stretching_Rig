/*
  SUPERPRESSURE BALLOON PRESSURIZING RIG

    Arduino Pro Mini 5V 8MHz (powered through VCC by 5V LDO regulator)
    LD1117 low drop voltage regulator (Vout: 5V, Vin: 6-15V)
    2x Mitsumi R-14 A221 pump (5-7V input) + 12N65 N-Channel MOSFET (220Ω and 360kΩ resistors at gate)
    3-way valve (5-6V, 190mA) + BC547C NPN transistor (220Ω resistor at base)
    YF-S402 water flow sensor (5-42V, 15mA)
    MPS20N0040D pressure sensor with HX711 analog-to-digital converter (2.6-5.5V)
    DS18B20 digital thermometer (3-5.5V, 4.7kΩ pull-up resistor between VCC and DQ)
    SPP-C Bluetooth module (3.3-6V, 3.3kΩ and 6.2kΩ voltage divider at RX pin)
    OLED display (3.3-5V, 128x64 pixels)
    Wall Adapter 7.5V (2x pump, 5V regulator)
*/


#include <util/delay.h>


extern "C"{
  #include "fonts.h"
}


#define BLUETOOTH                                                     // uncomment to enable Bluetooth data logging / communication

#define SCK                           2                               // pin D2 (HX711 clock pin)
#define DT                            3                               // pin D3 (HX711 data pin)
#define BUTTON1                       4                               // pin D4 (button 1)
#define BUTTON2                       5                               // pin D5 (button 2)
#define BUTTON3                       6                               // pin D6 (button 3)
#define TEMP_PIN                      7                               // pin D7 (DS18B20 DQ pin) - must be on PORTD
#define FLOW_PIN                      9                               // pin D9 (YF-S402) - must be on PORTB
#define PUMP_PIN                      10                              // pin D10 (MOSFET's gate) - PWM pin (TIMER1), must be on PORTB
#define VALVE_PIN                     11                              // pin D11 (transistor's base) - must be on PORTB

#define HX_CALIB_0                    -1250700.0                      // zero offset at 20°C
#define HX_CALIB_1                    727.0                           // [raw/Pa] scaling coefficient
#define HX_CALIB_T                    -11715.0                        // [raw/°C] temperature compensation coefficient
#define YF_CALIB                      21.1                            // [L/Hz] air flow scaling coefficient
#define YF_CAP                        4.4                             // [L/min] cap just above pumps' output to limit occasional spikes

#define OLED_ADDRESS                  0x3C

#define B_PRESS                       7                               // 7 * 20ms = 140ms
#define B_DEBOUNCE                    6                               // 6 * 20ms = 120ms

#define TIMER1_RES                    20                              // [ms] interrupt fires every x ms
#define DISPLAY_RATE                  13                              // 13 * 20ms = 260ms (update rate of displayed data)
#define SENSOR_RATE                   5                               // 5 * 20ms = 100ms (sensor data acquisition rate)
#define CONTROL_RATE                  5                               // 5 * 20ms = 100ms (program/manual pump control rate)
#define DATA_RATE                     25                              // 25 * 20ms = 500ms (data transmission rate)

#define PRESSURE_STEP                 50                              // [Pa] for manual setting
#define RATE_STEP                     10                              // [Pa/h] for manual setting
#define PWM_STEP                      1285                            // 1285: 51 x 0.098V steps, 3855: 17 x 0.294V steps (0-65535)
#define PWM_MIN                       12850                           // minimum PWM value aside 0 
#define PWM_MAX                       65535                           // maximum allowed PWM value

#define DS18B20_RES                   9                               // 9: 9-bit, 10: 10-bit, 11: 11-bit, 12: 12-bit


uint16_t displayPressure_crnt         = 0;                            // [Pa] 0-65535 range (0-65535Pa)
uint16_t displayPressure_set          = 0;                            // [Pa] 0-65535 range (0-65535Pa)
uint16_t displayTemperature_crnt      = 0;                            // [0.1°C] 0-65535 range (0.0-6553.5°C)
uint16_t displayRate_set              = 0;                            // [Pa/h] 0-65535 range (0-65535Pa/h)
uint32_t displayTime_crnt             = 0;                            // [s] 0-4294967295 range (0-4294967295s, 49710d)
uint32_t displayVolume_total          = 0;                            // [10uL] 0-4294967295 range (0-42949L) 
uint16_t displayVolume_rate           = 0;                            // [mL/min] 0-65535 range (0-65.535L/min)
uint16_t displayVoltage_pumps         = 0;                            // [mV] 0-65535 range (0-65.535V)
uint8_t pressureNeg                   = 0;                            // signal the measured pressure is negative
uint8_t temperatureNeg                = 0;                            // signal the measured temperature is negative
uint8_t valve_open                    = 0;                            // signal open/closed valve

volatile uint8_t measuredSensors      = 0;
volatile uint8_t measuredDisplay      = 0;
volatile uint8_t measuredControl      = 0;
volatile uint8_t measuredData         = 0;
volatile uint32_t measuredDuration    = 0;

uint32_t sensorsPrev                  = 0;
uint32_t pumpPrev                     = 0;

uint8_t sys_state                     = 0;                            // 0: Menu, 1: Program Menu, 2: Program, 3: Pressure, 4: Rate, 5: Valve, 6: Volume
uint8_t menu_sel                      = 0;                            // 0: Program, 1: Pressure, 2: Rate, 3: Valve, 4: Volume
uint8_t menu_sel_pg                   = 0;                            // 0: Page 1, 1: Page 2
uint8_t program_sel                   = 0;                            // 0: Program 1, 1: Program 2, 2: Program 3, 3: Program 4

uint8_t pause                         = 0;                            // flag to signal running/not running system , 0: continue program 1: pause program
uint8_t program_stage                 = 0;                            // keeps track of the current stage of an automatic program
uint16_t pwm_val                      = 0;                            // 0-65535 Timer1 (16-bit)
uint32_t pressure_target              = 0;                            // [10uPa] 0-4294967295 range (0-42949Pa) derived from set pressure or gradually increased based on rate

volatile uint8_t pressButton1         = 0;
volatile uint8_t pressButton2         = 0;
volatile uint8_t pressButton3         = 0;
volatile uint8_t ButtonDebounce1      = 0;
volatile uint8_t ButtonDebounce2      = 0;
volatile uint8_t ButtonDebounce3      = 0;
volatile uint8_t sensors_measure      = 0;
volatile uint8_t pump_control         = 0;
volatile uint8_t data_transmit        = 0;

volatile uint16_t update_flags        = 0;                            // Bit 0: update Menu screen
                                                                      // Bit 1: update Active screen
                                                                      // Bit 2: update Active Volume screen
                                                                      // Bit 3: update Active Program screen
                                                                      // Bit 4: update Program Menu
                                                                      // Bit 5: clear screen

/* Pressure compensation for when the pumps are active */
uint8_t pump_adj_step[] = {0, 10, 13, 18, 24, 32, 38, 51};
uint32_t pump_adj[] = {0, 32969, 50995, 74493, 94476, 109202, 118387, 132374};

/* 1st pressure level [Pa], rate to 1st pressure level [Pa/h], 2nd pressure level [Pa],  rate to 2nd pressure level [Pa/h], ... */
uint16_t program1[] = {250, 5000, 1000, 100, 2000, 50};
uint16_t program2[] = {250, 5000, 800, 100, 1500, 50};
uint16_t program3[] = {3000, 30000, 5000, 2000, 7000, 200};
uint16_t program4[] = {500, 5000, 800, 200, 1500, 100, 2000, 50};


// Setup function ---------------------------------------------------------------------------------
void setup()
{
#ifdef BLUETOOTH

  Serial.begin(115200);

#endif /* BLUETOOTH */

  I2C_init();                                                         // Arduino <-> OLED

  OLED_init();
  OLED_clear();
  OLED_menu_screen();                                                 // show initial screen

  DDRD &= ~(1 << BUTTON1);                                            // D4 set as INPUT (Button)
  DDRD &= ~(1 << BUTTON2);                                            // D5 set as INPUT (Button)
  DDRD &= ~(1 << BUTTON3);                                            // D6 set as INPUT (Button)

  DDRB &= ~(1 << (FLOW_PIN - 8));                                     // D9 set as INPUT

  DDRB |= (1 << (PUMP_PIN - 8));                                      // D10 set as OUTPUT
  PORTB &= ~(1 << (PUMP_PIN - 8));                                    // D10 set LOW

  DDRB |= (1 << (VALVE_PIN - 8));                                     // D11 set as OUTPUT
  PORTB &= ~(1 << (VALVE_PIN - 8));                                   // D11 set LOW
  
  DS18B20_init();
  DS18B20_resolution(DS18B20_RES);
  DS18B20_init_conversion();

  HX711_init();

  Timer2_setup();                                                     // application timing
  Timer1_setup();                                                     // pump PWM driver

  measuredDuration = millis();
}


// Loop function ----------------------------------------------------------------------------------
void loop()
{
  /* Button 1 Press */
  if(pressButton1 >= B_PRESS)
  {
    switch(sys_state)
    {
      case 0:                                                         // MENU
        if(menu_sel > 0)
        {
          menu_sel--;
          update_flags |= 0b0000000000000001;                         // update Menu screen

          if(menu_sel == 0 && menu_sel_pg == 1) menu_sel_pg = 0;
        }
        break;

      case 1:                                                         // PROGRAM MENU
        if(program_sel > 0)
        {
          program_sel--;
          update_flags |= 0b0000000000010000;                         // update Program Menu screen
        }
        break;

      case 2:                                                         // PROGRAM
        break;

      case 3:                                                         // PRESSURE
        if(displayPressure_set >= PRESSURE_STEP) displayPressure_set -= PRESSURE_STEP;
        else displayPressure_set = 0;
        update_flags |= 0b0000000000000010;                           // update Active screen
        pressure_target = (uint32_t)displayPressure_crnt * 100000;
        break;

      case 4:                                                         // RATE
        if(displayRate_set >= RATE_STEP) displayRate_set -= RATE_STEP;
        else displayRate_set = 0;
        update_flags |= 0b0000000000000010;                           // update Active screen
        break;

      case 5:                                                         // VALVE
        break;

      case 6:                                                         // VOLUME
        if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
        else pwm_val = 0;
        update_flags |= 0b0000000000000100;                           // update Active Volume screen
        break;
      
      default:
        break;
    }
        
    pressButton1 = 0;
    ButtonDebounce1 = B_DEBOUNCE;
  }

  /* Button 2 Press */
  if(pressButton2 >= B_PRESS)
  {
    switch(sys_state)
    {
      case 0:                                                         // MENU
        if(menu_sel < 4)
        {
          menu_sel++;
          update_flags |= 0b0000000000000001;                         // update Menu screen

          if(menu_sel == 4 && menu_sel_pg == 0) menu_sel_pg = 1;
        }
        break;

      case 1:                                                         // PROGRAM MENU
        if(program_sel < 3)
        {
          program_sel++;
          update_flags |= 0b0000000000010000;                         // update Program Menu screen
        }
        break;

      case 2:                                                         // PROGRAM
        if(pause)
        {
          pause = 0;
        }
        else
        {
          pause = 1;
        }
        break;

      case 3:                                                         // PRESSURE
        if(displayPressure_set <= 9999 - PRESSURE_STEP) displayPressure_set += PRESSURE_STEP;
        else displayPressure_set = 9999;
        update_flags |= 0b0000000000000010;                           // update Active screen
        pressure_target = (uint32_t)displayPressure_crnt * 100000;
        break;

      case 4:                                                         // RATE
        if(displayRate_set <= 999 - RATE_STEP) displayRate_set += RATE_STEP;
        else displayRate_set = 999;
        update_flags |= 0b0000000000000010;                           // update Active screen
        break;

      case 5:                                                         // VALVE
        if(valve_open)
        {
          PORTB &= ~(1 << (VALVE_PIN - 8));                           // D11 set LOW
          valve_open = 0;
        }
        else
        {
          PORTB |= (1 << (VALVE_PIN - 8));                            // D11 set HIGH
          valve_open = 1;
        }
        update_flags |= 0b0000000000000010;                           // update Active screen
        break;

      case 6:                                                         // VOLUME
        if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
        else pwm_val = PWM_MAX;
        if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        update_flags |= 0b0000000000000100;                           // update Active Volume screen
        break;
      
      default:
        break;
    }
    
    pressButton2 = 0;
    ButtonDebounce2 = B_DEBOUNCE;
  }

  /* Button 3 Press */
  if(pressButton3 >= B_PRESS)
  {
    switch(sys_state)
    {
      case 0:                                                         // MENU
        switch(menu_sel)
        {
          case 0:                                                     // MENU: PROGRAM
            sys_state = 1;
            menu_sel = 0;
            program_sel = 0;
            pressure_target = 0;
            pwm_val = 0;
            program_stage = 0;
            valve_open = 0;
            PORTB &= ~(1 << (VALVE_PIN - 8));                         // D11 set LOW
            update_flags |= 0b0000000000110000;                       // clear screen, update Program Menu screen
            break;
          case 1:                                                     // MENU: PRESSURE
            sys_state = 3;
            menu_sel = 0;
            update_flags |= 0b0000000000100010;                       // clear screen, update Active screen
            break;
          case 2:                                                     // MENU: RATE
            sys_state = 4;
            menu_sel = 0;
            update_flags |= 0b0000000000100010;                       // clear screen, update Active screen
            break;
          case 3:                                                     // MENU: VALVE
            sys_state = 5;
            menu_sel = 0;
            update_flags |= 0b0000000000100010;                       // clear screen, update Active screen
            break;
          case 4:                                                     // MENU: VOLUME
            sys_state = 6;
            menu_sel = 0;
            pwm_val = 0;
            measuredDuration = millis();
            displayVolume_total = 0;
            valve_open = 0;
            PORTB &= ~(1 << (VALVE_PIN - 8));                         // D11 set LOW
            update_flags |= 0b0000000000100100;                       // clear screen, update Active Volume screen
            break;
          default:
            break;
        }
        break;

      case 1:                                                         // PROGRAM MENU
        sys_state = 2;
        measuredDuration = millis();
        update_flags |= 0b0000000000101000;                           // clear screen, update Active Program screen
        break;

      case 2:                                                         // PROGRAM
        sys_state = 0;
        menu_sel = 0;
        menu_sel_pg = 0;
        pause = 0;
        pressure_target = 0;
        displayRate_set = 0;
        displayPressure_set = 0;
        pwm_val = 0;
        measuredDuration = millis();
        update_flags |= 0b0000000000100001;                           // clear screen, update Menu screen
        break;

      case 3:                                                         // PRESSURE
        sys_state = 0;
        menu_sel = 0;
        menu_sel_pg = 0;
        update_flags |= 0b0000000000100001;                           // clear screen, update Menu screen
        break;

      case 4:                                                         // RATE
        sys_state = 0;
        menu_sel = 0;
        menu_sel_pg = 0;
        update_flags |= 0b0000000000100001;                           // clear screen, update Menu screen
        break;

      case 5:                                                         // VALVE
        sys_state = 0;
        menu_sel = 0;
        menu_sel_pg = 0;
        update_flags |= 0b0000000000100001;                           // clear screen, update Menu screen
        break;

      case 6:                                                         // VOLUME
        sys_state = 0;
        menu_sel = 0;
        menu_sel_pg = 0;
        pwm_val = 0;
        measuredDuration = millis();
        update_flags |= 0b0000000000100001;                           // clear screen, update Menu screen
        break;
      
      default:
        break;
    }
    
    pressButton3 = 0;
    ButtonDebounce3 = B_DEBOUNCE;
  }

  /* Sensors */
  if(sensors_measure)
  {
    /* Temperature */  
    int16_t temperature = DS18B20_get_temperature();
    float temperature_F = DS18B20_get_result(temperature, DS18B20_RES) * 10.0;

    if(temperature_F >= -550.0 && temperature_F <= 1250.0)
    {
      if(temperature_F < 0.0)
      {
        displayTemperature_crnt = 65535 - (uint16_t)temperature_F & 0x7FFF;
        temperatureNeg = 1;
      }
      else
      {
        displayTemperature_crnt = (uint16_t)temperature_F;
        temperatureNeg = 0;
      }
    }

    DS18B20_init_conversion();

    /* Pressure */
    while(!HX711_check_for_data());
    
    int32_t pressure = HX711_get_value(1);

    float _tempF = 0.0;                                               // to avoid occasional wrong temperature reading corrupting pressure calculation

    if(temperatureNeg) _tempF = (float)displayTemperature_crnt / 10.0 * -1.0;
    else _tempF = (float)displayTemperature_crnt / 10.0;
    
    pressure = pressure + (int32_t)(HX_CALIB_T * (20.0 - _tempF));
    
    int32_t compensation = 0;
    uint8_t pv = (uint8_t)(pwm_val / PWM_STEP);

    for(uint8_t i = 0; i < sizeof(pump_adj_step); i++)
    {
      if(pv == 0) break;
      if(pv > pump_adj_step[i]) continue;

      float ratio = ((float)pv - (float)pump_adj_step[i-1]) / ((float)pump_adj_step[i] - (float)pump_adj_step[i-1]);
      compensation = (int32_t)((float)(pump_adj[i] - pump_adj[i-1]) * ratio) + (int32_t)pump_adj[i-1];
      break;
    }

    pressure = pressure - compensation;
    
    float pressure_F = ((float)pressure + HX_CALIB_0) / HX_CALIB_1;

    if(pressure_F < 0.0)
    {
      if(pressure_F > -1.0) displayPressure_crnt = 0;
      else displayPressure_crnt = (65535 - (uint16_t)pressure_F & 0x7FFF) + 1;
      pressureNeg = 1;
    }
    else
    {
      displayPressure_crnt = (uint16_t)pressure_F;
      pressureNeg = 0;
    }

    /* Air Flow */
    uint32_t duration = 0;
    uint32_t dt = 0;
    float frequency = 0.0;
    float liter = 0.0;
    
    if(sys_state == 6)
    {
      duration = pulseIn(9, LOW, 30000);

      if(duration > 0)
      {
        frequency = 1.0 / (float)(duration * 2) * 1000000.0;
        liter = frequency / YF_CALIB;
        if(liter > YF_CAP) liter = YF_CAP;
      }
      
      displayVolume_rate = (uint16_t)(liter * 1000.0);
      dt = millis() - sensorsPrev;
      displayVolume_total = (uint32_t)(liter * ((float)dt / 60000.0) * 100000.0) + displayVolume_total;
    }
    else
    {
      displayVolume_rate = 0;
    }
    
    sensorsPrev = millis();
    sensors_measure = 0;
  }

  /* Pump Control */
  if(pump_control)
  {
    uint32_t _press = 0;
    uint32_t _rate = 0;
    int32_t _press_crnt = 0;

    if(pressureNeg) _press_crnt = (int32_t)displayPressure_crnt * (-1);
    else _press_crnt = (int32_t)displayPressure_crnt;
    
    uint32_t dt = millis() - pumpPrev;
    
    switch(sys_state)
    {
      case 0:                                                         // MENU
        if(pressure_target / 100000 < displayPressure_set)
        {
          pressure_target = pressure_target + (uint32_t)displayRate_set * dt / 36;
        }
        else
        {
          pressure_target = (uint32_t)displayPressure_set * 100000;
        }

        if(pressure_target <= 0) {pwm_val = 0; break;}

        if(_press_crnt < (int32_t)(pressure_target / 100000))
        {
          if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
          else pwm_val = PWM_MAX;
          if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        }
        else
        {
          if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
          else pwm_val = 0;
        }
        break;

      case 1:                                                         // PROGRAM MENU
        pwm_val = 0;
        break;

      case 2:                                                         // PROGRAM
        switch(program_sel)
        {
          case 0:                                                     // Program 1
            for(uint8_t i = 0; i < sizeof(program1) / 4; i++)
            {
              if(_press_crnt < (int32_t)program1[i*2])
              {
                if(program_stage < i) program_stage = i;
                break;
              }
              if(i >= sizeof(program1) / 4 - 1) program_stage = i;
            }
            
            _press = program1[program_stage * 2];
            _rate = program1[program_stage * 2 + 1];
            break;

          case 1:                                                     // Program 2
            for(uint8_t i = 0; i < sizeof(program2) / 4; i++)
            {
              if(_press_crnt < (int32_t)program2[i*2])
              {
                if(program_stage < i) program_stage = i;
                break;
              }
              if(i >= sizeof(program2) / 4 - 1) program_stage = i;
            }
            
            _press = program2[program_stage * 2];
            _rate = program2[program_stage * 2 + 1];
            break;

          case 2:                                                     // Program 3
            for(uint8_t i = 0; i < sizeof(program3) / 4; i++)
            {
              if(_press_crnt < (int32_t)program3[i*2])
              {
                if(program_stage < i) program_stage = i;
                break;
              }
              if(i >= sizeof(program3) / 4 - 1) program_stage = i;
            }
            
            _press = program3[program_stage * 2];
            _rate = program3[program_stage * 2 + 1];
            break;

          case 3:                                                     // Program 4
            for(uint8_t i = 0; i < sizeof(program4) / 4; i++)
            {
              if(_press_crnt < (int32_t)program4[i*2])
              {
                if(program_stage < i) program_stage = i;
                break;
              }
              if(i >= sizeof(program4) / 4 - 1) program_stage = i;
            }
            
            _press = program4[program_stage * 2];
            _rate = program4[program_stage * 2 + 1];
            break;

          default:
            break;
        }

        if(_press > 9999) displayPressure_set = 9999;
        else displayPressure_set = _press;

        if(_rate > 999) displayRate_set = 999;
        else displayRate_set = _rate;

        if(pressure_target / 100000 < _press)
        {
          pressure_target = pressure_target + _rate * dt / 36;
        }
        else
        {
          pressure_target = _press * 100000;
        }

        if(_press_crnt < (int32_t)(pressure_target / 100000))
        {
          if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
          else pwm_val = PWM_MAX;
          if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        }
        else
        {
          if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
          else pwm_val = 0;
        }
        break;
      
      case 3:                                                         // PRESSURE
        if(pressure_target / 100000 < displayPressure_set)
        {
          pressure_target = pressure_target + (uint32_t)displayRate_set * dt / 36;
        }
        else
        {
          pressure_target = (uint32_t)displayPressure_set * 100000;
        }

        if(pressure_target <= 0) {pwm_val = 0; break;}

        if(_press_crnt < (int32_t)(pressure_target / 100000))
        {
          if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
          else pwm_val = PWM_MAX;
          if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        }
        else
        {
          if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
          else pwm_val = 0;
        }
        break;

      case 4:                                                         // RATE
        if(pressure_target / 100000 < displayPressure_set)
        {
          pressure_target = pressure_target + (uint32_t)displayRate_set * dt / 36;
        }
        else
        {
          pressure_target = (uint32_t)displayPressure_set * 100000;
        }

        if(pressure_target <= 0) {pwm_val = 0; break;}

        if(_press_crnt < (int32_t)(pressure_target / 100000))
        {
          if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
          else pwm_val = PWM_MAX;
          if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        }
        else
        {
          if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
          else pwm_val = 0;
        }
        break;

      case 5:                                                         // VALVE
        if(pressure_target / 100000 < displayPressure_set)
        {
          pressure_target = pressure_target + (uint32_t)displayRate_set * dt / 36;
        }
        else
        {
          pressure_target = (uint32_t)displayPressure_set * 100000;
        }

        if(pressure_target <= 0) {pwm_val = 0; break;}

        if(_press_crnt < (int32_t)(pressure_target / 100000))
        {
          if(pwm_val <= PWM_MAX - PWM_STEP) pwm_val += PWM_STEP;
          else pwm_val = PWM_MAX;
          if(pwm_val <= PWM_MIN) pwm_val = PWM_MIN;
        }
        else
        {
          if(pwm_val >= PWM_MIN + PWM_STEP) pwm_val -= PWM_STEP;
          else pwm_val = 0;
        }
        break;
      
      case 6:                                                         // VOLUME
        break;
        
      default:
        break;
    }
    
    if(!pause)
    {
      PWM_set_duty_cycle(pwm_val);
    }
    else
    {
      PWM_set_duty_cycle(0);
    }

    pumpPrev = millis();
    pump_control = 0;
  }
  
  /* Pump Voltage */
  displayVoltage_pumps = (uint16_t)((-0.027 * (((float)pwm_val - 12850.0) / 1285.0 + 1.0) + 2.529) * ((float)pwm_val / 65535.0 * 5.0) * 1000.0);

  /* Duration */
  displayTime_crnt = (millis() - measuredDuration) / 1000;

  /* Data Transmission */
#ifdef BLUETOOTH

  if(data_transmit)
  {
    Serial.print(millis());                                           // [ms]
    Serial.print(",");
    Serial.print(displayTime_crnt);                                   // [s]
    Serial.print(",");
    if(pressureNeg) Serial.print("-");
    Serial.print(displayPressure_crnt);                               // [Pa]
    Serial.print(",");
    if(temperatureNeg) Serial.print("-");
    Serial.print(displayTemperature_crnt);                            // [0.1°C]
    Serial.print(",");
    Serial.print(displayVolume_rate);                                 // [mL/min]
    Serial.print(",");
    Serial.println(displayVolume_total);                              // [10uL]
    
    data_transmit = 0;
  }
    
#endif /* BLUETOOTH */
  
  /* Update OLED */
  if(update_flags & 0b0000000000100000)                               // clear screen
  {
    update_flags &= 0b1111111111011111;
    OLED_clear();
  }
  
  if(update_flags & 0b0000000000000001)                               // update Menu screen
  {
    update_flags &= 0b1111111111111110;
    OLED_update_menu_screen();
  }

  if(update_flags & 0b0000000000000010)                               // update Active screen
  {
    update_flags &= 0b1111111111111101;
    OLED_update_active_screen();
  }

  if(update_flags & 0b0000000000000100)                               // update Active Volume screen
  {
    update_flags &= 0b1111111111111011;
    OLED_update_active_volume_screen();
  }

  if(update_flags & 0b0000000000001000)                               // update Active Program screen
  {
    update_flags &= 0b1111111111110111;
    OLED_update_active_program_screen();
  }

  if(update_flags & 0b0000000000010000)                               // update Program Menu screen
  {
    update_flags &= 0b1111111111101111;
    OLED_update_program_menu_screen();
  }
}


// Timer1 functions -------------------------------------------------------------------------------
/*
  Timer0 used by Arduino's core (delay(), millis(), micros()) - pins D5 and D6.
  Timer1 16-bit used by Servo library - pins D9 and D10.
  Timer2 8-bit used by Tone library - pins D3 and D11.
  
  D4 - button 1 (91k resistor to GND)
  D5 - button 2 (91k resistor to GND)
  D6 - button 3 (100k resistor to GND)
*/


/*
  Timer2 in CTC Mode running in the background providing 20ms time pulse.
*/
void Timer2_setup(void)
{
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);                   // 8MHz/1024 prescaling, CLK=7.8125kHz, CTC mode
  TCCR2A = (1 << WGM21);                                              // CTC mode (OCR2A - top)
  OCR2A = 156;                                                        // output compare match every 19.968ms  
  TCNT2 = 0;                                                          // initialize counter
  TIMSK2 |= (1 << OCIE2A);                                            // enable output compare A match interrupt
  sei();
}


/*
  Timer/Counter2 Compare Match A interrupt service routine is called when TCNT2 matches OCR2A.
*/
ISR(TIMER2_COMPA_vect)
{
  TCNT2 = 0;                                                          // reset counter

  uint8_t buttons = PIND;                                             // read D4, D5 and D6
  
  /* Button 1 Press */
  if(ButtonDebounce1)
  {
    ButtonDebounce1--;
  }
  else
  {
    if(buttons & (1 << BUTTON1)) pressButton1++;
    else pressButton1 = 0;
  }

  /* Button 2 Press */
  if(ButtonDebounce2)
  {
    ButtonDebounce2--;
  }
  else
  {
    if(buttons & (1 << BUTTON2)) pressButton2++;
    else pressButton2 = 0;
  }

  /* Button 3 Press */
  if(ButtonDebounce3)
  {
    ButtonDebounce3--;
  }
  else
  {
    if(buttons & (1 << BUTTON3)) pressButton3++;
    else pressButton3 = 0;
  }

  /* Active - Periodic Display Updates */
  if(sys_state == 2 || sys_state == 3 || sys_state == 4 || sys_state == 5 || sys_state == 6) measuredDisplay++;
  else measuredDisplay = 0;

  if(measuredDisplay >= DISPLAY_RATE)
  {
    switch(sys_state)
    {
      case 0:                                                           // MENU
        break;
  
      case 1:                                                           // PROGRAM MENU
        break;
  
      case 2:                                                           // PROGRAM
        update_flags |= 0b0000000000001000;                             // update Active Program screen
        break;
  
      case 3:                                                           // PRESSURE
        update_flags |= 0b0000000000000010;                             // update Active screen
        break;
  
      case 4:                                                           // RATE
        update_flags |= 0b0000000000000010;                             // update Active screen
        break;
  
      case 5:                                                           // VALVE
        update_flags |= 0b0000000000000010;                             // update Active screen
        break;
  
      case 6:                                                           // VOLUME
        update_flags |= 0b0000000000000100;                             // update Active Volume screen
        break;
  
      default:
        break;
    }
    measuredDisplay = 0;
  }

  /* Sensors */
  measuredSensors++;
  
  if(measuredSensors >= SENSOR_RATE)
  {
    measuredSensors = 0;
    sensors_measure = 1;
  }

  /* Pump */
  measuredControl++;

  if(measuredControl >= CONTROL_RATE)
  {
    measuredControl = 0;
    pump_control = 1;
  }

  /* Data */
  measuredData++;

  if(measuredData >= DATA_RATE)
  {
    measuredData = 0;
    data_transmit = 1;
  }
}


// DS18B20 functions ------------------------------------------------------------------------------
/*
  Temperature Range:  -55 to 125°C
  Accuracy:           +-0.5°C (-10 to 85°C)
                      +-2.0°C (-55 to 125°C)
  
  Resolution  Increment   Conversion
  9-bit       0.5°C       93.75ms
  10-bit      0.25°C      187.5ms
  11-bit      0.125°C     375ms
  12-bit      0.0625°C    750ms
*/


/*
  
*/
void DS18B20_init(void)
{
  DDRD &= ~(1 << TEMP_PIN);                                           // set as input
  PORTD &= ~(1 << TEMP_PIN);                                          // set LOW
}


/*
  
*/
void DS18B20_OneWire_reset(void)
{
  DDRD |= (1 << TEMP_PIN);                                            // set as output
  PORTD &= ~(1 << TEMP_PIN);                                          // set LOW
  _delay_us(500);
  
  DDRD &= ~(1 << TEMP_PIN);                                           // set as input
  PORTD &= ~(1 << TEMP_PIN);                                          // set LOW
  _delay_us(500);
}


/*
  
*/
void DS18B20_OneWire_send_byte(uint8_t data)
{
  for(uint8_t i = 8; i > 0; i--)
  {
    if(data & 1)
    {
      DDRD |= (1 << TEMP_PIN);                                        // set as output
      PORTD &= ~(1 << TEMP_PIN);                                      // set LOW
      _delay_us(5);
      
      DDRD &= ~(1 << TEMP_PIN);                                       // set as input
      PORTD &= ~(1 << TEMP_PIN);                                      // set LOW
      _delay_us(60);
    }else{
      DDRD |= (1 << TEMP_PIN);                                        // set as output
      PORTD &= ~(1 << TEMP_PIN);                                      // set LOW
      _delay_us(60);
      
      DDRD &= ~(1 << TEMP_PIN);                                       // set as input
      PORTD &= ~(1 << TEMP_PIN);                                      // set LOW
    }
    
    data >>= 1;
  }
}


/*
  
*/
uint8_t DS18B20_OneWire_get_byte(void)
{
  uint8_t data_bit;
  uint8_t data = 0;
  
  for(uint8_t i = 0; i < 8; i++)
  {
    DDRD |= (1 << TEMP_PIN);                                          // set as output
    PORTD &= ~(1 << TEMP_PIN);                                        // set LOW
    _delay_us(5);
    
    DDRD &= ~(1 << TEMP_PIN);                                         // set as input
    PORTD &= ~(1 << TEMP_PIN);                                        // set LOW
    _delay_us(5);
    
    if(PIND & (1 << TEMP_PIN)) data_bit = 1;
    else data_bit = 0;
    
    data = (data >> 1) | (data_bit << 7);
    _delay_us(50);
  }
  
  return data;
}


/*
  
*/
void DS18B20_init_conversion(void)
{
  DS18B20_OneWire_reset();
  
  DS18B20_OneWire_send_byte(0xCC);                                    // skip ROM command
  DS18B20_OneWire_send_byte(0x44);                                    // initiates temperature conversion
}


/*
  
*/
int16_t DS18B20_get_temperature(void)
{
  int16_t data;

  DS18B20_OneWire_reset();
  
  DS18B20_OneWire_send_byte(0xCC);                                    // skip ROM command
  DS18B20_OneWire_send_byte(0xBE);                                    // reads bytes from scratchpad (first two bytes temperature data)
  
  data = DS18B20_OneWire_get_byte();                                  // Low Byte
  data |= ((uint16_t)DS18B20_OneWire_get_byte() << 8);                // High Byte
  
  return data;
}


/*

*/
float DS18B20_get_result(int16_t raw, uint8_t resolution)
{
  float result = 0.0;
  
  switch(resolution)
  {
    case 9:
      result = (float)(raw >> 3) * 0.5;
      break;

    case 10:
      result = (float)(raw >> 2) * 0.25;
      break;

    case 11:
      result = (float)(raw >> 1) * 0.125;
      break;

    case 12:
      result = (float)(raw) * 0.0625;
      break;
    
    default:
      result = (float)(raw >> 3) * 0.5;
      break;
  }

  return result;
}


/*
  Resolution  Increment   Conversion  0b0xx11111
  9-bit       0.5°C       93.75ms     00
  10-bit      0.25°C      187.5ms     01
  11-bit      0.125°C     375ms       10
  12-bit      0.0625°C    750ms       11
*/
void DS18B20_resolution(uint8_t resolution)
{
  uint8_t data;
  
  if(resolution >= 9 && resolution <= 12) data = ((resolution - 9) << 5) | 0b00011111;
  else data = 0b00011111;
  
  DS18B20_OneWire_reset();
  
  DS18B20_OneWire_send_byte(0xCC);                                    // skip ROM command
  DS18B20_OneWire_send_byte(0x4E);                                    // write Scratchpad command
  DS18B20_OneWire_send_byte(0x00);                                    // TH
  DS18B20_OneWire_send_byte(0x00);                                    // TL
  DS18B20_OneWire_send_byte(data);                                    // Config
  
  DS18B20_OneWire_reset();
  
  DS18B20_OneWire_send_byte(0xCC);                                    // skip ROM command
  DS18B20_OneWire_send_byte(0x48);                                    // copy Scratchpad command
  
  DS18B20_OneWire_reset();
}


// HX711 functions --------------------------------------------------------------------------------
/*
  24-bit data output in 2's complement format.
  10Hz output data rate.
  Gain of 128. With a 5V power supply corresponds to differential input voltage of +-20mV.
*/


/*

*/
void HX711_init(void)
{
  DDRD &= ~(1 << DT);                                                 // set DT as input
  DDRD |= (1 << SCK);                                                 // set SCK as output
  PORTD &= ~(1 << SCK);                                               // set SCK LOW
}


/*
  When PD_SCK pin changes from low to high and stays at high for longer than 60µs, HX711 enters power down mode.
  When PD_SCK returns to low, chip will reset and enter normal operation mode.
  After a reset or power-down event, input selection is default to Channel A with a gain of 128.
*/
void HX711_power_down(void)
{
  PORTD &= ~(1 << SCK);                                               // set SCK LOW
  PORTD |= (1 << SCK);                                                // set SCK HIGH
  _delay_us(100);
}


/*
  Output settling time is 50ms at 80Hz output data rate and 400ms at 10Hz.
  Settling time refers to the time from power up, reset, input channel change and gain change to valid stable output data. 
*/
void HX711_power_up(void)
{
  PORTD &= ~(1 << SCK);                                               // set SCK LOW
}


/*
  25-27 positive pulses on CLK pin.
  Clocked out 24-bit output value + 1 to 3 pulses setting gain and output channel.
  
    Pulses  Channel   Gain    uint8_t channel
    25      A         128     1
    26      B         32      2
    27      A         64      3
*/
int32_t HX711_get_value(uint8_t channel)
{
  int32_t data;
  uint8_t pulses = channel;

  if(pulses < 1 || pulses > 3) pulses = 1;

  // first clock out 24 data bits
  for(uint8_t i = 0; i < 24; i++)
  {
    PORTD |= (1 << SCK);                                              // set SCK HIGH
    _delay_us(2);

    if(PIND & (1 << DT)) data |= 0x01;                                // read DT pin
    if(i < 23) data = data << 1;                                      // don't shift after the last bit
    
    PORTD &= ~(1 << SCK);                                             // set SCK LOW
    _delay_us(2);
  }

  // then remaining pulses
  for(uint8_t i = 0; i < pulses; i++)
  {
    PORTD |= (1 << SCK);                                              // set SCK HIGH
    _delay_us(2);
    PORTD &= ~(1 << SCK);                                             // set SCK LOW
    _delay_us(2);
  }

  // complete 32-bit signed integer
  if(data & 0x800000) data |= 0xFF000000;
  
  return data;
}


/*
  When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock input PD_SCK
  should be low. When DOUT goes to low, it indicates data is ready for retrieval.
*/
uint8_t HX711_check_for_data(void)
{
  uint8_t reg = PIND;
  
  if(reg & (1 << DT)) return 0;
  else return 1;
}


// PWM functions ----------------------------------------------------------------------------------
/*
  TIMER0  8-bit   D5, D6    delay(), millis(), micros()
  TIMER1  16-bit  D9, D10   Servo
  TIMER2  8-bit   D3, D11   Tone
*/


/*
  Timer1 in Fast PWM Mode providing output on OC1B (PB2/D10).
*/
void Timer1_setup(void)
{
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);                 // 8MHz/1 prescaling, F_PWM=122Hz, Fast PWM mode
  TCCR1A = (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);               // clear OC1B on compare match - set at BOTTOM, Fast PWM mode
  OCR1B = 0;                                                          // initially 0% duty cycle
  OCR1A = 65535;                                                      // TOP value - initially full range
  TCNT1 = 0;                                                          // initialize counter
}


/*
  Sets a 16-bit value to the Timer1's output compare register (OCR1B).
*/
void PWM_set_duty_cycle(uint16_t val)
{
  OCR1B = val;
}


/*
  Sets a 16-bit value to the Timer1's output compare register (OCR1A) which represents the Timer's TOP value.
*/
void PWM_set_top(uint16_t val)
{
  OCR1A = val;
}


// I2C functions ----------------------------------------------------------------------------------
/*
  I2C interface between Arduino and OLED display. 
*/


/*
  
*/
void I2C_init(void)
{
  TWSR = 0x00;                                                        // set the prescaler value to 1
  //TWBR = 0x48;                                                        // set the division factor for 100kHz clock signal (0x48 -> 16000000/(16+2*72*1)=100000)
  TWBR = 0x02;                                                        // set the division factor for 400kHz clock signal (0x02 -> 8000000/(16+2*2*1)=400000)
  TWCR = (1<<TWEN);                                                   // I2C enable
}


/*

*/
void I2C_start(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);                             // TWINT - clearing the 'job finished' flag, TWSTA - if the bus is clear become Master, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


/*
  
*/
void I2C_stop(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);                             // TWSTO - generate a Stop condition
}


/*

*/
void I2C_write_byte(uint8_t u8data)
{
  TWDR = u8data;                                                      // fill the Data Register
  TWCR = (1<<TWINT)|(1<<TWEN);                                        // TWINT - clearing the 'job finished' flag, TWEN - I2C enable
  while ((TWCR & (1<<TWINT)) == 0);                                   // waiting for the 'job finished' flag
}


// OLED functions ---------------------------------------------------------------------------------
/*
  OLED display 128x64 pixels (I2C 400kHz max).
  Approximate transmission duration via 100 and 400kHz I2C.
    OLED_init()                     27 bytes          2.4ms                 0.6ms
    OLED_draw_string_8x16()         16+16*n bytes     1.8ms + 1.44ms * n    0.45ms + 0.36ms * n
    OLED_draw_char_16x32()          96 bytes          8.6ms                 2.15ms
    OLED_clear()                    1034 bytes        93.1ms                23.3ms
    OLED_update_menu_screen()                         65.9ms                16.5ms
    OLED_update_active_screen()                       105.3ms               26.3ms
    OLED_update_active_volume_screen()                113.3ms               28.3ms
    OLED_update_program_menu_screen()                 71.6ms                17.9ms
    OLED_update_active_program_screen()               105.3ms               26.3ms
  OLED responds immediatelly. No gaps between successive bytes via I2C.
*/


/*
  keywords:
    SEG (segment) = COL (column) = byte of data (bits represent 8 rows within the column)
    COM = row
    Page = 8 rows of pixels of 128 columns
    Display = 8 pages
*/
void OLED_init(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0xAE);                                               // DISPLAY_OFF
  I2C_write_byte(0xA8);                                               // SET_MUX_RATIO
  I2C_write_byte(0x3F);
  I2C_write_byte(0xD3);                                               // SET_DISPLAY_OFFSET
  I2C_write_byte(0x00);
  I2C_write_byte(0x40);                                               // SET_DISPLAY_START_LINE
  I2C_write_byte(0xA1);                                               // SET_SEGMENT_REMAP
  I2C_write_byte(0xC8);                                               // SET_COM_SCAN_MODE
  I2C_write_byte(0xDA);                                               // SET_COM_PIN_MAP
  I2C_write_byte(0x12);
  I2C_write_byte(0x81);                                               // SET_CONTRAST
  I2C_write_byte(0x7F);
  I2C_write_byte(0xA4);                                               // DISPLAY_RAM
  I2C_write_byte(0xA6);                                               // DISPLAY NORMAL
  I2C_write_byte(0xD5);                                               // SET_DISPLAY_CLK_DIV
  I2C_write_byte(0x80);
  I2C_write_byte(0x8D);                                               // SET_CHARGE_PUMP
  I2C_write_byte(0x14);
  I2C_write_byte(0xD9);                                               // SET_PRECHARGE
  I2C_write_byte(0x22);
  I2C_write_byte(0xDB);                                               // SET_VCOMH_DESELECT
  I2C_write_byte(0x30);
  I2C_write_byte(0x20);                                               // SET_MEMORY_ADDR_MODE
  I2C_write_byte(0x00);
  I2C_write_byte(0xAF);                                               // DISPLAY_ON
  I2C_stop();
}


/*

*/
void OLED_draw_string_8x16(uint8_t * string, uint8_t column, uint8_t page, uint16_t stringsize)
{
  uint8_t * stringf = string;
  
  /* First Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *stringf - 0x20;
    uint16_t x = (uint16_t)c * 16;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    stringf++;
  }
  
  I2C_stop();

  /* Second Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page+1);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < stringsize; i++)
  {
    uint8_t c = *string - 0x20;
    uint16_t x = (uint16_t)c * 16 + 8;
    
    for(uint8_t y = 0; y < 8; y++)
    {
      I2C_write_byte(pgm_read_byte(&font8x16[x]));
      x++;
    }
    
    string++;
  }
  
  I2C_stop();
}


/*
  C     CHAR  OFFSET
  0     0     0
  1     1     64
  2     2     128
  3     3     192
  4     4     256
  5     5     320
  6     6     384
  7     7     448
  8     8     512
  9     9     576
  10    :     640
  11    ,     704
  12    -     768
  13    .     832
  14    /     896
  15          960
  PAGE
    for 2 rows only 0 or 4
    for 1 row 0 to 4
*/
void OLED_draw_char_16x32(uint8_t c, uint8_t column, uint8_t page)
{
  uint16_t offset = c * 64;
  
  for(uint8_t r = 0; r < 4; r++)
  {
    I2C_start();
    I2C_write_byte(OLED_ADDRESS << 1);                                // ADDRESS
    I2C_write_byte(0x00);                                             // BYTE_CMD_STREAM
    I2C_write_byte(0x21);                                             // SET_COLUMN_ADDRESS
    I2C_write_byte(column);
    I2C_write_byte(0x7F);
    I2C_write_byte(0x22);                                             // SET_PAGE_ADDRESS
    I2C_write_byte(page+r);
    I2C_write_byte(0x07);
    I2C_stop();
  
    I2C_start();
    I2C_write_byte(OLED_ADDRESS << 1);                                // ADDRESS
    I2C_write_byte(0x40);                                             // BYTE_DATA_STREAM
  
    for(uint16_t i = 0; i < 16; i++)
    {
      I2C_write_byte(pgm_read_byte(&font16x32[offset + (3 - r) + i * 4]));
    }
  
    I2C_stop();
  }
}


/*
  Example:
    [0xE0, 0xE0, 0xE0]
    -> 0b11100000, 0b11100000, 0b11100000 (a 3x3 pixel dot)
*/
void OLED_draw_chars_1x8(uint8_t * chars, uint8_t column, uint8_t page, uint8_t len)
{
  uint8_t * _ch = chars;
  
  /* Row */
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(column);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(page);
  I2C_write_byte(0x07);
  I2C_stop();
  
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint8_t i = 0; i < len; i++)
  {
    I2C_write_byte(*_ch++);
  }
  
  I2C_stop();
}


/*

*/
void OLED_clear(void)
{
  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x00);                                               // BYTE_CMD_STREAM
  I2C_write_byte(0x21);                                               // SET_COLUMN_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x7F);
  I2C_write_byte(0x22);                                               // SET_PAGE_ADDRESS
  I2C_write_byte(0x00);
  I2C_write_byte(0x07);
  I2C_stop();

  I2C_start();
  I2C_write_byte(OLED_ADDRESS << 1);                                  // ADDRESS
  I2C_write_byte(0x40);                                               // BYTE_DATA_STREAM
  
  for(uint16_t i = 0; i < 1024; i++)
  {
    I2C_write_byte(0x00);
  }
  
  I2C_stop();
}


/*
  Draws menu screen arrangement.
*/
void OLED_menu_screen(void)
{
  OLED_draw_string_8x16((uint8_t*)"Program ", 40, 0, 8);
  OLED_draw_string_8x16((uint8_t*)"Pressure", 40, 2, 8);
  OLED_draw_string_8x16((uint8_t*)"Rate    ", 40, 4, 8);
  OLED_draw_string_8x16((uint8_t*)"Valve   ", 40, 6, 8);

  uint8_t dot1[] = {0xC0, 0xC0, 0xC0, 0xC0};
  uint8_t dot2[] = {0x03, 0x03, 0x03, 0x03};
  OLED_draw_chars_1x8(dot1, 18, menu_sel * 2, 4);
  OLED_draw_chars_1x8(dot2, 18, menu_sel * 2 + 1, 4);
}


/*
  Draws initial screen arrangement.
*/
void OLED_update_menu_screen(void)
{
  uint8_t dot1[] = {0xC0, 0xC0, 0xC0, 0xC0};
  uint8_t dot2[] = {0x03, 0x03, 0x03, 0x03};
  uint8_t empty[] = {0x00, 0x00, 0x00, 0x00};
  
  for(uint8_t i = 0; i < 8; i++)
  {
    OLED_draw_chars_1x8(empty, 18, i, 4);
  }

  if(menu_sel_pg == 0)
  {
    OLED_draw_string_8x16((uint8_t*)"Program ", 40, 0, 8);
    OLED_draw_string_8x16((uint8_t*)"Pressure", 40, 2, 8);
    OLED_draw_string_8x16((uint8_t*)"Rate    ", 40, 4, 8);
    OLED_draw_string_8x16((uint8_t*)"Valve   ", 40, 6, 8);
    
    OLED_draw_chars_1x8(dot1, 18, menu_sel * 2, 4);
    OLED_draw_chars_1x8(dot2, 18, menu_sel * 2 + 1, 4);
  }
  else if(menu_sel_pg == 1)
  {
    OLED_draw_string_8x16((uint8_t*)"Pressure", 40, 0, 8);
    OLED_draw_string_8x16((uint8_t*)"Rate    ", 40, 2, 8);
    OLED_draw_string_8x16((uint8_t*)"Valve   ", 40, 4, 8);
    OLED_draw_string_8x16((uint8_t*)"Volume  ", 40, 6, 8);
    
    OLED_draw_chars_1x8(dot1, 18, (menu_sel - 1) * 2, 4);
    OLED_draw_chars_1x8(dot2, 18, (menu_sel - 1) * 2 + 1, 4);
  }
}


/*
  Draws active screen arrangement (Pressure, Rate, Valve).
*/
void OLED_update_active_screen(void)
{
  /* Pressure Current */
  if(displayPressure_crnt >= 1000) OLED_draw_char_16x32(displayPressure_crnt / 1000 % 10, 0, 0);
  else OLED_draw_char_16x32(15, 0, 0);
  if(displayPressure_crnt >= 100) OLED_draw_char_16x32(displayPressure_crnt / 100 % 10, 16, 0);
  else OLED_draw_char_16x32(15, 16, 0);
  if(displayPressure_crnt >= 10) OLED_draw_char_16x32(displayPressure_crnt / 10 % 10, 32, 0);
  else OLED_draw_char_16x32(15, 32, 0);
  OLED_draw_char_16x32(displayPressure_crnt % 10, 48, 0);
  OLED_draw_string_8x16((uint8_t*)"Pa", 1, 4, 2);

  if(pressureNeg) OLED_draw_string_8x16((uint8_t*)"-", 24, 4, 1);
  else OLED_draw_string_8x16((uint8_t*)" ", 24, 4, 1);
  
  /* Pressure Set */
  uint8_t _pressure[6];
 
  if(displayPressure_set >= 1000) _pressure[0] = displayPressure_set / 1000 % 10 + '0';
  else _pressure[0] = ' ';
  if(displayPressure_set >= 100) _pressure[1] = displayPressure_set / 100 % 10 + '0';
  else _pressure[1] = ' ';
  if(displayPressure_set >= 10) _pressure[2] = displayPressure_set / 10 % 10 + '0';
  else _pressure[2] = ' ';
  _pressure[3] = displayPressure_set % 10 + '0';
  _pressure[4] = 'P';
  _pressure[5] = 'a';
  
  OLED_draw_string_8x16(_pressure, 0, 6, 6);

  /* Temperature Current */
  if(displayTemperature_crnt >= 100) OLED_draw_char_16x32(displayTemperature_crnt / 100 % 10, 75, 0);
  else OLED_draw_char_16x32(15, 75, 0);
  
  OLED_draw_char_16x32(displayTemperature_crnt / 10 % 10, 91, 0);
  
  uint8_t dot[] = {0xE0, 0xE0, 0xE0};
  OLED_draw_chars_1x8(dot, 107, 3, 3);
  
  OLED_draw_char_16x32(displayTemperature_crnt % 10, 112, 0);
  OLED_draw_string_8x16((uint8_t*)"^C", 110, 4, 2);

  if(temperatureNeg) OLED_draw_string_8x16((uint8_t*)"-", 96, 4, 1);
  else OLED_draw_string_8x16((uint8_t*)" ", 96, 4, 1);

  /* Rate Set */
  uint8_t _rate[7];

  if(displayRate_set >= 100) _rate[0] = displayRate_set / 100 % 10 + '0';
  else _rate[0] = ' ';
  if(displayRate_set >= 10) _rate[1] = displayRate_set / 10 % 10 + '0';
  else _rate[1] = ' ';
  _rate[2] = displayRate_set % 10 + '0';
  _rate[3] = 'P';
  _rate[4] = 'a';
  _rate[5] = '/';
  _rate[6] = 'h';
  
  OLED_draw_string_8x16(_rate, 72, 6, 7);

  /* Time Current */
  uint8_t hour = displayTime_crnt / 3600;
  uint8_t minute = displayTime_crnt / 60 % 60;

  if(displayTime_crnt > 359999) {hour = 99; minute = 99;}

  uint8_t _time[5];

  _time[0] = hour / 10 % 10 + '0';
  _time[1] = hour % 10 + '0';
  _time[2] = ':';
  _time[3] = minute / 10 % 10 + '0';
  _time[4] = minute % 10 + '0';

  OLED_draw_string_8x16(_time, 45, 4, 5);

  /* Valve */
  if(valve_open) OLED_draw_string_8x16((uint8_t*)"V", 56, 6, 1);
  else OLED_draw_string_8x16((uint8_t*)" ", 56, 6, 1);
}


/*
  Draws active volume screen arrangement.
*/
void OLED_update_active_volume_screen(void)
{
  /* Volume Total */
  if(displayVolume_total >= 100000000) OLED_draw_char_16x32(displayVolume_total / 100000000 % 10, 56, 0);
  else OLED_draw_char_16x32(15, 56, 0);
  if(displayVolume_total >= 10000000) OLED_draw_char_16x32(displayVolume_total / 10000000 % 10, 72, 0);
  else OLED_draw_char_16x32(15, 72, 0);
  if(displayVolume_total >= 1000000) OLED_draw_char_16x32(displayVolume_total / 1000000 % 10, 88, 0);
  else OLED_draw_char_16x32(15, 88, 0);
  OLED_draw_char_16x32(displayVolume_total / 100000 % 10, 104, 0);
  OLED_draw_string_8x16((uint8_t*)"L", 120, 2, 1);
  
  /* Volume Rate */
  uint8_t _rate[6];
  uint16_t dspl = 0;

  if(displayVolume_rate > 9900) dspl = 9900;
  else dspl = displayVolume_rate;

  _rate[0] = dspl / 1000 % 10 + '0';
  _rate[1] = '.';
  _rate[2] = dspl / 100 % 10 + '0';
  _rate[3] = 'L';
  _rate[4] = '/';
  _rate[5] = 'm';

  OLED_draw_string_8x16(_rate, 0, 2, 6);

  /* Pump Voltage */
  uint8_t _volts[5];

  _rate[0] = displayVoltage_pumps / 1000 % 10 + '0';
  _rate[1] = '.';
  _rate[2] = displayVoltage_pumps / 100 % 10 + '0';
  _rate[3] = displayVoltage_pumps / 10 % 10 + '0';
  _rate[4] = 'V';

  OLED_draw_string_8x16(_rate, 0, 0, 5);

  /* Pressure Current */
  uint8_t minus[12] = {0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07};
  
  if(displayPressure_crnt >= 1000) OLED_draw_char_16x32(displayPressure_crnt / 1000 % 10, 48, 4);
  else OLED_draw_char_16x32(15, 48, 4);
  if(displayPressure_crnt >= 100) OLED_draw_char_16x32(displayPressure_crnt / 100 % 10, 64, 4);
  else OLED_draw_char_16x32(15, 64, 4);
  if(displayPressure_crnt >= 10) OLED_draw_char_16x32(displayPressure_crnt / 10 % 10, 80, 4);
  else OLED_draw_char_16x32(15, 80, 4);
  OLED_draw_char_16x32(displayPressure_crnt % 10, 96, 4);
  OLED_draw_string_8x16((uint8_t*)"Pa", 112, 6, 2);

  if(pressureNeg)
  {
    if(displayPressure_crnt >= 100 && displayPressure_crnt < 1000) OLED_draw_chars_1x8(minus, 50, 6, 12);
    else if(displayPressure_crnt >= 10 && displayPressure_crnt < 100) OLED_draw_chars_1x8(minus, 66, 6, 12);
    else if(displayPressure_crnt < 10) OLED_draw_chars_1x8(minus, 82, 6, 12);
  }

  /* Time Current */
  uint8_t hour = displayTime_crnt / 3600;
  uint8_t minute = displayTime_crnt / 60 % 60;

  if(displayTime_crnt > 359999) {hour = 99; minute = 99;}

  uint8_t _time[5];

  _time[0] = hour / 10 % 10 + '0';
  _time[1] = hour % 10 + '0';
  _time[2] = ':';
  _time[3] = minute / 10 % 10 + '0';
  _time[4] = minute % 10 + '0';

  OLED_draw_string_8x16(_time, 0, 4, 5);

  /* Temperature Current */
  uint8_t _temp[5];

  if(temperatureNeg && displayTemperature_crnt >= 100) _temp[0] = '-';
  else _temp[0] = ' ';
  if(displayTemperature_crnt >= 100) _temp[1] = displayTemperature_crnt / 100 % 10 + '0';
  else
  {
    if(temperatureNeg) _temp[1] = '-';
    else _temp[1] = ' ';
  }
  _temp[2] = displayTemperature_crnt / 10 % 10 + '0';
  _temp[3] = '^';
  _temp[4] = 'C';
  
  OLED_draw_string_8x16(_temp, 0, 6, 5);
}


/*
  Draws Program Menu screen arrangement.
*/
void OLED_update_program_menu_screen(void)
{
  uint8_t dot1[] = {0xC0, 0xC0, 0xC0, 0xC0};
  uint8_t dot2[] = {0x03, 0x03, 0x03, 0x03};
  uint8_t empty[] = {0x00, 0x00, 0x00, 0x00};
  
  for(uint8_t i = 0; i < 8; i++)
  {
    OLED_draw_chars_1x8(empty, 18, i, 4);
  }

  OLED_draw_string_8x16((uint8_t*)"Program 1", 40, 0, 9);
  OLED_draw_string_8x16((uint8_t*)"Program 2", 40, 2, 9);
  OLED_draw_string_8x16((uint8_t*)"Program 3", 40, 4, 9);
  OLED_draw_string_8x16((uint8_t*)"Program 4", 40, 6, 9);
    
  OLED_draw_chars_1x8(dot1, 18, program_sel * 2, 4);
  OLED_draw_chars_1x8(dot2, 18, program_sel * 2 + 1, 4);
}


/*
  Draws Active Program screen arrangement.
*/
void OLED_update_active_program_screen(void)
{
  /* Pressure Current */
  if(displayPressure_crnt >= 1000) OLED_draw_char_16x32(displayPressure_crnt / 1000 % 10, 0, 0);
  else OLED_draw_char_16x32(15, 0, 0);
  if(displayPressure_crnt >= 100) OLED_draw_char_16x32(displayPressure_crnt / 100 % 10, 16, 0);
  else OLED_draw_char_16x32(15, 16, 0);
  if(displayPressure_crnt >= 10) OLED_draw_char_16x32(displayPressure_crnt / 10 % 10, 32, 0);
  else OLED_draw_char_16x32(15, 32, 0);
  OLED_draw_char_16x32(displayPressure_crnt % 10, 48, 0);
  OLED_draw_string_8x16((uint8_t*)"Pa", 1, 4, 2);

  if(pressureNeg) OLED_draw_string_8x16((uint8_t*)"-", 24, 4, 1);
  else OLED_draw_string_8x16((uint8_t*)" ", 24, 4, 1);
  
  /* Pressure Set */
  uint8_t _pressure[6];
 
  if(displayPressure_set >= 1000) _pressure[0] = displayPressure_set / 1000 % 10 + '0';
  else _pressure[0] = ' ';
  if(displayPressure_set >= 100) _pressure[1] = displayPressure_set / 100 % 10 + '0';
  else _pressure[1] = ' ';
  if(displayPressure_set >= 10) _pressure[2] = displayPressure_set / 10 % 10 + '0';
  else _pressure[2] = ' ';
  _pressure[3] = displayPressure_set % 10 + '0';
  _pressure[4] = 'P';
  _pressure[5] = 'a';
  
  OLED_draw_string_8x16(_pressure, 0, 6, 6);

  /* Temperature Current */
  if(displayTemperature_crnt >= 100) OLED_draw_char_16x32(displayTemperature_crnt / 100 % 10, 75, 0);
  else OLED_draw_char_16x32(15, 75, 0);
  
  OLED_draw_char_16x32(displayTemperature_crnt / 10 % 10, 91, 0);
  
  uint8_t dot[] = {0xE0, 0xE0, 0xE0};
  OLED_draw_chars_1x8(dot, 107, 3, 3);
  
  OLED_draw_char_16x32(displayTemperature_crnt % 10, 112, 0);
  OLED_draw_string_8x16((uint8_t*)"^C", 110, 4, 2);

  if(temperatureNeg) OLED_draw_string_8x16((uint8_t*)"-", 96, 4, 1);
  else OLED_draw_string_8x16((uint8_t*)" ", 96, 4, 1);

  /* Rate Set */
  uint8_t _rate[7];

  if(displayRate_set >= 100) _rate[0] = displayRate_set / 100 % 10 + '0';
  else _rate[0] = ' ';
  if(displayRate_set >= 10) _rate[1] = displayRate_set / 10 % 10 + '0';
  else _rate[1] = ' ';
  _rate[2] = displayRate_set % 10 + '0';
  _rate[3] = 'P';
  _rate[4] = 'a';
  _rate[5] = '/';
  _rate[6] = 'h';
  
  OLED_draw_string_8x16(_rate, 72, 6, 7);

  /* Time Current */
  uint8_t hour = displayTime_crnt / 3600;
  uint8_t minute = displayTime_crnt / 60 % 60;

  if(displayTime_crnt > 359999) {hour = 99; minute = 99;}

  uint8_t _time[5];

  _time[0] = hour / 10 % 10 + '0';
  _time[1] = hour % 10 + '0';
  _time[2] = ':';
  _time[3] = minute / 10 % 10 + '0';
  _time[4] = minute % 10 + '0';

  OLED_draw_string_8x16(_time, 45, 4, 5);

  /* Program */
  uint8_t _prg[1];
  _prg[0] = program_sel + 1 + '0';
  
  OLED_draw_string_8x16(_prg, 56, 6, 1);
}













