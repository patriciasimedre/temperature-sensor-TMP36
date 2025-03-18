#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PID_v1_bc.h>  

/* ------------------- PINURI ------------------- */
#define BUTTON_OK     6
#define BUTTON_CANCEL 7
#define BUTTON_NEXT   8  // + inainte
#define BUTTON_PREV   9  // - inapoi

#define LAMP_PWM_PIN 10  // PWM catre tranzistor/bec
#define LM35_PIN     A0  // senzor temperatura LM35

/* ------------------ ENUM BUTOANE ------------------ */
enum Buttons {
  EV_OK = 0,
  EV_CANCEL,
  EV_NEXT,
  EV_PREV,
  EV_NONE,
  EV_MAX_NUM
};

/* ------------------ ENUM MENIU ------------------ */
enum Menus {
  MENU_MAIN = 0,
  MENU_START,
  MENU_TSET,
  MENU_TINC,
  MENU_TMNT,
  MENU_TRAC,
  MENU_MAX_NUM
};

/* ---------------- STATE PROCES ---------------- */
enum ProcessState {
  STATE_IDLE = 0,
  STATE_INCALZIRE,
  STATE_MENTINERE,
  STATE_RACIRE,
  STATE_DONE
};

/* -------------- OBIECT LCD -------------- */
// RS=12, E=11, D4=5, D5=4, D6=3, D7=2 (exemplu)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/* -------------- VAR GLOBALE -------------- */
const int EE_ADDR_TSET = 0;   
const int EE_ADDR_TINC = 4;   
const int EE_ADDR_TMNT = 8;   
const int EE_ADDR_TRAC = 12;  

float g_Tset    = 50.0;   
unsigned long g_tinc = 30; 
unsigned long g_tmnt = 30;
unsigned long g_trac = 30;

ProcessState processState = STATE_IDLE;
unsigned long stageStartTime = 0; 

Menus scroll_menu  = MENU_MAIN;  
Menus current_menu = MENU_MAIN;

// PID
double Input, Output, Setpoint;
PID myPID(&Input, &Output, &Setpoint, 2.0, 0.5, 1.0, DIRECT);

/* ---- DEFINIM SETURI DE PID DIFERITE PER ETAPA ---- 
   Le poți ajusta în funcție de nevoile tale.
   incKp, incKi, incKd => încălzire
   menKp, menKi, menKd => menținere
   racKp, racKi, racKd => răcire
*/
double incKp = 5.0,  incKi = 0.2,  incKd = 1.0;  // reacție puternică la încălzire
double menKp = 2.0,  menKi = 0.1,  menKd = 1.0;  // reacție mai moderată la menținere
double racKp = 3.0,  racKi = 0.0,  racKd = 2.0;  // reacție pentru răcire (bec aproape stins)

/* ---------------- PROTOTIPURI ---------------- */
Buttons GetButtons(void);
void print_menu(Menus menu);

void enter_menu();
void go_home();
void go_next();
void go_prev();

void inc_Tset();
void dec_Tset();
void inc_tinc();
void dec_tinc();
void inc_tmnt();
void dec_tmnt();
void inc_trac();
void dec_trac();

void readEEPROM();
void writeEEPROM();

void start_process();
void updateProcess();
void stop_process();

/* Tabel de functii (state machine) */
typedef void (*state_machine_handler_t)(void);
state_machine_handler_t sm[MENU_MAX_NUM][EV_MAX_NUM] =
{
  // MENU_MAIN -> (OK, CANCEL, NEXT, PREV)
  {enter_menu, go_home, go_next, go_prev},

  // MENU_START -> OK -> start_process
  {start_process, go_home, go_next, go_prev},

  // MENU_TSET
  {writeEEPROM, go_home, inc_Tset, dec_Tset},
  // MENU_TINC
  {writeEEPROM, go_home, inc_tinc, dec_tinc},
  // MENU_TMNT
  {writeEEPROM, go_home, inc_tmnt, dec_tmnt},
  // MENU_TRAC
  {writeEEPROM, go_home, inc_trac, dec_trac},
};

/* =================================================
   SETUP
==================================================== */
void setup()
{
  Serial.begin(9600);
  lcd.begin(16,2);

  // Setăm Timer 1 pentru PWM cu prescaler 1 (~62.5 kHz)
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);  // Fast PWM, 8-bit
  TCCR1B = _BV(WGM12) | _BV(CS10);  // Prescaler 1

  // Butoane cu rezistente EXTERNE de pull-down (apasat=HIGH)
  pinMode(BUTTON_OK,     INPUT);
  digitalWrite(BUTTON_OK, LOW);
  pinMode(BUTTON_CANCEL, INPUT);
  digitalWrite(BUTTON_CANCEL, LOW);
  pinMode(BUTTON_NEXT,   INPUT);
  digitalWrite(BUTTON_NEXT, LOW);
  pinMode(BUTTON_PREV,   INPUT);
  digitalWrite(BUTTON_PREV, LOW);

  pinMode(LAMP_PWM_PIN, OUTPUT);
  analogWrite(LAMP_PWM_PIN, 0);

  readEEPROM();

  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

  print_menu(scroll_menu);
}

/* =================================================
   LOOP
==================================================== */
void loop()
{
  Buttons ev = GetButtons();
  if (ev != EV_NONE) { sm[current_menu][ev](); print_menu(scroll_menu); }
  if (processState != STATE_IDLE && processState != STATE_DONE) { updateProcess(); }
  delay(100);
}

/* =================================================
   GET BUTTONS
==================================================== */
Buttons GetButtons(void)
{
  if (digitalRead(BUTTON_OK) == HIGH) {
    delay(20);
    if (digitalRead(BUTTON_OK) == HIGH) return EV_OK;
  }
  else if (digitalRead(BUTTON_CANCEL) == HIGH) {
    delay(20);
    if (digitalRead(BUTTON_CANCEL) == HIGH) return EV_CANCEL;
  }
  else if (digitalRead(BUTTON_NEXT) == HIGH) {
    delay(20);
    if (digitalRead(BUTTON_NEXT) == HIGH) return EV_NEXT;
  }
  else if (digitalRead(BUTTON_PREV) == HIGH) {
    delay(20);
    if (digitalRead(BUTTON_PREV) == HIGH) return EV_PREV;
  }
  return EV_NONE;
}

/* =================================================
   MENIU - ACȚIUNI
==================================================== */
void enter_menu() 
{
  current_menu = scroll_menu;
}

void go_home() 
{
  scroll_menu  = MENU_MAIN;
  current_menu = MENU_MAIN;
}

void go_next()
{
  scroll_menu = (Menus)((int)scroll_menu + 1);
  if (scroll_menu >= MENU_MAX_NUM) {
    scroll_menu = MENU_MAIN;
  }
  current_menu = MENU_MAIN;
}

void go_prev()
{
  if (scroll_menu == MENU_MAIN) {
    scroll_menu = (Menus)(MENU_MAX_NUM - 1);
  } else {
    scroll_menu = (Menus)((int)scroll_menu - 1);
  }
  current_menu = MENU_MAIN;
}

/* =================================================
   PRINT MENU
==================================================== */
void print_menu(Menus menu)
{
  lcd.clear();
  switch(menu)
  {
    case MENU_MAIN:
      lcd.print("MENIU PRINCIPAL");
      lcd.setCursor(0,1);
      if (processState == STATE_IDLE) {
        lcd.print("Select sub-meniu");
      } 
      else if (processState == STATE_DONE) {
        lcd.print("Proces TERMINAT");
      } 
      else {
        lcd.print("Proces ACTIV...");
      }
      break;

    case MENU_START:
      lcd.print("START PROCES?");
      lcd.setCursor(0,1);
      lcd.print("OK->Start  C->Back");
      break;

    case MENU_TSET:
      lcd.print("Tset=");
      lcd.print(g_Tset);
      lcd.print("C");
      lcd.setCursor(0,1);
      lcd.print("OK->Save  +/-->val");
      break;

    case MENU_TINC:
      lcd.print("t_incalz=");
      lcd.print(g_tinc);
      lcd.print("s");
      lcd.setCursor(0,1);
      lcd.print("OK->Save +/-->val");
      break;

    case MENU_TMNT:
      lcd.print("t_mentin=");
      lcd.print(g_tmnt);
      lcd.print("s");
      lcd.setCursor(0,1);
      lcd.print("OK->Save +/-->val");
      break;

    case MENU_TRAC:
      lcd.print("t_racire=");
      lcd.print(g_trac);
      lcd.print("s");
      lcd.setCursor(0,1);
      lcd.print("OK->Save +/-->val");
      break;

    default:
      lcd.print("PS 2020");
      break;
  }
}

/* =================================================
   FUNCȚII INC/DEC parametri
==================================================== */
void inc_Tset()  { g_Tset += 1; }
void dec_Tset()  { if(g_Tset>1) g_Tset -= 1; }
void inc_tinc()  { g_tinc += 5; }
void dec_tinc()  { if(g_tinc>5) g_tinc -= 5; }
void inc_tmnt()  { g_tmnt += 5; }
void dec_tmnt()  { if(g_tmnt>5) g_tmnt -= 5; }
void inc_trac()  { g_trac += 5; }
void dec_trac()  { if(g_trac>5) g_trac -= 5; }

/* =================================================
   EEPROM
==================================================== */
void readEEPROM()
{
  EEPROM.get(EE_ADDR_TSET, g_Tset);
  EEPROM.get(EE_ADDR_TINC, g_tinc);
  EEPROM.get(EE_ADDR_TMNT, g_tmnt);
  EEPROM.get(EE_ADDR_TRAC, g_trac);

  if (isnan(g_Tset) || g_Tset < 0 || g_Tset>200) {
    g_Tset = 50.0;
    g_tinc = 30;
    g_tmnt = 30;
    g_trac = 30;
  }
}

void writeEEPROM()
{
  EEPROM.put(EE_ADDR_TSET, g_Tset);
  EEPROM.put(EE_ADDR_TINC, g_tinc);
  EEPROM.put(EE_ADDR_TMNT, g_tmnt);
  EEPROM.put(EE_ADDR_TRAC, g_trac);

  Serial.println("EEPROM updated!");
  go_home(); 
}

/* =================================================
   PROCES: START / STOP
==================================================== */
void start_process()
{
  if(processState == STATE_IDLE) {
    processState = STATE_INCALZIRE;
    stageStartTime = millis();
    Serial.println("Proces pornit!");
  }
}
void stop_process()
{
  processState = STATE_IDLE;
  analogWrite(LAMP_PWM_PIN, 0);
  Serial.println("Proces oprit!");
}

/* =================================================
   UPDATE PROCES (3 etape)
==================================================== */
void updateProcess() {
    static unsigned long lastUpdateTime = 0; // Timpul ultimei actualizări
    unsigned long currentTime = millis();

    // Verificăm dacă au trecut cel puțin 1 secunde de la ultima actualizare
    if (currentTime - lastUpdateTime < 1000) {
        return;
    }
    lastUpdateTime = currentTime;

    // Citire temperatura actuală de la senzorul LM35
    int rawADC = analogRead(LM35_PIN);
    double currentTemp = ((rawADC * 5.0 / 1023.0) * 100.0) - 5; // Conversie la grade Celsius

    unsigned long nowSec = (currentTime - stageStartTime) / 1000;
    unsigned long remainingTime = 0;

    switch (processState) {
        /* ====================== */
        /* 1) ETAPA DE INCALZIRE */
        /* ====================== */
        case STATE_INCALZIRE: {
            remainingTime = (nowSec < g_tinc) ? (g_tinc - nowSec) : 0;

            lcd.setCursor(0, 0);
            lcd.print("INCALZIRE   ");
            lcd.setCursor(0, 1);
            lcd.print("T=");
            lcd.print(currentTemp);
            lcd.print("C R=");
            lcd.print(remainingTime);
            lcd.print("s ");

            if (currentTemp >= g_Tset) {
                // Bec stins dacă depășește temperatura setată
                analogWrite(LAMP_PWM_PIN, 0);
            } else if (currentTemp >= g_Tset + 1) {
                // Putere redusă la 30%
                analogWrite(LAMP_PWM_PIN, 255 * 0.3);
            } else {
                // Putere maximă pentru încălzire
                analogWrite(LAMP_PWM_PIN, 255);
            }

            if (nowSec >= g_tinc) {
                processState = STATE_MENTINERE;
                stageStartTime = millis();
            }
        } break;

        /* ====================== */
        /* 2) ETAPA DE MENTINERE */
        /* ====================== */
        case STATE_MENTINERE: {
            remainingTime = (nowSec < g_tmnt) ? (g_tmnt - nowSec) : 0;

            lcd.setCursor(0, 0);
            lcd.print("MENTINERE   ");
            lcd.setCursor(0, 1);
            lcd.print("T=");
            lcd.print(currentTemp);
            lcd.print("C R=");
            lcd.print(remainingTime);
            lcd.print("s ");

            if (currentTemp <= g_Tset - 1) {
                // Putere maximă dacă scade cu mai mult de 2 grade
                analogWrite(LAMP_PWM_PIN, 255);
            } else if (currentTemp >= g_Tset + 3) {
                // Putere redusă la 20% dacă depășește cu 2 grade
                analogWrite(LAMP_PWM_PIN, 255 * 0.2);
            } else {
                // Putere medie între cele două limite
                analogWrite(LAMP_PWM_PIN, 255 * 0.5);
            }

            if (nowSec >= g_tmnt) {
                processState = STATE_RACIRE;
                stageStartTime = millis();
            }
        } break;

        /* ====================== */
        /* 3) ETAPA DE RACIRE    */
        /* ====================== */
        case STATE_RACIRE: {
            remainingTime = (nowSec < g_trac) ? (g_trac - nowSec) : 0;

            lcd.setCursor(0, 0);
            lcd.print("RACIRE      ");
            lcd.setCursor(0, 1);
            lcd.print("T=");
            lcd.print(currentTemp);
            lcd.print("C R=");
            lcd.print(remainingTime);
            lcd.print("s ");

            if (nowSec >= g_trac - 5) {
                // Bec oprit complet în ultimele 5 secunde
                analogWrite(LAMP_PWM_PIN, 0);
            } else {
                // Putere redusă la 10%
                analogWrite(LAMP_PWM_PIN, 255 * 0.1);
            }

            if (nowSec >= g_trac) {
                processState = STATE_DONE;
            }
        } break;

        /* ====================== */
        /* 4) PROCES FINALIZAT   */
        /* ====================== */
        case STATE_DONE: {
            lcd.setCursor(0, 0);
            lcd.print("PROCES FINAL");
            lcd.setCursor(0, 1);
            lcd.print("T=");
            lcd.print(currentTemp);
            lcd.print("C ");

            // Bec oprit complet
            analogWrite(LAMP_PWM_PIN, 0);

            delay(1000); // Pauză pentru afișaj
            stop_process();
            go_home();
        } break;

        default:
            break;
    }
}

