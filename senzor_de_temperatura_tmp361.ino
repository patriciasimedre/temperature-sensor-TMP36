#include <LiquidCrystal.h>
#include <avr/interrupt.h>

#define TP36_SENSOR_CHANNEL 0  // Setare pin senzor de temp A0
#define ADC_REF_VOLTAGE 5.0    // Tensiune de referinta 5V

// setare pini LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

// declarare lcd
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// variabile ceas
volatile int seconds = 0;
volatile int minutes = 0;
volatile int hours = 12; // ora de start

void setup() {
    // Configurare ADC
    ADMUX |= (1 << REFS0); // Setam tensiunea de referinta la AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler de 128
    ADCSRA |= (1 << ADEN); // Activam ADC-ul

    // Configurare Timer1 pentru a genera o întrerupere la fiecare secundă
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, prescaler 1024
    OCR1A = 15624; // Valoare pentru 1 secunda (16 MHz / 1024 prescaler)
    TIMSK1 |= (1 << OCIE1A); // Activăm întreruperea la comparare
    sei(); // Activăm întreruperile globale

    lcd.begin(16, 2);
    Serial.begin(9600);
}

ISR(TIMER1_COMPA_vect) {
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0;
            }
        }
    }
}

void loop() {
    // Setăm canalul ADC pentru senzorul de temperatură (A0)
    ADMUX = (ADMUX & 0xF0) | TP36_SENSOR_CHANNEL;
    // Pornim conversia ADC
    ADCSRA |= (1 << ADSC);
    // Așteptăm să se termine conversia
    while (ADCSRA & (1 << ADSC));
    // Citim valoarea ADC și calculăm temperatura
    uint16_t adc_value = ADC;
    float voltage = (float)adc_value * ADC_REF_VOLTAGE / 1024.0;
    int temperature = (voltage - 0.5) * 100.0;

    // Afișare pe LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T = ");
    lcd.print(temperature);
    lcd.print("C");

    lcd.setCursor(0, 1);
    if (hours < 10) lcd.print("0");
    lcd.print(hours);
    lcd.print(":");
    if (minutes < 10) lcd.print("0");
    lcd.print(minutes);
    lcd.print(":");
    if (seconds < 10) lcd.print("0");
    lcd.print(seconds);

    delay(1000);
}
