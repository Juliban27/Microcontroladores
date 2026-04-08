/*
 * Control de velocidad de motor DC - Lazo abierto
 * Usando registros de bajo nivel ATmega328P
 *
 * PINES:
 *   D9  (PB1) → ENA  L293D  — PWM via Timer1 (OC1A)
 *   D4  (PD4) → IN1  L293D
 *   D5  (PD5) → IN2  L293D
 *   D2  (PD2) → BTN STOP  (INT0)
 *   D3  (PD3) → BTN DIR   (INT1)
 *   A0  (PC0) → Potenciómetro
 *   D8  (PB0) → LED parada
 *   D10 (PB2) → LED dirección
 *   D11 (PB3) → LED velocidad (PWM via Timer2 OC2A)
 *
 * REGISTROS USADOS:
 *   DDRB, DDRD, PORTB, PORTD  → configuración y escritura de pines digitales
 *   TCCR1A, TCCR1B, OCR1A     → Timer1 para PWM en D9  (ENA, velocidad motor)
 *   TCCR2A, TCCR2B, OCR2A     → Timer2 para PWM en D11 (LED velocidad)
 *   ADMUX, ADCSRA, ADCH/ADCL  → ADC para leer potenciómetro en A0
 *   EICRA, EIMSK              → Interrupciones externas INT0 e INT1
 *   SREG (sei/cli)            → Habilitar/deshabilitar interrupciones globales
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// ──────────────────────────────────────────────
//  PARÁMETROS
// ──────────────────────────────────────────────
#define PWM_MIN      60
#define PWM_MAX      255
#define DEBOUNCE_MS  200

// ──────────────────────────────────────────────
//  ESTADO GLOBAL
// ──────────────────────────────────────────────
volatile bool motorDetenido = true;
volatile bool dirAdelante   = true;
volatile unsigned long t_stop = 0;
volatile unsigned long t_dir  = 0;

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  setup_gpio();
  setup_pwm();
  setup_adc();
  setup_interrupts();

  disable();
  Serial.println(F("Sistema listo - registros de bajo nivel"));
}

// ──────────────────────────────────────────────
//  CONFIGURACIÓN DE PINES (DDR y PORT)
// ──────────────────────────────────────────────
void setup_gpio() {
  /*
   * DDRx: Data Direction Register
   *   bit=1 → salida (OUTPUT)
   *   bit=0 → entrada (INPUT)
   *
   * PORTx cuando es entrada:
   *   bit=1 → activa pull-up interno
   *   bit=0 → entrada flotante (sin pull-up)
   */

  // PUERTO B — salidas: PB0(D8 LED), PB1(D9 ENA), PB2(D10 LED), PB3(D11 LED)
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);

  // PUERTO D — salidas: PD4(D4 IN1), PD5(D5 IN2)
  DDRD |= (1 << PD4) | (1 << PD5);

  // PUERTO D — entradas con pull-up: PD2(D2 INT0 BTN_STOP), PD3(D3 INT1 BTN_DIR)
  // DDR en 0 (entrada) ya es el valor por defecto, solo activar pull-up en PORT
  DDRD  &= ~((1 << PD2) | (1 << PD3));  // PD2, PD3 como entradas
  PORTD |=  (1 << PD2)  | (1 << PD3);   // Pull-up interno activo

  // PUERTO C — entrada ADC: PC0(A0) — DDR en 0 por defecto, sin pull-up
  DDRC  &= ~(1 << PC0);
  PORTC &= ~(1 << PC0);
}

// ──────────────────────────────────────────────
//  CONFIGURACIÓN PWM
// ──────────────────────────────────────────────
void setup_pwm() {
  /*
   * TIMER 1 → PWM en D9 (PB1 = OC1A), 8 bits, para ENA del L293D
   *
   * TCCR1A:
   *   COM1A1=1, COM1A0=0 → modo no-inversor en OC1A
   *   WGM11=0, WGM10=1   → combinado con TCCR1B para Fast PWM 8-bit (modo 5)
   *
   * TCCR1B:
   *   WGM13=0, WGM12=1   → Fast PWM 8-bit (TOP = 0xFF)
   *   CS11=1, CS10=1     → prescaler 64 → ~976 Hz (audible pero funciona)
   *
   * OCR1A: valor de comparación (0–255), determina el duty cycle
   *   duty = OCR1A / 255 × 100%
   */
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
  OCR1A  = 0;  // Iniciar en 0 (motor detenido)

  /*
   * TIMER 2 → PWM en D11 (PB3 = OC2A), para LED de velocidad
   *
   * TCCR2A:
   *   COM2A1=1 → modo no-inversor en OC2A
   *   WGM21=1, WGM20=1 → Fast PWM (TOP = 0xFF)
   *
   * TCCR2B:
   *   CS22=1, CS21=1, CS20=0 → prescaler 256 → ~245 Hz
   *
   * OCR2A: duty cycle del LED (0=apagado, 255=máximo brillo)
   */
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS22) | (1 << CS21);
  OCR2A  = 0;
}

// ──────────────────────────────────────────────
//  CONFIGURACIÓN ADC
// ──────────────────────────────────────────────
void setup_adc() {
  /*
   * ADMUX — Multiplexor ADC:
   *   REFS1=0, REFS0=1 → referencia = AVCC (5V del Arduino)
   *   ADLAR=0          → resultado alineado a derecha (usar ADCH:ADCL, 10 bits)
   *   MUX3:0 = 0000    → canal ADC0 = pin A0
   *
   * ADCSRA — Control y estado:
   *   ADEN=1  → habilitar el ADC
   *   ADPS2=1, ADPS1=1, ADPS0=1 → prescaler 128 → 16MHz/128 = 125kHz
   *     (ADC requiere entre 50kHz y 200kHz para máxima resolución)
   */
  ADMUX  = (1 << REFS0);                          // AVCC como referencia, canal 0
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Habilitar, prescaler 128
}

// ──────────────────────────────────────────────
//  CONFIGURACIÓN INTERRUPCIONES EXTERNAS
// ──────────────────────────────────────────────
void setup_interrupts() {
  /*
   * EICRA — External Interrupt Control Register A:
   *   ISC01=1, ISC00=0 → INT0 dispara en flanco de bajada (FALLING)
   *   ISC11=1, ISC10=0 → INT1 dispara en flanco de bajada (FALLING)
   *   (flanco de bajada = pulsador presionado con pull-up)
   *
   * EIMSK — External Interrupt Mask Register:
   *   INT0=1 → habilitar INT0
   *   INT1=1 → habilitar INT1
   *
   * sei() → habilitar interrupciones globales (bit I del SREG)
   */
  EICRA |= (1 << ISC01) | (1 << ISC11);  // Flanco de bajada para INT0 e INT1
  EICRA &= ~((1 << ISC00) | (1 << ISC10));
  EIMSK |= (1 << INT0) | (1 << INT1);    // Habilitar INT0 e INT1
  sei();                                  // Interrupciones globales ON
}

// ──────────────────────────────────────────────
//  LECTURA ADC (bloqueante)
// ──────────────────────────────────────────────
uint16_t leer_adc() {
  /*
   * Iniciar conversión: ADSC=1
   * Esperar hasta que ADSC vuelva a 0 (conversión completa)
   * Leer ADCL primero (obligatorio), luego ADCH
   * Resultado = ADCH:ADCL (10 bits, 0–1023)
   */
  ADCSRA |= (1 << ADSC);           // Iniciar conversión
  while (ADCSRA & (1 << ADSC));    // Esperar fin de conversión
  return ADC;                       // Registro ADC = ADCL + ADCH combinados
}

// ──────────────────────────────────────────────
//  LOOP PRINCIPAL
// ──────────────────────────────────────────────
void loop() {
  // 1. Leer potenciómetro y mapear a PWM
  uint16_t adc = leer_adc();
  uint8_t  pwm = (uint8_t)((uint32_t)adc * PWM_MAX / 1023);

  // 2. LED velocidad: brillo proporcional (Timer2 OC2A)
  OCR2A = pwm;

  // 3. Control del motor
  if (motorDetenido) {
    disable();
    // LED parada ON: PB0 = 1
    PORTB |= (1 << PB0);
    // Parpadeo LED dirección: PB2 toggle cada ~400ms
    if ((millis() / 400) % 2) PORTB |=  (1 << PB2);
    else                       PORTB &= ~(1 << PB2);

  } else {
    // LED parada OFF
    PORTB &= ~(1 << PB0);
    // LED dirección según sentido
    if (dirAdelante) PORTB |=  (1 << PB2);
    else             PORTB &= ~(1 << PB2);

    if (pwm < PWM_MIN) {
      disable();
    } else {
      set_bridge_pwm(pwm, dirAdelante);
    }
  }

  // 4. Debug serial cada 500ms
  static unsigned long t_serial = 0;
  if (millis() - t_serial > 500) {
    t_serial = millis();
    Serial.print(F("ADC="));   Serial.print(adc);
    Serial.print(F(" PWM="));  Serial.print(pwm);
    Serial.print(F(" Dir="));  Serial.print(dirAdelante ? F("ADELANTE") : F("REVERSA"));
    Serial.print(F(" Stop=")); Serial.println(motorDetenido ? F("SI") : F("NO"));
  }

  delay(30);
}

// ──────────────────────────────────────────────
//  CONTROL DEL PUENTE H (registros PORT)
// ──────────────────────────────────────────────

void set_bridge_pwm(uint8_t pwm, bool adelante) {
  /*
   * Dirección: manipular PD4 (IN1) y PD5 (IN2) directamente
   * Velocidad: escribir en OCR1A (Timer1 OC1A = D9)
   *
   * Tabla de verdad L293D:
   *   IN1=1, IN2=0 → adelante
   *   IN1=0, IN2=1 → reversa
   */
  if (adelante) {
    PORTD |=  (1 << PD4);  // IN1 = HIGH
    PORTD &= ~(1 << PD5);  // IN2 = LOW
  } else {
    PORTD &= ~(1 << PD4);  // IN1 = LOW
    PORTD |=  (1 << PD5);  // IN2 = HIGH
  }
  OCR1A = pwm;  // Duty cycle del PWM en ENA
}

void disable() {
  // ENA = 0 (PWM a 0), IN1 = IN2 = 0
  OCR1A  = 0;
  PORTD &= ~((1 << PD4) | (1 << PD5));
}

void fast_stop() {
  // Freno eléctrico: IN1 = IN2 = 1 con ENA activo
  PORTD |= (1 << PD4) | (1 << PD5);
  OCR1A  = 255;
}

// Mantener funciones originales (ahora usan registros internamente)
void clockwise()         { set_bridge_pwm(255, true);  }
void counter_clockwise() { set_bridge_pwm(255, false); }

// ──────────────────────────────────────────────
//  ISR — INTERRUPCIONES EXTERNAS
// ──────────────────────────────────────────────

ISR(INT0_vect) {
  // INT0 → PD2 → BTN STOP
  unsigned long ahora = millis();
  if (ahora - t_stop > DEBOUNCE_MS) {
    motorDetenido = !motorDetenido;
    t_stop = ahora;
  }
}

ISR(INT1_vect) {
  // INT1 → PD3 → BTN DIR
  unsigned long ahora = millis();
  if (ahora - t_dir > DEBOUNCE_MS) {
    if (!motorDetenido) dirAdelante = !dirAdelante;
    t_dir = ahora;
  }
}