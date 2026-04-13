/**
 * main.ino
 * Archivo principal — Convertidor Boost Ideal con MPPT P&O + Control PID
 *
 * Descripción:
 *   Este sketch de Arduino implementa el control de un convertidor Boost para
 *   seguimiento del punto de máxima potencia (MPPT) de un panel solar,
 *   usando el algoritmo de Perturbación y Observación (P&O) como lazo externo
 *   y un controlador PID como lazo interno para regular el voltaje de salida.
 *
 * Lazos de control (no bloqueantes mediante millis()):
 *   - Lazo PID:  se ejecuta cada TS_PID_MS  ms (lazo rápido, ej. 10 ms)
 *   - Lazo MPPT: se ejecuta cada TS_MPPT_MS ms (lazo lento, ej. 100 ms)
 *   - Serial:    se reporta cada TS_SERIAL_MS ms (ej. 500 ms)
 *
 * Configuración del Timer1:
 *   Se configura Timer1 en modo Fast PWM de 8 bits con prescaler = 1,
 *   generando ~31.4 kHz en el pin 9, adecuado para el MOSFET del Boost.
 *
 * Hardware requerido:
 *   - Arduino Uno / Nano / Mega
 *   - Divisor de tensión en A0 (Vpanel) y A2 (Vout)
 *   - Sensor de corriente (ej. ACS712) en A1 (Ipanel)
 *   - Driver de gate + MOSFET en pin 9
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#include <Arduino.h>
#include "config.h"
#include "mppt_po.h"
#include "pid_controller.h"

// ---------------------------------------------------------------------------
// Objetos de control
// ---------------------------------------------------------------------------

// MPPT P&O: paso de perturbación = DELTA_D * V_REF_MAX [V], límites de setpoint
MPPT_PO mppt(DELTA_D * V_REF_MAX, DUTY_MIN * V_REF_MAX, DUTY_MAX * V_REF_MAX);

// PID: parámetros desde config.h, salida en fracción [DUTY_MIN, DUTY_MAX]
PIDController pid(KP, KI, KD, TS_PID_MS, DUTY_MIN, DUTY_MAX);

// ---------------------------------------------------------------------------
// Variables de temporización (millis)
// ---------------------------------------------------------------------------
unsigned long lastTimePID    = 0;
unsigned long lastTimeMPPT   = 0;
unsigned long lastTimeSerial = 0;

// ---------------------------------------------------------------------------
// Variables de estado global
// ---------------------------------------------------------------------------
float v_panel   = 0.0f;   // Voltaje del panel solar [V]
float i_panel   = 0.0f;   // Corriente del panel solar [A]
float v_out     = 0.0f;   // Voltaje de salida del convertidor [V]
float p_panel   = 0.0f;   // Potencia del panel solar [W]
float setpoint  = 0.0f;   // Setpoint de voltaje dado por MPPT [V]
float duty      = 0.0f;   // Duty cycle actual (fracción 0–1)

// ---------------------------------------------------------------------------
// Funciones auxiliares de lectura de sensores
// ---------------------------------------------------------------------------

/**
 * Lee el voltaje del panel solar en voltios.
 * Se asume divisor de tensión que mapea [0, V_REF_MAX] → [0, 5V] → ADC [0, 1023].
 */
float readVPanel() {
    int raw = analogRead(PIN_V_PANEL);
    return (raw / ADC_RESOLUTION) * V_REF_MAX;
}

/**
 * Lee la corriente del panel solar en amperios.
 * Se asume sensor de corriente que mapea [0, I_REF_MAX] → [0, 5V] → ADC [0, 1023].
 */
float readIPanel() {
    int raw = analogRead(PIN_I_PANEL);
    return (raw / ADC_RESOLUTION) * I_REF_MAX;
}

/**
 * Lee el voltaje de salida del convertidor en voltios.
 * Se asume divisor de tensión que mapea [0, V_REF_MAX] → [0, 5V] → ADC [0, 1023].
 */
float readVOut() {
    int raw = analogRead(PIN_V_OUT);
    return (raw / ADC_RESOLUTION) * V_REF_MAX;
}

// ---------------------------------------------------------------------------
// Configuración del Timer1 para PWM de alta frecuencia (~31.4 kHz)
// ---------------------------------------------------------------------------

/**
 * Configura Timer1 en modo Fast PWM de 8 bits con prescaler = 1.
 * Frecuencia de PWM = 16 MHz / (1 * 256) ≈ 62.5 kHz
 *
 * TCCR1A: COM1A1=1 (non-inverting), WGM10=1 (Fast PWM 8-bit)
 * TCCR1B: WGM12=1 (Fast PWM 8-bit), CS10=1 (prescaler 1 → ~62.5 kHz)
 *
 * Nota: Solo válido para Arduino Uno/Nano (ATmega328P).
 *       Para Mega (ATmega2560) los registros son iguales para Timer1.
 */
void configureTimer1PWM() {
    // Deshabilitar interrupciones durante la configuración
    cli();

    // Limpiar registros de control del Timer1
    TCCR1A = 0;
    TCCR1B = 0;

    // Modo Fast PWM de 8 bits: WGM12=1, WGM10=1
    // Salida no inversora en OC1A (pin 9): COM1A1=1
    TCCR1A = (1 << COM1A1) | (1 << WGM10);

    // Prescaler = 1 (CS10=1) → f_PWM = 16 MHz / 256 ≈ 62.5 kHz
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Habilitar interrupciones
    sei();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    // Comunicación serial para monitoreo
    Serial.begin(115200);
    Serial.println(F("=== Control Boost MPPT P&O + PID ==="));
    Serial.println(F("Inicializando..."));

    // Configurar pin PWM como salida
    pinMode(PIN_PWM, OUTPUT);
    analogWrite(PIN_PWM, 0);  // Duty cycle = 0 al inicio (seguridad)

    // Configurar Timer1 para PWM de alta frecuencia
    configureTimer1PWM();

    // Leer voltaje inicial del panel para el setpoint de arranque
    float v_init = readVPanel();

    // Inicializar MPPT con el voltaje actual del panel
    mppt.begin(v_init);

    // Inicializar PID
    pid.begin();

    // Setpoint inicial: voltaje actual del panel
    setpoint = mppt.getSetpoint();

    // Inicializar marcas de tiempo
    lastTimePID    = millis();
    lastTimeMPPT   = millis();
    lastTimeSerial = millis();

    Serial.println(F("Sistema listo."));
    Serial.println(F("V_panel[V], I_panel[A], P_panel[W], V_out[V], Duty[%]"));
}

// ---------------------------------------------------------------------------
// Loop principal (no bloqueante)
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    // ------------------------------------------------------------------
    // Lazo MPPT (lento): cada TS_MPPT_MS ms
    // Actualiza el setpoint de voltaje buscando el MPP
    // ------------------------------------------------------------------
    if ((now - lastTimeMPPT) >= TS_MPPT_MS) {
        lastTimeMPPT = now;

        // Leer sensores del panel
        v_panel = readVPanel();
        i_panel = readIPanel();

        // Ejecutar un paso del algoritmo P&O
        setpoint = mppt.update(v_panel, i_panel);

        // Guardar potencia para reporte
        p_panel = mppt.getPower();

        // Cuando el MPPT cambia el setpoint, resetear el integrador del PID
        // para evitar un transitorio brusco
        pid.reset();
    }

    // ------------------------------------------------------------------
    // Lazo PID (rápido): cada TS_PID_MS ms
    // Regula el voltaje de salida para seguir el setpoint del MPPT
    // ------------------------------------------------------------------
    if ((now - lastTimePID) >= TS_PID_MS) {
        lastTimePID = now;

        // Leer voltaje de salida
        v_out = readVOut();

        // Calcular nuevo duty cycle con el PID
        duty = pid.update(setpoint, v_out);

        // Aplicar duty cycle al PWM
        // duty es fracción [DUTY_MIN, DUTY_MAX], se escala a [0, PWM_MAX]
        int pwm_val = (int)(duty * PWM_MAX);
        pwm_val = constrain(pwm_val, (int)(DUTY_MIN * PWM_MAX), (int)(DUTY_MAX * PWM_MAX));
        analogWrite(PIN_PWM, pwm_val);
    }

    // ------------------------------------------------------------------
    // Reporte Serial: cada TS_SERIAL_MS ms
    // ------------------------------------------------------------------
    if ((now - lastTimeSerial) >= TS_SERIAL_MS) {
        lastTimeSerial = now;

        // Imprimir variables de estado en formato CSV
        Serial.print(v_panel, 3);   Serial.print(F(","));
        Serial.print(i_panel, 3);   Serial.print(F(","));
        Serial.print(p_panel, 3);   Serial.print(F(","));
        Serial.print(v_out,   3);   Serial.print(F(","));
        Serial.print(duty * 100.0f, 1);  // en porcentaje
        Serial.println(F("%"));
    }
}
