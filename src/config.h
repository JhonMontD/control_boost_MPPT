/**
 * config.h
 * Configuración y parámetros del sistema para el Convertidor Boost con MPPT P&O + PID
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Pines de Arduino
// ---------------------------------------------------------------------------
#define PIN_V_PANEL   A0   // Voltaje del panel solar (divisor de tensión)
#define PIN_I_PANEL   A1   // Corriente del panel solar (sensor de corriente)
#define PIN_V_OUT     A2   // Voltaje de salida del convertidor (divisor de tensión)
#define PIN_PWM       9    // Salida PWM para el gate del MOSFET (Timer1)

// ---------------------------------------------------------------------------
// Parámetros del sistema de medición
// ---------------------------------------------------------------------------
#define V_REF_MAX      30.0    // Voltaje máximo medible [V] (con divisor de tensión)
#define I_REF_MAX       5.0   // Corriente máxima medible [A] (con sensor de corriente)
#define ADC_RESOLUTION 1023.0  // Resolución del ADC de 10 bits (0–1023)

// ---------------------------------------------------------------------------
// Parámetros PID (ajustables)
// ---------------------------------------------------------------------------
#define KP          2.0    // Ganancia proporcional
#define KI          0.5    // Ganancia integral
#define KD          0.1    // Ganancia derivativa
#define TS_PID_MS   10     // Periodo de muestreo PID [ms]

// ---------------------------------------------------------------------------
// Parámetros MPPT P&O
// ---------------------------------------------------------------------------
#define DELTA_D       0.005  // Paso de perturbación del duty cycle [fracción]
#define TS_MPPT_MS    100    // Periodo de muestreo MPPT [ms]

// ---------------------------------------------------------------------------
// Límites del duty cycle
// ---------------------------------------------------------------------------
#define DUTY_MIN   0.1    // Duty cycle mínimo (10%)
#define DUTY_MAX   0.9    // Duty cycle máximo (90%)
#define PWM_MAX    255    // Valor máximo para analogWrite (8 bits)

// ---------------------------------------------------------------------------
// Periodo de reporte serial
// ---------------------------------------------------------------------------
#define TS_SERIAL_MS  500   // Intervalo de impresión por Serial [ms]

#endif // CONFIG_H
