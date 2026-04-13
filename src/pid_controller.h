/**
 * pid_controller.h
 * Módulo de Control PID para el convertidor Boost
 *
 * Características:
 *  - Control PID discreto con tiempo de muestreo configurable
 *  - Anti-windup por saturación del integrador
 *  - Limitación de salida entre DUTY_MIN y DUTY_MAX
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class PIDController {
public:
    /**
     * Constructor
     * @param kp        Ganancia proporcional
     * @param ki        Ganancia integral
     * @param kd        Ganancia derivativa
     * @param ts_ms     Tiempo de muestreo [ms]
     * @param out_min   Límite inferior de la salida (duty cycle)
     * @param out_max   Límite superior de la salida (duty cycle)
     */
    PIDController(float kp, float ki, float kd,
                  float ts_ms,
                  float out_min, float out_max);

    /**
     * Inicializa el controlador PID reseteando el estado interno.
     */
    void begin();

    /**
     * Ejecuta un paso del controlador PID.
     * Debe llamarse cada ts_ms milisegundos.
     *
     * @param setpoint   Referencia (voltaje/corriente objetivo) [unidad física]
     * @param measured   Valor medido actual [unidad física]
     * @return           Duty cycle calculado (fracción entre out_min y out_max)
     */
    float update(float setpoint, float measured);

    /**
     * Retorna el último duty cycle calculado.
     */
    float getDuty() const;

    /**
     * Permite actualizar las ganancias en tiempo de ejecución.
     */
    void setGains(float kp, float ki, float kd);

    /**
     * Resetea el estado interno (integral y error anterior) sin cambiar ganancias.
     */
    void reset();

private:
    float _kp;           // Ganancia proporcional
    float _ki;           // Ganancia integral
    float _kd;           // Ganancia derivativa
    float _ts;           // Tiempo de muestreo [s]
    float _out_min;      // Límite inferior de salida
    float _out_max;      // Límite superior de salida

    float _integral;     // Acumulador integral
    float _error_prev;   // Error en el ciclo anterior
    float _duty;         // Último duty cycle calculado
};

#endif // PID_CONTROLLER_H
