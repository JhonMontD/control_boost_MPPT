/**
 * pid_controller.cpp
 * Implementación del Control PID discreto con anti-windup
 *
 * Ecuaciones discretas:
 *   error      = setpoint - medición
 *   integral  += error * Ts              (con clamping anti-windup)
 *   derivada   = (error - error_prev) / Ts
 *   output     = Kp*error + Ki*integral + Kd*derivada
 *   duty       = saturar(output, out_min, out_max)
 *
 * Anti-windup: el integrador solo acumula cuando la salida NO está saturada,
 * o cuando la contribución integral reduce la saturación.
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#include "pid_controller.h"

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
PIDController::PIDController(float kp, float ki, float kd,
                             float ts_ms,
                             float out_min, float out_max)
    : _kp(kp),
      _ki(ki),
      _kd(kd),
      _ts(ts_ms / 1000.0f),   // convertir ms → s
      _out_min(out_min),
      _out_max(out_max),
      _integral(0.0f),
      _error_prev(0.0f),
      _duty(out_min)
{
}

// ---------------------------------------------------------------------------
// Inicialización
// ---------------------------------------------------------------------------
void PIDController::begin() {
    reset();
}

// ---------------------------------------------------------------------------
// Reset del estado interno
// ---------------------------------------------------------------------------
void PIDController::reset() {
    _integral   = 0.0f;
    _error_prev = 0.0f;
    _duty       = _out_min;
}

// ---------------------------------------------------------------------------
// Paso del controlador PID
// ---------------------------------------------------------------------------
float PIDController::update(float setpoint, float measured) {
    // 1. Calcular error
    float error = setpoint - measured;

    // 2. Término proporcional
    float p_term = _kp * error;

    // 3. Término derivativo (diferencia hacia atrás)
    float d_term = _kd * (error - _error_prev) / _ts;

    // 4. Calcular salida sin integrador para verificar saturación (anti-windup)
    float output_pd = p_term + d_term;

    // 5. Término integral con anti-windup por clamping:
    //    Solo integra si la salida NO está saturada, o si la integración
    //    reduce la magnitud de la saturación (signo contrario al error de saturación).
    float output_prev = output_pd + _ki * _integral; // salida con integral actual

    bool saturated_high = (output_prev >= _out_max);
    bool saturated_low  = (output_prev <= _out_min);

    // Permitir integración solo si no agrava la saturación
    if (!((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f))) {
        _integral += error * _ts;
    }

    // 6. Calcular salida completa
    float output = p_term + _ki * _integral + d_term;

    // 7. Saturar el duty cycle entre los límites configurados
    _duty = constrain(output, _out_min, _out_max);

    // 8. Actualizar error anterior
    _error_prev = error;

    return _duty;
}

// ---------------------------------------------------------------------------
// Getters / Setters
// ---------------------------------------------------------------------------
float PIDController::getDuty() const {
    return _duty;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
