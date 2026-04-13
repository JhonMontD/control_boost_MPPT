/**
 * mppt_po.cpp
 * Implementación del algoritmo MPPT con Perturbación y Observación (P&O)
 *
 * Algoritmo (ejecutado cada TS_MPPT_MS ms):
 *  1. Medir V_panel, I_panel
 *  2. P_actual = V_panel * I_panel
 *  3. dP = P_actual - P_anterior
 *  4. dV = V_panel - V_anterior
 *  5. Si |dP| < umbral (zona muerta): no perturbar (ya estamos cerca del MPP)
 *  6. Si dP > 0:
 *       Si dV > 0 → incrementar setpoint de voltaje
 *       Si dV < 0 → decrementar setpoint de voltaje
 *  7. Si dP < 0:
 *       Si dV > 0 → decrementar setpoint de voltaje
 *       Si dV < 0 → incrementar setpoint de voltaje
 *  8. Actualizar P_anterior, V_anterior
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#include "mppt_po.h"

// Umbral mínimo de cambio de potencia para considerar que el MPP fue perturbado.
// Evita oscilaciones innecesarias cuando la potencia no cambia significativamente.
static const float DP_THRESHOLD = 0.01f;  // [W]

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MPPT_PO::MPPT_PO(float delta_d, float v_min, float v_max)
    : _delta_d(delta_d),
      _v_min(v_min),
      _v_max(v_max),
      _setpoint(0.0f),
      _p_prev(0.0f),
      _v_prev(0.0f),
      _p_actual(0.0f)
{
}

// ---------------------------------------------------------------------------
// Inicialización
// ---------------------------------------------------------------------------
void MPPT_PO::begin(float v_initial) {
    _setpoint = constrain(v_initial, _v_min, _v_max);
    _p_prev   = 0.0f;
    _v_prev   = v_initial;
    _p_actual = 0.0f;
}

// ---------------------------------------------------------------------------
// Paso del algoritmo P&O
// ---------------------------------------------------------------------------
float MPPT_PO::update(float v_panel, float i_panel) {
    // Paso 2: calcular potencia actual
    _p_actual = v_panel * i_panel;

    // Pasos 3-4: variaciones respecto al ciclo anterior
    float dP = _p_actual - _p_prev;
    float dV = v_panel   - _v_prev;

    // Paso 5: zona muerta — si el cambio de potencia es despreciable,
    // no perturbar para evitar oscilaciones alrededor del MPP.
    if (dP > DP_THRESHOLD || dP < -DP_THRESHOLD) {
        // Pasos 6-7: lógica de perturbación
        if (dP > 0.0f) {
            // La potencia aumentó
            if (dV > 0.0f) {
                // Seguimos en la dirección correcta → aumentar setpoint
                _setpoint += _delta_d;
            } else {
                // Disminución de V llevó a aumento de P → decrementar setpoint
                _setpoint -= _delta_d;
            }
        } else {
            // La potencia disminuyó
            if (dV > 0.0f) {
                // Aumento de V causó disminución de P → decrementar setpoint
                _setpoint -= _delta_d;
            } else {
                // Disminución de V causó disminución de P → aumentar setpoint
                _setpoint += _delta_d;
            }
        }

        // Saturar el setpoint dentro de los límites permitidos
        _setpoint = constrain(_setpoint, _v_min, _v_max);
    }
    // Si |dP| <= DP_THRESHOLD, mantenemos el setpoint actual (estamos cerca del MPP)

    // Paso 8: actualizar valores anteriores
    _p_prev = _p_actual;
    _v_prev = v_panel;

    return _setpoint;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------
float MPPT_PO::getSetpoint() const {
    return _setpoint;
}

float MPPT_PO::getPower() const {
    return _p_actual;
}

