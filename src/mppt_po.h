/**
 * mppt_po.h
 * Módulo MPPT con algoritmo de Perturbación y Observación (P&O)
 *
 * El algoritmo MPPT P&O ajusta iterativamente el setpoint de voltaje
 * del panel solar para encontrar el Punto de Máxima Potencia (MPP).
 *
 * Proyecto: Control Boost MPPT
 * Plataforma: Arduino Uno / Nano / Mega
 */

#ifndef MPPT_PO_H
#define MPPT_PO_H

#include <Arduino.h>
#include "config.h"

class MPPT_PO {
public:
    /**
     * Constructor
     * @param delta_d  Paso de perturbación del setpoint de voltaje [V]
     * @param v_min    Voltaje mínimo del setpoint [V]
     * @param v_max    Voltaje máximo del setpoint [V]
     */
    MPPT_PO(float delta_d, float v_min, float v_max);

    /**
     * Inicializa el módulo MPPT con un setpoint de voltaje inicial.
     * @param v_initial  Voltaje inicial del setpoint [V]
     */
    void begin(float v_initial);

    /**
     * Ejecuta un paso del algoritmo P&O.
     * Debe llamarse cada TS_MPPT_MS milisegundos.
     *
     * @param v_panel  Voltaje medido del panel solar [V]
     * @param i_panel  Corriente medida del panel solar [A]
     * @return         Nuevo setpoint de voltaje [V]
     */
    float update(float v_panel, float i_panel);

    /**
     * Retorna el setpoint de voltaje actual [V].
     */
    float getSetpoint() const;

    /**
     * Retorna la última potencia calculada [W].
     */
    float getPower() const;

private:
    float _delta_d;      // Paso de perturbación [V]
    float _v_min;        // Límite inferior del setpoint [V]
    float _v_max;        // Límite superior del setpoint [V]

    float _setpoint;     // Setpoint de voltaje actual [V]
    float _p_prev;       // Potencia en el ciclo anterior [W]
    float _v_prev;       // Voltaje en el ciclo anterior [V]
    float _p_actual;     // Última potencia calculada [W]
};

#endif // MPPT_PO_H
