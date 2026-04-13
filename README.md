# Control Boost MPPT

Control de un **Convertidor Boost Ideal** en Arduino que integra:

1. **Algoritmo MPPT con Perturbación y Observación (P&O)** — lazo externo lento
2. **Control PID** — lazo interno rápido para regular el ciclo de trabajo (duty cycle) del PWM

---

## Estructura del proyecto

```
control_boost_MPPT/
├── src/
│   ├── main.ino           ← Sketch principal de Arduino
│   ├── config.h           ← Parámetros y pines configurables
│   ├── mppt_po.h          ← Declaración del módulo MPPT P&O
│   ├── mppt_po.cpp        ← Implementación del módulo MPPT P&O
│   ├── pid_controller.h   ← Declaración del controlador PID
│   └── pid_controller.cpp ← Implementación del controlador PID
└── README.md
```

---

## Diagrama del circuito Boost

```
         L              D
 Vd ──+──UUUU──+──|>|──+──── Vo
      |        |        |
     [Panel]  [S1]     [C]   [R]
      |    MOSFET|      |     |
      +──────────+──────+─────+
                 |
               PWM (pin 9)
               Arduino
```

| Componente | Descripción |
|------------|-------------|
| **Vd** | Fuente de entrada: panel solar |
| **L** | Inductor de almacenamiento de energía |
| **D** | Diodo de paso |
| **S1** | MOSFET controlado por PWM (~62.5 kHz) |
| **C** | Capacitor de filtrado de salida |
| **R** | Carga (resistencia equivalente) |
| **Vo** | Voltaje de salida del convertidor |

---

## Diagrama de conexiones de hardware (Arduino Uno/Nano)

```
Arduino         Sensor / Circuito
─────────────────────────────────────────────────────
A0 ←────────── Divisor de tensión (Voltaje panel Vd)
A1 ←────────── Sensor de corriente ACS712 (Ipanel)
A2 ←────────── Divisor de tensión (Voltaje salida Vo)
D9 ──────────→ Driver de gate MOSFET (PWM ~62.5 kHz)
GND ────────── Tierra común
5V  ────────── Alimentación sensores
```

**Divisores de tensión** (para mapear V_REF_MAX=30 V → 5 V en ADC):

```
                R1 = 50 kΩ
Vx ────┬────[R1]────── 5V (Arduino)
       │
      [R2]  R2 = 10 kΩ
       │
      GND
```
> Ajusta R1/R2 según tu V_REF_MAX. Verifica que la tensión en A0/A1/A2 no supere 5 V.

---

## Algoritmo MPPT P&O

El algoritmo se ejecuta cada `TS_MPPT_MS` milisegundos:

```
1. Medir V_panel, I_panel
2. P_actual = V_panel × I_panel
3. dP = P_actual − P_anterior
4. dV = V_panel − V_anterior
5. Si dP > 0:
     Si dV > 0 → setpoint += DELTA_D   (seguimos en buena dirección)
     Si dV < 0 → setpoint -= DELTA_D
6. Si dP ≤ 0:
     Si dV > 0 → setpoint -= DELTA_D
     Si dV < 0 → setpoint += DELTA_D   (invertir dirección)
7. setpoint = saturar(setpoint, V_min, V_max)
8. Actualizar P_anterior, V_anterior
```

El setpoint resultante es la **referencia de voltaje** que el PID intentará seguir en la salida del convertidor.

---

## Control PID (lazo interno)

El PID se ejecuta cada `TS_PID_MS` milisegundos:

```
1. Leer voltaje de salida Vo
2. error = setpoint − Vo
3. integral += error × Ts        (con anti-windup por clamping)
4. derivada = (error − error_prev) / Ts
5. output = Kp×error + Ki×integral + Kd×derivada
6. duty = saturar(output, DUTY_MIN, DUTY_MAX)
7. analogWrite(PIN_PWM, duty × PWM_MAX)
```

**Anti-windup:** el integrador solo acumula cuando la salida NO está saturada, o cuando la acumulación reduce la saturación (signo contrario al error de saturación).

---

## Configuración de parámetros (`src/config.h`)

| Parámetro | Valor por defecto | Descripción |
|-----------|:-----------------:|-------------|
| `KP` | `2.0` | Ganancia proporcional del PID |
| `KI` | `0.5` | Ganancia integral del PID |
| `KD` | `0.1` | Ganancia derivativa del PID |
| `TS_PID_MS` | `10` | Periodo de muestreo del PID [ms] |
| `DELTA_D` | `0.005` | Paso de perturbación MPPT (fracción de V_REF_MAX) |
| `TS_MPPT_MS` | `100` | Periodo de muestreo del MPPT [ms] |
| `V_REF_MAX` | `30.0` | Voltaje máximo medible [V] |
| `I_REF_MAX` | `5.0` | Corriente máxima medible [A] |
| `DUTY_MIN` | `0.1` | Duty cycle mínimo (10%) |
| `DUTY_MAX` | `0.9` | Duty cycle máximo (90%) |
| `PWM_MAX` | `255` | Resolución PWM (8 bits) |

### Ajuste de Kp, Ki, Kd

1. Comenzar con **Kp** pequeño, **Ki=0**, **Kd=0**.
2. Aumentar **Kp** hasta obtener respuesta rápida sin oscilaciones excesivas.
3. Añadir **Ki** para eliminar el error en estado estacionario.
4. Añadir **Kd** para amortiguar oscilaciones transitorias.
5. Observar la respuesta con el **Serial Monitor** (115200 baud).

### Ajuste de DELTA_D y TS_MPPT_MS

- Un **DELTA_D mayor** → convergencia más rápida al MPP pero mayor oscilación en estado estable.
- Un **TS_MPPT_MS mayor** → más tiempo para que el PID se estabilice antes de la siguiente perturbación.

---

## Monitoreo serial

El sketch reporta cada 500 ms en formato CSV (115200 baud):

```
V_panel[V], I_panel[A], P_panel[W], V_out[V], Duty[%]
12.340,1.023,12.630,24.100,55.3%
```

Puedes visualizarlo con el **Serial Plotter** de Arduino IDE para ver la convergencia al MPP.

---

## Cómo subir el código al Arduino

1. Instalar **Arduino IDE** (versión 1.8.x o 2.x).
2. Abrir `src/main.ino` en Arduino IDE.
3. Arduino IDE incluirá automáticamente los archivos `.h` y `.cpp` que están en la misma carpeta `src/`.
4. Seleccionar la placa: **Herramientas → Placa → Arduino Uno** (o Nano/Mega).
5. Seleccionar el puerto COM correspondiente.
6. Clic en **Subir** (Ctrl+U).

> **Nota:** Los archivos `.cpp` deben estar en la misma carpeta que el `.ino` para que Arduino IDE los compile automáticamente.

---

## Dependencias / librerías

Este proyecto **no requiere librerías externas**. Usa únicamente:

- `Arduino.h` (incluida en el IDE)
- Registro `TCCR1A`/`TCCR1B` del ATmega328P para configurar el Timer1 (compatible con Uno, Nano y Mega)

---

## Notas importantes

- El código usa `millis()` para los lazos de tiempo, **no `delay()`**, garantizando ejecución no bloqueante.
- El Timer1 se reconfigura para ~62.5 kHz, lo que **deshabilita** el PWM estándar en los pines 9 y 10.  
  Si necesitas usar esos pines para otra cosa, revierte la configuración del timer.
- Los límites `DUTY_MIN=0.1` y `DUTY_MAX=0.9` protegen el convertidor de condiciones de saturación y discontinuidad extrema.

