"""
Parametri attuatori EXCITE

Questo modulo contiene le specifiche di tutti gli attuatori del satellite EXCITE:
- Reaction Wheels (RW) - 4 ruote in configurazione piramidale
- Magnetorquers (MTB) - 3 magnetorquer ortogonali
- Thruster chimico H2O2 - Propulsore monopropellente
"""

import numpy as np
from Basilisk.utilities import macros

# ===== Reaction Wheels (CubeSpace CW0162) =====

# Numero di Reaction Wheels
NUM_RW = 4

# Configurazione piramidale
# Elevazione dalla plane XY [gradi]
RW_PYRAMID_ELEVATION_DEG = 26.57

# Azimuth angoli nel piano XY [gradi]
# Distribuzione simmetrica a 90° tra le ruote
RW_PYRAMID_AZIMUTHS_DEG = [45.0, 135.0, 225.0, 315.0]

# Specifiche performance CW0162
RW_MAX_TORQUE_NM = 0.2e-3  # N*m - coppia massima in uscita
RW_MAX_SPEED_RPM = 10000   # RPM - velocità massima
RW_INERTIA_KG_M2 = 5.0e-6  # kg*m^2 - momento d'inerzia ruota

# Posizioni RW rispetto al COM in body frame [m]
RW_POSITIONS_B = [
    np.array([0.023, 0.023, 0.046]),   # RW1
    np.array([-0.023, 0.023, 0.046]),  # RW2
    np.array([0.023, -0.023, 0.046]),  # RW3
    np.array([-0.023, -0.023, 0.046])  # RW4
]

# Modello attrito realistico per RW
# Questi parametri modellano la risposta non ideale delle reaction wheel fisiche

# Attrito coulombiano [N*m]
# Coppia costante resistiva indipendente dalla velocità
RW_FRICTION_COULOMB_NM = 1.5e-5

# Attrito statico [N*m]
# Coppia necessaria per vincere inerzia iniziale (da fermo)
RW_FRICTION_STATIC_NM = 2.0e-5

# Coefficiente attrito viscoso [N*m*s]
# Resistenza proporzionale alla velocità
RW_FRICTION_VISCOUS_NM_S = 1.0e-7

# Parametro Stribeck [rad/s]
# Modella la transizione smooth tra attrito statico e coulombiano
RW_STRIBECK_BETA_RAD_S = 0.01

# Limiti operativi
RW_VOLTAGE_MIN_V = 5.0    # V - tensione minima operativa
RW_VOLTAGE_MAX_V = 12.0   # V - tensione massima

# Consumo energetico
RW_POWER_NOMINAL_W = 0.5  # W - potenza nominale per ruota in operazione
RW_POWER_STANDBY_W = 0.1  # W - potenza in standby

# ===== Magnetorquers (MTB) =====

# Numero di Magnetorquers (3 assi ortogonali)
NUM_MTB = 3

# Dipolo magnetico massimo per asse [A*m^2]
MTB_MAX_DIPOLE_AM2 = 0.2

# Assi MTB allineati con body frame
MTB_AXES = np.eye(3)  # [X, Y, Z] identity matrix

# Orientazione MTB in body frame
MTB_AXIS_X = np.array([1.0, 0.0, 0.0])  # MTB lungo asse X
MTB_AXIS_Y = np.array([0.0, 1.0, 0.0])  # MTB lungo asse Y
MTB_AXIS_Z = np.array([0.0, 0.0, 1.0])  # MTB lungo asse Z

# Consumo energetico MTB
MTB_POWER_MAX_W = 1.0     # W - potenza massima per MTB
MTB_POWER_NOMINAL_W = 0.5 # W - potenza nominale
MTB_VOLTAGE_V = 5.0       # V - tensione operativa

# Resistenza elettrica
MTB_RESISTANCE_OHM = 10.0  # Ω - resistenza bobina

# Limiti corrente
MTB_CURRENT_MAX_A = 0.5    # A - corrente massima

# ===== Thruster Chimico H2O2 (UniPi IOD) =====

# Tipo propellente
THRUSTER_PROPELLANT = "H2O2"  # Perossido di idrogeno (monopropellente)

# Specifiche performance
THRUSTER_MAX_THRUST_N = 0.5   # N - spinta nominale
THRUSTER_ISP_S = 180.0        # s - impulso specifico
THRUSTER_MIN_IMPULSE_BIT_NS = 0.01  # N*s - minimo impulso erogabile

# Posizione thruster rispetto al COM in body frame [m]
# Posizionato al centro della faccia -Z
THRUSTER_LOCATION_B = np.array([0.0, 0.0, 0.0])

# Direzione thrust in body frame (vettore unitario)
# Accende in direzione -Z per manovre Delta-V
THRUSTER_DIRECTION_B = np.array([0.0, 0.0, -1.0])

# Parametri propellente
THRUSTER_PROPELLANT_MASS_KG = 2.0  # kg - massa propellente disponibile
THRUSTER_TANK_CAPACITY_L = 2.5     # L - capacità serbatoio

# Limiti operativi
THRUSTER_MIN_PULSE_WIDTH_MS = 20   # ms - durata minima accensione
THRUSTER_DUTY_CYCLE_MAX = 0.8      # Duty cycle massimo (80%)

# Consumo energetico
THRUSTER_POWER_IGNITION_W = 10.0   # W - potenza per ignizione
THRUSTER_POWER_STEADY_W = 5.0      # W - potenza steady state

# Tempo di risposta
THRUSTER_RESPONSE_TIME_MS = 10     # ms - tempo da comando a thrust nominale

# ===== Power Consumption Summary =====

# Consumo totale attuatori in diverse modalità operative

# Detumbling (B-dot con MTB)
POWER_DETUMBLING_W = NUM_MTB * MTB_POWER_NOMINAL_W  # ~1.5 W

# Pointing (RW attive + MTB desaturation)
POWER_POINTING_W = NUM_RW * RW_POWER_NOMINAL_W + MTB_POWER_STANDBY_W  # ~2.1 W

# Delta-V maneuver (Thruster + RW + MTB)
POWER_MANEUVER_W = (THRUSTER_POWER_STEADY_W +
                    NUM_RW * RW_POWER_NOMINAL_W +
                    NUM_MTB * MTB_POWER_STANDBY_W)  # ~7.5 W

# Standby mode
MTB_POWER_STANDBY_W = 0.05  # W per MTB
POWER_STANDBY_W = (NUM_RW * RW_POWER_STANDBY_W +
                   NUM_MTB * MTB_POWER_STANDBY_W)  # ~0.55 W
