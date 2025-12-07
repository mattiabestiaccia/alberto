"""
Parametri controllori e guadagni EXCITE

Questo modulo contiene tutti i guadagni e parametri di tuning per i controllori:
- MRP Steering (outer loop attitude control)
- Rate Servo (inner loop rate tracking)
- B-dot Controller (detumbling)
- Momentum Management (RW desaturation)
"""

import numpy as np

# ===== MRP Steering Controller (Outer Loop) =====

# Guadagno proporzionale
# Controlla la "stiffness" lineare per piccoli errori di assetto
# Aumentare = tracking più aggressivo, ma rischio oscillazioni
MRP_STEERING_K1 = 0.55

# Guadagno cubico
# Controlla l'approccio al limite omega_max
# Aumentare = transizione più smooth verso omega_max
MRP_STEERING_K3 = 0.01

# Velocità angolare massima comandata [rad/s]
# PARAMETRO CRITICO per prevenire saturazione RW
# Ridotto per manovre grandi più smooth e limitare accumulo momento angolare
MRP_STEERING_OMEGA_MAX_RAD_S = 0.006  # 0.006 rad/s = 0.34 deg/s

# Limiti errore attitudinale (MRP switching threshold)
# Se |sigma| > 1, switch to shadow set per evitare singolarità
MRP_SWITCH_THRESHOLD = 0.9

# ===== Rate Servo Controller (Inner Loop) =====

# Guadagno proporzionale
# Ridotto da 1.5 a 0.9 per prevenire picchi di saturazione RW
RATE_SERVO_P = 0.9

# Guadagno integrale
# Ridotto per evitare overshoot e accumulo integrator windup
RATE_SERVO_Ki = 0.005

# Limite integrale (anti-windup)
# Previene accumulo eccessivo del termine integrale
RATE_SERVO_INTEGRAL_LIMIT = 2.0 / RATE_SERVO_Ki * 0.1  # ~40.0

# Guadagno matriciale (per controllo avanzato)
# Matrice P per stabilizzazione robusta
RATE_SERVO_P_MATRIX = 150.0 * np.eye(3)  # Guadagno scalato per identità

# ===== B-dot Controller (Detumbling) =====

# Guadagno B-dot [A*m^2/T/s]
# Controlla aggressività detumbling con magnetorquers
# Valore negativo per dissipazione energia
# Aumentare magnitudine = detumbling più rapido ma maggior consumo energia
BDOT_GAIN = -3e5

# Threshold velocità angolare per considerare detumbling completo [rad/s]
# Quando |omega| < threshold, passa a sun-safe mode
BDOT_OMEGA_THRESHOLD_RAD_S = 1e-2  # 0.01 rad/s = 0.57 deg/s

# ===== Momentum Management (RW Desaturation) =====

# Threshold momento angolare RW per attivare desaturazione [N*m*s]
# Quando momento accumul ato > threshold, usa MTB per scaricare RW
RW_DESATURATION_THRESHOLD_NMS = 5.0

# Guadagno desaturazione con magnetorquers
# Controlla aggressività desaturazione
MTB_DESATURATION_GAIN = 1e-4

# Frequenza controllo desaturazione [Hz]
DESATURATION_UPDATE_RATE_HZ = 0.1  # Ogni 10 secondi

# ===== Low-Pass Filter (Torque Command Filtering) =====

# Cutoff frequency per filtro passa-basso su comandi coppia [rad/s]
# Riduce noise ad alta frequenza e jitter negli attuatori
TORQUE_FILTER_OMEGA_C_RAD_S = 2.0 * np.pi * 0.01  # 0.01 Hz cutoff

# Time constant [s]
TORQUE_FILTER_TAU_S = 1.0 / (TORQUE_FILTER_OMEGA_C_RAD_S / (2.0 * np.pi))

# ===== Panel Deployment Controller =====

# Guadagno proporzionale motori pannelli (deployment)
PANEL_MOTOR_P_DEPLOYMENT = 0.1  # Moderate damping durante deployment

# Guadagno proporzionale motori pannelli (operazioni)
PANEL_MOTOR_P_OPERATIONS = 0.03  # Ridotto per movimento più smooth

# Velocità deployment [rad/s]
PANEL_DEPLOYMENT_RATE_RAD_S = 0.1  # Lenta per sicurezza

# ===== Pointing Modes Thresholds =====

# Sun-Safe Pointing
SUN_POINTING_ACCURACY_THRESHOLD_DEG = 5.0  # deg - accuratezza richiesta

# Nadir Pointing
NADIR_POINTING_ACCURACY_THRESHOLD_DEG = 3.0  # deg

# Ground Station Pointing
GS_POINTING_ACCURACY_THRESHOLD_DEG = 2.0  # deg - più stringente per comunicazione

# ===== Controller Saturation Limits =====

# Limite coppia totale comando [N*m]
MAX_TORQUE_COMMAND_NM = 0.2  # Limite fisico RW

# Limite dipolo MTB comando [A*m^2]
MAX_MTB_DIPOLE_AM2 = 0.2  # Limite fisico MTB

# ===== Advanced Control Parameters =====

# Deadband per errore attitudinale [deg]
# Se errore < deadband, non applica correzione (risparmio energia)
ATTITUDE_ERROR_DEADBAND_DEG = 0.1

# Deadband per rate error [deg/s]
RATE_ERROR_DEADBAND_DEG_S = 0.01

# Slew rate limit [deg/s]
# Velocità massima manovra (safety limit)
MAX_SLEW_RATE_DEG_S = 1.0

# Acceleration limit [deg/s^2]
# Accelerazione angolare massima
MAX_ANGULAR_ACCEL_DEG_S2 = 0.1

# ===== Tuning Guidelines =====

# Linee guida per tuning (da documentare in docs/)
TUNING_GUIDELINES = {
    'oscillatory': 'Ridurre K1, aumentare K3',
    'slow_tracking': 'Aumentare K1 e RATE_SERVO_P',
    'rw_saturation': 'Ridurre OMEGA_MAX o aumentare DESATURATION_GAIN',
    'overshoot': 'Ridurre OMEGA_MAX, ridurre Ki',
    'detumbling_slow': 'Aumentare BDOT_GAIN (magnitudine)',
    'jitter': 'Aumentare TORQUE_FILTER_OMEGA_C_RAD_S'
}

# ===== Performance Metrics Target =====

# Obiettivi di performance per validazione
PERFORMANCE_TARGETS = {
    'detumbling_time_h': 4.0,           # Detumbling < 4 ore
    'steady_state_error_deg': 1.0,      # Errore < 1°
    'rms_tracking_error_deg': 0.5,      # RMS < 0.5°
    'max_rw_speed_rpm': 6000,           # Velocità RW < 6000 RPM
    'settling_time_s': 300              # Settling < 5 min
}
