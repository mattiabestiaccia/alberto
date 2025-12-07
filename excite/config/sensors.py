"""
Parametri sensori EXCITE

Questo modulo contiene le specifiche di tutti i sensori del satellite EXCITE:
- Star Tracker - Determinazione assetto ad alta precisione
- IMU (Inertial Measurement Unit) - Giroscopi e accelerometri
- TAM (Three-Axis Magnetometer) - Misura campo magnetico
- CSS (Coarse Sun Sensors) - Determinazione direzione solare
- GPS/GNSS - Posizione e velocità orbitale
"""

import numpy as np
from Basilisk.utilities import macros as mc

# ===== Star Tracker =====

# Accuratezza misura assetto [deg]
STAR_TRACKER_ACCURACY_3SIGMA_DEG = 0.01  # 0.01° (3-sigma)

# Noise standard deviation [rad]
STAR_TRACKER_NOISE_STD_RAD = (STAR_TRACKER_ACCURACY_3SIGMA_DEG / 3.0) * mc.D2R

# Field of View [deg]
STAR_TRACKER_FOV_DEG = 20.0

# Update rate [Hz]
STAR_TRACKER_UPDATE_RATE_HZ = 4.0

# Consumo energetico
STAR_TRACKER_POWER_W = 3.0  # W

# Temperatura operativa [°C]
STAR_TRACKER_TEMP_MIN_C = -40.0
STAR_TRACKER_TEMP_MAX_C = 60.0

# ===== IMU Custom (Giroscopi + Accelerometri) =====

# Accuratezza velocità angolare [deg/s] (3-sigma)
IMU_GYRO_ACCURACY_3SIGMA_DEG_S = 0.05

# Noise standard deviation [rad/s]
IMU_GYRO_NOISE_STD_RAD_S = (IMU_GYRO_ACCURACY_3SIGMA_DEG_S / 3.0) * mc.D2R

# Bias drift rate [deg/s/sqrt(hr)]
IMU_GYRO_BIAS_DRIFT_DEG_S_SQRTHR = 0.01

# Bias stability [deg/hr]
IMU_GYRO_BIAS_STABILITY_DEG_HR = 1.0

# Range misura [deg/s]
IMU_GYRO_RANGE_DEG_S = 300.0

# Accelerometer accuracy [m/s^2] (3-sigma)
IMU_ACCEL_ACCURACY_3SIGMA_M_S2 = 0.01

# Noise standard deviation [m/s^2]
IMU_ACCEL_NOISE_STD_M_S2 = IMU_ACCEL_ACCURACY_3SIGMA_M_S2 / 3.0

# Update rate [Hz]
IMU_UPDATE_RATE_HZ = 100.0

# Consumo energetico
IMU_POWER_W = 0.5  # W

# ===== TAM (Three-Axis Magnetometer) =====

# Accuratezza misura campo magnetico [nT] (3-sigma)
TAM_ACCURACY_3SIGMA_NT = 50.0  # 50 nanoTesla

# Noise standard deviation [T]
TAM_NOISE_STD_T = (TAM_ACCURACY_3SIGMA_NT / 3.0) * 1e-9

# Range misura [T]
TAM_RANGE_T = 100e-6  # ±100 μT (campo Terra ~30-60 μT)

# Update rate [Hz]
TAM_UPDATE_RATE_HZ = 10.0

# Consumo energetico
TAM_POWER_W = 0.2  # W

# Offset bias [T]
TAM_BIAS_T = np.array([0.0, 0.0, 0.0])  # Calibrato a zero

# ===== CSS (Coarse Sun Sensors) =====

# Numero di sensori CSS (tipicamente 6 per copertura omnidirezionale)
NUM_CSS = 6

# Accuratezza direzione sole [deg] (3-sigma)
CSS_ACCURACY_3SIGMA_DEG = 1.0

# Noise standard deviation [rad]
CSS_NOISE_STD_RAD = (CSS_ACCURACY_3SIGMA_DEG / 3.0) * mc.D2R

# Field of View per sensore [deg]
CSS_FOV_DEG = 120.0

# Albedo (riflettività solare)
CSS_ALBEDO = 0.3

# Update rate [Hz]
CSS_UPDATE_RATE_HZ = 10.0

# Consumo energetico
CSS_POWER_SINGLE_W = 0.05  # W per sensore
CSS_POWER_TOTAL_W = NUM_CSS * CSS_POWER_SINGLE_W  # ~0.3 W

# Posizioni CSS sul corpo (normali alle facce del CubeSat)
CSS_NORMALS_B = [
    np.array([1.0, 0.0, 0.0]),   # +X face
    np.array([-1.0, 0.0, 0.0]),  # -X face
    np.array([0.0, 1.0, 0.0]),   # +Y face
    np.array([0.0, -1.0, 0.0]),  # -Y face
    np.array([0.0, 0.0, 1.0]),   # +Z face
    np.array([0.0, 0.0, -1.0])   # -Z face
]

# ===== GPS/GNSS =====

# Accuratezza posizione [m] (3-sigma)
GPS_POSITION_ACCURACY_3SIGMA_M = 10.0

# Noise standard deviation position [m]
GPS_POSITION_NOISE_STD_M = GPS_POSITION_ACCURACY_3SIGMA_M / 3.0

# Accuratezza velocità [m/s] (3-sigma)
GPS_VELOCITY_ACCURACY_3SIGMA_M_S = 0.1

# Noise standard deviation velocity [m/s]
GPS_VELOCITY_NOISE_STD_M_S = GPS_VELOCITY_ACCURACY_3SIGMA_M_S / 3.0

# Update rate [Hz]
GPS_UPDATE_RATE_HZ = 1.0

# Consumo energetico
GPS_POWER_W = 1.5  # W

# ===== SimpleNav Configuration =====

# SimpleNav fornisce "truth data" con errori realistici per tutti i sensori
# Questa configurazione aggrega i contributi di GPS, Star Tracker, IMU, CSS

# Position standard deviation [m]
SIMPLENAV_POSITION_STD_M = GPS_POSITION_NOISE_STD_M

# Velocity standard deviation [m/s]
SIMPLENAV_VELOCITY_STD_M_S = GPS_VELOCITY_NOISE_STD_M_S

# Attitude standard deviation [rad]
SIMPLENAV_ATTITUDE_STD_RAD = STAR_TRACKER_NOISE_STD_RAD

# Angular rate standard deviation [rad/s]
SIMPLENAV_RATE_STD_RAD_S = IMU_GYRO_NOISE_STD_RAD_S

# Sun direction standard deviation [rad]
SIMPLENAV_SUN_STD_RAD = CSS_NOISE_STD_RAD

# P Matrix diagonal (covariance matrix)
SIMPLENAV_P_MATRIX_DIAG = [
    SIMPLENAV_POSITION_STD_M, SIMPLENAV_POSITION_STD_M, SIMPLENAV_POSITION_STD_M,
    SIMPLENAV_VELOCITY_STD_M_S, SIMPLENAV_VELOCITY_STD_M_S, SIMPLENAV_VELOCITY_STD_M_S,
    SIMPLENAV_ATTITUDE_STD_RAD, SIMPLENAV_ATTITUDE_STD_RAD, SIMPLENAV_ATTITUDE_STD_RAD,
    SIMPLENAV_RATE_STD_RAD_S, SIMPLENAV_RATE_STD_RAD_S, SIMPLENAV_RATE_STD_RAD_S,
    SIMPLENAV_SUN_STD_RAD, SIMPLENAV_SUN_STD_RAD, SIMPLENAV_SUN_STD_RAD
]

# Walk bounds (random walk limits)
SIMPLENAV_WALK_BOUNDS = [
    SIMPLENAV_POSITION_STD_M, SIMPLENAV_POSITION_STD_M, SIMPLENAV_POSITION_STD_M,
    SIMPLENAV_VELOCITY_STD_M_S, SIMPLENAV_VELOCITY_STD_M_S, SIMPLENAV_VELOCITY_STD_M_S,
    SIMPLENAV_ATTITUDE_STD_RAD, SIMPLENAV_ATTITUDE_STD_RAD, SIMPLENAV_ATTITUDE_STD_RAD,
    SIMPLENAV_RATE_STD_RAD_S, SIMPLENAV_RATE_STD_RAD_S, SIMPLENAV_RATE_STD_RAD_S,
    SIMPLENAV_SUN_STD_RAD, SIMPLENAV_SUN_STD_RAD, SIMPLENAV_SUN_STD_RAD
]

# ===== Power Consumption Summary =====

# Consumo totale sensori
SENSORS_POWER_TOTAL_W = (STAR_TRACKER_POWER_W +
                         IMU_POWER_W +
                         TAM_POWER_W +
                         CSS_POWER_TOTAL_W +
                         GPS_POWER_W)  # ~5.2 W
