"""
Parametri ambiente spaziale EXCITE

Questo modulo contiene i parametri dell'ambiente spaziale:
- Modelli di gravità (Terra, Sole, Luna)
- Atmosfera e drag
- Solar Radiation Pressure (SRP)
- Campo magnetico terrestre
- Eclissi
"""

import numpy as np

# ===== Gravity Bodies =====

# Corpi gravitazionali considerati nella simulazione
GRAVITY_BODIES = ['earth', 'sun', 'moon']

# Parametro gravitazionale Terra [m^3/s^2]
MU_EARTH = 3.986004418e14

# Parametro gravitazionale Sole [m^3/s^2]
MU_SUN = 1.32712440018e20

# Parametro gravitazionale Luna [m^3/s^2]
MU_MOON = 4.9028e12

# Gravitational harmonics (J2, J3, J4 per Terra)
EARTH_J2 = 1.08263e-3
EARTH_J3 = -2.532e-6
EARTH_J4 = -1.619e-6

# Abilita gradiente gravitazionale
ENABLE_GRAVITY_GRADIENT = True

# ===== Atmospheric Model =====

# Modello atmosfera esponenziale
# Valido per LEO (200-1000 km)

# Densità atmosferica al livello del mare [kg/m^3]
ATMOSPHERE_RHO_0 = 1.225

# Scala altezza atmosfera [m]
# Varia con altitudine, valore medio per 500 km
ATMOSPHERE_SCALE_HEIGHT_M = 88667.0  # ~88.7 km

# Altitudine riferimento per densità [m]
ATMOSPHERE_REF_ALTITUDE_M = 500e3  # 500 km

# Densità atmosferica a 500 km [kg/m^3]
# Valore approssimativo per attività solare moderata
ATMOSPHERE_DENSITY_500KM = 1.0e-12  # kg/m^3

# Parametri atmosfera WGS84
ATMOSPHERE_WGS84_PLANET = "Earth"

# ===== Solar Radiation Pressure (SRP) =====

# Costante solare a 1 AU [W/m^2]
SOLAR_CONSTANT_W_M2 = 1367.0

# Pressione radiazione solare a 1 AU [N/m^2]
# P_srp = S / c, dove c = velocità luce
SRP_PRESSURE_1AU_N_M2 = SOLAR_CONSTANT_W_M2 / 299792458.0  # ~4.56e-6 N/m^2

# Coefficiente riflettività medio spacecraft
# Varia da 0 (assorbimento totale) a 2 (riflessione speculare)
SRP_REFLECTIVITY_COEFF = 0.9  # Pannelli solari ad alta riflettività

# Abilita SRP
ENABLE_SRP = True

# ===== Magnetic Field Model =====

# Modello campo magnetico: WMM (World Magnetic Model)
MAGNETIC_FIELD_MODEL = "WMM"

# Anno epoch per WMM
WMM_EPOCH_YEAR = 2025

# Componenti dipolo magnetico terrestre [T]
# Approssimazione dipolo centrato
EARTH_MAGNETIC_DIPOLE_T = 7.94e-5  # ~79.4 μT all'equatore

# Abilita campo magnetico
ENABLE_MAGNETIC_FIELD = True

# ===== Eclipse Model =====

# Modello eclissi: cilindrico vs conico
ECLIPSE_MODEL = "cylindrical"  # Più semplice e veloce

# Abilita calcolo eclissi
ENABLE_ECLIPSE = True

# Threshold ombra per considerare eclissi
# 0.0 = ombra totale (umbra)
# 0.1-0.9 = penombra
# 1.0 = sole pieno
ECLIPSE_SHADOW_THRESHOLD = 0.1

# ===== Space Environment Disturbances =====

# Magnetic Disturbance Torque
# Coppia disturbo dovuta a interazione dipolo residuo con campo magnetico

# Dipolo magnetico residuo spacecraft [A*m^2]
# (Definito anche in spacecraft.py)
SPACECRAFT_MAGNETIC_DIPOLE_AM2 = np.array([0.035, 0.035, 0.02])

# Abilita disturbo magnetico
ENABLE_MAGNETIC_DISTURBANCE = True

# ===== Atmospheric Drag Parameters =====

# Coefficienti drag per diverse superfici
# (Definiti anche in spacecraft.py)

# Drag coefficient medio spacecraft
DRAG_COEFF_MEAN = 2.0

# Abilita drag atmosferico
ENABLE_ATMOSPHERIC_DRAG = True

# Velocità vento relativo [m/s]
# Per co-rotating atmosphere model
ATMOSPHERE_COROTATION = True

# ===== Third Body Perturbations =====

# Abilita perturbazione gravitazionale Sole
ENABLE_SUN_GRAVITY = True

# Abilita perturbazione gravitazionale Luna
ENABLE_MOON_GRAVITY = True

# ===== Space Weather Parameters =====

# Indice attività solare F10.7 [SFU]
# Solar Flux Unit: 1 SFU = 10^-22 W/m^2/Hz
# Range tipico: 70-250 SFU
# 150 = attività moderata
F10_7_SOLAR_FLUX = 150.0

# Indice geomagnetico Kp
# Range: 0-9 (0=quieto, 9=tempesta severa)
KP_INDEX = 3.0  # Attività moderata

# Questi parametri possono influenzare densità atmosferica
# tramite modelli avanzati (NRLMSISE-00, JB2008)

# ===== Environment Summary =====

ENVIRONMENT_CONFIG = {
    'gravity': {
        'bodies': GRAVITY_BODIES,
        'earth_harmonics': [EARTH_J2, EARTH_J3, EARTH_J4],
        'gradient': ENABLE_GRAVITY_GRADIENT
    },
    'atmosphere': {
        'model': 'exponential',
        'density_500km': ATMOSPHERE_DENSITY_500KM,
        'scale_height': ATMOSPHERE_SCALE_HEIGHT_M,
        'drag_enabled': ENABLE_ATMOSPHERIC_DRAG
    },
    'srp': {
        'enabled': ENABLE_SRP,
        'solar_constant': SOLAR_CONSTANT_W_M2,
        'reflectivity': SRP_REFLECTIVITY_COEFF
    },
    'magnetic_field': {
        'model': MAGNETIC_FIELD_MODEL,
        'enabled': ENABLE_MAGNETIC_FIELD,
        'disturbance_enabled': ENABLE_MAGNETIC_DISTURBANCE
    },
    'eclipse': {
        'model': ECLIPSE_MODEL,
        'enabled': ENABLE_ECLIPSE
    },
    'space_weather': {
        'f10_7': F10_7_SOLAR_FLUX,
        'kp': KP_INDEX
    }
}
