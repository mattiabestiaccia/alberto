"""
Parametri missione e timeline EXCITE

Questo modulo contiene i parametri della missione EXCITE:
- Elementi orbitali e condizioni iniziali
- Timeline missione e fasi FSM
- Ground station configuration
- Condizioni iniziali di assetto
"""

import numpy as np
from Basilisk.utilities import macros

# ===== Parametri Orbitali =====

# Elementi orbitali classici
ORBITAL_ELEMENTS = {
    'semi_major_axis_m': 6878.137e3,  # m - semiasse maggiore (raggio Terra 6378 km + 500 km altitudine)
    'eccentricity': 0.01,              # eccentricità (quasi circolare)
    'inclination_deg': 97.99,          # deg - inclinazione per orbita eliosincrona (SSO)
    'raan_deg': 70.0,                  # deg - Right Ascension of Ascending Node (ottimizzato per Pisa)
    'argument_periapsis_deg': 0.0,     # deg - argomento del periasse
    'true_anomaly_deg': 0.0            # deg - anomalia vera iniziale
}

# Conversioni rad
ORBITAL_ELEMENTS_RAD = {
    'semi_major_axis_m': ORBITAL_ELEMENTS['semi_major_axis_m'],
    'eccentricity': ORBITAL_ELEMENTS['eccentricity'],
    'inclination_rad': ORBITAL_ELEMENTS['inclination_deg'] * macros.D2R,
    'raan_rad': ORBITAL_ELEMENTS['raan_deg'] * macros.D2R,
    'argument_periapsis_rad': ORBITAL_ELEMENTS['argument_periapsis_deg'] * macros.D2R,
    'true_anomaly_rad': ORBITAL_ELEMENTS['true_anomaly_deg'] * macros.D2R
}

# Altitudine orbita
ORBIT_ALTITUDE_KM = 500.0  # km
ORBIT_PERIOD_MIN = 94.6    # min - periodo orbitale approssimativo

# ===== Condizioni Iniziali Assetto =====

# Assetto iniziale in Modified Rodriguez Parameters (MRP)
# Rappresenta orientazione iniziale dopo separazione dal lanciatore
INITIAL_ATTITUDE_MRP = np.array([0.0, 0.2, -0.3])

# Velocità angolare iniziale [rad/s]
# Satellite tumbling dopo separazione
INITIAL_ANGULAR_VELOCITY_RAD_S = np.array([0.4, -0.4, 0.5])

# ===== Timeline Missione =====

# Durata simulazione totale [ore]
MISSION_DURATION_HOURS = 24.0

# Durata simulazione [secondi]
MISSION_DURATION_SECONDS = MISSION_DURATION_HOURS * 3600.0

# ===== Fasi Missione FSM (Finite State Machine) =====

# Fase 1: DEPLOYMENT
# Deployment pannelli solari dopo separazione
DEPLOYMENT_DURATION_S = 60.0  # secondi - tempo per deployment completo

# Fase 2: DETUMBLING
# Detumbling con B-dot controller per ridurre velocità angolare
DETUMBLING_MAX_DURATION_HOURS = 12.0  # ore - timeout backup se detumbling non converge
DETUMBLING_COMPLETE_THRESHOLD_RAD_S = 1e-2  # rad/s - soglia |omega| per considerare detumbling completo

# Fase 3: SUN-SAFE POINTING
# Pointing pannelli solari verso sole dopo detumbling
POST_DEPLOYMENT_WAIT_HOURS = 1.0  # ore - attesa stabilizzazione dopo deployment

# Fase 4: INITIAL CHARGE
# Ricarica batteria in sun-safe mode
INITIAL_CHARGE_DURATION_HOURS = 1.0  # ore

# Fase 5: PAYLOAD MODE A (Earth-facing)
# Payload punta verso Terra
PAYLOAD_A_DURATION_HOURS = 2.0  # ore

# Fase 6: PAYLOAD MODE B (Nadir-pointing)
# Payload in nadir pointing
PAYLOAD_B_DURATION_HOURS = 1.5  # ore

# Fase 7: GS CONTACT
# Ground station contact window
GS_CONTACT_MIN_DURATION_MIN = 5.0  # min - durata minima contatto utile

# ===== Soglie Transizioni FSM =====

# Eclissi threshold (0.0 = eclissi totale, 1.0 = sole pieno)
ECLIPSE_SHADOW_THRESHOLD = 0.1

# Battery charge thresholds [%]
BATTERY_LOW_THRESHOLD_PERCENT = 30.0
BATTERY_HIGH_THRESHOLD_PERCENT = 80.0

# Pointing accuracy threshold [deg]
POINTING_ACCURACY_THRESHOLD_DEG = 5.0

# ===== Ground Station Configuration =====

# Ground Station Pisa (UniPi)
GS_LOCATION = {
    'name': 'PisaGroundStation',
    'latitude_deg': 43.7,      # gradi Nord
    'longitude_deg': 10.4,     # gradi Est
    'altitude_m': 0.0,         # m slm
    'min_elevation_deg': 10.0  # gradi - elevazione minima per visibilità
}

# Parametri comunicazione
GS_MAX_RANGE_KM = 2000.0  # km - range massimo comunicazione
GS_ANTENNA_GAIN_DB = 15.0  # dB - guadagno antenna ground station

# ===== Epoch e Tempo =====

# Data inizio missione (UTC)
# Utilizzato per calcolo effemeridi SPICE
MISSION_EPOCH_UTC = "2025-06-01T12:00:00"

# Simulation start time [s]
SIM_START_TIME_S = 0.0

# ===== Rate di Esecuzione =====

# Dynamics update rate
DYNAMICS_RATE_HZ = 2.0  # Hz
DYNAMICS_TIMESTEP_S = 1.0 / DYNAMICS_RATE_HZ  # 0.5 s

# FSW update rate
FSW_RATE_HZ = 1.0  # Hz
FSW_TIMESTEP_S = 1.0 / FSW_RATE_HZ  # 1.0 s

# IMPORTANTE: fswRate <= dynRate per garantire message ordering
# FSW legge messaggi scritti da Dynamics, quindi FSW deve essere <= Dynamics
assert FSW_TIMESTEP_S >= DYNAMICS_TIMESTEP_S, \
    "FSW timestep must be >= Dynamics timestep for correct message passing"

# ===== Mission Success Criteria =====

# Criteri di successo missione
SUCCESS_CRITERIA = {
    'detumbling_time_max_h': DETUMBLING_MAX_DURATION_HOURS,
    'final_omega_max_rad_s': 0.02,  # rad/s
    'pointing_accuracy_deg': POINTING_ACCURACY_THRESHOLD_DEG,
    'battery_survival': True,  # Batteria non deve mai scendere < 10%
    'gs_contacts_min': 2  # Almeno 2 contatti con GS in 24h
}

# ===== Fasi Missione Dettagliate =====

# Descrizione sequenza fasi per reference
MISSION_PHASES = {
    1: {
        'name': 'DEPLOYMENT',
        'description': 'Deployment pannelli solari',
        'duration_s': DEPLOYMENT_DURATION_S,
        'start_trigger': 'Mission start',
        'end_trigger': 'Deployment complete'
    },
    2: {
        'name': 'DETUMBLING',
        'description': 'Riduzione velocità angolare con B-dot',
        'duration_s': DETUMBLING_MAX_DURATION_HOURS * 3600,
        'start_trigger': 'Deployment complete',
        'end_trigger': f'|omega| < {DETUMBLING_COMPLETE_THRESHOLD_RAD_S} rad/s'
    },
    3: {
        'name': 'SUN_SAFE',
        'description': 'Sun-safe pointing per ricarica',
        'duration_s': POST_DEPLOYMENT_WAIT_HOURS * 3600,
        'start_trigger': 'Detumbling complete',
        'end_trigger': 'Stabilization complete'
    },
    4: {
        'name': 'INITIAL_CHARGE',
        'description': 'Ricarica batteria iniziale',
        'duration_s': INITIAL_CHARGE_DURATION_HOURS * 3600,
        'start_trigger': 'Sun-safe stable',
        'end_trigger': 'Battery charged'
    },
    5: {
        'name': 'PAYLOAD_A',
        'description': 'Payload mode A - Earth facing',
        'duration_s': PAYLOAD_A_DURATION_HOURS * 3600,
        'start_trigger': 'Charge complete',
        'end_trigger': 'Time elapsed'
    },
    6: {
        'name': 'PAYLOAD_B',
        'description': 'Payload mode B - Nadir pointing',
        'duration_s': PAYLOAD_B_DURATION_HOURS * 3600,
        'start_trigger': 'Payload A complete',
        'end_trigger': 'Time elapsed'
    },
    7: {
        'name': 'GS_CONTACT',
        'description': 'Ground station pointing (Pisa)',
        'duration_s': None,  # Event-driven
        'start_trigger': 'GS in view',
        'end_trigger': 'GS out of view'
    }
}
