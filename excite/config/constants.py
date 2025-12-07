"""
Costanti fisiche e conversioni EXCITE

Questo modulo contiene costanti fisiche universali e fattori di conversione
utilizzati in tutta la simulazione.
"""

import numpy as np

# ===== Costanti Fisiche Universali =====

# Velocità della luce nel vuoto [m/s]
SPEED_OF_LIGHT_M_S = 299792458.0

# Costante gravitazionale universale [m^3/(kg*s^2)]
GRAVITATIONAL_CONSTANT = 6.67430e-11

# Costante di Boltzmann [J/K]
BOLTZMANN_CONSTANT = 1.380649e-23

# Costante di Stefan-Boltzmann [W/(m^2*K^4)]
STEFAN_BOLTZMANN_CONSTANT = 5.670374419e-8

# Numero di Avogadro [1/mol]
AVOGADRO_NUMBER = 6.02214076e23

# ===== Costanti Astronomiche =====

# Unità Astronomica (distanza media Terra-Sole) [m]
ASTRONOMICAL_UNIT_M = 1.495978707e11

# Raggio equatoriale Terra (WGS84) [m]
EARTH_RADIUS_EQUATORIAL_M = 6378137.0

# Raggio polare Terra (WGS84) [m]
EARTH_RADIUS_POLAR_M = 6356752.314245

# Raggio medio Terra [m]
EARTH_RADIUS_MEAN_M = 6371000.0

# Eccentricità Terra (WGS84)
EARTH_ECCENTRICITY = 0.081819190842622

# Velocità angolare rotazione Terra [rad/s]
EARTH_ROTATION_RATE_RAD_S = 7.2921159e-5  # ~15.04 deg/hr

# Giorno siderale [s]
SIDEREAL_DAY_S = 86164.0905

# Anno siderale [s]
SIDEREAL_YEAR_S = 31558149.7635

# Obliquità dell'eclittica (inclinazione asse Terra) [deg]
EARTH_OBLIQUITY_DEG = 23.43928

# Massa Terra [kg]
EARTH_MASS_KG = 5.972168e24

# Massa Sole [kg]
SUN_MASS_KG = 1.98892e30

# Massa Luna [kg]
MOON_MASS_KG = 7.346e22

# Raggio Sole [m]
SUN_RADIUS_M = 6.96e8

# Raggio Luna [m]
MOON_RADIUS_M = 1.7374e6

# ===== Parametri Orbitali Terra =====

# Parametro gravitazionale Terra μ = G*M [m^3/s^2]
MU_EARTH = 3.986004418e14

# Parametro gravitazionale Sole [m^3/s^2]
MU_SUN = 1.32712440018e20

# Parametro gravitazionale Luna [m^3/s^2]
MU_MOON = 4.9028e12

# Velocità di fuga dalla superficie terrestre [km/s]
EARTH_ESCAPE_VELOCITY_KM_S = 11.186

# ===== Conversioni Angolari =====

# Gradi to radianti
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi

# Arcminuti to radianti
ARCMIN_TO_RAD = DEG_TO_RAD / 60.0
RAD_TO_ARCMIN = RAD_TO_DEG * 60.0

# Arcsecondi to radianti
ARCSEC_TO_RAD = ARCMIN_TO_RAD / 60.0
RAD_TO_ARCSEC = RAD_TO_ARCMIN * 60.0

# Rivoluzioni to radianti
REV_TO_RAD = 2.0 * np.pi
RAD_TO_REV = 1.0 / (2.0 * np.pi)

# RPM to rad/s
RPM_TO_RAD_S = 2.0 * np.pi / 60.0
RAD_S_TO_RPM = 60.0 / (2.0 * np.pi)

# ===== Conversioni Tempo =====

# Secondi in un minuto
SECONDS_PER_MINUTE = 60.0

# Secondi in un'ora
SECONDS_PER_HOUR = 3600.0

# Secondi in un giorno
SECONDS_PER_DAY = 86400.0

# Minuti in un'ora
MINUTES_PER_HOUR = 60.0

# Ore in un giorno
HOURS_PER_DAY = 24.0

# Giorni in un anno (medio)
DAYS_PER_YEAR = 365.25

# Nanosecondi in un secondo
NANO_PER_SECOND = 1e9

# Microsecondi in un secondo
MICRO_PER_SECOND = 1e6

# Millisecondi in un secondo
MILLI_PER_SECOND = 1e3

# ===== Conversioni Distanza =====

# Chilometri to metri
KM_TO_M = 1000.0
M_TO_KM = 0.001

# Centimetri to metri
CM_TO_M = 0.01
M_TO_CM = 100.0

# Millimetri to metri
MM_TO_M = 0.001
M_TO_MM = 1000.0

# Miglia nautiche to metri
NM_TO_M = 1852.0
M_TO_NM = 1.0 / 1852.0

# Piedi to metri
FT_TO_M = 0.3048
M_TO_FT = 1.0 / 0.3048

# ===== Conversioni Massa =====

# Grammi to kilogrammi
G_TO_KG = 0.001
KG_TO_G = 1000.0

# Libbre to kilogrammi
LB_TO_KG = 0.45359237
KG_TO_LB = 1.0 / 0.45359237

# ===== Conversioni Forza/Coppia =====

# Newton to kilogrammi-forza
N_TO_KGF = 1.0 / 9.80665
KGF_TO_N = 9.80665

# Libbra-forza to Newton
LBF_TO_N = 4.4482216152605
N_TO_LBF = 1.0 / 4.4482216152605

# ===== Conversioni Energia/Potenza =====

# Joule to Watt-ora
J_TO_WH = 1.0 / 3600.0
WH_TO_J = 3600.0

# Kilowatt-ora to Joule
KWH_TO_J = 3.6e6
J_TO_KWH = 1.0 / 3.6e6

# ===== Conversioni Pressione =====

# Pascal to bar
PA_TO_BAR = 1e-5
BAR_TO_PA = 1e5

# Pascal to atmosfere
PA_TO_ATM = 1.0 / 101325.0
ATM_TO_PA = 101325.0

# Pascal to PSI (pound per square inch)
PA_TO_PSI = 1.0 / 6894.757
PSI_TO_PA = 6894.757

# ===== Conversioni Temperatura =====

# Kelvin offset
CELSIUS_TO_KELVIN_OFFSET = 273.15

# Conversione Celsius -> Kelvin
def celsius_to_kelvin(temp_c):
    """Converte temperatura da Celsius a Kelvin"""
    return temp_c + CELSIUS_TO_KELVIN_OFFSET

# Conversione Kelvin -> Celsius
def kelvin_to_celsius(temp_k):
    """Converte temperatura da Kelvin a Celsius"""
    return temp_k - CELSIUS_TO_KELVIN_OFFSET

# Conversione Fahrenheit -> Celsius
def fahrenheit_to_celsius(temp_f):
    """Converte temperatura da Fahrenheit a Celsius"""
    return (temp_f - 32.0) * 5.0 / 9.0

# Conversione Celsius -> Fahrenheit
def celsius_to_fahrenheit(temp_c):
    """Converte temperatura da Celsius a Fahrenheit"""
    return temp_c * 9.0 / 5.0 + 32.0

# ===== Costanti Basilisk-Specific =====

# Conversioni macros Basilisk
# (Disponibili anche da Basilisk.utilities.macros)

# Massa Terra per Basilisk
BSK_MU_EARTH = MU_EARTH

# Raggio Terra per Basilisk
BSK_EARTH_RADIUS = EARTH_RADIUS_MEAN_M

# Velocità rotazione Terra per Basilisk
BSK_EARTH_OMEGA = EARTH_ROTATION_RATE_RAD_S

# ===== Limiti Numerici =====

# Epsilon per confronti floating point
FLOAT_EPSILON = 1e-12

# Infinito per calcoli
INFINITY = np.inf

# Pi greco (alta precisione)
PI = np.pi

# Numero di Eulero
E = np.e

# ===== Unit Test Tolerances =====

# Tolleranze per unit test

# Tolleranza posizione [m]
TEST_TOLERANCE_POSITION_M = 1.0

# Tolleranza velocità [m/s]
TEST_TOLERANCE_VELOCITY_M_S = 0.01

# Tolleranza assetto [rad]
TEST_TOLERANCE_ATTITUDE_RAD = 1e-6

# Tolleranza velocità angolare [rad/s]
TEST_TOLERANCE_RATE_RAD_S = 1e-8
