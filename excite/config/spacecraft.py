"""
Parametri configurazione spacecraft EXCITE

Questo modulo contiene tutti i parametri fisici e geometrici del satellite EXCITE:
- Massa e tensore d'inerzia
- Geometria e dimensioni
- Proprietà superficiali per drag e SRP
- Dipolo magnetico residuo
- Configurazione pannelli solari
"""

import numpy as np

# ===== Massa e Inerzia =====

# Massa totale satellite [kg]
TOTAL_MASS_KG = 14.0

# Tensore d'inerzia rispetto al centro di massa in body frame [kg*m^2]
# Formato: [Ixx, Ixy, Ixz,
#           Iyx, Iyy, Iyz,
#           Izx, Izy, Izz]
INERTIA_TENSOR_KG_M2 = np.array([
    [0.3280, -0.006412, -0.003930],
    [-0.006412, 0.3458, -0.004116],
    [-0.003930, -0.004116, 0.1508]
])

# Posizione centro di massa rispetto al punto di riferimento body B [m]
# Nel sistema di coordinate body-fixed
CENTER_OF_MASS_B = np.array([0.0, 0.0, 0.0])

# ===== Geometria Spacecraft =====

# Dimensioni CubeSat 12U [m]
# CubeSat configuration: 3U x 2U x 2U
SPACECRAFT_DIMENSIONS_M = {
    'length_x': 0.34,   # m - 3U lungo asse X
    'width_y': 0.20,    # m - 2U lungo asse Y
    'height_z': 0.20    # m - 2U lungo asse Z
}

# Aree superficiali per drag e SRP [m^2]
# Top/bottom faces (perpendicular to Z axis)
BODY_TOP_BOTTOM_AREA_M2 = 0.34 * 0.20  # 0.068 m^2

# Side faces
BODY_SIDE_AREA_M2 = 0.34 * 0.20  # 0.068 m^2

# ===== Proprietà Superficiali =====

# Coefficienti di drag per diverse superfici
# Lista per 14 faccette (body panels + deployable panels)
DRAG_COEFFICIENTS = [
    1.9,  # +Z top
    1.8,  # -Z bottom
    1.8, 1.8, 1.8, 1.8,  # Lati body
    2.0,  # Deploy panel 1
    2.0,  # Deploy panel 2
    2.0,  # Deploy panel 3
    2.0,  # Deploy panel 4
    2.0,  # Deploy panel 5 (spare)
    2.0,  # Deploy panel 6 (spare)
    2.0,  # Deploy panel 7 (spare)
    2.0   # Deploy panel 8 (spare)
]

# Riflettività per SRP (Solar Radiation Pressure)
# 0.0 = assorbimento totale, 1.0 = riflessione speculare
SRP_REFLECTIVITY = 0.9  # Pannelli solari ad alta riflettività

# ===== Dipolo Magnetico Residuo =====

# Dipolo magnetico residuo dello spacecraft [A*m^2]
# Dovuto a magnetizzazione permanente e correnti interne
MAGNETIC_DIPOLE_AM2 = np.array([0.035, 0.035, 0.02])

# ===== Pannelli Solari =====

# Numero di pannelli solari deployable
NUM_DEPLOYABLE_PANELS = 4

# Area singolo pannello solare [m^2]
PANEL_AREA_M2 = 0.03

# Efficienza celle solari
PANEL_EFFICIENCY = 0.29  # 29%

# Normale ai pannelli in panel frame (4 pannelli dispiegabili)
# Dopo deployment, puntano verso +X, -X, +Y, -Y
PANEL_NORMAL_VECTORS = [
    np.array([1.0, 0.0, 0.0]),   # Panel 1: +X direction
    np.array([-1.0, 0.0, 0.0]),  # Panel 2: -X direction
    np.array([0.0, 1.0, 0.0]),   # Panel 3: +Y direction
    np.array([0.0, -1.0, 0.0])   # Panel 4: -Y direction
]

# Posizioni pannelli rispetto al COM in body frame [m]
# Prima del deployment (stowed configuration)
PANEL_POSITIONS_STOWED_B = [
    np.array([0.023, 0.023, 0.046]),   # Panel 1
    np.array([-0.023, 0.023, 0.046]),  # Panel 2
    np.array([0.023, -0.023, 0.046]),  # Panel 3
    np.array([-0.023, -0.023, 0.046])  # Panel 4
]

# ===== Facets per Drag/SRP =====

# Numero totale di faccette per calcolo drag/SRP
NUM_FACETS = 14  # 6 body + 8 deployable (4 attivi + 4 spare)

# Aree faccette [m^2]
FACET_AREAS_M2 = [
    BODY_TOP_BOTTOM_AREA_M2,  # +Z top
    BODY_TOP_BOTTOM_AREA_M2,  # -Z bottom
    BODY_SIDE_AREA_M2,        # +X side
    BODY_SIDE_AREA_M2,        # -X side
    BODY_SIDE_AREA_M2,        # +Y side
    BODY_SIDE_AREA_M2,        # -Y side
    PANEL_AREA_M2,            # Deploy panel 1
    PANEL_AREA_M2,            # Deploy panel 2
    PANEL_AREA_M2,            # Deploy panel 3
    PANEL_AREA_M2,            # Deploy panel 4
    PANEL_AREA_M2,            # Spare 5
    PANEL_AREA_M2,            # Spare 6
    PANEL_AREA_M2,            # Spare 7
    PANEL_AREA_M2             # Spare 8
]

# Vettori normali alle faccette in facet frame
FACET_NORMALS_F = [np.array([0.0, 0.0, 1.0]) for _ in range(NUM_FACETS)]

# Vettori di rotazione per faccette articolate (pannelli deployable)
FACET_ROTATION_AXES_F = [np.array([1.0, 0.0, 0.0]) for _ in range(NUM_FACETS)]

# Posizioni faccette rispetto al COM in body frame [m]
FACET_POSITIONS_B = [
    np.array([0.0, 0.0, 0.10]),      # +Z top center
    np.array([0.0, 0.0, -0.10]),     # -Z bottom center
    np.array([0.17, 0.0, 0.0]),      # +X side center
    np.array([-0.17, 0.0, 0.0]),     # -X side center
    np.array([0.0, 0.10, 0.0]),      # +Y side center
    np.array([0.0, -0.10, 0.0]),     # -Y side center
    np.array([0.17, 0.10, 0.046]),   # Deploy panel 1
    np.array([-0.17, 0.10, 0.046]),  # Deploy panel 2
    np.array([0.17, -0.10, 0.046]),  # Deploy panel 3
    np.array([-0.17, -0.10, 0.046]), # Deploy panel 4
    np.array([0.0, 0.0, 0.0]),       # Spare 5 (dummy position)
    np.array([0.0, 0.0, 0.0]),       # Spare 6
    np.array([0.0, 0.0, 0.0]),       # Spare 7
    np.array([0.0, 0.0, 0.0])        # Spare 8
]

# Matrici DCM (Direction Cosine Matrix) per orientazione faccette
# Identity matrix per faccette fisse al body
FACET_DCM_F0B = [
    np.eye(3) for _ in range(NUM_FACETS)
]
