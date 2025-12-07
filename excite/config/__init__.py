"""
Excite Configuration Module

Questo modulo contiene tutti i parametri di configurazione del satellite EXCITE,
organizzati per categoria.
"""

from . import spacecraft
from . import actuators
from . import sensors
from . import mission
from . import control
from . import environment
from . import constants

__all__ = [
    'spacecraft',
    'actuators',
    'sensors',
    'mission',
    'control',
    'environment',
    'constants'
]
