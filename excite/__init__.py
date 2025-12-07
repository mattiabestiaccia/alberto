"""EXCITE AOCS Simulation Package"""
__version__ = "1.0.0"

# Import subpackages
from . import config
from . import dynamics
from . import fsw
from . import scenario
from . import analysis
from . import utils

__all__ = [
    'config',
    'dynamics',
    'fsw',
    'scenario',
    'analysis',
    'utils'
]
