
import sys
import pytest
from unittest.mock import MagicMock
import numpy as np

def pytest_configure(config):
    """
    Mock Basilisk modules if they are not installed.
    This allows testing the project structure and logic without the full C++ backend.
    """
    try:
        import Basilisk
    except ImportError:
        print("\nWARNING: Basilisk not found. Mocking it for testing purposes.\n")
        
        # Create a mock Basilisk package
        mock_bsk = MagicMock()
        mock_bsk.__path__ = ['/mock/path/to/Basilisk']
        sys.modules['Basilisk'] = mock_bsk
        
        # Mock submodules used by the project
        sys.modules['Basilisk.utilities'] = MagicMock()
        sys.modules['Basilisk.utilities.orbitalMotion'] = MagicMock()
        sys.modules['Basilisk.utilities.macros'] = MagicMock()
        sys.modules['Basilisk.architecture'] = MagicMock()
        sys.modules['Basilisk.architecture.swig_common_model'] = MagicMock()
        sys.modules['Basilisk.simulation'] = MagicMock()
        sys.modules['Basilisk.fswAlgorithms'] = MagicMock()
        sys.modules['Basilisk.fswAlgorithms.tamComm'] = MagicMock()
        
        # Link mocks to ensure explicit lookup works
        sys.modules['Basilisk'].utilities = sys.modules['Basilisk.utilities']
        sys.modules['Basilisk.utilities'].orbitalMotion = sys.modules['Basilisk.utilities.orbitalMotion']
        sys.modules['Basilisk.utilities'].macros = sys.modules['Basilisk.utilities.macros']
        
        sys.modules['Basilisk'].architecture = sys.modules['Basilisk.architecture']
        sys.modules['Basilisk.architecture'].swig_common_model = sys.modules['Basilisk.architecture.swig_common_model']
        
        sys.modules['Basilisk'].simulation = sys.modules['Basilisk.simulation']
        
        sys.modules['Basilisk'].fswAlgorithms = sys.modules['Basilisk.fswAlgorithms']
        sys.modules['Basilisk.fswAlgorithms'].tamComm = sys.modules['Basilisk.fswAlgorithms.tamComm']
        
        # Mock specific macros used
        sys.modules['Basilisk.utilities.macros'].sec2nano = lambda x: x * 1e9
        sys.modules['Basilisk.utilities.macros'].hour2nano = lambda x: x * 3600 * 1e9
        sys.modules['Basilisk.utilities.macros'].D2R = 0.017453292519943295
        
        # Configure orbitalMotion
        mock_orb = MagicMock()
        mock_orb.elem2rv.return_value = (np.zeros(3), np.zeros(3))
        mock_orb.rv2elem.return_value = None
        mock_orb.elem2rv.return_value = (np.zeros(3), np.zeros(3))
        mock_orb.rv2elem.return_value = None
        sys.modules['Basilisk.utilities.orbitalMotion'] = mock_orb
        # Also set it on the parent to be safe
        sys.modules['Basilisk.utilities'].orbitalMotion.elem2rv.return_value = (np.zeros(3), np.zeros(3))
        
        
        # Mock BSK_masters with actual classes for inheritance
        class MockBSKSim:
            def __init__(self, fswRate=0.1, dynRate=0.1): 
                self.fswRate = fswRate
                self.dynRate = dynRate
            def set_DynModel(self, model): pass
            def set_FswModel(self, model): pass
            def AddModelToTask(self, *args): pass
            def get_DynModel(self): 
                # Return a mock dynamic model with necessary attributes
                mock_dyn = MagicMock()
                mock_dyn.taskName = "DynTask"
                mock_dyn.scObject = MagicMock()
                mock_dyn.scObject.scStateOutMsg = MagicMock()
                mock_dyn.scObject.scStateOutMsg.read.return_value = MagicMock()
                # Ensure other attributes accessed in code are present
                return mock_dyn
            def get_FswModel(self):
                mock_fsw = MagicMock()
                mock_fsw.processTasksTimeStep = 0.1
                return mock_fsw
            def ConfigureStopTime(self, time): pass
            def ExecuteSimulation(self): pass

        class MockBSKScenario:
            def __init__(self): pass
            
        sys.modules['BSK_masters'] = MagicMock()
        sys.modules['BSK_masters'].BSKSim = MockBSKSim
        sys.modules['BSK_masters'].BSKScenario = MockBSKScenario
        
        # Mock BSK_Plotting
        sys.modules['BSK_Plotting'] = MagicMock()
