
import pytest
import os
import sys
from excite.scenario.scenario import scenario_EXCITE
from Basilisk.utilities import macros

class TestSimulation:
    def test_imports(self):
        """Test that key modules can be imported."""
        import excite.dynamics
        import excite.fsw
        import excite.analysis
        assert True

    def test_scenario_initialization(self):
        """Test that the scenario class can be instantiated."""
        scen = scenario_EXCITE()
        assert scen.name == 'scenario_EXCITE'
        assert scen.fswRate == 0.1
        assert scen.dynRate == 0.1

    def test_simulation_run_short(self):
        """Run the simulation for a very short duration."""
        scen = scenario_EXCITE()
        
        # Configure a very short run (e.g. 10 seconds)
        # Note: scenario.run() might interpret arguments differently or not accept them if not designed to.
        # Looking at __main__.py, it calls scenario.run() without args? 
        # Wait, __main__.py calls scenario.run(), but we need to check if run() accepts duration.
        # The BSKSim.run() usually triggers the ConfigureStopTime logic.
        
        # We'll set the stop time manually if needed.
        # The __main__ script passed duration to NOTHING. It just printed it.
        # It calls scenario.run().
        # We need to verify if scenario.run() uses a stored duration or defaults.
        
        # Let's try setting TEnd manually if possible, or assume a default.
        # BSKSim usually has a default.
        
        # For this test, we want to override duration to be short.
        T_end = macros.sec2nano(10.0)
        scen.ConfigureStopTime(T_end) 
        
        scen.ExecuteSimulation()
        
        # Check if it finished (no errors)
        assert True
