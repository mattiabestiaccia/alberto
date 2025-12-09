#!/usr/bin/env python3
"""
Test incrementale simulazione EXCITE
"""

import sys
import os
import Basilisk.utilities.macros as macros

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

print("=" * 70)
print("EXCITE Incremental Test - 1 HOUR")
print("=" * 70)

try:
    print("\n1. Importing scenario...")
    from excite.scenario.scenario import scenario_EXCITE
    print("   ✓ Import successful")

    print("\n2. Creating scenario...")
    scenario = scenario_EXCITE()
    print("   ✓ Scenario created")

    print("\n3. Initializing simulation...")
    scenario.InitializeSimulation()
    print("   ✓ Simulation initialized")

    print("\n4. Running 3600-second simulation (1 hour)...")
    print("   Start time: 0.0 s")
    scenario.ConfigureStopTime(macros.sec2nano(3600.0))
    scenario.ExecuteSimulation()
    print("   ✓ Simulation completed: 3600.0 s")

    print("\n" + "=" * 70)
    print("SUCCESS! 1-hour simulation completed")
    print("=" * 70)

except Exception as e:
    print("\n" + "=" * 70)
    print(f"ERROR: {type(e).__name__}")
    print("=" * 70)
    print(f"{e}")
    print()

    import traceback
    print("Full traceback:")
    traceback.print_exc()

    sys.exit(1)

sys.exit(0)
