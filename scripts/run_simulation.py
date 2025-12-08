#!/usr/bin/env python3
"""
Main script per eseguire simulazione EXCITE

Usage:
    python scripts/run_simulation.py [--duration HOURS] [--plots]
"""

import sys
import os
import argparse

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)


def main():
    """Run EXCITE AOCS Simulation"""
    parser = argparse.ArgumentParser(
        description='Run EXCITE AOCS Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/run_simulation.py
  python scripts/run_simulation.py --duration 6.0
  python scripts/run_simulation.py --plots
  python scripts/run_simulation.py --duration 2.0 --plots

Mission Phases:
  1. Deployment (60s)
  2. Detumbling (up to 12h)
  3. Sun-Safe Pointing
  4. Initial Charge (1h)
  5. Payload Mode A (2h)
  6. Payload Mode B (1.5h)
  7. GS Contact (event-driven)
        """
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=24.0,
        metavar='HOURS',
        help='Simulation duration in hours (default: 24.0)'
    )

    parser.add_argument(
        '--plots',
        action='store_true',
        help='Show plots at the end of simulation'
    )

    parser.add_argument(
        '--no-plots',
        action='store_true',
        help='Disable plots (useful for batch runs)'
    )

    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose output'
    )

    args = parser.parse_args()

    # Determine plot setting
    show_plots = args.plots and not args.no_plots

    print("="*70)
    print(" EXCITE Satellite AOCS Simulation")
    print("="*70)
    print(f"  Duration:  {args.duration} hours")
    print(f"  Plots:     {'Enabled' if show_plots else 'Disabled'}")
    print(f"  Verbose:   {'Yes' if args.verbose else 'No'}")
    print("="*70)
    print()

    try:
        # Import scenario class
        from excite.scenario.scenario import scenario_EXCITE

        print("Initializing EXCITE scenario...")
        scenario = scenario_EXCITE()

        print(f"Starting simulation ({args.duration} hours)...")
        print()

        # Run simulation
        scenario.run()

        print()
        print("="*70)
        print(" Simulation completed successfully!")
        print("="*70)

        # Note: Plotting is handled by scenario.run() internally
        # based on scenario configuration

        return 0

    except ImportError as e:
        print()
        print("="*70)
        print(" ERROR: Failed to import required modules")
        print("="*70)
        print(f"Details: {e}")
        print()
        print("Troubleshooting:")
        print("  1. Ensure Basilisk is installed:")
        print("     pip install basilisk-framework")
        print()
        print("  2. Check PYTHONPATH includes project root:")
        print(f"     export PYTHONPATH={project_root}:$PYTHONPATH")
        print()
        print("  3. Verify all dependencies:")
        print("     pip install -r requirements.txt")
        print("="*70)
        return 1

    except Exception as e:
        print()
        print("="*70)
        print(" ERROR: Simulation failed")
        print("="*70)
        print(f"Exception: {type(e).__name__}")
        print(f"Message: {e}")
        print()

        if args.verbose:
            import traceback
            print("Full traceback:")
            traceback.print_exc()

        print("="*70)
        return 1


if __name__ == '__main__':
    sys.exit(main())
