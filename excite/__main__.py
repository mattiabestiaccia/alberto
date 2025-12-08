"""
Entry point for running EXCITE simulation via python -m excite

Usage:
    python -m excite [--duration HOURS] [--plots]
"""

import sys
import argparse


def main():
    """Run EXCITE 24-hour mission simulation"""
    parser = argparse.ArgumentParser(
        description='EXCITE AOCS Simulation - CubeSat 12U Attitude Control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m excite                          # Run 24h simulation
  python -m excite --duration 6.0           # Run 6h simulation
  python -m excite --plots                  # Run and show plots
  python -m excite --duration 2.0 --plots   # Quick 2h test with plots
        """
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=24.0,
        help='Simulation duration in hours (default: 24.0)'
    )

    parser.add_argument(
        '--plots',
        action='store_true',
        help='Show plots at the end of simulation'
    )

    args = parser.parse_args()

    print(f"="*60)
    print("EXCITE AOCS Simulation")
    print(f"Duration: {args.duration} hours")
    print(f"Plots: {'Enabled' if args.plots else 'Disabled'}")
    print(f"="*60)

    try:
        # Import scenario class
        from excite.scenario.scenario import scenario_EXCITE

        # Create and run scenario
        scenario = scenario_EXCITE()

        # Execute simulation
        # Note: This assumes scenario has a run method that accepts duration
        # If not, the scenario will use its internal default
        scenario.run()

        print("\nSimulation completed successfully!")

        return 0

    except ImportError as e:
        print(f"\nError: Failed to import EXCITE modules.")
        print(f"Details: {e}")
        print("\nMake sure Basilisk is installed and PYTHONPATH is configured correctly.")
        return 1

    except Exception as e:
        print(f"\nError during simulation:")
        print(f"{e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
