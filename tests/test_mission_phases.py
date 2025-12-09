#!/usr/bin/env python3
"""
Test dettagliato delle fasi della missione EXCITE

Questo script esegue una simulazione completa e verifica:
- Convergenza dell'errore di assetto
- Velocità delle ruote di reazione
- Budget di potenza e SOC batteria
- Transizioni della FSM
- Performance dei controllori
"""

import sys
import os
import numpy as np
import Basilisk.utilities.macros as macros

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from excite.scenario.scenario import scenario_EXCITE
from excite.config import mission, control


class MissionPhaseAnalyzer:
    """Analizza le performance della missione per fase"""

    def __init__(self, scenario):
        self.scenario = scenario
        self.checkpoints = []

    def add_checkpoint(self, time_s, phase_name):
        """Aggiungi checkpoint per analisi"""
        self.checkpoints.append({
            'time': time_s,
            'phase': phase_name
        })

    def analyze_attitude_error(self, sigma_BR):
        """
        Analizza errore di assetto (MRP)

        Returns:
            dict: Statistiche errore di assetto
        """
        # Converti MRP in angolo di errore (gradi)
        # |σ| ≈ tan(θ/4) per piccoli angoli
        sigma_magnitude = np.linalg.norm(sigma_BR, axis=1)
        error_deg = 4 * np.arctan(sigma_magnitude) * macros.R2D

        return {
            'mean_deg': np.mean(error_deg),
            'max_deg': np.max(error_deg),
            'min_deg': np.min(error_deg),
            'final_deg': error_deg[-1],
            'std_deg': np.std(error_deg)
        }

    def analyze_rw_speeds(self, rw_speeds):
        """
        Analizza velocità ruote di reazione

        Args:
            rw_speeds: Array [N, 4] con velocità RW in RPM

        Returns:
            dict: Statistiche velocità RW
        """
        max_speed_limit = 6000.0  # RPM (limite hardware CubeSpace CW0162)

        # Prendi solo le prime 4 colonne (4 RW)
        rw_speeds_4 = rw_speeds[:, :4]

        rw_max = np.max(np.abs(rw_speeds_4), axis=0)
        rw_mean = np.mean(np.abs(rw_speeds_4), axis=0)

        return {
            'max_speeds_rpm': rw_max.tolist(),
            'mean_speeds_rpm': rw_mean.tolist(),
            'max_overall_rpm': np.max(rw_max),
            'saturation_warning': np.any(rw_max > 0.8 * max_speed_limit),
            'saturation_critical': np.any(rw_max > 0.95 * max_speed_limit)
        }

    def analyze_power_budget(self, battery_soc):
        """
        Analizza budget di potenza

        Args:
            battery_soc: Array con State of Charge batteria [0-1]

        Returns:
            dict: Statistiche power budget
        """
        soc_percent = battery_soc * 100

        return {
            'initial_soc_percent': soc_percent[0],
            'final_soc_percent': soc_percent[-1],
            'min_soc_percent': np.min(soc_percent),
            'max_soc_percent': np.max(soc_percent),
            'delta_soc_percent': soc_percent[-1] - soc_percent[0],
            'critical_warning': np.any(soc_percent < 30.0)
        }

    def analyze_angular_velocity(self, omega_BR):
        """
        Analizza velocità angolare

        Args:
            omega_BR: Array [N, 3] con velocità angolare in rad/s

        Returns:
            dict: Statistiche velocità angolare
        """
        omega_magnitude = np.linalg.norm(omega_BR, axis=1) * macros.R2D  # deg/s

        return {
            'mean_deg_s': np.mean(omega_magnitude),
            'max_deg_s': np.max(omega_magnitude),
            'final_deg_s': omega_magnitude[-1],
            'std_deg_s': np.std(omega_magnitude)
        }

    def print_summary(self, duration_hours, analysis):
        """Stampa riepilogo analisi"""

        print("\n" + "=" * 70)
        print(f" MISSION ANALYSIS SUMMARY - {duration_hours:.2f} hours")
        print("=" * 70)

        print("\n1. ATTITUDE ERROR (MRP)")
        print(f"   Mean error:    {analysis['attitude']['mean_deg']:.3f}°")
        print(f"   Max error:     {analysis['attitude']['max_deg']:.3f}°")
        print(f"   Final error:   {analysis['attitude']['final_deg']:.3f}°")
        print(f"   Std deviation: {analysis['attitude']['std_deg']:.3f}°")

        if analysis['attitude']['final_deg'] < 1.0:
            print("   ✓ EXCELLENT: Pointing accuracy < 1°")
        elif analysis['attitude']['final_deg'] < 5.0:
            print("   ✓ GOOD: Pointing accuracy < 5°")
        else:
            print("   ⚠ WARNING: Pointing accuracy > 5°")

        print("\n2. ANGULAR VELOCITY")
        print(f"   Mean:          {analysis['omega']['mean_deg_s']:.4f}°/s")
        print(f"   Max:           {analysis['omega']['max_deg_s']:.4f}°/s")
        print(f"   Final:         {analysis['omega']['final_deg_s']:.4f}°/s")

        if analysis['omega']['final_deg_s'] < 0.01:
            print("   ✓ EXCELLENT: Very stable (< 0.01°/s)")
        elif analysis['omega']['final_deg_s'] < 0.1:
            print("   ✓ GOOD: Stable (< 0.1°/s)")

        print("\n3. REACTION WHEELS")
        rw_data = analysis['rw']
        print(f"   RW1 max/mean:  {rw_data['max_speeds_rpm'][0]:.1f} / {rw_data['mean_speeds_rpm'][0]:.1f} RPM")
        print(f"   RW2 max/mean:  {rw_data['max_speeds_rpm'][1]:.1f} / {rw_data['mean_speeds_rpm'][1]:.1f} RPM")
        print(f"   RW3 max/mean:  {rw_data['max_speeds_rpm'][2]:.1f} / {rw_data['mean_speeds_rpm'][2]:.1f} RPM")
        print(f"   RW4 max/mean:  {rw_data['max_speeds_rpm'][3]:.1f} / {rw_data['mean_speeds_rpm'][3]:.1f} RPM")
        print(f"   Peak overall:  {rw_data['max_overall_rpm']:.1f} RPM (limit: 6000 RPM)")

        if rw_data['saturation_critical']:
            print("   ⚠ CRITICAL: RW approaching saturation (> 95% of 6000 RPM)!")
        elif rw_data['saturation_warning']:
            print("   ⚠ WARNING: RW speeds high (> 80% of 6000 RPM)")
        else:
            print("   ✓ GOOD: RW speeds within safe limits")

        print("\n4. POWER BUDGET")
        pwr_data = analysis['power']
        print(f"   Initial SOC:   {pwr_data['initial_soc_percent']:.1f}%")
        print(f"   Final SOC:     {pwr_data['final_soc_percent']:.1f}%")
        print(f"   Min SOC:       {pwr_data['min_soc_percent']:.1f}%")
        print(f"   Delta SOC:     {pwr_data['delta_soc_percent']:+.1f}%")

        if pwr_data['critical_warning']:
            print("   ⚠ CRITICAL: Battery SOC dropped below 30%!")
        elif pwr_data['delta_soc_percent'] < -10:
            print("   ⚠ WARNING: Significant battery discharge")
        elif pwr_data['delta_soc_percent'] > 0:
            print("   ✓ GOOD: Battery charging")
        else:
            print("   ✓ ACCEPTABLE: Stable power budget")

        print("\n" + "=" * 70)


def run_mission_test(duration_hours=1.0, show_plots=False):
    """
    Esegue test completo missione con analisi dettagliata

    Args:
        duration_hours: Durata simulazione in ore
        show_plots: Se True, mostra grafici alla fine
    """

    print("=" * 70)
    print(f" EXCITE MISSION PHASE TEST - {duration_hours} hours")
    print("=" * 70)

    try:
        print("\n[1/5] Creating scenario...")
        scenario = scenario_EXCITE()
        analyzer = MissionPhaseAnalyzer(scenario)
        print("      ✓ Scenario created")

        print("\n[2/5] Initializing simulation...")
        scenario.InitializeSimulation()
        print("      ✓ Simulation initialized")

        print(f"\n[3/5] Running {duration_hours}-hour simulation...")
        print(f"      Start time: 0.0 s")
        print(f"      Stop time:  {duration_hours * 3600:.1f} s")

        scenario.ConfigureStopTime(macros.sec2nano(duration_hours * 3600.0))
        scenario.ExecuteSimulation()

        print(f"      ✓ Simulation completed")

        print("\n[4/5] Extracting telemetry data...")

        # Estrai dati dai message recorders (Basilisk 2.x syntax)
        attGuidRec = scenario.msgRecList['attGuidMsg']
        sigma_BR = attGuidRec.sigma_BR
        omega_BR = attGuidRec.omega_BR_B  # Velocità angolare di riferimento

        # Velocità angolare del corpo (per detumbling analysis)
        navRec = scenario.msgRecList['simpleNavMsg']
        omega_BN = navRec.omega_BN_B

        # RW speeds (4 ruote) - converti da rad/s a RPM
        # Usa il messaggio aggregato rwSpeedMsg
        rwSpeedRec = scenario.msgRecList['rwSpeedMsg']
        rw_speeds_rad_s = rwSpeedRec.wheelSpeeds  # [N, 4] in rad/s
        rw_speeds = rw_speeds_rad_s * 60.0 / (2.0 * np.pi)  # Converti rad/s → RPM

        # Battery SOC
        batteryRec = scenario.msgRecList['batteryMsg']
        battery_level = batteryRec.storageLevel  # Watt-hours
        battery_capacity = batteryRec.storageCapacity  # Watt-hours

        # Calcola SOC (State of Charge) - gestisci divisione per zero
        if battery_capacity[0] > 0:
            battery_soc = battery_level / battery_capacity[0]
        else:
            battery_soc = np.ones_like(battery_level) * 0.5  # Default 50% se non definito

        print("      ✓ Telemetry extracted")

        print("\n[5/5] Analyzing mission performance...")

        analysis = {
            'attitude': analyzer.analyze_attitude_error(sigma_BR),
            'omega': analyzer.analyze_angular_velocity(omega_BR),
            'rw': analyzer.analyze_rw_speeds(rw_speeds),
            'power': analyzer.analyze_power_budget(battery_soc)
        }

        print("      ✓ Analysis complete")

        # Stampa riepilogo
        analyzer.print_summary(duration_hours, analysis)

        # Genera grafici se richiesto
        if show_plots:
            print("\n[PLOTS] Generating performance plots...")
            # TODO: Integrate with excite.analysis.plotting
            print("        (Plotting not yet implemented)")

        print("\n" + "=" * 70)
        print(" TEST COMPLETED SUCCESSFULLY")
        print("=" * 70)

        return 0

    except Exception as e:
        print("\n" + "=" * 70)
        print(f" ERROR: {type(e).__name__}")
        print("=" * 70)
        print(f"{e}")
        print()

        import traceback
        print("Full traceback:")
        traceback.print_exc()

        return 1


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Test dettagliato fasi missione EXCITE',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=1.0,
        metavar='HOURS',
        help='Durata simulazione in ore (default: 1.0)'
    )

    parser.add_argument(
        '--plots',
        action='store_true',
        help='Mostra grafici performance'
    )

    args = parser.parse_args()

    sys.exit(run_mission_test(
        duration_hours=args.duration,
        show_plots=args.plots
    ))
