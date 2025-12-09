#!/usr/bin/env python3
"""
EXCITE Tracked Simulation Runner

Esegue simulazione con logging automatico completo in executions/
"""

import sys
import os
import argparse
import numpy as np
import Basilisk.utilities.macros as macros

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from excite.scenario.scenario import scenario_EXCITE
from excite.utils.execution_logger import ExecutionLogger, create_execution_index
from excite.config import mission, control, spacecraft, actuators


def run_tracked_simulation(duration_hours=1.0, description="test"):
    """
    Esegue simulazione con logging completo

    Args:
        duration_hours: Durata in ore
        description: Descrizione breve dell'esecuzione
    """

    # Crea logger
    logger = ExecutionLogger(description=description)

    console_output = []

    def log_print(msg):
        """Print e salva in console output"""
        print(msg)
        console_output.append(msg)

    try:
        log_print("=" * 70)
        log_print(f" EXCITE TRACKED SIMULATION - {duration_hours} hours")
        log_print("=" * 70)

        # Salva configurazione
        config = {
            'duration_hours': duration_hours,
            'spacecraft': {
                'mass_kg': spacecraft.TOTAL_MASS_KG,
                'inertia_kgm2': spacecraft.INERTIA_TENSOR_KG_M2.tolist(),
                'com_m': spacecraft.CENTER_OF_MASS_B.tolist()
            },
            'actuators': {
                'rw_count': 4,
                'rw_max_speed_rpm': actuators.RW_MAX_SPEED_RPM,
                'rw_max_torque_nm': actuators.RW_MAX_TORQUE_NM
            },
            'control': {
                'mrp_k1': control.MRP_STEERING_K1,
                'mrp_k3': control.MRP_STEERING_K3,
                'rate_servo_p': control.RATE_SERVO_P
            },
            'mission': {
                'orbital_altitude_km': 550,
                'inclination_deg': 97.4,
                'orbit_period_min': mission.ORBIT_PERIOD_MIN
            }
        }
        logger.log_config(config)

        log_print("\n[1/6] Creating scenario...")
        scenario = scenario_EXCITE()
        log_print("      ✓ Scenario created")

        log_print("\n[2/6] Initializing simulation...")
        scenario.InitializeSimulation()
        log_print("      ✓ Simulation initialized")

        log_print(f"\n[3/6] Running {duration_hours}-hour simulation with periodic plotting...")
        
        # Define milestones for plotting (in hours)
        # These correspond to expected end times of phases: 
        # Detumbling (<12h), SunSafe (14-17h), GS (17-19h), Payload A (19-20.5h), Payload B (20.5-22h), Imaging (22-24h)
        default_milestones = [1.0, 13.0, 15.0, 18.0, 20.0, 21.0, 23.0, 24.0]
        
        # Filter milestones that are within the requested duration
        milestones = [m for m in default_milestones if m < duration_hours]
        if duration_hours not in milestones:
            milestones.append(duration_hours)
            
        log_print(f"      Milestones: {milestones}")
        log_print(f"      Plots will be saved to: {logger.get_run_dir()}/plots/")

        current_time_hours = 0.0
        
        for milestone in milestones:
            log_print(f"\n      >>> Executing to {milestone} hours...")
            
            # Execute to milestone
            scenario.ConfigureStopTime(macros.sec2nano(milestone * 3600.0))
            scenario.ExecuteSimulation()
            
            current_time_hours = milestone
            log_print(f"          Reached {milestone} hours.")
            
            # Generate intermediate plots
            plot_prefix = f"plot_{milestone:.1f}h"
            plots_dir = os.path.join(logger.get_run_dir(), "plots")
            
            log_print(f"          Generating plots for {milestone}h...")
            try:
                scenario.pull_outputs(showPlots=False, savePrefix=plot_prefix, outputDir=plots_dir)
                log_print(f"          ✓ Plots saved")
            except Exception as plot_err:
                 log_print(f"          ⚠️ Error generating plots: {plot_err}")

        log_print("      ✓ Simulation completed")

        log_print("\n[4/6] Extracting telemetry...")

        # Estrai dati telemetrici
        attGuidRec = scenario.msgRecList['attGuidMsg']
        navRec = scenario.msgRecList['simpleNavMsg']
        rwSpeedRec = scenario.msgRecList['rwSpeedMsg']
        
        batteryRec = scenario.msgRecList.get('batteryMsg')
        scStateRec = scenario.msgRecList.get('scStateMsg')

        telemetry = {
            'time': attGuidRec.times() * macros.NANO2HOUR,  # ore
            'sigma_BR': attGuidRec.sigma_BR,
            'omega_BR_B': attGuidRec.omega_BR_B,
            'omega_BN_B': navRec.omega_BN_B,
            'rw_speeds_rad_s': rwSpeedRec.wheelSpeeds,
            'battery_level_wh': batteryRec.storageLevel if batteryRec else np.zeros(len(attGuidRec.times())),
            'battery_capacity_wh': batteryRec.storageCapacity if batteryRec else np.ones(len(attGuidRec.times())),
            'sc_position_m': scStateRec.r_BN_N if scStateRec else np.zeros((len(attGuidRec.times()), 3)),
            'sc_velocity_m_s': scStateRec.v_BN_N if scStateRec else np.zeros((len(attGuidRec.times()), 3))
        }

        # Salva telemetria (NumPy e CSV)
        logger.log_telemetry(telemetry)
        logger.log_telemetry_csv(telemetry)

        log_print("      ✓ Telemetry extracted and saved")

        log_print("\n[5/6] Calculating metrics...")

        # Calcola metriche
        sigma_magnitude = np.linalg.norm(telemetry['sigma_BR'], axis=1)
        error_deg = 4 * np.arctan(sigma_magnitude) * macros.R2D

        omega_magnitude = np.linalg.norm(telemetry['omega_BR_B'], axis=1) * macros.R2D

        rw_speeds_rpm = telemetry['rw_speeds_rad_s'][:, :4] * 60.0 / (2.0 * np.pi)
        rw_max = np.max(np.abs(rw_speeds_rpm), axis=0)
        rw_mean = np.mean(np.abs(rw_speeds_rpm), axis=0)

        # SOC batteria
        if telemetry['battery_capacity_wh'][0] > 0:
            battery_soc = telemetry['battery_level_wh'] / telemetry['battery_capacity_wh'][0]
        else:
            battery_soc = np.ones_like(telemetry['battery_level_wh']) * 0.5

        metrics = {
            'attitude_error': {
                'mean_deg': float(np.mean(error_deg)),
                'max_deg': float(np.max(error_deg)),
                'final_deg': float(error_deg[-1]),
                'std_deg': float(np.std(error_deg))
            },
            'angular_velocity': {
                'mean_deg_s': float(np.mean(omega_magnitude)),
                'max_deg_s': float(np.max(omega_magnitude)),
                'final_deg_s': float(omega_magnitude[-1])
            },
            'reaction_wheels': {
                'max_speeds_rpm': rw_max.tolist(),
                'mean_speeds_rpm': rw_mean.tolist(),
                'peak_overall_rpm': float(np.max(rw_max)),
                'saturation_percent': float(np.max(rw_max) / 6000.0 * 100)
            },
            'power_budget': {
                'initial_soc_percent': float(battery_soc[0] * 100),
                'final_soc_percent': float(battery_soc[-1] * 100),
                'min_soc_percent': float(np.min(battery_soc) * 100),
                'delta_soc_percent': float((battery_soc[-1] - battery_soc[0]) * 100)
            }
        }

        logger.log_metrics(metrics)

        log_print("      ✓ Metrics calculated and saved")

        log_print("\n[6/6] Generating report...")

        # Genera report testuale
        report = generate_text_report(duration_hours, metrics, logger.run_id)
        logger.log_text_report(report, "REPORT.md")

        log_print("      ✓ Report generated")

        # Salva console output
        logger.log_console_output('\n'.join(console_output))

        # Finalizza
        logger.finalize(status='completed')

        log_print("\n" + "=" * 70)
        log_print(" SIMULATION COMPLETED SUCCESSFULLY")
        log_print("=" * 70)
        log_print(f"\nResults saved in: {logger.get_run_dir()}")
        log_print(f"Periodic plots in: {logger.get_run_dir()}/plots/")

        # Aggiorna indice
        create_execution_index()

        return 0

    except Exception as e:
        log_print("\n" + "=" * 70)
        log_print(f" ERROR: {type(e).__name__}")
        log_print("=" * 70)
        log_print(f"{e}")

        import traceback
        log_print("\nFull traceback:")
        log_print(traceback.format_exc())

        logger.log_console_output('\n'.join(console_output))
        logger.finalize(status='failed', error_msg=str(e))

        return 1


def generate_text_report(duration_hours, metrics, run_id):
    """Genera report testuale dettagliato"""

    att = metrics['attitude_error']
    omega = metrics['angular_velocity']
    rw = metrics['reaction_wheels']
    pwr = metrics['power_budget']

    # Valutazioni
    att_rating = "EXCELLENT" if att['final_deg'] < 1.0 else "GOOD" if att['final_deg'] < 5.0 else "WARNING"
    omega_rating = "EXCELLENT" if omega['final_deg_s'] < 0.01 else "GOOD" if omega['final_deg_s'] < 0.1 else "WARNING"
    rw_rating = "GOOD" if rw['saturation_percent'] < 80 else "WARNING" if rw['saturation_percent'] < 95 else "CRITICAL"

    report = f"""# EXCITE Simulation Report

**Run ID**: `{run_id}`
**Duration**: {duration_hours} hours
**Status**: ✅ COMPLETED

---

## Performance Summary

### 1. Attitude Control

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| Mean Error | {att['mean_deg']:.4f}° | < 1° | {'✅' if att['mean_deg'] < 1.0 else '⚠️'} |
| Max Error | {att['max_deg']:.4f}° | < 5° | {'✅' if att['max_deg'] < 5.0 else '⚠️'} |
| Final Error | {att['final_deg']:.4f}° | < 1° | {'✅' if att['final_deg'] < 1.0 else '⚠️'} |
| Std Deviation | {att['std_deg']:.4f}° | - | - |

**Rating**: {att_rating}

### 2. Angular Velocity Stability

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| Mean | {omega['mean_deg_s']:.4f}°/s | < 0.1°/s | {'✅' if omega['mean_deg_s'] < 0.1 else '⚠️'} |
| Max | {omega['max_deg_s']:.4f}°/s | < 0.5°/s | {'✅' if omega['max_deg_s'] < 0.5 else '⚠️'} |
| Final | {omega['final_deg_s']:.4f}°/s | < 0.01°/s | {'✅' if omega['final_deg_s'] < 0.01 else '⚠️'} |

**Rating**: {omega_rating}

### 3. Reaction Wheels

| RW | Max Speed (RPM) | Mean Speed (RPM) | Utilization |
|----|-----------------|------------------|-------------|
| RW1 | {rw['max_speeds_rpm'][0]:.1f} | {rw['mean_speeds_rpm'][0]:.1f} | {rw['max_speeds_rpm'][0]/6000*100:.2f}% |
| RW2 | {rw['max_speeds_rpm'][1]:.1f} | {rw['mean_speeds_rpm'][1]:.1f} | {rw['max_speeds_rpm'][1]/6000*100:.2f}% |
| RW3 | {rw['max_speeds_rpm'][2]:.1f} | {rw['mean_speeds_rpm'][2]:.1f} | {rw['max_speeds_rpm'][2]/6000*100:.2f}% |
| RW4 | {rw['max_speeds_rpm'][3]:.1f} | {rw['mean_speeds_rpm'][3]:.1f} | {rw['max_speeds_rpm'][3]/6000*100:.2f}% |

**Peak Overall**: {rw['peak_overall_rpm']:.1f} RPM ({rw['saturation_percent']:.2f}% of 6000 RPM limit)

**Rating**: {rw_rating}

### 4. Power Budget

| Metric | Value |
|--------|-------|
| Initial SOC | {pwr['initial_soc_percent']:.1f}% |
| Final SOC | {pwr['final_soc_percent']:.1f}% |
| Min SOC | {pwr['min_soc_percent']:.1f}% |
| Delta SOC | {pwr['delta_soc_percent']:+.1f}% |

---

## Files Generated

- `telemetry/*.npy` - Raw telemetry (NumPy format)
- `telemetry/*.csv` - Telemetry CSV (for external analysis)
- `metrics.json` - Performance metrics
- `config.json` - Simulation configuration
- `logs/console.log` - Full console output

---

## Recommendations

"""

    # Aggiungi raccomandazioni
    if att['final_deg'] > 5.0:
        report += "- ⚠️ **Attitude error high** - Review control gains\n"
    if rw['saturation_percent'] > 80:
        report += "- ⚠️ **RW approaching saturation** - Consider desaturation strategy\n"
    if pwr['min_soc_percent'] < 30:
        report += "- ⚠️ **Low battery SOC** - Review power budget\n"

    if att['final_deg'] < 1.0 and omega['final_deg_s'] < 0.01 and rw['saturation_percent'] < 50:
        report += "- ✅ **All systems nominal** - Ready for extended mission\n"

    report += f"""
---

**Generated**: {run_id}
"""

    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run EXCITE simulation with automatic logging',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=1.0,
        metavar='HOURS',
        help='Simulation duration in hours (default: 1.0)'
    )

    parser.add_argument(
        '--description',
        type=str,
        default='test',
        help='Short description for this run (default: test)'
    )

    args = parser.parse_args()

    sys.exit(run_tracked_simulation(
        duration_hours=args.duration,
        description=args.description
    ))
