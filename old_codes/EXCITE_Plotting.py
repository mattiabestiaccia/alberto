# This file contains plotting functions for EXCITE mission analysis

import numpy as np
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics


def plot_rw_angular_velocities(timeData, rwSpeeds, phaseMarkers, eclipseData, figNum=9,
                                add_gs_windows_func=None, add_sweep_markers_func=None):
    """
    Plot reaction wheel angular velocities for all 4 wheels.

    Args:
        timeData: Time array [hours]
        rwSpeeds: RW speeds [rad/s] (N x 4 array)
        phaseMarkers: Dictionary with phase times [hours]
        eclipseData: Eclipse periods (optional, can be None)
        figNum: Figure number
        add_gs_windows_func: Function to add GS access windows to plot (optional)
        add_sweep_markers_func: Function to add sweep start markers to plot (optional)
    """
    plt.figure(figNum, figsize=(14, 10))

    # Convert rad/s to RPM for better readability
    rwSpeeds_rpm = rwSpeeds * 60.0 / (2.0 * np.pi)

    wheelLabels = ['RW1', 'RW2', 'RW3', 'RW4']
    wheelColors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']  # Blue, Orange, Green, Red

    for i in range(4):
        ax = plt.subplot(2, 2, i+1)
        ax.plot(timeData, rwSpeeds_rpm[:, i], color=wheelColors[i], linewidth=1.5, label=f'{wheelLabels[i]} Speed')
        ax.set_xlabel('Time [hours]', fontsize=10)
        ax.set_ylabel('Angular Velocity [RPM]', fontsize=10)
        ax.set_title(f'{wheelLabels[i]} Angular Velocity', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add phase markers if provided
        if phaseMarkers:
            # Add vertical lines for phase transitions
            for phase_name, phase_time in phaseMarkers.items():
                if 'detumble_start' in phase_name:
                    ax.axvline(x=phase_time, color='green', linestyle='--', alpha=0.5, linewidth=1.0, label='Detumble Start')
                elif 'detumble_end' in phase_name:
                    ax.axvline(x=phase_time, color='orange', linestyle='--', alpha=0.5, linewidth=1.0, label='Detumble End')
                elif 'deployment' in phase_name:
                    ax.axvline(x=phase_time, color='red', linestyle='--', alpha=0.7, linewidth=1.5, label='Deployment')

        # Add GS access windows if function provided
        if add_gs_windows_func is not None:
            add_gs_windows_func(ax, add_vertical_lines=(i == 0))  # Lines only on first subplot

        # Add sweep start markers if function provided
        if add_sweep_markers_func is not None:
            add_sweep_markers_func(ax, add_markers=(i == 0))  # Markers only on first subplot

        ax.legend(loc='best', fontsize=8)

    plt.tight_layout()


def plot_orbital_elements(timeData, r_BN_N, v_BN_N, mu, maneuverPhases=None):
    """
    Plot orbital elements evolution (altitude, apogee/perigee, eccentricity).

    Args:
        timeData: Time array [hours]
        r_BN_N: Position vectors [m] (N x 3 array)
        v_BN_N: Velocity vectors [m/s] (N x 3 array)
        mu: Gravitational parameter [m^3/s^2]
        maneuverPhases: Dictionary with maneuver phase times [hours] (optional)
    """
    # Calculate orbital elements for each time step
    N = len(timeData)
    altitude_km = np.zeros(N)
    apogee_km = np.zeros(N)
    perigee_km = np.zeros(N)
    eccentricity = np.zeros(N)

    R_earth = 6378137.0  # [m] Earth radius

    for i in range(N):
        r_vec = r_BN_N[i, :]
        v_vec = v_BN_N[i, :]

        # Calculate altitude
        r_mag = np.linalg.norm(r_vec)
        altitude_km[i] = (r_mag - R_earth) / 1000.0

        # Calculate orbital elements
        oe = orbitalMotion.rv2elem(mu, r_vec, v_vec)
        # oe is a ClassicElements object with attributes: a, e, i, Omega, omega, f
        a = oe.a  # semi-major axis [m]
        e = oe.e  # eccentricity

        eccentricity[i] = e
        apogee_km[i] = (a * (1 + e) - R_earth) / 1000.0
        perigee_km[i] = (a * (1 - e) - R_earth) / 1000.0

    # Create figure with 3 vertical subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Subplot 1: Altitude
    ax1 = axes[0]
    ax1.plot(timeData, altitude_km, 'b-', linewidth=1.5, label='Altitude')
    ax1.set_ylabel('Altitude [km]', fontsize=11)
    ax1.set_title('Orbital Altitude Evolution', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)

    # Subplot 2: Apogee and Perigee
    ax2 = axes[1]
    ax2.plot(timeData, apogee_km, 'r-', linewidth=1.5, label='Apogee')
    ax2.plot(timeData, perigee_km, 'g-', linewidth=1.5, label='Perigee')
    ax2.set_ylabel('Altitude [km]', fontsize=11)
    ax2.set_title('Apogee and Perigee Evolution', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)

    # Subplot 3: Eccentricity
    ax3 = axes[2]
    ax3.plot(timeData, eccentricity, 'm-', linewidth=1.5, label='Eccentricity')
    ax3.set_xlabel('Time [hours]', fontsize=11)
    ax3.set_ylabel('Eccentricity [-]', fontsize=11)
    ax3.set_title('Orbital Eccentricity Evolution', fontsize=13, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=9)

    # Add maneuver phase markers if provided
    if maneuverPhases:
        firing_numbers = set()
        for key in maneuverPhases.keys():
            if key.startswith('firing_') and key.endswith('_start'):
                parts = key.split('_')
                if len(parts) == 3 and parts[1].isdigit():
                    firing_numbers.add(int(parts[1]))

        for ax in axes:
            for firing_num in sorted(firing_numbers):
                start_key = f'firing_{firing_num}_start'
                end_key = f'firing_{firing_num}_end'

                if start_key in maneuverPhases and end_key in maneuverPhases:
                    t_start = maneuverPhases[start_key]
                    t_end = maneuverPhases[end_key]

                    if firing_num == 1:
                        ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.5, linewidth=0.8, label='Firing Start')
                        ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.5, linewidth=0.8, label='Firing End')
                    else:
                        ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.5, linewidth=0.8)
                        ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.5, linewidth=0.8)

                    ax.axvspan(t_start, t_end, alpha=0.1, color='yellow')

    plt.tight_layout()


def plot_mtb_dipoles(timeData, mtbDipoleCmd, mtbDipoleActual, phaseMarkers, figNum=5):
    """
    Plot magnetorquer dipole commands and actual values (3 axes).

    Args:
        timeData: Time array [hours]
        mtbDipoleCmd: MTB dipole commands [A*m^2] (N x 3 array)
        mtbDipoleActual: MTB actual dipoles [A*m^2] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    # Command (solid lines)
    ax.plot(timeData, mtbDipoleCmd[:, 0], 'r-', linewidth=1.0, label='Cmd X')
    ax.plot(timeData, mtbDipoleCmd[:, 1], 'g-', linewidth=1.0, label='Cmd Y')
    ax.plot(timeData, mtbDipoleCmd[:, 2], 'b-', linewidth=1.0, label='Cmd Z')
    # Actual (dashed lines)
    ax.plot(timeData, mtbDipoleActual[:, 0], 'r--', linewidth=1.0, alpha=0.7, label='Actual X')
    ax.plot(timeData, mtbDipoleActual[:, 1], 'g--', linewidth=1.0, alpha=0.7, label='Actual Y')
    ax.plot(timeData, mtbDipoleActual[:, 2], 'b--', linewidth=1.0, alpha=0.7, label='Actual Z')

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Magnetic Dipole [A*m^2]', fontsize=11)
    ax.set_title('Magnetorquer Dipoles (Cmd vs Actual)', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9, ncol=2)

    # Add phase markers
    if phaseMarkers:
        for phase_name, phase_time in phaseMarkers.items():
            if 'detumble_start' in phase_name:
                ax.axvline(x=phase_time, color='green', linestyle='--', alpha=0.5, linewidth=1.0)
            elif 'detumble_end' in phase_name:
                ax.axvline(x=phase_time, color='orange', linestyle='--', alpha=0.5, linewidth=1.0)

    plt.tight_layout()


def plot_solar_panel_angles(timeData, panelAngles, phaseMarkers, figNum=6):
    """
    Plot solar panel angles in 2x2 grid (one subplot per panel).

    Args:
        timeData: Time array [hours]
        panelAngles: Panel angles [rad] - list of 4 arrays or (N x 4) array
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(14, 10))

    panelLabels = ['Panel 1', 'Panel 2', 'Panel 3', 'Panel 4']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']

    for i in range(4):
        ax = plt.subplot(2, 2, i+1)
        # Handle both list of arrays and 2D array
        if isinstance(panelAngles, list):
            angles_deg = np.rad2deg(panelAngles[i])
        else:
            angles_deg = np.rad2deg(panelAngles[:, i])
        ax.plot(timeData, angles_deg, color=colors[i], linewidth=1.5, label=panelLabels[i])

        # Add reference lines for stowed (-180°) and deployed (-90°) positions
        ax.axhline(-180, color='gray', linestyle=':', linewidth=1, alpha=0.5, label='Stowed')
        ax.axhline(-90, color='green', linestyle=':', linewidth=1, alpha=0.5, label='Deployed')

        ax.set_xlabel('Time [hours]', fontsize=10)
        ax.set_ylabel('Angle [deg]', fontsize=10)
        ax.set_title(f'{panelLabels[i]} Angle', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        ax.set_ylim(-185, -85)  # Focus on deployment range

    plt.tight_layout()


def plot_rw_torques(timeData, rwTorqueCmd, rwTorqueActual, phaseMarkers, figNum=8):
    """
    Plot reaction wheel torques (commanded vs actual) for all 4 wheels.

    Args:
        timeData: Time array [hours]
        rwTorqueCmd: Commanded RW torques [Nm] (N x 4 array)
        rwTorqueActual: Actual RW torques [Nm] (N x 4 array)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(14, 10))

    wheelLabels = ['RW1', 'RW2', 'RW3', 'RW4']

    for i in range(4):
        ax = plt.subplot(2, 2, i+1)
        ax.plot(timeData, rwTorqueCmd[:, i] * 1000, 'b-', linewidth=1.0, alpha=0.7, label='Commanded')
        ax.plot(timeData, rwTorqueActual[:, i] * 1000, 'r--', linewidth=1.0, alpha=0.7, label='Actual')
        ax.set_xlabel('Time [hours]', fontsize=10)
        ax.set_ylabel('Torque [mNm]', fontsize=10)
        ax.set_title(f'{wheelLabels[i]} Torque', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)

    plt.tight_layout()


def plot_attitude_tracking_errors(timeData, sigma_BR, phaseMarkers, eclipseData=None, settling_idx=None, figNum=2):
    """
    Plot attitude tracking errors (Roll, Pitch, Yaw angles).

    Args:
        timeData: Time array [hours]
        sigma_BR: MRP attitude error (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours]
        eclipseData: Dictionary with 'entry_times_hours' and 'exit_times_hours' lists (optional)
        settling_idx: Settling time index (optional, can be None)
        figNum: Figure number
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    # Convert MRP to Euler angles (roll, pitch, yaw) for each timestep
    roll = np.zeros(len(sigma_BR))
    pitch = np.zeros(len(sigma_BR))
    yaw = np.zeros(len(sigma_BR))

    for i in range(len(sigma_BR)):
        # Convert MRP to DCM
        dcm = rbk.MRP2C(sigma_BR[i, :])
        # Convert DCM to 321 Euler angles (yaw-pitch-roll)
        euler321 = rbk.C2Euler321(dcm)
        # Extract roll, pitch, yaw (in radians, then convert to degrees)
        yaw[i] = np.rad2deg(euler321[0])      # yaw (rotation about z-axis)
        pitch[i] = np.rad2deg(euler321[1])    # pitch (rotation about y-axis)
        roll[i] = np.rad2deg(euler321[2])     # roll (rotation about x-axis)

    # Unwrap angles to remove discontinuities at +/-180 deg
    roll = np.unwrap(roll, period=360)
    pitch = np.unwrap(pitch, period=360)
    yaw = np.unwrap(yaw, period=360)

    plt.figure(figNum, figsize=(14, 8))

    ax = plt.gca()

    # Add eclipse shading first (so it's behind the data)
    if eclipseData is not None:
        entry_times = eclipseData.get('entry_times_hours', [])
        exit_times = eclipseData.get('exit_times_hours', [])
        for i, (t_entry, t_exit) in enumerate(zip(entry_times, exit_times)):
            label = 'Eclipse' if i == 0 else ''
            ax.axvspan(t_entry, t_exit, alpha=0.2, color='gray', label=label, zorder=0)

    ax.plot(timeData, roll, 'r-', linewidth=1.0, label='Roll')
    ax.plot(timeData, pitch, 'g-', linewidth=1.0, label='Pitch')
    ax.plot(timeData, yaw, 'b-', linewidth=1.0, label='Yaw')

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angle [deg]', fontsize=11)
    ax.set_title('Attitude Tracking Errors (Roll, Pitch, Yaw)', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Add phase markers with labels
    _add_phase_markers_with_labels(ax, phaseMarkers)

    # Add settling time marker if provided
    if settling_idx is not None and settling_idx < len(timeData):
        settling_time = timeData[settling_idx]
        ax.axvline(x=settling_time, color='purple', linestyle=':', alpha=0.8, linewidth=2.0)
        ymin, ymax = ax.get_ylim()
        ax.text(settling_time, ymax * 0.95, 'Settled', rotation=90, verticalalignment='top',
                horizontalalignment='right', fontsize=7, color='purple', fontweight='bold', alpha=0.8)

    ax.legend(loc='best', fontsize=8)
    plt.tight_layout()

def plot_altitude_comparison(timeData, altitude_controlled, altitude_dummy, phaseMarkers, figNum=12):
    """
    Plot altitude comparison between controlled and dummy spacecraft (side by side).

    Args:
        timeData: Time array [hours]
        altitude_controlled: Altitude of controlled spacecraft [km]
        altitude_dummy: Altitude of dummy spacecraft (no control) [km]
        phaseMarkers: Dictionary with firing start/end times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(14, 6))

    # Subplot 1: Controlled spacecraft altitude
    ax1 = plt.subplot(1, 2, 1)
    ax1.plot(timeData, altitude_controlled, 'b-', linewidth=1.5, label='Controlled (with PPT)')
    ax1.set_xlabel('Time [hours]', fontsize=12)
    ax1.set_ylabel('Altitude [km]', fontsize=12)
    ax1.set_title('Controlled Spacecraft Altitude', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)

    # Add firing markers for controlled spacecraft
    if phaseMarkers:
        firing_numbers = set()
        for key in phaseMarkers.keys():
            if key.startswith('firing_') and key.endswith('_start'):
                parts = key.split('_')
                if len(parts) == 3 and parts[1].isdigit():
                    firing_numbers.add(int(parts[1]))

        for firing_num in sorted(firing_numbers):
            start_key = f'firing_{firing_num}_start'
            end_key = f'firing_{firing_num}_end'

            if start_key in phaseMarkers and end_key in phaseMarkers:
                t_start = phaseMarkers[start_key]
                t_end = phaseMarkers[end_key]

                if firing_num == 1:
                    ax1.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing Start')
                    ax1.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing End')
                else:
                    ax1.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0)
                    ax1.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0)

                ax1.axvspan(t_start, t_end, alpha=0.15, color='yellow')

    ax1.legend(loc='best', fontsize=9)

    # Subplot 2: Dummy spacecraft altitude
    ax2 = plt.subplot(1, 2, 2)
    ax2.plot(timeData, altitude_dummy, 'r-', linewidth=1.5, label='Dummy (no control)')
    ax2.set_xlabel('Time [hours]', fontsize=12)
    ax2.set_ylabel('Altitude [km]', fontsize=12)
    ax2.set_title('Dummy Spacecraft Altitude (No Control)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)

    plt.tight_layout()


def plot_altitude_difference(timeData, altitude_diff, phaseMarkers, figNum=13):
    """
    Plot altitude difference between controlled and dummy spacecraft.

    Args:
        timeData: Time array [hours]
        altitude_diff: Altitude difference (controlled - dummy) [km]
        phaseMarkers: Dictionary with firing start/end times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    ax.plot(timeData, altitude_diff, 'g-', linewidth=2, label='Altitude Difference (Controlled - Dummy)')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
    ax.set_xlabel('Time [hours]', fontsize=12)
    ax.set_ylabel('Altitude Difference [km]', fontsize=12)
    ax.set_title('Altitude Difference: Effect of PPT Maneuvers', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Add firing markers
    if phaseMarkers:
        firing_numbers = set()
        for key in phaseMarkers.keys():
            if key.startswith('firing_') and key.endswith('_start'):
                parts = key.split('_')
                if len(parts) == 3 and parts[1].isdigit():
                    firing_numbers.add(int(parts[1]))

        for firing_num in sorted(firing_numbers):
            start_key = f'firing_{firing_num}_start'
            end_key = f'firing_{firing_num}_end'

            if start_key in phaseMarkers and end_key in phaseMarkers:
                t_start = phaseMarkers[start_key]
                t_end = phaseMarkers[end_key]

                if firing_num == 1:
                    ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing Start')
                    ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing End')
                else:
                    ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0)
                    ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0)

                ax.axvspan(t_start, t_end, alpha=0.15, color='yellow')

    ax.legend(loc='best', fontsize=10)
    plt.tight_layout()


# ============================================================================
# RECOVERED FUNCTIONS (from user's paste - lines 755-1592 of original file)
# ============================================================================


def plot_thruster_force(timeData, thrustForces, phaseMarkers, figNum=20):
    """
    Plot thruster forces for PPT thrusters.

    Args:
        timeData: Time array [hours]
        thrustForces: Thrust forces [N] (N x 4 array for 4 thrusters)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 7))

    ax = plt.gca()
    thrusterLabels = ['PPT1', 'PPT2', 'PPT3', 'PPT4']
    colors = ['r', 'g', 'b', 'm']

    for i in range(thrustForces.shape[1]):
        ax.plot(timeData, thrustForces[:, i] * 1e6, color=colors[i],
                linewidth=1.0, label=thrusterLabels[i])

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Thrust Force [uN]', fontsize=11)
    ax.set_title('PPT Thrust Forces', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    # Add phase markers
    if phaseMarkers:
        for phase_name, phase_time in phaseMarkers.items():
            if 'firing_start' in phase_name:
                ax.axvline(x=phase_time, color='orange', linestyle='--', alpha=0.5, linewidth=1.0)
            elif 'firing_end' in phase_name:
                ax.axvline(x=phase_time, color='red', linestyle='--', alpha=0.5, linewidth=1.0)

    plt.tight_layout()


def add_maneuver_phase_markers(ax, phaseMarkers):
    """
    Helper function to add maneuver phase markers to a plot axis.

    Args:
        ax: Matplotlib axis object
        phaseMarkers: Dictionary with phase times [hours]
    """
    if phaseMarkers:
        for phase_name, phase_time in phaseMarkers.items():
            if 'detumble_start' in phase_name:
                ax.axvline(x=phase_time, color='green', linestyle='--', alpha=0.5, linewidth=1.0, label='Detumble Start')
            elif 'detumble_end' in phase_name:
                ax.axvline(x=phase_time, color='orange', linestyle='--', alpha=0.5, linewidth=1.0, label='Detumble End')
            elif 'deployment' in phase_name:
                ax.axvline(x=phase_time, color='red', linestyle='--', alpha=0.7, linewidth=1.5, label='Deployment')


def plot_maneuver_attitude_errors(timeData, sigma_BR, omega_BR_B, phaseMarkers, figNum=21):
    """
    Plot attitude errors during maneuvers.

    Args:
        timeData: Time array [hours]
        sigma_BR: MRP attitude error (N x 3 array)
        omega_BR_B: Angular velocity error [rad/s] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Subplot 1: MRP errors
    ax1 = axes[0]
    ax1.plot(timeData, sigma_BR[:, 0], 'r-', linewidth=1.0, label='sigma_1')
    ax1.plot(timeData, sigma_BR[:, 1], 'g-', linewidth=1.0, label='sigma_2')
    ax1.plot(timeData, sigma_BR[:, 2], 'b-', linewidth=1.0, label='sigma_3')
    ax1.set_ylabel('MRP [-]', fontsize=11)
    ax1.set_title('Attitude Error (MRP)', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)
    add_maneuver_phase_markers(ax1, phaseMarkers)

    # Subplot 2: Angular velocity errors
    ax2 = axes[1]
    ax2.plot(timeData, omega_BR_B[:, 0] * 180/np.pi, 'r-', linewidth=1.0, label='omega_x')
    ax2.plot(timeData, omega_BR_B[:, 1] * 180/np.pi, 'g-', linewidth=1.0, label='omega_y')
    ax2.plot(timeData, omega_BR_B[:, 2] * 180/np.pi, 'b-', linewidth=1.0, label='omega_z')
    ax2.set_xlabel('Time [hours]', fontsize=11)
    ax2.set_ylabel('Angular Velocity [deg/s]', fontsize=11)
    ax2.set_title('Angular Velocity Error', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)
    add_maneuver_phase_markers(ax2, phaseMarkers)

    plt.tight_layout()


def plot_detumbling_angular_velocity(timeData, omega_BN_B, phaseMarkers, figNum=22):
    """
    Plot angular velocity during detumbling phase.

    Args:
        timeData: Time array [hours]
        omega_BN_B: Angular velocity [rad/s] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()

    # Convert to deg/s
    omega_deg = omega_BN_B * 180.0 / np.pi

    ax.plot(timeData, omega_deg[:, 0], 'r-', linewidth=1.0, label='omega_x')
    ax.plot(timeData, omega_deg[:, 1], 'g-', linewidth=1.0, label='omega_y')
    ax.plot(timeData, omega_deg[:, 2], 'b-', linewidth=1.0, label='omega_z')

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angular Velocity [deg/s]', fontsize=11)
    ax.set_title('Angular Velocity During Detumbling', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    add_maneuver_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_deployment_impact_zoom(timeData, omega_BN_B, deployment_time, figNum=23):
    """
    Plot zoomed view of angular velocity around deployment event.

    Args:
        timeData: Time array [hours]
        omega_BN_B: Angular velocity [rad/s] (N x 3 array)
        deployment_time: Deployment time [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()

    # Convert to deg/s
    omega_deg = omega_BN_B * 180.0 / np.pi

    ax.plot(timeData, omega_deg[:, 0], 'r-', linewidth=1.5, label='omega_x')
    ax.plot(timeData, omega_deg[:, 1], 'g-', linewidth=1.5, label='omega_y')
    ax.plot(timeData, omega_deg[:, 2], 'b-', linewidth=1.5, label='omega_z')

    # Add deployment marker
    ax.axvline(x=deployment_time, color='red', linestyle='--', alpha=0.7, linewidth=2.0, label='Deployment')

    # Zoom around deployment
    time_window = 0.5  # hours
    ax.set_xlim([deployment_time - time_window, deployment_time + time_window])

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angular Velocity [deg/s]', fontsize=11)
    ax.set_title('Angular Velocity Around Deployment Event', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    plt.tight_layout()


def add_ppt_phase_markers(ax, phaseMarkers):
    """
    Helper function to add PPT firing phase markers to a plot axis.
    Shows all firing events with vertical lines and shaded regions.

    Args:
        ax: Matplotlib axis object
        phaseMarkers: Dictionary with firing start/end times [hours]
    """
    if phaseMarkers:
        firing_numbers = set()
        for key in phaseMarkers.keys():
            if key.startswith('firing_') and key.endswith('_start'):
                parts = key.split('_')
                if len(parts) == 3 and parts[1].isdigit():
                    firing_numbers.add(int(parts[1]))

        for firing_num in sorted(firing_numbers):
            start_key = f'firing_{firing_num}_start'
            end_key = f'firing_{firing_num}_end'

            if start_key in phaseMarkers and end_key in phaseMarkers:
                t_start = phaseMarkers[start_key]
                t_end = phaseMarkers[end_key]

                # Only add labels for first firing to avoid legend clutter
                if firing_num == 1:
                    ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing Start')
                    ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0, label='Firing End')
                else:
                    ax.axvline(x=t_start, color='orange', linestyle='--', alpha=0.7, linewidth=1.0)
                    ax.axvline(x=t_end, color='red', linestyle='--', alpha=0.7, linewidth=1.0)

                # Add shaded region for firing duration
                ax.axvspan(t_start, t_end, alpha=0.15, color='yellow')


def plot_ppt_individual_forces(timeData, pptForces, phaseMarkers, figNum=24):
    """
    Plot individual PPT thruster forces.

    Args:
        timeData: Time array [hours]
        pptForces: PPT forces [N] (N x 4 array)
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(14, 10))

    thrusterLabels = ['PPT1', 'PPT2', 'PPT3', 'PPT4']

    for i in range(4):
        ax = plt.subplot(2, 2, i+1)
        ax.plot(timeData, pptForces[:, i] * 1e6, 'b-', linewidth=1.0, label=thrusterLabels[i])
        ax.set_xlabel('Time [hours]', fontsize=10)
        ax.set_ylabel('Thrust [uN]', fontsize=10)
        ax.set_title(f'{thrusterLabels[i]} Thrust', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)

        add_ppt_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_ppt_torque_z(timeData, torque_z, phaseMarkers, figNum=25):
    """
    Plot Z-axis torque during PPT firings.

    Args:
        timeData: Time array [hours]
        torque_z: Z-axis torque [Nm] (N array)
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    ax.plot(timeData, torque_z * 1000, 'b-', linewidth=1.5, label='Torque Z')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1.0)

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Torque Z [mNm]', fontsize=11)
    ax.set_title('Z-Axis Torque During PPT Firings', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    add_ppt_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_ppt_omega_z(timeData, omega_z, phaseMarkers, figNum=26):
    """
    Plot Z-axis angular velocity during PPT firings.

    Args:
        timeData: Time array [hours]
        omega_z: Z-axis angular velocity [rad/s] (N array)
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    omega_z_deg = omega_z * 180.0 / np.pi
    ax.plot(timeData, omega_z_deg, 'b-', linewidth=1.5, label='Omega Z')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1.0)

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angular Velocity Z [deg/s]', fontsize=11)
    ax.set_title('Z-Axis Angular Velocity During PPT Firings', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    add_ppt_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_ppt_performance(timeData, dvData, altitudeData, phaseMarkers, figNum=27):
    """
    Plot PPT performance metrics (delta-V and altitude change).

    Args:
        timeData: Time array [hours]
        dvData: Delta-V data [m/s] (dictionary with components)
        altitudeData: Altitude data [km]
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Subplot 1: Delta-V accumulation
    ax1 = axes[0]
    if 'CN_N' in dvData:
        ax1.plot(timeData, dvData['CN_N'], 'b-', linewidth=1.5, label='Delta-V (inertial)')
    ax1.set_ylabel('Delta-V [m/s]', fontsize=11)
    ax1.set_title('Delta-V Accumulation', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)
    add_ppt_phase_markers(ax1, phaseMarkers)

    # Subplot 2: Altitude evolution
    ax2 = axes[1]
    ax2.plot(timeData, altitudeData, 'g-', linewidth=1.5, label='Altitude')
    ax2.set_xlabel('Time [hours]', fontsize=11)
    ax2.set_ylabel('Altitude [km]', fontsize=11)
    ax2.set_title('Altitude Evolution', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)
    add_ppt_phase_markers(ax2, phaseMarkers)

    plt.tight_layout()


def plot_ppt_omega_z_pre_end_detumbling(timeData, omega_z, detumble_end_time, figNum=28):
    """
    Plot Z-axis angular velocity before end of detumbling.

    Args:
        timeData: Time array [hours]
        omega_z: Z-axis angular velocity [rad/s]
        detumble_end_time: End time of detumbling [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    omega_z_deg = omega_z * 180.0 / np.pi
    ax.plot(timeData, omega_z_deg, 'b-', linewidth=1.5, label='Omega Z')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1.0)
    ax.axvline(x=detumble_end_time, color='orange', linestyle='--', alpha=0.7, linewidth=2.0, label='Detumble End')

    # Zoom to pre-detumble end period
    time_window = 1.0  # hours
    ax.set_xlim([detumble_end_time - time_window, detumble_end_time + 0.1])

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angular Velocity Z [deg/s]', fontsize=11)
    ax.set_title('Z-Axis Angular Velocity Before End of Detumbling', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    plt.tight_layout()


def plot_all_angular_rates_full_mission(timeData, omega_BN_B, phaseMarkers, figNum=29):
    """
    Plot all three angular rates for the full mission duration.

    Args:
        timeData: Time array [hours]
        omega_BN_B: Angular velocity [rad/s] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours]
        figNum: Figure number
    """
    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()

    # Convert to deg/s
    omega_deg = omega_BN_B * 180.0 / np.pi

    ax.plot(timeData, omega_deg[:, 0], 'r-', linewidth=1.0, label='omega_x')
    ax.plot(timeData, omega_deg[:, 1], 'g-', linewidth=1.0, label='omega_y')
    ax.plot(timeData, omega_deg[:, 2], 'b-', linewidth=1.0, label='omega_z')

    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Angular Velocity [deg/s]', fontsize=11)
    ax.set_title('Angular Velocity - Full Mission', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    add_maneuver_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_delta_v_accumulators(timeData, dvAccum_CN_B, dvAccum_BN_B, dvAccum_CN_N, phaseMarkers, figNum=10):
    """
    Plot delta-v accumulators in different frames (3 components on same plot with ALL firing markers).

    Args:
        timeData: Time array [hours]
        dvAccum_CN_B: Delta-v accumulator of center of mass in body frame coordinates [m/s] (N x 3 array)
        dvAccum_BN_B: Delta-v accumulator of body frame origin in body frame coordinates [m/s] (N x 3 array)
        dvAccum_CN_N: Delta-v accumulator of center of mass in inertial frame coordinates [m/s] (N x 3 array)
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Subplot 1: dvAccum_CN_B (Center of mass in body frame)
    ax1 = axes[0]
    ax1.plot(timeData, dvAccum_CN_B[:, 0], 'r-', linewidth=1.0, label='dV_x (body)')
    ax1.plot(timeData, dvAccum_CN_B[:, 1], 'g-', linewidth=1.0, label='dV_y (body)')
    ax1.plot(timeData, dvAccum_CN_B[:, 2], 'b-', linewidth=1.0, label='dV_z (body)')
    ax1.set_ylabel('Delta-V [m/s]', fontsize=10)
    ax1.set_title('Delta-V Accumulator: Center of Mass in Body Frame', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=8)

    # Subplot 2: dvAccum_BN_B (Body frame origin in body frame)
    ax2 = axes[1]
    ax2.plot(timeData, dvAccum_BN_B[:, 0], 'r-', linewidth=1.0, label='dV_x (body)')
    ax2.plot(timeData, dvAccum_BN_B[:, 1], 'g-', linewidth=1.0, label='dV_y (body)')
    ax2.plot(timeData, dvAccum_BN_B[:, 2], 'b-', linewidth=1.0, label='dV_z (body)')
    ax2.set_ylabel('Delta-V [m/s]', fontsize=10)
    ax2.set_title('Delta-V Accumulator: Body Frame Origin in Body Frame', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    # Subplot 3: dvAccum_CN_N (Center of mass in inertial frame)
    ax3 = axes[2]
    ax3.plot(timeData, dvAccum_CN_N[:, 0], 'r-', linewidth=1.0, label='dV_x (inertial)')
    ax3.plot(timeData, dvAccum_CN_N[:, 1], 'g-', linewidth=1.0, label='dV_y (inertial)')
    ax3.plot(timeData, dvAccum_CN_N[:, 2], 'b-', linewidth=1.0, label='dV_z (inertial)')
    ax3.set_xlabel('Time [hours]', fontsize=10)
    ax3.set_ylabel('Delta-V [m/s]', fontsize=10)
    ax3.set_title('Delta-V Accumulator: Center of Mass in Inertial Frame', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=8)

    # Add ALL firing phase markers to all subplots (using helper function)
    for ax in axes:
        add_ppt_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_semi_major_axis(timeData, r_BN_N, v_BN_N, mu, phaseMarkers, figNum=30):
    """
    Plot semi-major axis evolution over time.

    Args:
        timeData: Time array [hours]
        r_BN_N: Position vectors [m] (N x 3 array)
        v_BN_N: Velocity vectors [m/s] (N x 3 array)
        mu: Gravitational parameter [m^3/s^2]
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    # Calculate semi-major axis for each time step
    N = len(timeData)
    sma_km = np.zeros(N)

    for i in range(N):
        r_vec = r_BN_N[i, :]
        v_vec = v_BN_N[i, :]

        # Calculate orbital elements
        oe = orbitalMotion.rv2elem(mu, r_vec, v_vec)
        a = oe.a  # semi-major axis [m]
        sma_km[i] = a / 1000.0

    plt.figure(figNum, figsize=(12, 6))

    ax = plt.gca()
    ax.plot(timeData, sma_km, 'b-', linewidth=1.5, label='Semi-Major Axis')
    ax.set_xlabel('Time [hours]', fontsize=11)
    ax.set_ylabel('Semi-Major Axis [km]', fontsize=11)
    ax.set_title('Semi-Major Axis Evolution', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    add_ppt_phase_markers(ax, phaseMarkers)

    plt.tight_layout()


def plot_drift_evaluation(timeData, altitude_controlled, altitude_dummy, phaseMarkers, figNum=31):
    """
    Plot drift evaluation: compare controlled vs uncontrolled spacecraft altitude.

    Args:
        timeData: Time array [hours]
        altitude_controlled: Altitude with PPT control [km]
        altitude_dummy: Altitude without control [km]
        phaseMarkers: Dictionary with firing phase times [hours]
        figNum: Figure number
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Subplot 1: Both altitudes
    ax1 = axes[0]
    ax1.plot(timeData, altitude_controlled, 'b-', linewidth=1.5, label='Controlled (with PPT)')
    ax1.plot(timeData, altitude_dummy, 'r-', linewidth=1.5, alpha=0.7, label='Uncontrolled (no PPT)')
    ax1.set_ylabel('Altitude [km]', fontsize=11)
    ax1.set_title('Altitude Comparison: Controlled vs Uncontrolled', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=9)
    add_ppt_phase_markers(ax1, phaseMarkers)

    # Subplot 2: Altitude difference
    ax2 = axes[1]
    altitude_diff = altitude_controlled - altitude_dummy
    ax2.plot(timeData, altitude_diff, 'g-', linewidth=1.5, label='Altitude Difference')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1.0)
    ax2.set_xlabel('Time [hours]', fontsize=11)
    ax2.set_ylabel('Altitude Difference [km]', fontsize=11)
    ax2.set_title('Altitude Difference (Controlled - Uncontrolled)', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=9)
    add_ppt_phase_markers(ax2, phaseMarkers)

    plt.tight_layout()


def plot_sweep_verification(timeData, sigma_BR, sigma_BN, sigma_R0N, gsAccessWindows=None, sweepDelay_s=120.0, figNum=10):
    """
    Plot sweep verification: compare sigma_BR (error vs final reference with sweep)
    against sigma_BR0 (error vs base reference without sweep).

    If sweep is working correctly:
    - sigma_BR should be ~0 (spacecraft follows the sweeping reference)
    - sigma_BR0 should oscillate +/-30 deg (spacecraft deviates from base GS pointing)

    Args:
        timeData: Time array [hours]
        sigma_BR: MRP error vs final reference R (with sweep) [N x 3 array]
        sigma_BN: Body attitude MRP (vs inertial) [N x 3 array]
        sigma_R0N: Base reference MRP (GS pointing only, no sweep) [N x 3 array]
        gsAccessWindows: Dictionary with 'starts' and 'ends' lists [hours] (optional)
        sweepDelay_s: Delay before sweep starts after GS contact [seconds]
        figNum: Figure number
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    # Calculate sigma_BR0 = sigma_BN (-) sigma_R0N (MRP subtraction)
    sigma_BR0 = np.zeros_like(sigma_BR)
    for i in range(len(sigma_BN)):
        sigma_BR0[i, :] = rbk.subMRP(sigma_BN[i, :], sigma_R0N[i, :])

    # Convert MRP to principal rotation angle for clearer visualization
    def mrp_to_angle_deg(sigma_array):
        """Convert MRP array to principal rotation angles in degrees"""
        angles = np.zeros(len(sigma_array))
        for i in range(len(sigma_array)):
            sigma_norm = np.linalg.norm(sigma_array[i, :])
            angles[i] = 4.0 * np.arctan(sigma_norm) * 180.0 / np.pi
        return angles

    angle_BR = mrp_to_angle_deg(sigma_BR)    # Error w.r.t. final reference (with sweep)
    angle_BR0 = mrp_to_angle_deg(sigma_BR0)  # Error w.r.t. base reference (GS pointing only)

    # Convert sigma to Euler angles for pitch comparison
    pitch_BR = np.zeros(len(sigma_BR))
    pitch_BR0 = np.zeros(len(sigma_BR0))
    for i in range(len(sigma_BR)):
        # sigma_BR -> Euler 321 -> pitch
        if np.linalg.norm(sigma_BR[i, :]) > 1e-10:
            dcm_BR = rbk.MRP2C(sigma_BR[i, :])
            euler_BR = rbk.C2Euler321(dcm_BR)
            pitch_BR[i] = euler_BR[1] * 180.0 / np.pi  # pitch in degrees
        # sigma_BR0 -> Euler 321 -> pitch
        if np.linalg.norm(sigma_BR0[i, :]) > 1e-10:
            dcm_BR0 = rbk.MRP2C(sigma_BR0[i, :])
            euler_BR0 = rbk.C2Euler321(dcm_BR0)
            pitch_BR0[i] = euler_BR0[1] * 180.0 / np.pi  # pitch in degrees

    # Helper function to add GS access windows
    def add_gs_windows(ax):
        if gsAccessWindows and 'starts' in gsAccessWindows and 'ends' in gsAccessWindows:
            for i, (start_h, end_h) in enumerate(zip(gsAccessWindows['starts'], gsAccessWindows['ends'])):
                label_shade = 'GS Access' if i == 0 else ''
                ax.axvspan(start_h, end_h, alpha=0.15, color='green', label=label_shade, zorder=0)
                if i == 0:
                    ax.axvline(x=start_h, color='green', linestyle='--', linewidth=1.5, alpha=0.6, label='GS Start')
                    ax.axvline(x=end_h, color='red', linestyle='--', linewidth=1.5, alpha=0.6, label='GS End')
                else:
                    ax.axvline(x=start_h, color='green', linestyle='--', linewidth=1.5, alpha=0.6)
                    ax.axvline(x=end_h, color='red', linestyle='--', linewidth=1.5, alpha=0.6)

    # Helper function to add sweep start markers
    def add_sweep_markers(ax):
        if gsAccessWindows and 'starts' in gsAccessWindows:
            sweep_delay_h = sweepDelay_s / 3600.0
            for i, start_h in enumerate(gsAccessWindows['starts']):
                sweep_start_h = start_h + sweep_delay_h
                label = 'Sweep Start' if i == 0 else ''
                ax.axvline(x=sweep_start_h, color='purple', linestyle=':', linewidth=2.0, alpha=0.8, label=label)

    # Create figure with 2 subplots
    plt.figure(figNum, figsize=(14, 8))

    # Subplot 1: Principal rotation angles
    plt.subplot(2, 1, 1)
    plt.plot(timeData, angle_BR, 'b-', linewidth=1.5, label=r'$|\sigma_{BR}|$ (vs R with sweep)')
    plt.plot(timeData, angle_BR0, 'r-', linewidth=1.5, label=r'$|\sigma_{BR_0}|$ (vs R0 base pointing)')
    plt.axhline(y=30, color='g', linestyle='--', alpha=0.5, label='Sweep amplitude (30 deg)')
    plt.ylabel('Principal Rotation Angle [deg]', fontsize=11)
    plt.title('Sweep Verification: Tracking Error vs Base Pointing Error', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    ax = plt.gca()
    add_gs_windows(ax)
    add_sweep_markers(ax)
    plt.legend(loc='best', fontsize=10)
    plt.ylim([0, 50])

    # Subplot 2: Pitch component comparison
    plt.subplot(2, 1, 2)
    plt.plot(timeData, pitch_BR, 'b-', linewidth=1.5, label=r'Pitch $\sigma_{BR}$ (vs R with sweep)')
    plt.plot(timeData, pitch_BR0, 'r-', linewidth=1.5, label=r'Pitch $\sigma_{BR_0}$ (vs R0 base pointing)')
    plt.axhline(y=30, color='g', linestyle='--', alpha=0.5, label='+30 deg')
    plt.axhline(y=-30, color='g', linestyle='--', alpha=0.5, label='-30 deg')
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.ylabel('Pitch Error [deg]', fontsize=11)
    plt.xlabel('Time [hours]', fontsize=11)
    plt.grid(True, alpha=0.3)
    ax = plt.gca()
    add_gs_windows(ax)
    add_sweep_markers(ax)
    plt.legend(loc='best', fontsize=10)
    plt.ylim([-45, 45])

    plt.tight_layout()


def plot_antenna_sweep_verification(timeData, sigma_RN, sigma_R0N, sigma_BN,
                                     add_gs_windows_func, add_sweep_markers_func,
                                     figNum=10):
    """
    Plot antenna sweep verification with three subplots showing pitch angles.

    This function visualizes the antenna elevation sweep by comparing:
    1. Base GS pointing direction (before sweep)
    2. Final reference with sweep applied
    3. Actual spacecraft body attitude

    Args:
        timeData: Time array [hours]
        sigma_RN: MRP reference attitude with sweep (N x 3 array)
        sigma_R0N: MRP base GS pointing attitude without sweep (N x 3 array)
        sigma_BN: MRP body attitude (N x 3 array)
        add_gs_windows_func: Function to add GS access windows to plot
        add_sweep_markers_func: Function to add sweep start markers to plot
        figNum: Figure number (default: 10)
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    plt.figure(figNum, figsize=(14, 10))

    # Convert MRPs to Euler 321 pitch angles
    pitch_ref = np.zeros(len(sigma_RN))
    pitch_base = np.zeros(len(sigma_R0N))
    pitch_body = np.zeros(len(sigma_BN))

    for i in range(len(sigma_RN)):
        # Final reference pitch (with sweep)
        dcm_ref = rbk.MRP2C(sigma_RN[i, :])
        euler_ref = rbk.C2Euler321(dcm_ref)
        pitch_ref[i] = np.rad2deg(euler_ref[1])

        # Base GS pointing pitch (no sweep)
        dcm_base = rbk.MRP2C(sigma_R0N[i, :])
        euler_base = rbk.C2Euler321(dcm_base)
        pitch_base[i] = np.rad2deg(euler_base[1])

        # Body pitch
        dcm_body = rbk.MRP2C(sigma_BN[i, :])
        euler_body = rbk.C2Euler321(dcm_body)
        pitch_body[i] = np.rad2deg(euler_body[1])

    # Unwrap to avoid discontinuities at +/-180 deg
    pitch_ref = np.unwrap(pitch_ref, period=360)
    pitch_base = np.unwrap(pitch_base, period=360)
    pitch_body = np.unwrap(pitch_body, period=360)

    # Subplot 1: Inertial pitch angles comparison
    ax1 = plt.subplot(3, 1, 1)
    plt.plot(timeData, pitch_base, 'g-', linewidth=2.0, label=r'Base GS Pointing $\theta_{R_0}$')
    plt.plot(timeData, pitch_ref, 'b--', linewidth=1.5, label=r'Reference with Sweep $\theta_{R}$')
    plt.plot(timeData, pitch_body, 'r:', linewidth=1.0, label=r'Spacecraft Body $\theta_{B}$')
    plt.ylabel('Pitch Angle [deg]', fontsize=11)
    plt.title('Inertial Pitch Angle Comparison', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3)
    add_gs_windows_func(ax1, add_vertical_lines=True)
    add_sweep_markers_func(ax1, add_markers=True)
    plt.legend(loc='upper right', fontsize=9)

    # Subplot 2: Antenna elevation sweep angle
    ax2 = plt.subplot(3, 1, 2)
    sweep_angle = pitch_ref - pitch_base
    plt.plot(timeData, sweep_angle, 'm-', linewidth=1.5,
             label=r'Sweep Angle $\theta_{sweep} = \theta_{R} - \theta_{R_0}$')
    plt.axhline(y=30, color='k', linestyle='--', linewidth=0.8, alpha=0.6, label=r'Target: $\pm 30\degree$')
    plt.axhline(y=-30, color='k', linestyle='--', linewidth=0.8, alpha=0.6)
    plt.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.4)
    plt.ylabel('Sweep Angle [deg]', fontsize=11)
    plt.title('Antenna Elevation Sweep Angle', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3)
    add_gs_windows_func(ax2, add_vertical_lines=True)
    add_sweep_markers_func(ax2, add_markers=True)
    plt.legend(loc='upper right', fontsize=9)

    # Subplot 3: Attitude tracking error
    ax3 = plt.subplot(3, 1, 3)
    pitch_error = pitch_body - pitch_ref
    plt.plot(timeData, pitch_error, 'c-', linewidth=1.0,
             label=r'Tracking Error $\theta_{B} - \theta_{R}$')
    plt.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.6)
    plt.ylabel('Tracking Error [deg]', fontsize=11)
    plt.xlabel('Time [hours]', fontsize=11)
    plt.title('Pitch Attitude Tracking Error', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3)
    add_gs_windows_func(ax3, add_vertical_lines=True)
    add_sweep_markers_func(ax3, add_markers=True)
    plt.legend(loc='upper right', fontsize=9)

    plt.tight_layout()


def plot_attitude_estimation_error_smekf(timeData, euler_true, euler_smekf, phaseMarkers=None, figNum=13):
    """
    Plot attitude estimation error: Truth vs SMEKF estimated Euler angles.

    Shows the difference between true spacecraft attitude and SMEKF estimated attitude
    in terms of roll, pitch, and yaw angles.

    Args:
        timeData: Time array [hours]
        euler_true: True Euler angles [rad] (N x 3 array: roll, pitch, yaw)
        euler_smekf: SMEKF estimated Euler angles [rad] (N x 3 array: roll, pitch, yaw)
        phaseMarkers: Dictionary with phase times [hours] (optional)
        figNum: Figure number (default: 13)
    """
    plt.figure(figNum, figsize=(14, 10))

    # Convert to degrees for plotting
    euler_true_deg = np.rad2deg(euler_true)
    euler_smekf_deg = np.rad2deg(euler_smekf)

    # Calculate errors
    euler_error_deg = euler_true_deg - euler_smekf_deg

    # Unwrap to avoid discontinuities
    for i in range(3):
        euler_error_deg[:, i] = np.unwrap(euler_error_deg[:, i], period=360)

    labels = ['Roll', 'Pitch', 'Yaw']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green

    # Subplot 1: Roll error
    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(timeData, euler_error_deg[:, 0], color=colors[0], linewidth=1.0,
             label=f'{labels[0]} Error (Truth - SMEKF)')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_ylabel(f'{labels[0]} Error [deg]', fontsize=11)
    ax1.set_title('SMEKF Attitude Estimation Error: Roll', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax1, phaseMarkers)

    # Subplot 2: Pitch error
    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(timeData, euler_error_deg[:, 1], color=colors[1], linewidth=1.0,
             label=f'{labels[1]} Error (Truth - SMEKF)')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax2.set_ylabel(f'{labels[1]} Error [deg]', fontsize=11)
    ax2.set_title('SMEKF Attitude Estimation Error: Pitch', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax2, phaseMarkers)

    # Subplot 3: Yaw error
    ax3 = plt.subplot(3, 1, 3)
    ax3.plot(timeData, euler_error_deg[:, 2], color=colors[2], linewidth=1.0,
             label=f'{labels[2]} Error (Truth - SMEKF)')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax3.set_ylabel(f'{labels[2]} Error [deg]', fontsize=11)
    ax3.set_xlabel('Time [hours]', fontsize=11)
    ax3.set_title('SMEKF Attitude Estimation Error: Yaw', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax3, phaseMarkers)

    plt.suptitle('Attitude Estimation Error: Truth vs SMEKF', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()


def plot_attitude_estimation_error_quest(timeData, sigma_true, sigma_quest, phaseMarkers=None, figNum=14, startTime_hours=None, eclipseData=None):
    """
    Plot attitude estimation error: Truth vs QUEST estimated attitude in MRP (Modified Rodrigues Parameters).

    Shows the difference between true spacecraft attitude and QUEST estimated attitude
    in terms of MRP error (sigma_1, sigma_2, sigma_3).

    Args:
        timeData: Time array [hours]
        sigma_true: True attitude MRP (N x 3 array: sigma_1, sigma_2, sigma_3)
        sigma_quest: QUEST estimated attitude MRP (N x 3 array: sigma_1, sigma_2, sigma_3)
        phaseMarkers: Dictionary with phase times [hours] (optional)
        figNum: Figure number (default: 14)
        startTime_hours: Start time for plotting [hours] (optional). If provided, only data
                         from this time onwards will be plotted (e.g., QUEST activation time)
        eclipseData: Dictionary with 'entry_times_hours' and 'exit_times_hours' lists (optional)
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    plt.figure(figNum, figsize=(14, 10))

    # Filter data from startTime if provided
    if startTime_hours is not None:
        start_idx = np.searchsorted(timeData, startTime_hours)
        timeData = timeData[start_idx:]
        sigma_true = sigma_true[start_idx:, :]
        sigma_quest = sigma_quest[start_idx:, :]

    # Calculate MRP error: sigma_error = sigma_true (-) sigma_quest (MRP subtraction)
    # This represents the rotation from QUEST estimate to true attitude
    sigma_error = np.zeros_like(sigma_true)
    for i in range(len(sigma_true)):
        sigma_error[i, :] = rbk.subMRP(sigma_true[i, :], sigma_quest[i, :])

    labels = [r'$\sigma_1$', r'$\sigma_2$', r'$\sigma_3$']
    colors = ['#d62728', '#9467bd', '#8c564b']  # Red, Purple, Brown

    # Helper function to add eclipse shading
    def add_eclipse_shading(ax, eclipseData):
        if eclipseData is None:
            return
        entry_times = eclipseData.get('entry_times_hours', [])
        exit_times = eclipseData.get('exit_times_hours', [])
        for i, (t_entry, t_exit) in enumerate(zip(entry_times, exit_times)):
            label = 'Eclipse' if i == 0 else ''
            ax.axvspan(t_entry, t_exit, alpha=0.2, color='gray', label=label, zorder=0)

    # Subplot 1: sigma_1 error
    ax1 = plt.subplot(3, 1, 1)
    add_eclipse_shading(ax1, eclipseData)
    ax1.plot(timeData, sigma_error[:, 0], color=colors[0], linewidth=1.0,
             label=f'{labels[0]} Error (Truth - QUEST)')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_ylabel(f'{labels[0]} Error [-]', fontsize=11)
    ax1.set_title(r'QUEST Attitude Estimation Error: $\sigma_1$', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax1, phaseMarkers)
    ax1.legend(loc='upper right', fontsize=8)

    # Subplot 2: sigma_2 error
    ax2 = plt.subplot(3, 1, 2)
    add_eclipse_shading(ax2, eclipseData)
    ax2.plot(timeData, sigma_error[:, 1], color=colors[1], linewidth=1.0,
             label=f'{labels[1]} Error (Truth - QUEST)')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax2.set_ylabel(f'{labels[1]} Error [-]', fontsize=11)
    ax2.set_title(r'QUEST Attitude Estimation Error: $\sigma_2$', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax2, phaseMarkers)
    ax2.legend(loc='upper right', fontsize=8)

    # Subplot 3: sigma_3 error
    ax3 = plt.subplot(3, 1, 3)
    add_eclipse_shading(ax3, eclipseData)
    ax3.plot(timeData, sigma_error[:, 2], color=colors[2], linewidth=1.0,
             label=f'{labels[2]} Error (Truth - QUEST)')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax3.set_ylabel(f'{labels[2]} Error [-]', fontsize=11)
    ax3.set_xlabel('Time [hours]', fontsize=11)
    ax3.set_title(r'QUEST Attitude Estimation Error: $\sigma_3$', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax3, phaseMarkers)
    ax3.legend(loc='upper right', fontsize=8)

    plt.suptitle('Attitude Estimation Error: Truth vs QUEST (MRP)', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()


def plot_quest_attitude_angles(timeData, sigma_quest, phaseMarkers=None, figNum=16, startTime_hours=None, eclipseData=None):
    """
    Plot QUEST estimated attitude angles (Roll, Pitch, Yaw).

    Shows the attitude estimated by QUEST algorithm in terms of Euler angles.

    Args:
        timeData: Time array [hours]
        sigma_quest: QUEST estimated attitude MRP (N x 3 array: sigma_1, sigma_2, sigma_3)
        phaseMarkers: Dictionary with phase times [hours] (optional)
        figNum: Figure number (default: 16)
        startTime_hours: Start time for plotting [hours] (optional). If provided, only data
                         from this time onwards will be plotted (e.g., QUEST activation time)
        eclipseData: Dictionary with 'entry_times_hours' and 'exit_times_hours' lists (optional)
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    plt.figure(figNum, figsize=(14, 10))

    # Filter data from startTime if provided
    if startTime_hours is not None:
        start_idx = np.searchsorted(timeData, startTime_hours)
        timeData = timeData[start_idx:]
        sigma_quest = sigma_quest[start_idx:, :]

    # Convert MRP to Euler angles (roll, pitch, yaw) for each timestep
    roll = np.zeros(len(sigma_quest))
    pitch = np.zeros(len(sigma_quest))
    yaw = np.zeros(len(sigma_quest))

    for i in range(len(sigma_quest)):
        # Convert MRP to DCM
        dcm = rbk.MRP2C(sigma_quest[i, :])
        # Convert DCM to 321 Euler angles (yaw-pitch-roll)
        euler321 = rbk.C2Euler321(dcm)
        # Extract roll, pitch, yaw (in radians, then convert to degrees)
        yaw[i] = np.rad2deg(euler321[0])      # yaw (rotation about z-axis)
        pitch[i] = np.rad2deg(euler321[1])    # pitch (rotation about y-axis)
        roll[i] = np.rad2deg(euler321[2])     # roll (rotation about x-axis)

    # Unwrap angles to remove discontinuities at +/-180 deg
    roll = np.unwrap(roll, period=360)
    pitch = np.unwrap(pitch, period=360)
    yaw = np.unwrap(yaw, period=360)

    labels = ['Roll', 'Pitch', 'Yaw']
    colors = ['#d62728', '#9467bd', '#8c564b']  # Red, Purple, Brown
    data = [roll, pitch, yaw]

    # Helper function to add eclipse shading
    def add_eclipse_shading(ax, eclipseData):
        if eclipseData is None:
            return
        entry_times = eclipseData.get('entry_times_hours', [])
        exit_times = eclipseData.get('exit_times_hours', [])
        for i, (t_entry, t_exit) in enumerate(zip(entry_times, exit_times)):
            label = 'Eclipse' if i == 0 else ''
            ax.axvspan(t_entry, t_exit, alpha=0.2, color='gray', label=label, zorder=0)

    # Subplot 1: Roll
    ax1 = plt.subplot(3, 1, 1)
    add_eclipse_shading(ax1, eclipseData)
    ax1.plot(timeData, roll, color=colors[0], linewidth=1.0, label='Roll (QUEST)')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_ylabel('Roll [deg]', fontsize=11)
    ax1.set_title('QUEST Estimated Attitude: Roll', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax1, phaseMarkers)
    ax1.legend(loc='upper right', fontsize=8)

    # Subplot 2: Pitch
    ax2 = plt.subplot(3, 1, 2)
    add_eclipse_shading(ax2, eclipseData)
    ax2.plot(timeData, pitch, color=colors[1], linewidth=1.0, label='Pitch (QUEST)')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax2.set_ylabel('Pitch [deg]', fontsize=11)
    ax2.set_title('QUEST Estimated Attitude: Pitch', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax2, phaseMarkers)
    ax2.legend(loc='upper right', fontsize=8)

    # Subplot 3: Yaw
    ax3 = plt.subplot(3, 1, 3)
    add_eclipse_shading(ax3, eclipseData)
    ax3.plot(timeData, yaw, color=colors[2], linewidth=1.0, label='Yaw (QUEST)')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax3.set_ylabel('Yaw [deg]', fontsize=11)
    ax3.set_xlabel('Time [hours]', fontsize=11)
    ax3.set_title('QUEST Estimated Attitude: Yaw', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    _add_phase_markers_with_labels(ax3, phaseMarkers)
    ax3.legend(loc='upper right', fontsize=8)

    plt.suptitle('QUEST Estimated Attitude (Euler Angles)', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()


def plot_omega_estimation_error_smekf(timeData, omega_true, omega_smekf, phaseMarkers=None, figNum=15):
    """
    Plot angular velocity estimation error: Truth vs SMEKF estimated omega_BN_B.

    Shows the difference between true spacecraft angular velocity and SMEKF estimated
    angular velocity in body frame (x, y, z components).

    Args:
        timeData: Time array [hours]
        omega_true: True angular velocity [rad/s] (N x 3 array: omega_x, omega_y, omega_z)
        omega_smekf: SMEKF estimated angular velocity [rad/s] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours] (optional)
        figNum: Figure number (default: 15)
    """
    plt.figure(figNum, figsize=(14, 10))

    # Convert to deg/s for better readability
    omega_true_degs = np.rad2deg(omega_true)
    omega_smekf_degs = np.rad2deg(omega_smekf)

    # Calculate errors
    omega_error_degs = omega_true_degs - omega_smekf_degs

    labels = [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green

    # Subplot 1: omega_x error
    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(timeData, omega_error_degs[:, 0], color=colors[0], linewidth=1.0,
             label=f'{labels[0]} Error (Truth - SMEKF)')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_ylabel(f'{labels[0]} Error [deg/s]', fontsize=11)
    ax1.set_title('SMEKF Angular Velocity Estimation Error: X-axis', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax1, phaseMarkers)

    # Subplot 2: omega_y error
    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(timeData, omega_error_degs[:, 1], color=colors[1], linewidth=1.0,
             label=f'{labels[1]} Error (Truth - SMEKF)')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax2.set_ylabel(f'{labels[1]} Error [deg/s]', fontsize=11)
    ax2.set_title('SMEKF Angular Velocity Estimation Error: Y-axis', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax2, phaseMarkers)

    # Subplot 3: omega_z error
    ax3 = plt.subplot(3, 1, 3)
    ax3.plot(timeData, omega_error_degs[:, 2], color=colors[2], linewidth=1.0,
             label=f'{labels[2]} Error (Truth - SMEKF)')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax3.set_ylabel(f'{labels[2]} Error [deg/s]', fontsize=11)
    ax3.set_xlabel('Time [hours]', fontsize=11)
    ax3.set_title('SMEKF Angular Velocity Estimation Error: Z-axis', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=9)
    _add_phase_markers(ax3, phaseMarkers)

    plt.suptitle('Angular Velocity Estimation Error: Truth vs SMEKF', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()


def plot_gyro_bias_estimation(timeData, bias_true, bias_smekf, phaseMarkers=None, figNum=16):
    """
    Plot gyroscope bias: True bias vs SMEKF estimated bias.

    Shows comparison between the true gyroscope bias (from IMU model) and
    the bias estimated by the SMEKF filter.

    Args:
        timeData: Time array [hours]
        bias_true: True gyro bias [rad/s] (N x 3 array: bias_x, bias_y, bias_z)
        bias_smekf: SMEKF estimated bias [rad/s] (N x 3 array)
        phaseMarkers: Dictionary with phase times [hours] (optional)
        figNum: Figure number (default: 16)
    """
    plt.figure(figNum, figsize=(14, 12))

    # Convert to deg/hr for better readability (standard unit for gyro bias)
    bias_true_deghr = np.rad2deg(bias_true) * 3600.0
    bias_smekf_deghr = np.rad2deg(bias_smekf) * 3600.0
    bias_error_deghr = bias_true_deghr - bias_smekf_deghr

    labels = ['X-axis', 'Y-axis', 'Z-axis']
    colors_true = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green
    colors_est = ['#aec7e8', '#ffbb78', '#98df8a']   # Light versions

    for i in range(3):
        # Subplot: True vs Estimated
        ax1 = plt.subplot(3, 2, 2*i + 1)
        ax1.plot(timeData, bias_true_deghr[:, i], color=colors_true[i], linewidth=1.5,
                 label=f'True Bias {labels[i]}')
        ax1.plot(timeData, bias_smekf_deghr[:, i], color=colors_est[i], linewidth=1.5,
                 linestyle='--', label=f'SMEKF Estimated {labels[i]}')
        ax1.axhline(y=0, color='k', linestyle=':', linewidth=0.5, alpha=0.5)
        ax1.set_ylabel(f'Bias {labels[i]} [deg/hr]', fontsize=10)
        if i == 0:
            ax1.set_title('Gyro Bias: True vs SMEKF Estimated', fontsize=11, fontweight='bold')
        if i == 2:
            ax1.set_xlabel('Time [hours]', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', fontsize=8)
        _add_phase_markers(ax1, phaseMarkers)

        # Subplot: Estimation Error
        ax2 = plt.subplot(3, 2, 2*i + 2)
        ax2.plot(timeData, bias_error_deghr[:, i], color=colors_true[i], linewidth=1.0,
                 label=f'Bias Error {labels[i]}')
        ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        ax2.set_ylabel(f'Error {labels[i]} [deg/hr]', fontsize=10)
        if i == 0:
            ax2.set_title('Bias Estimation Error (True - SMEKF)', fontsize=11, fontweight='bold')
        if i == 2:
            ax2.set_xlabel('Time [hours]', fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', fontsize=8)
        _add_phase_markers(ax2, phaseMarkers)

    plt.suptitle('Gyroscope Bias Estimation: IMU True vs SMEKF', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()


def _add_phase_markers(ax, phaseMarkers):
    """
    Helper function to add phase transition markers to a plot.

    Args:
        ax: Matplotlib axis object
        phaseMarkers: Dictionary with phase times [hours]
    """
    if phaseMarkers is None:
        return

    marker_colors = {
        'deployment': 'red',
        'sunPointing': 'orange',
        'gsPointing': 'green',
        'payloadA': 'blue',
        'payloadB': 'purple',
        'imaging': 'cyan'
    }

    for phase_name, phase_time in phaseMarkers.items():
        color = marker_colors.get(phase_name, 'gray')
        ax.axvline(x=phase_time, color=color, linestyle='--', linewidth=0.8, alpha=0.5)


def _add_phase_markers_with_labels(ax, phaseMarkers):
    """
    Helper function to add phase transition markers with labels to a plot.

    Args:
        ax: Matplotlib axis object
        phaseMarkers: Dictionary with phase times [hours]
    """
    if phaseMarkers is None:
        return

    # Define colors and display labels for each phase
    phase_config = {
        'deployment': {'color': 'red', 'label': 'Deploy'},
        'sunSafe': {'color': 'orange', 'label': 'SunSafe'},
        'sunPointing': {'color': 'orange', 'label': 'SunSafe'},
        'gsPointing': {'color': 'green', 'label': 'GS'},
        'payloadA': {'color': 'blue', 'label': 'PayloadA'},
        'payloadB': {'color': 'purple', 'label': 'PayloadB'},
        'imaging': {'color': 'cyan', 'label': 'Imaging'},
        'eclipse': {'color': 'gray', 'label': 'Eclipse'}
    }

    for phase_name, phase_time in phaseMarkers.items():
        config = phase_config.get(phase_name, {'color': 'gray', 'label': phase_name})
        color = config['color']
        label = config['label']

        # Add vertical line
        ax.axvline(x=phase_time, color=color, linestyle='--', linewidth=1.2, alpha=0.7)

        # Add label at top of plot
        ymin, ymax = ax.get_ylim()
        ax.text(phase_time, ymax * 0.95, label, rotation=90, verticalalignment='top',
                horizontalalignment='right', fontsize=7, color=color, fontweight='bold', alpha=0.8)
