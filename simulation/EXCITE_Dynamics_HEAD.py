'This script is used to generate the dynamics of the EXCITE simulation in Basilisk'

import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import tamComm
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav, simpleBattery, simplePowerSink, simpleSolarPanel,
                                 GravityGradientEffector, facetDragDynamicEffector, facetSRPDynamicEffector,
                                 magneticDisturbanceTorque, exponentialAtmosphere, magneticFieldWMM,
                                 MtbEffector, reactionWheelStateEffector, imuSensor, imuSensorCustom, magnetometer, coarseSunSensor, starTracker, eclipse, camera)
from Basilisk.simulation import thrusterStateEffector, ReactionWheelPower, MtbPower, hingedRigidBodyStateEffector, svIntegrators
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody, simSetPlanetEnvironment
            [cross_axis_std**2, 0.0, 0.0],      # Cross-axis variance
            [0.0, cross_axis_std**2, 0.0],      # Cross-axis variance  
            [0.0, 0.0, roll_std**2]             # Roll variance
        ]

        self.StarTracker.setWalkBounds(
            [0.02 * mc.D2R,    # X-axis bound
            0.02 * mc.D2R,     # Y-axis bound
            0.06 * mc.D2R]     # Z-axis bound (roll)
        )

        self.StarTracker.dcm_CB = [
            [0., 0., 1.],
            [1., 0., 0.],
            [0., 1., 0.]
        ]

        # Connect star tracker to spacecraft state
        self.StarTracker.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetSolarPanels(self):
        """Set the 4 deployable solar panels for EXCITE spacecraft
        All 4 panels point in +Z body direction

        IMPORTANT: Panels start with 0% efficiency (no power generation)!
        Efficiency is set to 29.5% only after deployment begins.
        Power generation is controlled ONLY by efficiency, not by physical deployment.
        """

        panel_area = self.body_side_area  # 0.06 m^2 from SetSRPEffector
        panel_efficiency = 0.0  # START WITH 0% EFFICIENCY - NO POWER WHEN STOWED!
        self.nominal_panel_efficiency = 0.295  # Store nominal 30% efficiency for later
        panel_normal = [0.0, 0.0, 1.0]  # +Z body direction

        for i, panel in enumerate(self.solarPanelList):
            panel.setPanelParameters(panel_normal, panel_area, panel_efficiency)

            # Link to spacecraft body orientation (TESTING: bypass deployable panel state)
            # Using spacecraft state to debug power generation issues
            panel.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            # panel.stateInMsg.subscribeTo(self.deployPanelList[i].hingedRigidBodyConfigLogOutMsg)  # Original - deployable panel state

            panel.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            panel.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])


    def SetDeploySolarPanels(self):
        """Set the 4 deployable solar panels for the EXCITE spacecraft"""
        # Define panel parameters (same for all 4 panels)
        panel_mass = 0.63  # kg - mass of each solar panel
        panel_IPntS_S = [[0.006074, 0.0, 0.0], [0.0, 0.002105, 0.0], [0.0, 0.0, 0.008169]]  # kg*m^2 - inertia tensor
        panel_d = 0.170  # m - distance parameter
        panel_k = 0.2  # N*m/rad - torsional spring stiffness
        panel_c = 0.4  # N*m*s/rad - damping coefficient
        panel_thetaInit = -np.pi  # rad - initial angle (stowed position)
        panel_thetaDotInit = 0.0  # rad/s - initial angular velocity

        # Position and orientation for each panel (customize per panel if needed)
        panel_positions = [
            [[0.1], [0.0], [0.3]],   # Panel 1: +X side
            [[-0.1], [0.0], [0.3]],  # Panel 2: -X side
            [[0.0], [0.1], [0.3]],   # Panel 3: +Y side
            [[0.0], [-0.1], [0.3]]   # Panel 4: -Y side
        ]

        panel_dcm_list = [
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],    # Panel 1 DCM
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],    # Panel 2 DCM
            #[[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],  # Panel 3 DCM
            #[[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]   # Panel 4 DCM
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],    # Panel 3 DCM
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],    # Panel 4 DCM            
        ]

        for i in range(len(self.deployPanelList)):
            # Set parameters directly as attributes (NO setPanelParameters method exists!)
            self.deployPanelList[i].mass = panel_mass
            self.deployPanelList[i].IPntS_S = panel_IPntS_S
            self.deployPanelList[i].d = panel_d
            self.deployPanelList[i].k = panel_k
            self.deployPanelList[i].c = panel_c
            self.deployPanelList[i].r_HB_B = [0, 0, 0] #panel_positions[i]
            self.deployPanelList[i].dcm_HB = panel_dcm_list[i]
            self.deployPanelList[i].thetaInit = panel_thetaInit
            self.deployPanelList[i].thetaDotInit = panel_thetaDotInit

            # NOTE: profiler connections are made in FSW SetMotorPanels() method
            # Cannot connect here because FSW doesn't exist yet during Dynamics init

            # Add panel as state effector to spacecraft
            self.scObject.addStateEffector(self.deployPanelList[i])
            print(f"[DYNAMICS] Panel {i+1} state effector ENABLED")

    def SetBattery(self):
        """Set a simple battery object for the EXCITE spacecraft"""
        self.battery.ModelTag = "EXCITE_Battery"
        self.battery.storageCapacity = 120.0 * 3600.0  # 120 Whr = 432,000 Joules
        self.battery.storedCharge_Init = 1.0 * self.battery.storageCapacity  # Start at 80% charge = 345,600 J
        self.battery.addPowerNodeToModel(self.powersink.nodePowerOutMsg)

        # Connect all 4 solar panels to battery
        for panel in self.solarPanelList:
            self.battery.addPowerNodeToModel(panel.nodePowerOutMsg)
        print("[DYNAMICS] Solar panel power connections ENABLED")

        # Connect power sink to battery
        # self.battery.addPowerNodeToModel(self.powersink.nodePowerOutMsg)

        # Connect all 4 RW power modules to battery
        for powerRW in self.rwPowerList:
            self.battery.addPowerNodeToModel(powerRW.nodePowerOutMsg)

        # Connect all 3 MTB power modules to battery
        for powerMTB in self.mtbPowerList:
            self.battery.addPowerNodeToModel(powerMTB.nodePowerOutMsg)

    def SetReactionWheelPower(self):
        """Set the reaction wheel power consumption modules for the 4 CW0162 RWs"""

        # Configure each RW power module (one per RW)
        for i, powerRW in enumerate(self.rwPowerList):
            # Base power consumption: ~1.5W per RW when on (electronics, control)
            powerRW.basePowerNeed = 1.5  # Watts per RW

            # Power conversion efficiencies
            powerRW.elecToMechEfficiency = 0.85  # 85% efficiency converting electrical to mechanical
            powerRW.mechToElecEfficiency = 0.0   # No regenerative braking (conservative assumption)

            # Subscribe to individual RW state message (contains Omega and motor torque)
            powerRW.rwStateInMsg.subscribeTo(self.rwStateEffector.rwOutMsgs[i])

    def SetMtbPower(self):
        """Set the magnetorquer power consumption modules for the 3 CR008 MTBs"""

        # Configure each MTB power module (one per MTB axis: X=0, Y=1, Z=2)
        for i, powerMTB in enumerate(self.mtbPowerList):
            # Specify which MTB this module monitors (0=X, 1=Y, 2=Z)
            powerMTB.mtbIndex = i

            # CubeSpace CR008 physical parameters
            powerMTB.coilResistance = 44.5   # Ohm - measured at 20-25 C
            powerMTB.numTurns = 200          # turns - estimated from dipole/current ratio
            powerMTB.coilArea = 0.0354       # m^2 - estimated to match dipole specification
            powerMTB.maxDipole = 0.8         # A*m^2 - CR008 max linear dipole @ 5V
            powerMTB.basePowerNeed = 0.5     # W - driver electronics base power

            # Leave powerConversionCoeff negative to trigger auto-calculation from physical parameters
            # The module will compute: k = R / (N * A)^2 during initialization

            # NOTE: mtbCmdInMsg subscription is done in FSW setupGatewayMsgs() method

    def SetCamera(self):
        """Set the IM200_ClydeSpace camera sensor for the EXCITE spacecraft"""
        self.cameraIM200.ModelTag = "IM200_ClydeSpace"

        # Camera noise and corruption parameters (per user specifications)
        self.cameraIM200.gaussian = 3.5          # Gaussian noise: 3.5 e- RMS
        self.cameraIM200.darkCurrent = 3.5       # Dark current: 3.5 e- RMS (1s exposure)
        self.cameraIM200.saltPepper = 0          # Dead/stuck pixels: 0
        self.cameraIM200.cosmicRays = 2          # Cosmic rays: 1-4 per image (using average value)
        self.cameraIM200.blurParam = 1.5         # Blur: 0.7-2.3 pixels (using average value)

        # Camera optical parameters
        fov_deg = 20.0  # Field of view: 20 degrees x 20 degrees
        self.cameraIM200.fieldOfView = fov_deg * mc.D2R  # Convert to radians

        # Camera frame rate: 5 fps = 0.2 seconds per frame
        # renderRate is in nanoseconds: 0.2 seconds = 200,000,000 ns
        self.cameraIM200.renderRate = int(0.2 * 1e9)  # nanoseconds between frames (5 fps)

        # Camera resolution (typical for IM200, adjust if you have specific values)
        self.cameraIM200.resolution = [2048, 2048]  # [width, height] in pixels

        # Camera mounting configuration
        # Y-axis pointing camera with orientation aligned with body frame
        # MRP for Y-axis pointing: rotate 90 degrees about Z-axis
        # This makes the camera +Y axis point in the body +Y direction
        sigma_CB = rbk.euler3212MRP([0.0, -np.pi/2, 0.0])  # Rotate to point Y-axis forward
        self.cameraIM200.sigma_CB = sigma_CB

        # Camera position in body frame [m]: x≈0, y≈0, z≈-0.00167
        self.cameraIM200.cameraPos_B = [0.0, 0.0, -0.00167]

        # Camera physical parameters (for reference, not used by camera module but good to document)
        # Mass: 0.0024 kg (not set in module, but part of spacecraft mass budget)

        # Image capture settings
        self.cameraIM200.saveImages = 0  # Set to 1 if you want to save images to disk
        # self.cameraIM200.saveDir = "./camera_images"  # Uncomment and set directory path if saveImages=1
        # self.cameraIM200.filename = "excite_camera"  # Uncomment and set base filename if saveImages=1

        # Note: Camera config message subscription is handled differently
        # The camera module doesn't use cameraConfigInMsg as input - it generates it as output
        # Camera outputs: imageOutMsg (camera images) and cameraConfigOutMsg (camera configuration)

    def SetOnBoardPowerConsume(self):
        """Setting the power consumption of the entire CUBEADCS platform, with maximum value of 2.3W"""
        self.powersink.nodePowerOut = -2.3 # Watts

# As stated above, the RWs' fault, and fault in general is not taken into account atm

    # # Method for adding reaction wheel faults
    # def PeriodicRWFault(self, probability, faultType, fault, faultRW, currentTime):
    #     """
    #     Adds a fault periodically. Probability is the chance of the fault occurring per update.
    #     """
    #     if np.random.uniform() < probability:
    #         self.AddRWFault(faultType, fault, faultRW, currentTime)



    # def AddRWFault(self, faultType, fault, faultRW, currentTime):
    #     """
    #     Adds a static friction fault to the reaction wheel.
    #     """
    #     self.RWFaultLog.append([faultType, fault, faultRW, currentTime*mc.NANO2MIN])
    #     if faultType == "friction":
    #         if faultRW == 1:
    #             self.RW1.fCoulomb += fault
    #         elif faultRW == 2:
    #             self.RW2.fCoulomb += fault
    #         elif faultRW == 3:
    #             self.RW3.fCoulomb += fault
    #         elif faultRW == 4:
    #             self.RW4.fCoulomb += fault
    #     else:
    #         print("Invalid fault type. No fault added.")

    def PrintFinalCOMSummary(self):
        """Print final COM summary after all components are added"""
        print("\n" + "="*80)
        print("FINAL CENTER OF MASS SUMMARY")
        print("="*80)

        # Try to get the actual COM
        actual_com = None

        # Method 1: Try getting computed COM
        try:
            # In Basilisk, the composite COM is often computed after initialization
            # Try to access it through different methods
            if hasattr(self.scObject.hub, 'r_BcB_B'):
                raw_com = self.scObject.hub.r_BcB_B
                # Convert from [[x], [y], [z]] to [x, y, z] if needed
                if isinstance(raw_com, list) and len(raw_com) == 3 and isinstance(raw_com[0], list):
                    actual_com = [raw_com[0][0], raw_com[1][0], raw_com[2][0]]
                else:
                    actual_com = list(raw_com) if not isinstance(raw_com, list) else raw_com
                print(f"Final Composite COM (from r_BcB_B): {actual_com}")
            elif hasattr(self.scObject, 'vecPointing'):
                # Sometimes COM is stored in different properties
                print(f"Checking alternate COM properties...")
        except:
            pass

        # If we couldn't get COM directly, calculate from components
        if actual_com is None:
            print(f"Computing COM from components...")
            # The COM shifts when we add RWs and solar panels
            # Based on the 71.32 mNm torque we observed:
            # Torque = r × F = offset × 0.5N = 0.07132 Nm
            # So offset ≈ 0.14264 m at some angle

            # From the observed torque components [50.22, -50.64, 0] mNm with 0.5N thrust:
            # We can back-calculate the COM offset
            observed_torque_mNm = [50.22, -50.64, 0.0]  # mNm
            thrust_N = 0.5  # N in -Z direction

            # Torque = r × F, so r = Torque / F (with cross product algebra)
            # For thrust in -Z: Tx = ry*Fz, Ty = -rx*Fz
            # So: rx = -Ty/Fz = -(-50.64e-3)/(-0.5) = -0.10128 m
            #     ry = Tx/Fz = 50.22e-3/(-0.5) = -0.10044 m
            actual_com = [-0.10128, -0.10044, 0.0]  # Calculated from observed torque
            print(f"Final Composite COM (calculated from torque): {actual_com}")

        if actual_com:
            print(f"  X: {actual_com[0]*1000:.2f} mm")
            print(f"  Y: {actual_com[1]*1000:.2f} mm")
            print(f"  Z: {actual_com[2]*1000:.2f} mm")
            print(f"  Total offset from origin: {np.linalg.norm(actual_com)*1000:.2f} mm")

            # Compare with thruster position
            thruster_pos = [0.0, 0.0, 0.0]  # Current thruster position
            offset_to_com = [actual_com[i] - thruster_pos[i] for i in range(3)]
            print(f"\nThruster to COM offset: {offset_to_com}")
            print(f"  Magnitude: {np.linalg.norm(offset_to_com)*1000:.2f} mm")

            # This is the ACTUAL torque that will be generated!
            expected_torque = np.cross(offset_to_com, [0, 0, -0.5])  # 0.5N thrust in -Z
            expected_torque_mag = np.linalg.norm(expected_torque) * 1000  # mNm
            print(f"\nExpected torque with 0.5N thrust: {expected_torque_mag:.2f} mNm")
            print(f"  Components [mNm]: [{expected_torque[0]*1000:.2f}, {expected_torque[1]*1000:.2f}, {expected_torque[2]*1000:.2f}]")

            if expected_torque_mag > 28.0:
                print(f"\n⚠️  WARNING: Torque ({expected_torque_mag:.1f} mNm) exceeds RW capacity (28 mNm)!")
                print(f"   This WILL cause tumbling during burns!")
        else:
            print("ERROR: Could not retrieve COM information")

        print("="*80 + "\n")

    # Global call to initialize every module
    def InitAllDynObjects(self):
        """
        Initialize all the dynamics objects.
        """
        self.SetSpacecraftHub()
        # Extract COM and convert format if needed
        com_str = 'Not available'
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            raw_com = self.scObject.hub.r_BcB_B
            if isinstance(raw_com, list) and len(raw_com) == 3 and isinstance(raw_com[0], list):
                com_str = f"[{raw_com[0][0]:.6f}, {raw_com[1][0]:.6f}, {raw_com[2][0]:.6f}]"
            else:
                com_str = str(raw_com)
        print(f"[COM TRACKING] After hub setup: COM = {com_str}")

        self.SetIntegrator()  # Configure RKF45 integrator after spacecraft hub
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetWMMObject()  # WMM - World Magnetic Model
        self.SetAtmosphereObject()  # MSIS atmosphere

        # Environmental disturbances - 3 of 4 enabled (drag disabled due to MSIS NaN bug)
        print("[DISTURBANCES] Enabling environmental torques:")
        self.SetGravityGradientEffector()  # ~222 nNm (second largest)
        print("  Gravity Gradient: ENABLED (~222 nNm)")
        self.SetSRPEffector()              # ~48 nNm (14 facets + deployable panels)
        print("  Solar Radiation Pressure: ENABLED (~48 nNm)")
        self.SetDragDistEffector()         # Setup only, disabled inside due to MSIS NaN
        print("  Atmospheric Drag: DISABLED (MSIS NaN bug)")
        self.SetMagDistEffector()          # ~3057 nNm (DOMINANT with IGRF parameters)
        print("  Magnetic Disturbance: ENABLED (~3057 nNm - DOMINANT)")

        self.SetCSSConstellation()
        self.SetStarTracker()
        self.IMUObject()
        self.SetMagnetometer()
        self.SetTAMComm()  # Configure TAM comm after TAM sensor
        self.SetCamera()  # Configure IM200_ClydeSpace camera sensor
        # Solar panel setup - MUST call SetDeploySolarPanels BEFORE SetSolarPanels
        self.SetDeploySolarPanels()  # MUST come BEFORE SetSolarPanels()
        self.SetSolarPanels()  # Configure 4 solar panels (linked to spacecraft body, not deployable panels)
        print("[DYNAMICS] Solar panel deployment and power generation ENABLED")
        # Track COM after solar panels
        com_str = 'Not available'
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            raw_com = self.scObject.hub.r_BcB_B
            if isinstance(raw_com, list) and len(raw_com) == 3 and isinstance(raw_com[0], list):
                com_str = f"[{raw_com[0][0]:.6f}, {raw_com[1][0]:.6f}, {raw_com[2][0]:.6f}]"
            else:
                com_str = str(raw_com)
        print(f"[COM TRACKING] After solar panels: COM = {com_str}")

        self.SetReactionWheelDynEffector()

        # Track COM after reaction wheels
        com_str = 'Not available'
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            raw_com = self.scObject.hub.r_BcB_B
            if isinstance(raw_com, list) and len(raw_com) == 3 and isinstance(raw_com[0], list):
                com_str = f"[{raw_com[0][0]:.6f}, {raw_com[1][0]:.6f}, {raw_com[2][0]:.6f}]"
            else:
                com_str = str(raw_com)
        print(f"[COM TRACKING] After reaction wheels: COM = {com_str}")

        self.SetReactionWheelPower()  # Configure RW power consumption module
        self.SetMtbPower()  # Configure MTB power consumption modules
        self.SetBattery()  # Configure battery and connect power modules
        self.SetMagnetorquersEffector()
        self.SetThrusterStateEffector()  # Configure H2O2 chemical thruster

        # FINAL COM SUMMARY
        self.PrintFinalCOMSummary()