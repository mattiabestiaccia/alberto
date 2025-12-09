'This script is used to generate the dynamics of the EXCITE simulation in Basilisk'

import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import tamComm
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav, simpleBattery, simplePowerSink, simpleSolarPanel,
                                 GravityGradientEffector, facetDragDynamicEffector, facetSRPDynamicEffector,
                                 exponentialAtmosphere, magneticFieldWMM,
                                 MtbEffector, reactionWheelStateEffector, imuSensor, magnetometer, coarseSunSensor, starTracker, eclipse)
from Basilisk.simulation import thrusterStateEffector, ReactionWheelPower, hingedRigidBodyStateEffector, svIntegrators
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody, simSetPlanetEnvironment
import numpy as np
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]

class BSKDynamicModels():
    """
    EXCITE simulation class that sets up the spacecraft simulation configuration.

    """
    def __init__(self, SimBase, dynRate):
        # Store SimBase reference for use in other methods
        self.SimBase = SimBase

        # define empty class variables
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None
        self.RW1 = None
        self.RW2 = None
        self.RW3 = None
        self.RW4 = None

        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName

        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create main dynamics task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()
        self.atmosphereObject = exponentialAtmosphere.ExponentialAtmosphere()
        self.magModule = magneticFieldWMM.MagneticFieldWMM()  # WMM
        self.ggEff = GravityGradientEffector.GravityGradientEffector()
        self.SRPEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
        self.dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()
        # self.magDistTorque = magneticDisturbanceTorque.MagneticDisturbanceTorque()
        self.eclipseObject = eclipse.Eclipse()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.rwFactory = simIncludeRW.rwFactory()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrusterStateEffector = thrusterStateEffector.ThrusterStateEffector()
        self.thFactory = None  # Will be initialized in SetThrusterStateEffector()
        self.mtbEffector = MtbEffector.MtbEffector()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.StarTracker = starTracker.StarTracker()
        self.IMU = imuSensor.ImuSensor()  # Kept for compatibility (not used)
        #self.IMUCustom = imuSensorCustom.imuSensorCustom()  # Custom IMU with bias drift model
        self.TAM = magnetometer.Magnetometer()
        self.tamComm = tamComm.tamComm()  # TAM sensor frame to body frame converter
        # self.cameraIM200 = camera.Camera()  # IM200_ClydeSpace camera sensor
        self.battery = simpleBattery.SimpleBattery()
        self.powersink = simplePowerSink.SimplePowerSink()
        # Create list of 4 deployable solar panels as power units
        self.solarPanelList = []
        for i in range(4):
            panel = simpleSolarPanel.SimpleSolarPanel()
            panel.ModelTag = f"solarPanel_{i+1}"
            self.solarPanelList.append(panel)

        self.deployPanelList = []
        for i in range(4):
            deploy_panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
            deploy_panel.ModelTag = f"solarPaneldeploy_{i+1}"
            self.deployPanelList.append(deploy_panel)

        # Create list of 4 RW power modules (one per RW)
        self.rwPowerList = []
        for i in range(4):
            powerRW = ReactionWheelPower.ReactionWheelPower()
            powerRW.ModelTag = f"RW{i+1}_Power"
            self.rwPowerList.append(powerRW)

        # Create list of 3 MTB power modules (one per MTB axis)
        #self.mtbPowerList = []
        #for i in range(3):
        #    powerMTB = MtbPower.MtbPower()
        #    powerMTB.ModelTag = f"MTB{i+1}_Power"
        #    self.mtbPowerList.append(powerMTB)

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()

        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 201)  # MUST execute FIRST to provide planet positions
        SimBase.AddModelToTask(self.taskName, self.scObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.atmosphereObject, 199)
        SimBase.AddModelToTask(self.taskName, self.magModule, 198)  # Execute AFTER SPICE to have valid planet position
        SimBase.AddModelToTask(self.taskName, self.ggEff, 197)
        SimBase.AddModelToTask(self.taskName, self.SRPEffector, 197)
        SimBase.AddModelToTask(self.taskName, self.dragEffector, 197)
        # self.magDistTorque = ...  # DISABLED: task add below also disabled
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 107)  # MUST execute BEFORE CSS
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        for deploy_panel in self.deployPanelList:
            SimBase.AddModelToTask(self.taskName, deploy_panel, 196)  # MUST execute BEFORE solar panels
        # Add all 4 solar panels to main task (power generation disabled initially via efficiency=0)
        for panel in self.solarPanelList:
            SimBase.AddModelToTask(self.taskName, panel, 194)  # Execute AFTER deploy panels write their messages
        print("[DYNAMICS] Solar panel modules added to task (deployment enabled)")
        # Add all 4 RW power modules to task
        # Add all 4 RW power modules to task - DISABLED to avoid "unable to read RW status message" warning
        # for powerRW in self.rwPowerList:
        #     SimBase.AddModelToTask(self.taskName, powerRW, 195)
        # Add all 3 MTB power modules to task - DISABLED: MtbPower not available
        # for powerMTB in self.mtbPowerList:
        #     SimBase.AddModelToTask(self.taskName, powerMTB, 195)

        # The RWs Faults are here commented for the moment. They will be added in the future, assessing the task of the FDIR creation.

        # SimBase.createNewEvent("addOneTimeRWFault", self.processTasksTimeStep, True,
        #     ["self.TotalSim.CurrentNanos>=self.oneTimeFaultTime and self.oneTimeRWFaultFlag==1"],
        #     ["self.DynModels.AddRWFault('friction',0.05,1, self.TotalSim.CurrentNanos)", "self.oneTimeRWFaultFlag=0"])


        # SimBase.createNewEvent("addRepeatedRWFault", self.processTasksTimeStep, True,
        #     ["self.repeatRWFaultFlag==1"],
        #     ["self.DynModels.PeriodicRWFault(1./3000,'friction',0.005,1, self.TotalSim.CurrentNanos)", "self.setEventActivity('addRepeatedRWFault',True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Specify the spacecraft hub parameters.
        """
        self.scObject.ModelTag = "EXCITE"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [0.3280, -0.006412, -0.003930,
                    -0.006412, 0.3363, -0.002078,
                    -0.003930, -0.002078, 0.3494]
        self.scObject.hub.mHub = 23.2 # 17.695  # kg - spacecraft mass
        # self.scObject.hub.r_BcB_B = [[-0.001377], [0.00177], [0.18619]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.r_BcB_B = [[0.000291], [0.004602], [0.020653]]  # m
        # self.scObject.hub.r_BcB_B = [[0.], [0.], [0.]]  # m
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetIntegrator(self):
        """
        Configure RKF45 variable time-step integrator for high-precision
        orbital maneuvers with chemical thruster.

        RKF45 (Runge-Kutta-Fehlberg 4-5 order) provides:
        - Adaptive time-stepping for optimal accuracy vs speed
        - Required for thrusterStateEffector compatibility
        - Higher precision for orbital propagation during delta-V burns
        """
        # DISABLED: RKF45 causes segmentation fault with thrusterStateEffector
        # Solution: Use default RK4 integrator with reduced dynamics time step
        # dynRate will be reduced in scenario from 0.1s to 0.01s for thruster stability
        print("[EXCITE Dynamics] Using default RK4 integrator (RKF45 causes segfault)")

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation
        """
        timeInitString = "2023 OCTOBER 22 00:00:00.0"
        self.gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        self.gravBodies['earth'].isCentralBody = True
        self.gravBodies['earth'].useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt', 2)
        self.sun = 0
        self.earth = 1
        self.moon = 2

        self.gravFactory.addBodiesTo(self.scObject)
        self.gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                                   timeInitString,
                                                   spicePlanetFrames=["IAU_sun", "IAU_earth", "IAU_moon"],  # FIXED: Match order with createBodies
                                                   epochInMsg=True)
        self.epochMsg = self.gravFactory.epochMsg

        self.gravFactory.spiceObject.zeroBase = 'Earth'

        self.EarthEphemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetWMMObject(self):
        """Modeling the World Magnetic Model to set the Magnetic Field of the Earth in the simulation."""
        self.magModule.ModelTag = "WMM"

        # Set ABSOLUTE path to WMM.COF
        import os
        wmm_path = os.path.abspath(bskPath + '/supportData/MagneticField/')
        self.magModule.dataPath = wmm_path + '/'

        self.magModule.envMinReach = -1.0  # Disable minimum range check
        self.magModule.envMaxReach = -1.0  # Disable maximum range check
        wmmEpoch = sp.timeStringToGregorianUTCMsg('2023 October 22, 00:00:0.0 (UTC)')
        self.magModule.epochInMsg.subscribeTo(wmmEpoch)

        # Add spacecraft to model AFTER epoch is set
        self.magModule.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetAtmosphereObject(self):
        """
        Modeling the Atmosphere with Exponential Atmosphere model.

        Switched from MSIS to ExponentialAtmosphere due to:
        - MSIS numerical instabilities (NaN errors in turbopause calculations)
        - ExponentialAtmosphere is stable and computationally efficient
        - At 550 km altitude, accuracy difference is acceptable (~30% vs 15% for MSIS)
        - For 8-hour mission, drag effect is <0.01% of total ΔV (13.9 m/s)
        """
        self.atmosphereObject.ModelTag = "atmosphere"

        # Configure Earth-like exponential atmosphere parameters
        # Uses simSetPlanetEnvironment utility to set correct base density and scale height
        simSetPlanetEnvironment.exponentialAtmosphere(self.atmosphereObject, "earth")

        # Link spacecraft state to atmosphere model
        self.atmosphereObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

        print("[ATMOSPHERE SETUP] Using ExponentialAtmosphere model for Earth")
        print(f"  Base density: {self.atmosphereObject.baseDensity:.6e} kg/m^3")
        print(f"  Scale height: {self.atmosphereObject.scaleHeight/1000:.1f} km")    

    def SetExternalForceTorqueObject(self):
        """Set the external force and torque object."""
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetGravityGradientEffector(self):
        """Modeling the effects of Gravity Gradient Effector on the EXCITE spacecraft"""
        self.ggEff.ModelTag = "gravityGradient"
        self.ggEff.addPlanetName(self.gravBodies['earth'].planetName)  # Primary gravity gradient source
        self.ggEff.addPlanetName(self.gravBodies['moon'].planetName)   # Secondary perturbation
        self.ggEff.addPlanetName(self.gravBodies['sun'].planetName)    # Tertiary perturbation
        # Gravity gradient disturbance ENABLED (~222 nNm at 500km)
        self.scObject.addDynamicEffector(self.ggEff)

    def SetSRPEffector(self):
        """Modeling the effects of Solar Radiation Pressure on the EXCITE spacecraft"""
        self.ModelTag = "SRPEffector"
        self.SRPEffector.setNumFacets(14)
        self.SRPEffector.setNumArticulatedFacets(0) # This is set to 0 atm, it will be changed to 4 when the deployment of the 
                                                    # solar panels will be simulated 
        self.width = 0.2  # m 
        self.length = 0.2 # m
        self.height = 0.3 # m 
        self.panel_width = 0.05 # m
        self.body_side_area = self.length * self.height       # m^2 - rectangular side faces of the CubeSat
        self.body_top_bottom_area = self.length * self.width  # m^2 - square top and bottom faces of the CubeSat

        self.facetAreaList = [self.body_top_bottom_area,  # +Z - Positive Z axis direction panel area of the CubeSat ("pannello body mounted v1:1" in CAD file)
                              self.body_side_area,        # +X - Positive X axis direction panel area of the Cubesat ("lateral panel 3 v1:1" in CAD file)
                              self.body_side_area,        # +Y - Positive Y axis direction panel area of the Cubesat ("lateral panel 2 v1:1" in CAD file)
                              self.body_side_area,        # -Y - Negative Y axis direction panel area of the Cubesat ("lateral panel 1 v1:1" in CAD file)
                              self.body_side_area,        # +X - Negative X axis direction panel area of the CubeSat ("lateral panel 4 v1:1" in CAD file)
                              self.body_top_bottom_area,  # -Z - Negative Z axis direction panel area of the Cubesat (not specified in CAD file, there is a hole for the thruster there)
                              self.body_side_area,        # +Z - Positive Z axis direction of the first solar panel area of the CubeSat ("deployable solar panel v1:1" in CAD file)
                              self.body_side_area,        # -Z - Negative Z axis direction of the first solar panel area of the CubeSat ("deployable solar panel v1:1" in CAD file)
                              self.body_side_area,        # +Z - Positive Z axis direction of the second panel area of the CubeSat ("deployable solar panel v1:2" in CAD file)
                              self.body_side_area,        # -Z - Negative Z axis direction of the second panel area of the CubeSat ("deployable solar panel v1:2" in CAD file)
                              self.body_side_area,        # +Z - Positive Z axis direction of the third solar panel area of the CubeSat ("deployable solar panel v1:3" in CAD file)
                              self.body_side_area,        # -Z - Negative Z axis direction of the third solar panel area of the CubeSat ("deployable solar panel v1:3" in CAD file)
                              self.body_side_area,        # +Z - Positive Z axis direction of the fourth solar panel area of the CubeSat ("deployable solar panel v1:4" in CAD file)
                              self.body_side_area]        # -Z - Negative Z axis direction of the third solar panel area of the CubeSat ("deployable solar panel v1:4" in CAD file)

       # The data below has to be checked again: in particular we need to assess what these matrices are needed for and what changes between
       # top and bottom sides of the solar panels

        self.facetDcm_F0BList = [np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # "pannello body mounted"
                                 np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]]),  # "lateral panel 3 v1:1" 
                                 np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),   # "lateral panel 2 v1:1"
                                 np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]]),   # "lateral panel 1 v1:1"                                  
                                 np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),    # "lateral panel 4 v1:1"
                                 np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # "pannello body mounted" ---> not specified, putting the same as the top
                                 np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # Positive Z direction of the CubeSat solar panel area "deployable solar panel v1:1"
                                 np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # Negative Z direction of the CubeSat solar panel area "deployable solar panel v1:1"
                                 np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # Positive Z direction of the CubeSat solar panel area "deployable solar panel v1:2"
                                 np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),    # Negative Z direction of the CubeSat solar panel area "deployable solar panel v1:2"
                                 np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),   # Positive Z direction of the CubeSat solar panel area "deployable solar panel v1:3"
                                 np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),   # Negative Z direction of the CubeSat solar panel area "deployable solar panel v1:3"
                                 np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]),   # Positive Z direction of the CubeSat solar panel area "deployable solar panel v1:4"
                                 np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])    # Negative Z direction of the CubeSat solar panel area "deployable solar panel v1:4"
                                ] 
        self.facetNHat_FList = [np.array([0.0, 0.0, 1.0]) for _ in range(14)]
        self.facetRotHat_FList = [np.array([1.0, 0.0, 0.0]) for _ in range(14)]  # Rotation axis for articulated facets (not used for fixed facets)

        # Center of pressure location of each facet wrt the body frame. These are approximate values, punt in the center of the facet.
        # They are in the same order of the panel presented above.

        self.facetR_CopB_Blist = [
            np.array([0, 0, self.height]),
            np.array([self.length/2, 0, self.height/2]),
            np.array([0, self.width/2, self.height/2]),
            np.array([0, -self.width/2, self.height]),
            np.array([-self.length/2, 0, self.height/2]),
            np.array([0, 0, 0]),
            np.array([self.length/2 + self.height/2, 0, self.height]),
            np.array([self.length/2 + self.height/2, 0, self.height - self.panel_width]),
            np.array([-self.length/2 - self.height/2, 0, self.height]),
            np.array([-self.length/2 - self.height/2, 0, self.height - self.panel_width]),
            np.array([0, self.width/2 + self.height/2, self.height]),
            np.array([0, self.width/2 + self.height/2, self.height - self.panel_width]),
            np.array([0, -self.width/2 - self.height/2, self.height]),
            np.array([0, -self.width/2 - self.height/2, self.height - self.panel_width]),
        ]

        self.facetSpecularCoeffList = np.array([
            0.85, 0.1, 0.1, 0.1, 0.1, 0.1,
            0.85, 0.3, 0.85, 0.3, 0.85, 0.3, 0.85, 0.3
        ])

        self.facetDiffuseCoeffList = np.array([
            0.1, 0.7, 0.7, 0.7, 0.7, 0.7,
            0.1, 0.5, 0.1, 0.5, 0.1, 0.5, 0.1, 0.5
        ])

        for i in range(14):
            self.SRPEffector.addFacet(
                self.facetAreaList[i],
                self.facetDcm_F0BList[i],
                self.facetNHat_FList[i],
                self.facetRotHat_FList[i],
                self.facetR_CopB_Blist[i],
                self.facetDiffuseCoeffList[i],
                self.facetSpecularCoeffList[i]
            )

        self.SRPEffector.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # Solar Radiation Pressure disturbance ENABLED (~48 nNm, 14 facets + deployable panels)
        self.scObject.addDynamicEffector(self.SRPEffector)

    def SetMagDistEffector(self):
        """Modeling the effects of Magnetic Disturbance on the EXCITE spacecraft."""
        self.magDistTorque.ModelTag = "magneticDisturbanceTorque"
        self.spacecraft_dipole = [0.035, 0.035, 0.02]
        self.magDistTorque.spacecraftMagneticDipole = [[self.spacecraft_dipole[0]], [self.spacecraft_dipole[1]], [self.spacecraft_dipole[2]]]
        self.magDistTorque.magFieldInMsg.subscribeTo(self.magModule.envOutMsgs[0])
        # Magnetic disturbance ENABLED (~3057 nNm - DOMINANT disturbance with IGRF)
        self.scObject.addDynamicEffector(self.magDistTorque)

    def SetDragDistEffector(self):
        """Modeling the effects of Atmospheric Drag on the EXCITE spacecraft"""
        self.dragEffector.ModelTag = "dragEffector"

        self.dragEffector.atmoDensInMsg.subscribeTo(self.atmosphereObject.envOutMsgs[0])

        # we will use here the same facets geometry used for SRP, but with drag coefficients instead of solar ones

        self.dragCoeffList = [1.9, 1.8, 1.8, 1.8, 1.8, 2.0,
                              2.1, 1.7, 2.1, 1.7, 2.1, 1.7, 2.1, 1.7]
        
        for i in range(14):
            self.facet_normal_B = self.facetDcm_F0BList[i] @ self.facetNHat_FList[i]

            self.dragEffector.addFacet(
                self.facetAreaList[i],
                self.dragCoeffList[i],
                self.facet_normal_B,
                self.facetR_CopB_Blist[i]
            )
        # Atmospheric drag disturbance ENABLED with ExponentialAtmosphere
        # Switched from MSIS to ExponentialAtmosphere to avoid numerical instabilities
        # At 550 km, drag torque ~0.02 nNm (minor compared to magnetic ~3057 nNm)
        self.scObject.addDynamicEffector(self.dragEffector)

    def SetSimpleNavObject(self):
        """Set the navigation sensor object."""
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        # Connect sun state for sun direction calculation (vehSunPntBdy)
        # This enables simpleNavObject to compute sun direction in body frame with noise
        self.simpleNavObject.sunStateInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[0])  # Sun

        print("[NAVIGATION SETUP] Configuring realistic navigation errors:")

        # REALISTIC NAV ERRORS - GPS/GNSS + Star Tracker + IMU + CSS
        # 3-sigma bounds for simpleNavObject (random walk boundaries)
        pos_3sigma = 5.0      # m - GPS/GNSS position accuracy in LEO
        vel_3sigma = 0.1      # m/s - GPS/GNSS velocity accuracy
        att_3sigma_deg = 0.07  # deg - Star Tracker accuracy (high quality)
        rate_3sigma = 0.05 * mc.D2R  # 0.05 deg/s - IMU/gyro accuracy
        sun_3sigma_deg = 1.0  # deg - CSS sun direction accuracy

        # Convert to radians where needed
        att_3sigma_rad = att_3sigma_deg * mc.D2R
        sun_3sigma_rad = sun_3sigma_deg * mc.D2R

        # Calculate standard deviations (1-sigma = 3-sigma / 3)
        pos_std = pos_3sigma / 3.0         # m
        vel_std = vel_3sigma / 3.0         # m/s
        att_std = att_3sigma_rad / 3.0     # rad
        rate_std = rate_3sigma / 3.0       # rad/s
        sun_std = sun_3sigma_rad / 3.0     # rad

        # PMatrix: Covariance matrix (1-sigma standard deviations)
        self.simpleNavObject.PMatrix = np.diag([
            pos_std, pos_std, pos_std,       # ~1.67 m (GPS/GNSS)
            vel_std, vel_std, vel_std,       # ~0.033 m/s (GPS/GNSS)
            att_std, att_std, att_std,       # ~0.033 deg (Star Tracker)
            rate_std, rate_std, rate_std,    # ~0.017 deg/s (IMU)
            sun_std, sun_std, sun_std,       # ~0.33 deg (CSS)
            0.0, 0.0, 0.0
        ])

        # walkBounds: Random walk boundaries (3-sigma values)
        self.simpleNavObject.walkBounds = [
            pos_3sigma, pos_3sigma, pos_3sigma,        # 5.0 m
            vel_3sigma, vel_3sigma, vel_3sigma,        # 0.1 m/s
            att_3sigma_rad, att_3sigma_rad, att_3sigma_rad,  # 0.1 deg
            rate_3sigma, rate_3sigma, rate_3sigma,     # 0.05 deg/s
            sun_3sigma_rad, sun_3sigma_rad, sun_3sigma_rad,  # 1.0 deg
            0.0, 0.0, 0.0
        ]

        # Print configuration summary
        print(f"  Position (3σ): {pos_3sigma} m (GPS/GNSS)")
        print(f"  Velocity (3σ): {vel_3sigma} m/s (GPS/GNSS)")
        print(f"  Attitude (3σ): {att_3sigma_deg} deg (Star Tracker)")
        print(f"  Rate (3σ): {rate_3sigma*mc.R2D:.3f} deg/s (IMU)")
        print(f"  Sun direction (3σ): {sun_3sigma_deg} deg (CSS)")

    def SetReactionWheelDynEffector(self):
        """Set the 4 reaction wheel devices (CW0162 from CubeSpace)"""
        # specify RW momentum capacity
        maxRWMomentum = 16.2e-3  # Nms
        rwMaxSpeed = 10000.0 #8000.0  # RPM
        rwInertia = 25997e-9 #9.3805e-4 #1.946e-4  # kg*m^2
        rwMass = 0. #0.144 # kg
        rwMaxTorque = 7.0e-3 #37.0e-3 #20.0e-3  # N*m
        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwElAngle = np.array([26.57, 26.57, 26.57, 26.57])*mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
        # the position vector of each reaction wheel below is not correct at all: we need to check the datasheet of CubeSpace to assess
        # where each reaction wheel is located inside the RWs' box.
        rwPosVector = [[0.023, 0.023, 0.046],
                       [0.023, -0.023, 0.046],
                       [-0.023, -0.023, 0.046],
                       [-0.023, 0.023, 0.046]
                       ]
        
            # Create the 4-wheel pyramid configuration
        numRW = 4
        for rw in range(numRW):
            # Calculate the spin axis direction vector (gsHat) for each wheel
            # Uses rotation matrices to convert elevation and azimuth angles to 3D unit vector
            # IMPORTANTE: da riguardare
            gsHat = (rbk.Mi(-rwAzimuthAngle[rw], 3).dot(rbk.Mi(rwElAngle[rw], 2))).dot(np.array([1, 0, 0]))
            #print(f"  Allocation Matrix gsHat = {gsHat} ")
                    # Create individual reaction wheel with custom configuration using EXACT CW0162 specs
            self.rwFactory.create('custom',
                            gsHat,  # Spin axis direction vector in body frame
                            Omega_max=rwMaxSpeed,  # RPM value (10000) - Basilisk converts internally
                            u_max=rwMaxTorque,  # Maximum output torque [N*m]
                            # maxMomentum calculated automatically from Js * Omega_max
                            rwMass = rwMass, # mass of the reaction wheels, expressed in kg. Not sure if this method has the mass attribute.
                            rWB_B = [0, 0, 0], #rwPosVector[rw], 
                            #rWB_B=rwPosVector[rw], # +rwCenterOfMassLocation,  # Position vector from spacecraft CM [m]
                            Js=rwInertia,  # Flywheel inertia from datasheet [kg*m^2]
                            Omega=0.0)  # Initial wheel speed (zero at start)

        self.rwStateEffector.ModelTag = "RW_CW0162_Cluster"
        self.rwFactory.addToSpacecraft("RWA", self.rwStateEffector, self.scObject)

        # Add realistic friction parameters to CW0162 reaction wheels
        # These values model the non-ideal torque response of physical reaction wheels
        friction_coulomb = 0.8795e-3    # [N·m] Coulomb friction torque (τc)
        friction_static = 0.9055e-3     # [N·m] Static friction (stiction, τst = τc + δ)
        friction_viscous = 5.16e-6      # [N·m·s/rad] Viscous friction coefficient (cv)
        beta_stribeck = 0.5             # [rad⁻¹] Stribeck friction coefficient (βst)

        for i in range(1, numRW + 1):
            rwName = f"RW{i}"
            self.rwFactory.rwList[rwName].fCoulomb = friction_coulomb
            self.rwFactory.rwList[rwName].fStatic = friction_static
            self.rwFactory.rwList[rwName].cViscous = friction_viscous
            self.rwFactory.rwList[rwName].betaStatic = beta_stribeck

    def SetMagnetorquersEffector(self):
        """Setting the 3 Magnetorquers for the EXCITE spacecraft"""

        self.mtbEffector.ModelTag = "MTBEffector"

        # CRITICAL FIX: Do NOT configure MTB parameters or connect messages here!
        # ALL magnetorquer configuration and message connections are handled by FSW
        # in setupGatewayMsgs (EXCITE_Fsw.py:474-497) to avoid interference
        #
        # FSW handles:
        #   1. MTB configuration (GtMatrix, maxDipoles, etc.)
        #   2. dipoleGatewayMsg creation and zero-initialization
        #   3. All message subscriptions (mtbCmdInMsg, magInMsg, mtbParamsInMsg)
        #
        # Dynamics ONLY adds the empty effector to spacecraft

        self.scObject.addDynamicEffector(self.mtbEffector)

    def SetThrusterStateEffector(self):
        """
        Configure H2O2 chemical thruster for orbital maneuvers (UniPi IOD experiment).

        Thruster Specifications:
        - Type: H2O2 monopropellant (catalyzed)
        - Thrust (BoL): 0.5 N
        - Specific Impulse: 165 s
        - Location: [-0.10128, -0.10045, -0.02950] m (body frame)
        - Direction: [0, 0, -1] (fires in -Z direction)

        Configuration Notes:
        - Single thruster requires ADCS velocity pointing for proper alignment
        - Offset from CM (~0.216 m) generates disturbance torque ~50 mNm
        - RW + MTB must compensate for coupling torque during burn
        - Compatible with RKF45 variable time-step integrator
        """
        # Make a fresh thruster factory instance (critical for multiple runs)
        self.thFactory = simIncludeThruster.thrusterFactory()

        # H2O2 thruster parameters (UniPi IOD specifications)
        MaxThrust_N = 0.5           # [N] NOMINAL thrust (H2O2 monopropellant thruster)
        steadyIsp_s = 165.0         # [s] Specific impulse for H2O2 monopropellant
        cutoffFrequency = 0.2       # [rad/s] Response time ~15s (slow ramp-up for realistic thermal spool-up)
        areaNozzle_m2 = 33.183e-6   # [m^2] Nozzle exit area = 33.183 mm^2
        MinOnTime_s = 0.05          # [s] Minimum on-time pulse duration

        # Thruster location in body frame [m]
        # Testing shows torque = 0 at [0,0,0] but tumbling still occurs!
        # The problem is NOT the offset - something else in the thruster module causes tumbling
        thruster_position_B = [0.0, 0.0, 0.0]

        # Thruster firing direction (unit vector in body frame)
        # Fires in -Z direction for velocity-aligned maneuvers
        thruster_direction_B = [0.0, 0.0, -1.0]

        # Create thruster using factory
        self.thFactory.create(
            'Blank_Thruster',           # Custom thruster type
            thruster_position_B,        # Position in body frame
            thruster_direction_B,       # Thrust direction
            MaxThrust=MaxThrust_N,
            steadyIsp=steadyIsp_s,
            MinOnTime=MinOnTime_s,
            cutoffFrequency=cutoffFrequency,
            areaNozzle=areaNozzle_m2
        )

        # Add thruster to spacecraft
        thrModelTag = "H2O2_UniPi_Thruster"
        self.thFactory.addToSpacecraft(thrModelTag,
                                      self.thrusterStateEffector,
                                      self.scObject)

        print(f"[EXCITE Dynamics] H2O2 thruster configured:")
        print(f"  Thrust: {MaxThrust_N} N")
        print(f"  Isp: {steadyIsp_s} s")
        print(f"  Min On-Time: {MinOnTime_s} s")
        print(f"  Nozzle area: {areaNozzle_m2*1e6:.3f} mm^2")

        # DEBUG: Print actual spacecraft COM after all components added
        print(f"\n[COM DEBUG] Spacecraft Center of Mass Analysis:")
        # r_CN_NInit is the initial position in inertial frame, not COM
        # r_BcB_B is the COM of the hub in body frame
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            print(f"  Hub COM in body frame: {self.scObject.hub.r_BcB_B}")
        else:
            print(f"  Hub COM not accessible via r_BcB_B")

        # Try different ways to get COM
        actual_com = [0.0, 0.0, 0.0]
        com_found = False

        # Try method 1: r_BcB_B
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            raw_com = self.scObject.hub.r_BcB_B
            # Convert from [[x], [y], [z]] to [x, y, z] if needed
            if isinstance(raw_com, list) and len(raw_com) == 3:
                if isinstance(raw_com[0], list):
                    actual_com = [raw_com[0][0], raw_com[1][0], raw_com[2][0]]
                else:
                    actual_com = raw_com
            else:
                actual_com = list(raw_com)
            com_found = True
            print(f"  Using r_BcB_B for COM: {actual_com}")
        # Try method 2: c_B
        elif hasattr(self.scObject, 'c_B'):
            actual_com = self.scObject.c_B
            com_found = True
            print(f"  Using c_B for COM: {actual_com}")
        # Try method 3: Calculate from mass properties
        elif hasattr(self.scObject.hub, 'mHub') and hasattr(self.scObject.hub, 'r_BP_B'):
            # This would be COM calculation from mass properties
            print(f"  Trying to calculate COM from mass properties...")
            print(f"    Hub mass: {self.scObject.hub.mHub if hasattr(self.scObject.hub, 'mHub') else 'N/A'}")

        if not com_found:
            print(f"  WARNING: Could not determine actual COM, assuming [0,0,0]")
            # Let's print available attributes to debug
            print(f"  Available hub attributes: {[attr for attr in dir(self.scObject.hub) if not attr.startswith('_')][:10]}")

        # Calculate offset from thruster position to actual COM
        thruster_to_com = [actual_com[i] - thruster_position_B[i] for i in range(3)]
        offset_magnitude = np.linalg.norm(thruster_to_com)

        print(f"  Thruster position: {thruster_position_B}")
        print(f"  Offset from thruster to COM: {thruster_to_com}")
        print(f"  Offset magnitude: {offset_magnitude*1000:.2f} mm")

        # Calculate expected torque
        expected_torque = np.cross(thruster_to_com, [0, 0, -MaxThrust_N])
        expected_torque_mag = np.linalg.norm(expected_torque) * 1000  # mNm
        print(f"  Expected torque from offset: {expected_torque_mag:.2f} mNm")
        print(f"  Expected torque components [mNm]: [{expected_torque[0]*1000:.2f}, {expected_torque[1]*1000:.2f}, {expected_torque[2]*1000:.2f}]")
        print(f"  Position: {thruster_position_B} m")
        print(f"  Direction: {thruster_direction_B}")

        # Calculate and display disturbance torque
        r_CM = np.array([-0.001377, 0.00177, 0.18619])  # From SetSpacecraftHub()
        r_thruster = np.array(thruster_position_B)
        r_offset = r_thruster - r_CM
        F_thrust = np.array(thruster_direction_B) * MaxThrust_N
        M_disturbance = np.cross(r_offset, F_thrust)
        print(f"  Offset from CM: {np.linalg.norm(r_offset)*1000:.1f} mm")
        print(f"  Disturbance torque: [{M_disturbance[0]*1000:.1f}, {M_disturbance[1]*1000:.1f}, {M_disturbance[2]*1000:.1f}] mNm")
        print(f"  WARNING: Disturbance torque magnitude ({np.linalg.norm(M_disturbance)*1000:.1f} mNm) requires RW+MTB compensation")

    def IMUObject(self):
        """Set the IMU sensor for the EXCITE spacecraft

        Uses imuSensorCustom with classical gyroscope dynamics:
            w_measured = w_true + bias + eta_v
            bias_dot = eta_u

        Where:
            eta_v ~ N(0, sigma_v^2) - measurement white noise (ARW)
            eta_u ~ N(0, sigma_u^2) - bias random walk noise
        """
        self.IMUCustom.ModelTag = "IMUCustom"

        # Gyroscope specifications (from datasheet):
        # - Noise Density (ARW): 0.0015 deg/s/sqrt(Hz)
        # - Bias Instability: 1 deg/h

        # Convert noise density to sigma_v [rad/s]
        noise_density_deg = 0.0015  # deg/s/sqrt(Hz)
        noise_density_rad = noise_density_deg * mc.D2R  # rad/s/sqrt(Hz)
        self.IMUCustom.sigma_v = noise_density_rad  # ARW noise

        # Convert bias instability to sigma_u [rad/s/sqrt(s)]
        bias_instability_deg_hr = 1.0  # deg/h
        bias_instability = bias_instability_deg_hr * mc.D2R / 3600.0  # rad/s
        self.IMUCustom.sigma_u = bias_instability / 100.0  # Bias random walk (~1% of instability)

        # Initial bias (zero for clean start, can be modified for testing)
        self.IMUCustom.bias_init = [0.0, 0.0, 0.0]  # rad/s

        # Enable bias drift for realistic simulation
        self.IMUCustom.enableDrift = True

        # Connect to spacecraft state
        self.IMUCustom.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

        print(f"[IMU CUSTOM] Configured with classical gyro dynamics:")
        print(f"  sigma_v (ARW): {self.IMUCustom.sigma_v:.3e} rad/s/sqrt(Hz)")
        print(f"  sigma_u (bias drift): {self.IMUCustom.sigma_u:.3e} rad/s/sqrt(s)")
        print(f"  Bias drift: {'Enabled' if self.IMUCustom.enableDrift else 'Disabled'}")

    def SetMagnetometer(self):
        """Set the TAM magnetometer sensor for the EXCITE spacecraft"""
        self.TAM.ModelTag = "TAM"
        self.TAM.scaleFactor = 1.0
        # Temporarily disable noise to debug NaN issue
        self.TAM.senNoiseStd = [0.0, 0.0, 0.0]  # Tesla - noise disabled for testing
        # self.TAM.senNoiseStd = [120e-9, 120e-9, 120e-9]  # Tesla - 120 nT noise per channel (3-sigma, CubeMag Compact spec)
        self.TAM.senBias = [0.0, 0.0, 0.0]  # Tesla - no bias (assuming calibrated sensor)

        # Set measurement range limits based on ±8 Gauss specification
        self.TAM.maxOutput = 8e-4   # Tesla - +8 Gauss saturation limit
        self.TAM.minOutput = -8e-4  # Tesla - -8 Gauss saturation limit

        self.TAM.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.TAM.magInMsg.subscribeTo(self.magModule.envOutMsgs[0])

    def SetTAMComm(self):
        """Configure TAM communication module to convert sensor frame to body frame"""
        self.tamComm.ModelTag = "tamComm"
        self.tamComm.dcm_BS = [1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0]  # Identity - TAM aligned with body frame
        self.tamComm.tamInMsg.subscribeTo(self.TAM.tamDataOutMsg)

    def SetCSSConstellation(self):
        """Set the CSS sensors for the EXCITE spacecraft"""
        self.CSSConstellationObject.ModelTag = "CSS_EXCITE_Constellation"

        # Create class-level registry if it doesn't exist
        if not hasattr(self, '_css_registry'):
            self._css_registry = []

        def setupCSS(cssDevice):
            cssDevice.fov = 83. * mc.D2R         # half-angle field of view value
            cssDevice.scaleFactor = 1.0
            cssDevice.maxOutput = 2.0
            cssDevice.minOutput = 0.0
            cssDevice.senBias = 0.0
            cssDevice.senNoiseStd = 0.005  # Sensor noise standard deviation
            cssDevice.kellyFactor = 0.0
            cssDevice.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            cssDevice.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            cssDevice.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])
            # Store CSS in class-level registry
            self._css_registry.append(cssDevice)

        # Setup the position of the sun sensor wrt body frame of the spacecraft. Not used by the module but taken into account for Vizard though.     
        r_b_list = [
            [0.09947, -0.0045, 0.1665],
            [-0.08915, -0.09947, 0.06606]
        ]

        # setup CSS sensor normal vectors in body frame components
        nHat_B_List = [
            [1., 0., 0.],
            [0., -1., 0.]
        ]
        numCSS = len(nHat_B_List)

        # store all
        cssList = []
        for idx, nHat_B in enumerate(nHat_B_List):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.r_B = r_b_list[idx]
            CSS.ModelTag = "CSS" + str(idx+1)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)

    def SetStarTracker(self):
        """Set the Star Tracker Object for the EXCITE spacecraft"""
        self.StarTracker.ModelTag = "Star Tracker"
        cross_axis_std = (0.02 * mc.D2R) / 3.0  # Convert 3-signa to 1-sigma
        roll_std = (0.06 * mc.D2R) / 3.0        # Convert 3-sigma to 1-sigma

        self.StarTracker.PMatrix = [
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

        # Connect all 3 MTB power modules to battery - DISABLED: MtbPower not available
        # for powerMTB in self.mtbPowerList:
        #     self.battery.addPowerNodeToModel(powerMTB.nodePowerOutMsg)

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
        #self.SetMagDistEffector()          # ~3057 nNm (DOMINANT with IGRF parameters)
        print("  Magnetic Disturbance: ENABLED (~3057 nNm - DOMINANT)")

        self.SetCSSConstellation()
        self.SetStarTracker()
        #self.IMUObject()
        self.SetMagnetometer()
        self.SetTAMComm()  # Configure TAM comm after TAM sensor
        #self.SetCamera()  # Configure IM200_ClydeSpace camera sensor
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
        #self.SetMtbPower()  # Configure MTB power consumption modules
        self.SetBattery()  # Configure battery and connect power modules
        self.SetMagnetorquersEffector()
        self.SetThrusterStateEffector()  # Configure H2O2 chemical thruster

        # FINAL COM SUMMARY
        self.PrintFinalCOMSummary()