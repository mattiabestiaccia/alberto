"""
EXCITE_Fsw.py - Software di volo (FSW) del satellite EXCITE

Questo modulo implementa tutti gli algoritmi di bordo che girano sull'On-Board Computer (OBC):
- Navigazione: QUEST (determinazione assetto statica), SMEKF (filtro Kalman esteso moltiplicativo)
- Guida: Sun-safe pointing, Ground Station pointing, Nadir pointing, Payload pointing
- Controllo: MRP Steering (anello esterno), Rate Servo (anello interno), B-dot (detumbling)
- Gestione momento angolare: Desaturazione RW tramite MTB
- Macchina a stati finiti (FSM): Gestione automatica delle fasi di missione

Algoritmi di controllo:
- Anello esterno: MRP Steering con limitazione velocità angolare
- Anello interno: Rate Servo per inseguimento velocità
- B-dot controller: Per stabilizzazione iniziale (detumbling)

Algoritmi di navigazione:
- QUEST: Determinazione assetto da 2 vettori (Sole + Campo magnetico)
- SMEKF: Fusione ottima di Star Tracker, IMU e QUEST

Classe principale:
- BSKFswModels: Configura e gestisce tutti i moduli FSW e la logica FSM
"""

import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (attTrackingError, mrpSteering, rateServoFullNonlinear,
                                    lowPassFilterTorqueCommand,
                                    rwMotorTorque, B_dot_controller_C,
                                    sunSafePoint, locationPointing, cssWlsEst, sunlineEphem,
                                    mtbMomentumManagementSimple, torque2Dipole,
                                    dipoleMapping, rwNullSpace,
                                    questAttDet, SMEKF)
from Basilisk.simulation import ephemerisConverter, groundLocation, hingedBodyLinearProfiler, hingedRigidBodyMotor
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import macros as mc

bskPath = __path__[0]


class BSKFswModels:
    """
    Classe che gestisce il Flight Software (FSW) di EXCITE

    Implementa:
    - Algoritmi di Guida, Navigazione e Controllo (GNC)
    - Macchina a stati finiti (FSM) per gestione autonoma della missione
    - Gestione momento angolare (desaturazione RW)
    - Logica di detumbling e pointing
    """
    def __init__(self, SimBase, fswRate):
        """
        Inizializza i modelli FSW

        Args:
            SimBase: Riferimento alla simulazione base
            fswRate: Frequenza di aggiornamento FSW [s]
        """
        # Definisce variabili di classe vuote (messaggi di output)
        self.vcMsg = None  # Messaggio virtual constellation (non usato)
        self.fswRwConfigMsg = None  # Configurazione RW per FSW
        self.cmdTorqueMsg = None  # Coppia comandata (output rateServo)
        self.cmdTorqueFilteredMsg = None  # Coppia comandata filtrata per RW
        self.cmdTorqueDirectMsg = None  # Coppia comandata diretta (non filtrata)
        self.attRefMsg = None  # Riferimento di assetto
        self.attGuidMsg = None  # Guida di assetto (errore tracking)
        self.cmdRwMotorMsg = None  # Comando motori RW
        self.mtbConfigMsg = None  # Configurazione MTB
        self.dipoleGatewayMsg = None  # Gateway per comandi dipolo MTB

        # Generazione comandi random basata su FSM (sistema semplificato)
        self.availableCommands = [
            'imagingMode',  # Modo imaging (nadir pointing)
            'GSPoint',  # Comunicazione ground station
            'payloadModeA_ReconfANT',  # Esperimento ReconfANT
            'payloadModeB_GPUIOT'  # Esperimento GPU IoT
        ]
        self.currentCommand = None  # Comando attualmente generato (in attesa di accesso GS)
        self.commandGenInterval_min = 3.0 * 60.0  # Intervallo generazione minimo: 3 ore [min]
        self.commandGenInterval_max = 4.0 * 60.0  # Intervallo generazione massimo: 4 ore [min]
        self.lastCommandGenTime = None  # Ultimo tempo generazione comando [ns]
        self.nextCommandGenTime = None  # Prossimo tempo programmato generazione comando [ns]

        # Definisce nome processo e time-step di default per tutti i task FSW
        self.processName = SimBase.FSWProcessName
        self.processTasksTimeStep = mc.sec2nano(fswRate)

        # ===== Crea i moduli FSW =====

        # Moduli di guida (Guidance)
        self.sunSafePoint = sunSafePoint.sunSafePoint()  # Puntamento sun-safe (pannelli verso Sole)
        self.sunSafePoint.ModelTag = "sunSafePoint"

        self.PisaGroundStation = groundLocation.GroundLocation()  # Posizione GS Pisa
        self.PisaGroundStation.ModelTag = "PisaGroundStation"

        self.PisaGSPointing = locationPointing.locationPointing()  # Puntamento verso GS Pisa
        self.PisaGSPointing.ModelTag = "PisaGSPointing"

        self.nadirPointing = locationPointing.locationPointing()  # Puntamento nadir (asse Y camera verso Terra)
        self.nadirPointing.ModelTag = "nadirPointing"

        # Stima direzione Sole (per sun-safe pointing)
        self.cssWlsEst = cssWlsEst.cssWlsEst()  # Stima WLS da CSS
        self.cssWlsEst.ModelTag = "cssWlsEst"

        self.sunlineEphem = sunlineEphem.sunlineEphem()  # Direzione Sole da effemeridi (alternativa a CSS)
        self.sunlineEphem.ModelTag = "sunlineEphem"

        self.ephemConverter = ephemerisConverter.EphemerisConverter()  # Convertitore SPICE -> Ephemeris
        self.ephemConverter.ModelTag = "ephemConverter"

        # Modulo errore tracking assetto (usato in sunSafePointTask)
        self.trackingError = attTrackingError.attTrackingError()
        self.trackingError.ModelTag = "trackingError"

        # Moduli di controllo (Control)
        self.mrpSteering = mrpSteering.mrpSteering()  # MRP Steering (anello esterno - limitazione velocità)
        self.mrpSteering.ModelTag = "mrpSteering"

        self.rateServo = rateServoFullNonlinear.rateServoFullNonlinear()  # Rate Servo (anello interno - tracking velocità)
        self.rateServo.ModelTag = "rateServo"

        self.rwMotorTorque = rwMotorTorque.rwMotorTorque()  # Coppia motori RW
        self.rwMotorTorque.ModelTag = "rwMotorTorque"

        self.rwNullSpace = rwNullSpace.rwNullSpace()  # Controllo null space RW
        self.rwNullSpace.ModelTag = "rwNullSpace"

        self.b_dot_controller = B_dot_controller_C.B_dot_controller_C()  # Controllore B-dot per detumbling
        self.b_dot_controller.ModelTag = "B_dot_Controller"

        # Moduli gestione momento angolare (Momentum Management)
        self.mtbMomentumManagement = mtbMomentumManagementSimple.mtbMomentumManagementSimple()  # Desaturazione RW con MTB
        self.mtbMomentumManagement.ModelTag = "mtbMomentumManagementSimple"

        self.torque2Dipole = torque2Dipole.torque2Dipole()  # Conversione coppia -> dipolo MTB
        self.torque2Dipole.ModelTag = "torque2Dipole"

        self.dipoleMapping = dipoleMapping.dipoleMapping()  # Mapping dipolo per momentum management
        self.dipoleMapping.ModelTag = "dipoleMapping"

        self.dipoleMappingBdot = dipoleMapping.dipoleMapping()  # Mapping dipolo separato per B-dot
        self.dipoleMappingBdot.ModelTag = "dipoleMappingBdot"

        self.lpFilter = lowPassFilterTorqueCommand.lowPassFilterTorqueCommand()  # Filtro passa-basso coppia
        self.lpFilter.ModelTag = "lpFilter"

        # Moduli di navigazione (Navigation)
        self.questModule = questAttDet.questAttDet()  # QUEST (determinazione assetto statica)
        self.questModule.ModelTag = "questAttDet"

        # SMEKF Navigation Filter module
        self.smekfModule = SMEKF.SMEKF()
        self.smekfModule.ModelTag = "SMEKF"

        # Defining the profilers for the solar panel deployment
        self.profilers_list = []
        for i in range(4):
            profiler = hingedBodyLinearProfiler.HingedBodyLinearProfiler()
            profiler.ModelTag = f"deploymentProfiler_{i+1}"
            self.profilers_list.append(profiler)

        self.panel_motors_list = []
        for i in range(4):    
            motor = hingedRigidBodyMotor.HingedRigidBodyMotor()
            motor.ModelTag =f"hingedRigidBodyMotor_{i+1}"
            self.panel_motors_list.append(motor)

        # create the FSW module gateway messages
        self.setupGatewayMsgs(SimBase)

        # Initialize all modules
        self.InitAllFSWObjects(SimBase)

        # =========================================================================
        # Create 6 tasks
        # =========================================================================
        SimBase.fswProc.addTask(SimBase.CreateNewTask("detumblingTask", self.processTasksTimeStep), 20)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("GSMonitorTask", self.processTasksTimeStep), 19)  # High priority - always monitor GS
        SimBase.fswProc.addTask(SimBase.CreateNewTask("navigationTask", self.processTasksTimeStep), 18)  # Navigation filter task (QUEST + SMEKF)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("sunSafePointTask", self.processTasksTimeStep), 15)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("GSPointTask", self.processTasksTimeStep), 15)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("imagingTask", self.processTasksTimeStep), 15)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("controlTask", self.processTasksTimeStep), 10)
        SimBase.fswProc.addTask(SimBase.CreateNewTask("solarDeploymentTask", self.processTasksTimeStep), 15)

        # =========================================================================
        # TASK 1: detumblingTask - B-dot controller with MTB for detumbling
        # =========================================================================
        # Execution order: B-dot -> dipoleMapping -> MTB effector
        SimBase.AddModelToTask("detumblingTask", self.b_dot_controller, 9)  # Execute FIRST
        SimBase.AddModelToTask("detumblingTask", self.dipoleMappingBdot, 8) # Execute SECOND

        # =========================================================================
        # TASK 2: solar arrays deployment task -
        # =========================================================================
        for i in range(len(self.profilers_list)):
            SimBase.AddModelToTask("solarDeploymentTask", self.profilers_list[i], 12)
            SimBase.AddModelToTask("solarDeploymentTask", self.panel_motors_list[i], 11)
    
        # =========================================================================
        # TASK 3: sunSafePointTask - Sun-safe pointing guidance
        # =========================================================================
        # Execution order: ephemConverter -> sunlineEphem -> sunSafePoint -> trackingError
        SimBase.AddModelToTask("sunSafePointTask", self.ephemConverter, 11)
        SimBase.AddModelToTask("sunSafePointTask", self.sunlineEphem, 10)
        SimBase.AddModelToTask("sunSafePointTask", self.sunSafePoint, 9)
        SimBase.AddModelToTask("sunSafePointTask", self.trackingError, 8)

        # =========================================================================
        # TASK 3.5: GSMonitorTask - Ground Station visibility monitoring (always active)
        # =========================================================================
        # This task monitors GS visibility WITHOUT controlling attitude
        # PisaGroundStation calculates: hasAccess, elevation, azimuth, slantRange
        # Keep this task enabled in all operational states to continuously monitor GS access
        SimBase.AddModelToTask("GSMonitorTask", self.PisaGroundStation, 10)

        # =========================================================================
        # TASK 3.6: navigationTask - Attitude Determination and Navigation Filter
        # =========================================================================
        # This task runs QUEST + SMEKF for attitude estimation
        # Execution order: ephemConverter -> sunlineEphem -> QUEST -> SMEKF
        # QUEST: Computes attitude from sun vector + magnetometer measurements
        # SMEKF: Fuses QUEST, Star Tracker, and IMU for optimal attitude estimate
        # NOTE: ephemConverter and sunlineEphem are needed to provide sun vector to QUEST
        SimBase.AddModelToTask("navigationTask", self.ephemConverter, 15)  # Ephemeris first
        SimBase.AddModelToTask("navigationTask", self.sunlineEphem, 14)    # Sun direction second
        SimBase.AddModelToTask("navigationTask", self.questModule, 12)     # QUEST runs after sun vector is available
        # SimBase.AddModelToTask("navigationTask", self.smekfModule, 11)   # SMEKF DISABLED FOR DEBUG

        # =========================================================================
        # TASK 4: GSPointTask - Ground Station pointing guidance (active only during GS comm)
        # =========================================================================
        # This task controls attitude to point at GS (only enabled during GS communication)
        # PisaGSPointing writes attitude reference to attGuidMsg (overrides sunSafePoint if both enabled)
        SimBase.AddModelToTask("GSPointTask", self.PisaGSPointing, 9)
        SimBase.AddModelToTask("GSPointTask", self.trackingError, 8)

        # =========================================================================
        # TASK 5: imagingTask - Nadir pointing guidance for camera imaging
        # =========================================================================
        # Execution order: ephemConverter -> nadirPointing -> trackingError
        SimBase.AddModelToTask("imagingTask", self.ephemConverter, 10)
        SimBase.AddModelToTask("imagingTask", self.nadirPointing, 9)
        SimBase.AddModelToTask("imagingTask", self.trackingError, 8)

        # =========================================================================
        # TASK 6: controlTask - Attitude control with RW + MTB momentum management
        # =========================================================================
        # Execution order (2-level control):
        # 1. MRP Steering (outer loop: generates rate commands with omega_max limiting)
        # 2. Rate Servo (inner loop: tracks rate commands, generates torque)
        # 3. Low-pass filter (smooths torque command)
        # 4. RW motor torque (converts torque to RW commands)
        # 5. RW null space (adds null motion, writes to RW gateway)
        # 6. MTB momentum management (calculates desaturation torque from RW speeds)
        # 7. Torque2Dipole (converts torque to dipole)
        # 8. Dipole mapping (maps to MTB, writes to MTB gateway)
        SimBase.AddModelToTask("controlTask", self.mrpSteering, 17)  # Outer loop
        SimBase.AddModelToTask("controlTask", self.rateServo, 16)    # Inner loop
        # SimBase.AddModelToTask("controlTask", self.lpFilter, 15)     # Torque smoothing - DISABLED for testing
        SimBase.AddModelToTask("controlTask", self.rwMotorTorque, 14)
        SimBase.AddModelToTask("controlTask", self.rwNullSpace, 13)
        SimBase.AddModelToTask("controlTask", self.mtbMomentumManagement, 12)
        SimBase.AddModelToTask("controlTask", self.torque2Dipole, 11)
        SimBase.AddModelToTask("controlTask", self.dipoleMapping, 10)

        # =========================================================================
        # Create FSM events
        # =========================================================================

        # Disable all tasks initially
        SimBase.fswProc.disableAllTasks()


        # -------------------------------------------------------------------------
        # Event 1: Detumbling complete -> Start deployment
        # -------------------------------------------------------------------------
        SimBase.createNewEvent(
            "startDeployment",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'detumbling' and
                self.omegaNorm < 1e-2
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'deployment'),
                setattr(self, 'deploymentStartTime_ns', self.TotalSim.CurrentNanos),
                self.FSWModels.unlockSolarPanelsForDeployment(self.TotalSim.CurrentNanos, self),
                print(f">>> DEPLOYMENT STARTED at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h (omega={self.omegaNorm:.5f} rad/s)")
            )
        )

        # -------------------------------------------------------------------------
        # Event 2: Post-deployment detumbling complete -> Start sun-safe
        # -------------------------------------------------------------------------
        SimBase.createNewEvent(
            "startSunSafe",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'deployment' and
                self.deploymentStartTime_ns is not None and
                self.TotalSim.CurrentNanos > (self.deploymentStartTime_ns + 3600e9) and
                self.omegaNorm < 1e-2
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'sunSafe'),
                print(f">>> TRANSITIONING TO SUN-SAFE at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h (omega={self.omegaNorm:.5f} rad/s)")
            )
        )

        # -------------------------------------------------------------------------
        # Event 3-6: Timed transitions for nominal schedule
        # -------------------------------------------------------------------------
        SimBase.createNewEvent(
            "startGSPointing",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'sunSafe' and
                self.TotalSim.CurrentNanos >= 17.0 * 3600e9
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'GSPoint'),
                print(f">>> TRANSITIONING TO GS POINTING at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h")
            )
        )

        SimBase.createNewEvent(
            "startPayloadA",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'GSPoint' and
                self.TotalSim.CurrentNanos >= 19.0 * 3600e9
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'payloadModeA'),
                print(f">>> TRANSITIONING TO PAYLOAD MODE A (ReconfANT) at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h")
            )
        )

        SimBase.createNewEvent(
            "startImaging",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'payloadModeA' and
                self.TotalSim.CurrentNanos >= 20.5 * 3600e9
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'imagingMode'),
                print(f">>> TRANSITIONING TO IMAGING MODE at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h")
            )
        )

        SimBase.createNewEvent(
            "startPayloadB",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'imagingMode' and
                self.TotalSim.CurrentNanos >= 22.0 * 3600e9
            ),
            actionFunction=lambda self: (
                setattr(self, 'modeRequest', 'payloadModeB'),
                print(f">>> TRANSITIONING TO PAYLOAD MODE B (GPU IoT) at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h")
            )
        )

        # -------------------------------------------------------------------------
        # Event 7-8: Eclipse management (re-enable after each trigger)
        # -------------------------------------------------------------------------
        SimBase.createNewEvent(
            "enterEclipse",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and
                self.shadowFactor < 0.1 and
                not self.inEclipse and
                self.modeRequest not in ['detumbling', 'deployment']
            ),
            actionFunction=lambda self: (
                setattr(self, 'previousMode', self.modeRequest),
                setattr(self, 'inEclipse', True),
                setattr(self, 'eclipseCounter', self.eclipseCounter + 1),  # Increment eclipse counter
                self.eclipseEntryTimes.append(self.TotalSim.CurrentNanos),  # Record entry time [ns]
                # Disable sun vector for QUEST (sun not visible during eclipse)
                setattr(self.FSWModels.questModule, 'useSunVector', 0),
                print(f">>> ENTERING ECLIPSE at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h (shadow={self.shadowFactor:.3f}, mode={self.modeRequest} -> eclipse)"),
                print(f"    [QUEST] Sun vector DISABLED (useSunVector=0) - using magnetometer only"),
                setattr(self.eventMap['exitEclipse'], 'eventActive', True),    # Enable exit event (will disable itself)
                setattr(self.eventMap['activateEclipse'], 'eventActive', True) # Enable eclipse activation event
            )
        )

        SimBase.createNewEvent(
            "exitEclipse",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and
                self.shadowFactor > 0.9 and
                self.inEclipse
            ),
            actionFunction=lambda self: (
                # Don't restore previousMode - keep current modeRequest (may have changed during eclipse)
                setattr(self, 'currentMode', None),  # Reset currentMode to trigger reactivation
                setattr(self, 'inEclipse', False),
                self.eclipseExitTimes.append(self.TotalSim.CurrentNanos),  # Record exit time [ns]
                # Re-enable sun vector for QUEST (sun visible again)
                setattr(self.FSWModels.questModule, 'useSunVector', 1),
                print(f">>> EXITING ECLIPSE at T={self.TotalSim.CurrentNanos*1e-9/3600:.2f}h (shadow={self.shadowFactor:.3f}, eclipse -> {self.modeRequest})"),
                print(f"    [QUEST] Sun vector RE-ENABLED (useSunVector=1)"),

                # RE-ENABLE all activation events to allow proper mode restoration after eclipse
                setattr(self.eventMap['activateSunSafe'], 'eventActive', True),
                setattr(self.eventMap['activateGSPoint'], 'eventActive', True),
                setattr(self.eventMap['activatePayloadA'], 'eventActive', True),
                setattr(self.eventMap['activatePayloadB'], 'eventActive', True),
                setattr(self.eventMap['activateImaging'], 'eventActive', True),
                setattr(self.eventMap['enterEclipse'], 'eventActive', True)  # Re-enable enter event for next eclipse
            )
        )
        # -------------------------------------------------------------------------
        # Mode-based task activation events (reactive to modeRequest changes)
        # -------------------------------------------------------------------------

        # Detumbling mode
        SimBase.createNewEvent(
            "activateDetumbling",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'detumbling' and
                self.currentMode != 'detumbling'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                setattr(self.FSWModels.b_dot_controller, 'enableController', 1),
                self.enableTask("detumblingTask"),
                self.enableTask("solarDeploymentTask"),
                setattr(self, 'currentMode', 'detumbling'),
                print(f"  -> Activated: detumblingTask, solarDeploymentTask")
            )
        )

        # Deployment mode (detumbling continues during deployment + QUEST starts here)
        SimBase.createNewEvent(
            "activateDeployment",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.modeRequest == 'deployment' and
                self.currentMode != 'deployment'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                setattr(self.FSWModels.b_dot_controller, 'enableController', 1),
                self.enableTask("detumblingTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("navigationTask"),  # QUEST starts at deployment
                setattr(self, 'currentMode', 'deployment'),
                # Record navigation task activation time for plotting
                setattr(self, 'navigationTaskActivationTime_ns', self.TotalSim.CurrentNanos),
                print(f"  -> Activated: detumblingTask (continues), solarDeploymentTask, navigationTask (QUEST)")
            )
        )

        # Sun-safe mode
        SimBase.createNewEvent(
            "activateSunSafe",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'sunSafe' and
                not self.inEclipse and
                self.currentMode != 'sunSafe'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                self.enableTask("sunSafePointTask"),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF
                setattr(self, 'currentMode', 'sunSafe'),
                # Record activation time ONLY on first activation (preserve original timestamp)
                (setattr(self, 'sunSafeActivationTime_ns', self.TotalSim.CurrentNanos) if self.sunSafeActivationTime_ns is None else None),
                print(f"  -> Activated: sunSafePointTask, controlTask, GSMonitorTask, navigationTask")
            )
        )

        # GS pointing mode
        SimBase.createNewEvent(
            "activateGSPoint",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'GSPoint' and
                not self.inEclipse and
                self.currentMode != 'GSPoint'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                setattr(self.FSWModels.PisaGSPointing, 'pHat_B', [0.0, 1.0, 0.0]),  # Y-axis for S-band
                self.enableTask("GSPointTask"),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF
                setattr(self, 'currentMode', 'GSPoint'),
                # Record activation time ONLY on first activation (preserve original timestamp)
                (setattr(self, 'gsPointingActivationTime_ns', self.TotalSim.CurrentNanos) if self.gsPointingActivationTime_ns is None else None),
                print(f"  -> Activated: GSPointTask (Y-axis S-band), controlTask, navigationTask")
            )
        )

        # Payload Mode A (ReconfANT - X axis)
        SimBase.createNewEvent(
            "activatePayloadA",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'payloadModeA' and
                not self.inEclipse and
                self.currentMode != 'payloadModeA'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                setattr(self.FSWModels.PisaGSPointing, 'pHat_B', [1.0, 0.0, 0.0]),  # X-axis for ReconfANT
                self.enableTask("GSPointTask"),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF
                setattr(self, 'currentMode', 'payloadModeA'),
                # Record activation time ONLY on first activation (preserve original timestamp)
                (setattr(self, 'payloadAActivationTime_ns', self.TotalSim.CurrentNanos) if self.payloadAActivationTime_ns is None else None),
                print(f"  -> Activated: GSPointTask (X-axis ReconfANT), controlTask, navigationTask")
            )
        )

        # Payload Mode B (GPU IoT - negative X axis)
        SimBase.createNewEvent(
            "activatePayloadB",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'payloadModeB' and
                not self.inEclipse and
                self.currentMode != 'payloadModeB'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                setattr(self.FSWModels.PisaGSPointing, 'pHat_B', [-1.0, 0.0, 0.0]),  # -X-axis for GPU IoT
                self.enableTask("GSPointTask"),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF
                setattr(self, 'currentMode', 'payloadModeB'),
                # Record activation time ONLY on first activation (preserve original timestamp)
                (setattr(self, 'payloadBActivationTime_ns', self.TotalSim.CurrentNanos) if self.payloadBActivationTime_ns is None else None),
                print(f"  -> Activated: GSPointTask (-X-axis GPU IoT), controlTask, navigationTask")
            )
        )

        # Imaging mode (nadir pointing with Y-axis camera)
        SimBase.createNewEvent(
            "activateImaging",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.updateFSMStateVariables() and  # Update state variables FIRST
                self.modeRequest == 'imagingMode' and
                not self.inEclipse and
                self.currentMode != 'imagingMode'
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                self.FSWModels.zeroGateWayMsgs(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                self.enableTask("imagingTask"),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF
                setattr(self, 'currentMode', 'imagingMode'),
                # Record activation time ONLY on first activation (preserve original timestamp)
                (setattr(self, 'imagingActivationTime_ns', self.TotalSim.CurrentNanos) if self.imagingActivationTime_ns is None else None),
                print(f"  -> Activated: imagingTask (nadir Y-axis), controlTask, navigationTask")
            )
        )

        # Eclipse mode: Point to GS during eclipse
        SimBase.createNewEvent(
            "activateEclipse",
            self.processTasksTimeStep,
            True,
            conditionFunction=lambda self: (
                self.inEclipse and
                self.currentMode is not None and
                self.currentMode not in ['detumbling', 'deployment', 'eclipse']
            ),
            actionFunction=lambda self: (
                self.fswProc.disableAllTasks(),
                # setattr(self.FSWModels.lpFilter, 'reset', 1),  # DISABLED - lpFilter bypassed
                self.FSWModels.mrpSteering.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.rateServo.Reset(self.TotalSim.CurrentNanos),
                self.FSWModels.mtbMomentumManagement.Reset(self.TotalSim.CurrentNanos),
                # FORCE S-band antenna (+Y axis) to point at GS during ALL eclipses
                setattr(self.FSWModels.PisaGSPointing, 'pHat_B', [0.0, 1.0, 0.0]),
                self.enableTask("controlTask"),
                self.enableTask("solarDeploymentTask"),
                self.enableTask("GSMonitorTask"),
                self.enableTask("GSPointTask"),
                self.enableTask("navigationTask"),  # Enable QUEST + SMEKF (QUEST may fail in eclipse)
                setattr(self, 'currentMode', 'eclipse'),
                print(f"  -> ECLIPSE MODE: Pointing S-band (+Y) to GS, navigationTask active")
            )
        )

    # These are module-initialization methods

    def SetSunSafePointGuidance(self, SimBase):
        """Define the sun safe pointing guidance module"""
        self.sunSafePoint.imuInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        # Use sunlineEphem (ephemeris-based) for sun direction
        self.sunSafePoint.sunDirectionInMsg.subscribeTo(self.sunlineEphem.navStateOutMsg)
        self.sunSafePoint.sHatBdyCmd = [0.0, 0.0, 1.0]
        # sunSafePoint outputs AttGuidMsg directly (no need for trackingError conversion)
        messaging.AttGuidMsg_C_addAuthor(self.sunSafePoint.attGuidanceOutMsg, self.attGuidMsg)

    def SetPisaGS(self, SimBase):
        """Defining the location of the GS in the FSW Algorithm"""
        # Setting Earth's Radius
        self.PisaGroundStation.planetRadius = 6378.137e3
        # Defining Pisa's Cordinates in IAU frame
        pisa_latitude = np.radians(43.72075)
        pisa_longitude = np.radians(10.38348)
        pisa_altitude = 3.0
        # Configure Pisa location using specifyLocation method
        self.PisaGroundStation.specifyLocation(pisa_latitude, pisa_longitude, pisa_altitude)
        # Set operational constraints for ground station visibility
        # UPDATED: track from 0 deg (filter later), increased range for full LEO coverage
        self.PisaGroundStation.minimumElevation = np.radians(0.0)  # Track all, filter 3-25 deg (S-band) / 8-25 deg (ISM)
        self.PisaGroundStation.maximumRange = 11500e3  # Increased from 2500 km for full coverage
        self.PisaGroundStation.planetInMsg.subscribeTo(SimBase.DynModels.gravFactory.spiceObject.planetStateOutMsgs[SimBase.DynModels.earth])  # Earth index = 1
        self.PisaGroundStation.addSpacecraftToModel(SimBase.DynModels.scObject.scStateOutMsg)

    def SetGSPointiGuidance(self, SimBase):
        """Defining the GS pointing guidance module

        Note: pHat_B is dynamically changed by FSM events depending on operational mode:
        - GSPoint mode: Y-axis [0, 1, 0] for S-band antenna communication
        - PayloadModeA: X-axis [1, 0, 0] for ReconfANT antenna experiment
        - PayloadModeB: -X-axis [-1, 0, 0] for GPU IoT ISM antenna experiment
        """
        self.PisaGSPointing.pHat_B = [0.0, 1.0, 0.0]  # Default: +Y body axis (S-band antenna)
        self.PisaGSPointing.useBoresightRateDamping = 1
        self.PisaGSPointing.scAttInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.PisaGSPointing.scTransInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.PisaGSPointing.locationInMsg.subscribeTo(self.PisaGroundStation.currentGroundStateOutMsg)
        messaging.AttGuidMsg_C_addAuthor(self.PisaGSPointing.attGuidOutMsg, self.attGuidMsg)

    def SetNadirPointing(self, SimBase):
        """Configure nadir pointing guidance for IM200 camera imaging mode

        Points camera Y-axis toward Earth's center for nadir imaging.
        Uses Earth ephemeris from shared ephemConverter (ephemOutMsgs[1]).
        """
        # Camera Y-axis points toward Earth center (nadir pointing)
        self.nadirPointing.pHat_B = [0.0, 1.0, 0.0]  # +Y body axis (camera boresight)
        self.nadirPointing.useBoresightRateDamping = 1  # Enable 3D rate control

        # Connect to spacecraft navigation messages
        self.nadirPointing.scAttInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.nadirPointing.scTransInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)

        # Point at Earth's center using ephemeris message from ephemConverter
        # ephemConverter.ephemOutMsgs[1] = Earth ephemeris (configured in SetSunlineEphem)
        self.nadirPointing.celBodyInMsg.subscribeTo(self.ephemConverter.ephemOutMsgs[1])

        # Connect attitude guidance output
        messaging.AttGuidMsg_C_addAuthor(self.nadirPointing.attGuidOutMsg, self.attGuidMsg)

    def SetAttitudeTrackingError(self, SimBase):
        """Attitude tracking error - not needed for sunSafePoint but kept for flexibility"""
        # trackingError can be used if we switch to reference-based guidance later
        self.trackingError.attNavInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.trackingError.attRefInMsg.subscribeTo(self.attRefMsg)
        # Do NOT add as author to attGuidMsg - sunSafePoint already does this

    def SetCSSWlsEst(self, SimBase):
        """Set the FSW CSS configuration information"""
        cssConfig = messaging.CSSConfigMsgPayload()
        totalCSSList = []
        # Use only 2 CSS for sun direction estimation (minimal config)
        nHat_B_vec = [
            [1., 0., 0.],
            [0., -1., 0.]
        ]
        for CSSHat in nHat_B_vec:
            CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
            CSSConfigElement.CBias = 1.0
            CSSConfigElement.nHat_B = CSSHat
            totalCSSList.append(CSSConfigElement)
        cssConfig.cssVals = totalCSSList
        cssConfig.nCSS = len(nHat_B_vec)
        self.cssConfigMsg = messaging.CSSConfigMsg().write(cssConfig)

        self.cssWlsEst.cssDataInMsg.subscribeTo(SimBase.DynModels.CSSConstellationObject.constellationOutMsg)
        self.cssWlsEst.cssConfigInMsg.subscribeTo(self.cssConfigMsg)

        # Set threshold to filter out noise and weak signals
        # Using same value as unit test (0.15 = 15% of max signal)
        self.cssWlsEst.sensorUseThresh = 0.15

    def SetSunlineEphem(self, SimBase):
        """Configure sunlineEphem to compute sun direction from ephemeris

        Also configures ephemerisConverter for both Sun and Earth:
        - ephemOutMsgs[0]: Sun ephemeris (for sun-safe pointing)
        - ephemOutMsgs[1]: Earth ephemeris (for nadir pointing)
        """
        # Configure ephemerisConverter to convert SPICE -> Ephemeris messages
        self.ephemConverter.addSpiceInputMsg(SimBase.DynModels.gravFactory.spiceObject.planetStateOutMsgs[0])  # Sun (index 0)
        self.ephemConverter.addSpiceInputMsg(SimBase.DynModels.gravFactory.spiceObject.planetStateOutMsgs[1])  # Earth (index 1)

        # sunlineEphem requires 3 inputs:
        # 1. Sun position from ephemerisConverter (converted from SPICE)
        # 2. Spacecraft position from simpleNav (with noise)
        # 3. Spacecraft attitude from simpleNav (with noise)
        self.sunlineEphem.sunPositionInMsg.subscribeTo(self.ephemConverter.ephemOutMsgs[0])  # Use Sun output
        self.sunlineEphem.scPositionInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.transOutMsg)
        self.sunlineEphem.scAttitudeInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)

    def SetMRPSteering(self):
        """Set the MRP Steering module (outer loop - rate limiting)"""
        # MRP Steering parameters for rate-limited attitude control
        # omega_max is the KEY parameter to prevent RW saturation
        self.mrpSteering.omega_max = 0.006  # rad/s = 0.34 deg/s - REDUCED for smoother large maneuvers
        self.mrpSteering.K1 = 0.55          # Proportional gain (linear stiffness for small errors)
        self.mrpSteering.K3 = 0.01          # Cubic gain (controls approach to omega_max)
        self.mrpSteering.ignoreOuterLoopFeedforward = False  # Use feedforward for smoother tracking

        # Input: attitude guidance message
        self.mrpSteering.guidInMsg.subscribeTo(self.attGuidMsg)

        # Output: rate command message (goes to rateServo)
        # Note: mrpSteering outputs rateCmdOutMsg, not cmdTorqueOutMsg!

    def SetRateServo(self, SimBase):
        """Set the Rate Servo module (inner loop - rate tracking)"""
        # Rate servo gains - track the rate commands from mrpSteering
        self.rateServo.Ki = 0.005     # Integral gain - REDUCED to avoid overshoot
        self.rateServo.P = 0.9        # Proportional gain - REDUCED from 1.5 to prevent RW saturation spikes
        self.rateServo.integralLimit = 2.0 / self.rateServo.Ki * 0.1
        self.rateServo.knownTorquePntB_B = [0.0, 0.0, 0.0]  # No known external torques

        # Input messages
        self.rateServo.guidInMsg.subscribeTo(self.attGuidMsg)  # Attitude guidance
        self.rateServo.vehConfigInMsg.subscribeTo(self.vcMsg)  # Vehicle inertia
        self.rateServo.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)  # RW configuration
        self.rateServo.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)  # RW speeds
        self.rateServo.rateSteeringInMsg.subscribeTo(self.mrpSteering.rateCmdOutMsg)  # Rate commands from steering!

        # Output: torque command (goes to lpFilter then RW motor torque)
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.rateServo.cmdTorqueOutMsg, self.cmdTorqueMsg)

    def SetLowPassFilter(self):
        """Setting a LowPassFilter to smooth the torque commanded to RWs"""
        self.lpFilter.wc = 25.0  # [rad/s] Cutoff frequency - tuned for 50ms delay (tau = 1/wc = 0.05s)
        self.lpFilter.h = 0.1   # filter time step, matched with fswRate
        self.lpFilter.reset = 1 # initializing filter on the first run
        self.lpFilter.cmdTorqueInMsg.subscribeTo(self.cmdTorqueMsg)
        # Filtered output which goes to cmdTorqueMsg
        messaging.CmdTorqueBodyMsg_C_addAuthor(self.lpFilter.cmdTorqueOutMsg, self.cmdTorqueFilteredMsg)

    def SetVehicleConfiguration(self):
        """Set the spacecraft configuration information - EXCITE 12U inertia"""
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        # Use the same inertia in the FSW algorithm as in the simulation (from EXCITE_Dynamics.py)
        vehicleConfigOut.ISCPntB_B = [1.02196, -0.005, 0.00983,
                                       -0.005, 1.0127, -0.0039,
                                       0.0098, -0.0039, 0.39674]
        self.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    def SetRWConfigMsg(self):
        """Set the RW device information - CubeSpace CW0162 configuration"""
        # Configure RW pyramid exactly as it is in the Dynamics (i.e. FSW with perfect knowledge)
        # CW0162 specifications from EXCITE_Dynamics.py
        rwElAngle = np.array([26.57, 26.57, 26.57, 26.57]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        rwInertia = 25997e-9  # kg*m^2 - CW0162 inertia
        self.rwMaxTorque = 7.0e-3  # Nm - CW0162 max torque

        fswSetupRW.clearSetup()
        for elAngle, azAngle in zip(rwElAngle, rwAzimuthAngle):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            fswSetupRW.create(gsHat,  # spin axis
                              rwInertia,  # kg*m^2
                              self.rwMaxTorque)  # Nm

        self.fswRwConfigMsg = fswSetupRW.writeConfigMessage()

    def SetRWMotorTorque(self):
        """Set the RW motor torque information"""
        controlAxes_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.rwMotorTorque.controlAxes_B = controlAxes_B
        self.rwMotorTorque.vehControlInMsg.subscribeTo(self.cmdTorqueMsg) # BYPASSING lpFilter - direct connection from rateServo
        # self.rwMotorTorque.vehControlInMsg.subscribeTo(self.cmdTorqueFilteredMsg) # Uncomment to re-enable lpFilter
        self.rwMotorTorque.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        # rwMotorTorque does NOT write to gateway - rwNullSpace does that

    def SetRWNullSpace(self, SimBase):
        """Configure RW null space module"""
        self.rwNullSpace.OmegaGain = 0.0000003  # Null space gain (at the beginning 0.0000003)
        desiredOmega = [2000.0 * mc.rpm2radsec] * 4  # Desired RW speeds (at the beginning 1000 RPM)
        rwDesiredSpeedPayload = messaging.RWSpeedMsgPayload()
        rwDesiredSpeedPayload.wheelSpeeds = desiredOmega
        self.rwDesiredSpeedMsg = messaging.RWSpeedMsg().write(rwDesiredSpeedPayload)

        # Create RWConstellationMsg manually with CW0162 parameters
        rwMaxTorque = 7.0e-3  # N*m
        rwInertia = 25997e-9  # kg*m^2
        rwElAngle = 26.57 * mc.D2R
        rwAzimuthAngles = [45.0, 135.0, 225.0, 315.0]

        rwConstellationConfig = messaging.RWConstellationMsgPayload()
        rwConstellationConfig.numRW = 4
        rwConfigElementList = []

        for i, azimuth in enumerate(rwAzimuthAngles):
            rwConfigElement = messaging.RWConfigElementMsgPayload()
            # Calculate gsHat for this wheel
            azimuthRad = azimuth * mc.D2R
            gsHat = (rbk.Mi(-azimuthRad, 3).dot(rbk.Mi(rwElAngle, 2))).dot(np.array([1, 0, 0]))
            rwConfigElement.gsHat_B = gsHat
            rwConfigElement.Js = rwInertia
            rwConfigElement.uMax = rwMaxTorque
            rwConfigElementList.append(rwConfigElement)

        rwConstellationConfig.reactionWheels = rwConfigElementList
        self.rwConstellationConfigMsg = messaging.RWConstellationMsg().write(rwConstellationConfig)

        self.rwNullSpace.rwMotorTorqueInMsg.subscribeTo(self.rwMotorTorque.rwMotorTorqueOutMsg)
        self.rwNullSpace.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)
        self.rwNullSpace.rwConfigInMsg.subscribeTo(self.rwConstellationConfigMsg)
        self.rwNullSpace.rwDesiredSpeedsInMsg.subscribeTo(self.rwDesiredSpeedMsg)

        # rwNullSpace writes to the RW motor command gateway
        messaging.ArrayMotorTorqueMsg_C_addAuthor(self.rwNullSpace.rwMotorTorqueOutMsg, self.cmdRwMotorMsg)

    # EXCITE-specific FSW module configuration methods
    def SetBDotController(self, SimBase):
        """Configure B-dot controller for detumbling phase"""
        mu = 3.986004418e14  # m^3/s^2 - Earth gravitational parameter
        a = 6878.137e3  # m - Semi-major axis (500km altitude)
        i = 97.99 * mc.D2R  # rad - Sun-synchronous inclination

      # EXCITE 12U spacecraft inertia (kg*m^2)
        I = [1.02196, -0.005, 0.00983,
           -0.005, 1.0127, -0.0039,
           0.0098, -0.0039, 0.39674]
        n = np.sqrt(mu / a**3)  # [rad/s] Mean motion (orbital frequency)
        geomagnetic_offset = 11.5 * mc.D2R # [rad] Approximate geomagnetic equator tilt
        csi = abs(i - geomagnetic_offset) # [rad] Inclination relative to geomagnetic equator
        # Extract minimum moment of inertia from I matrix (I is defined as [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz])
        I_diag = [I[0], I[4], I[8]] # Extract diagonal elements [Ixx, Iyy, Izz]
        I_min = min(I_diag)  # [kg*m^2] Minimum principal moment of inertia
        K_avanzini = 2 * n * (1 + np.sin(csi)) * I_min  # [kg*m^2*rad/s] Dimensionless after normalization

        # Configure B-dot controller parameters (CRITICAL for C module to work!)
        # Omega-feedback gain (dimensionless)
        self.b_dot_controller.controlGain_K = -K_avanzini  # [dimensionless] - negative for detumbling

        # Classic B-dot gain (different dimensions!)
        # Estimated from: K = m_desired / (dB/dt_typical)
        # For LEO: dB/dt ~ 3e-6 T/s (during rotation), m ~ 0.1 Am^2 -> K ~ 3e4 Am^2*s/T
        # Using 5e4 for more aggressive detumbling, negative sign for detumbling
        self.b_dot_controller.classicBDotGain_K = -5.0e5  # [Am^2*s/T] - negative for detumbling

        # Common parameters
        self.b_dot_controller.maxDipoleRequest = 0.8  # [Am^2] - CR0008 MTB maximum dipole moment
        self.b_dot_controller.minMagFieldForControl = 1.0e-8  # [T] - Minimum field threshold
        self.b_dot_controller.enableController = 1  # Start DISABLED - will be enabled after warmup in scenario
        self.b_dot_controller.useClassicBDot = 1  # 0 = omega-feedback law (default), 1 = classic B-dot law (dB/dt)

        # Connect input messages
        self.b_dot_controller.tamDataInMsg.subscribeTo(SimBase.DynModels.tamComm.tamOutMsg)
        self.b_dot_controller.scStateInMsg.subscribeTo(SimBase.DynModels.scObject.scStateOutMsg)

        # Configure dipole mapping for B-dot path: DipoleRequestBodyMsg -> MTBCmdMsg
        self.dipoleMappingBdot.steeringMatrix = [
            1.0, 0.0, 0.0,  # MTB 1: X-axis
            0.0, 1.0, 0.0,  # MTB 2: Y-axis
            0.0, 0.0, 1.0   # MTB 3: Z-axis
        ]
        self.dipoleMappingBdot.dipoleRequestBodyInMsg.subscribeTo(self.b_dot_controller.dipoleRequestOutMsg)
        self.dipoleMappingBdot.mtbArrayConfigParamsInMsg.subscribeTo(self.mtbConfigMsg)
        # Dipole mapping writes to dipole gateway
        messaging.MTBCmdMsg_C_addAuthor(self.dipoleMappingBdot.dipoleRequestMtbOutMsg, self.dipoleGatewayMsg)

    def SetMTBConfigMsg(self, SimBase):
        """MTB configuration already done in setupGatewayMsgs - this is a placeholder"""
        pass

    def SetMomentumManagement(self, SimBase):
        """Configure momentum management module"""
        self.mtbMomentumManagement.Kp = 0.003  # Proportional gain for momentum dumping (initial gain set to 0.003)
        self.mtbMomentumManagement.rwParamsInMsg.subscribeTo(self.fswRwConfigMsg)
        self.mtbMomentumManagement.rwSpeedsInMsg.subscribeTo(SimBase.DynModels.rwStateEffector.rwSpeedOutMsg)

    def SetTorque2Dipole(self, SimBase):
        """Configure torque to dipole conversion module"""
        self.torque2Dipole.tauRequestInMsg.subscribeTo(self.mtbMomentumManagement.tauMtbRequestOutMsg)
        self.torque2Dipole.tamSensorBodyInMsg.subscribeTo(SimBase.DynModels.tamComm.tamOutMsg)

    def SetDipoleMapping(self):
        """Configure dipole mapping module for 3-axis MTB"""
        # For 3 orthogonal MTBs, steering matrix is identity
        self.dipoleMapping.steeringMatrix = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.dipoleMapping.dipoleRequestBodyInMsg.subscribeTo(self.torque2Dipole.dipoleRequestOutMsg)
        self.dipoleMapping.mtbArrayConfigParamsInMsg.subscribeTo(self.mtbConfigMsg)
        # Dipole mapping writes to dipole gateway
        messaging.MTBCmdMsg_C_addAuthor(self.dipoleMapping.dipoleRequestMtbOutMsg, self.dipoleGatewayMsg)

    def SetQuestAttDet(self, SimBase):
        """Configure QUEST attitude determination module

        QUEST (QUaternion ESTimator) computes spacecraft attitude from
        two vector observations: sun direction and magnetic field.

        Input measurements:
        - Sun vector (body frame): from sunlineEphem via navStateOutMsg
        - Sun ephemeris (inertial): from ephemConverter
        - Magnetometer (body frame): from TAM via tamComm
        - Magnetic field reference (inertial): from WMM magModule
        """
        # Configure QUEST algorithm parameters
        self.questModule.lambda0 = 1.0  # Initial eigenvalue for Newton-Raphson

        # Observation weights (inversely proportional to sensor noise variance)
        # Sun sensor: ~0.5 deg accuracy -> weight = 1/(0.5*D2R)^2
        sunSensorNoise_rad = 0.5 * mc.D2R
        self.questModule.sunWeight = 1.0 / (sunSensorNoise_rad ** 2)

        # Magnetometer: ~6 deg accuracy (TAM + WMM model error) -> weight
        magSensorNoise_rad = 6.0 * mc.D2R
        self.questModule.magWeight = 1.0 / (magSensorNoise_rad ** 2)

        # Enable both vector observations
        self.questModule.useSunVector = 1
        self.questModule.useMagVector = 1

        # Algorithm convergence
        self.questModule.maxIterations = 10
        self.questModule.tolerance = 1.0e-8

        # Connect input messages
        # Sun direction in body frame: from simpleNavObject (includes noise, always available)
        # This replaces sunlineEphem which required navigationTask to be active
        self.questModule.sunlineInMsg.subscribeTo(SimBase.DynModels.simpleNavObject.attOutMsg)
        self.questModule.sunEphemerisInMsg.subscribeTo(self.ephemConverter.ephemOutMsgs[0])
        self.questModule.scPositionInMsg.subscribeTo(SimBase.DynModels.scObject.scStateOutMsg)
        self.questModule.tamSensorInMsg.subscribeTo(SimBase.DynModels.tamComm.tamOutMsg)
        self.questModule.magFieldInMsg.subscribeTo(SimBase.DynModels.magModule.envOutMsgs[0])

        print(f"[QUEST] Configured:")
        print(f"  Sun direction: from simpleNavObject (with noise)")
        print(f"  Sun weight: {self.questModule.sunWeight:.2e}")
        print(f"  Mag weight: {self.questModule.magWeight:.2e}")
        print(f"  Max iterations: {self.questModule.maxIterations}")

    def SetSMEKF(self, SimBase):
        """Configure SMEKF (Sequential Multiplicative Extended Kalman Filter)

        SMEKF estimates spacecraft attitude quaternion and gyroscope bias using:
        - Propagation: IMU gyroscope measurements
        - Measurement updates: QUEST attitude + Star Tracker (sequential fusion)

        State vector: [delta_theta (3), bias (3)]
        """
        # Process noise (matched to IMU specifications)
        # sigma_v: ARW from IMU custom module
        self.smekfModule.sigma_v = SimBase.DynModels.IMUCustom.sigma_v
        # sigma_u: Bias drift from IMU custom module
        self.smekfModule.sigma_u = SimBase.DynModels.IMUCustom.sigma_u

        # QUEST measurement noise covariance (3x3 diagonal)
        # Based on expected QUEST accuracy (~0.2 deg per axis)
        quest_noise_rad2 = (0.2 * mc.D2R) ** 2
        self.smekfModule.R_quest = [
            quest_noise_rad2, 0.0, 0.0,
            0.0, quest_noise_rad2, 0.0,
            0.0, 0.0, quest_noise_rad2
        ]

        # Star Tracker measurement noise (from datasheet)
        # Cross-boresight: 6 arcsec, Boresight: 30 arcsec
        st_cross_rad2 = (6.0 / 3600.0 * mc.D2R) ** 2
        st_bore_rad2 = (30.0 / 3600.0 * mc.D2R) ** 2
        self.smekfModule.R_st = [
            st_cross_rad2, 0.0, 0.0,
            0.0, st_cross_rad2, 0.0,
            0.0, 0.0, st_bore_rad2
        ]

        # Enable measurement sources
        self.smekfModule.useQuest = 1  # Use QUEST measurements
        self.smekfModule.useST = 1     # Use Star Tracker measurements
        self.smekfModule.propagationOnly = 0  # Full filter (not propagation only)

        # Initial conditions
        self.smekfModule.quat_BN_init = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
        self.smekfModule.bias_init = [0.0, 0.0, 0.0]  # Zero initial bias

        # Initial covariance (6x6)
        att_var = (1.0 * mc.D2R) ** 2  # 1 deg initial attitude uncertainty
        bias_var = (0.001 * mc.D2R) ** 2  # 0.001 deg/s initial bias uncertainty
        P0 = np.zeros((6, 6))
        P0[0:3, 0:3] = np.eye(3) * att_var
        P0[3:6, 3:6] = np.eye(3) * bias_var
        self.smekfModule.covar_init = P0.flatten().tolist()

        # Connect input messages
        self.smekfModule.questAttInMsg.subscribeTo(self.questModule.navStateOutMsg)
        self.smekfModule.imuSensorInMsg.subscribeTo(SimBase.DynModels.IMUCustom.sensorOutMsg)
        self.smekfModule.stSensorInMsg.subscribeTo(SimBase.DynModels.StarTracker.sensorOutMsg)

        print(f"[SMEKF] Configured:")
        print(f"  sigma_v (ARW): {self.smekfModule.sigma_v:.3e} rad/s")
        print(f"  sigma_u (bias drift): {self.smekfModule.sigma_u:.3e} rad/s/sqrt(s)")
        print(f"  QUEST noise: {np.sqrt(quest_noise_rad2)*mc.R2D:.3f} deg")
        print(f"  ST noise (cross): {np.sqrt(st_cross_rad2)*mc.R2D*3600:.1f} arcsec")
        print(f"  useQuest: {self.smekfModule.useQuest}, useST: {self.smekfModule.useST}")

    def SetSolarPanelProfilers(self):
        """Configuring the panel profiler - INITIALLY LOCKED in stowed position

        Panels start LOCKED (startTheta = endTheta = -pi) during detumbling.
        The FSM event 'initiateSolarPanelDeployment' will reconfigure these
        profilers to enable deployment trajectory.
        """
        for i in range(len(self.profilers_list)):
            # LOCKED CONFIGURATION: no movement, panels held at stowed position
            self.profilers_list[i].startTheta = -np.pi
            self.profilers_list[i].endTheta = -np.pi  # Same as startTheta = LOCKED
            self.profilers_list[i].startTime = mc.sec2nano(0)
            self.profilers_list[i].endTime = mc.sec2nano(0.000001)  # Short duration (doesn't matter when locked)

    def SetMotorPanels(self, SimBase):
        """Configuring the motors for the deploying of the solar panels

        Motors start with MODERATE gains during detumbling to resist panel oscillations
        without generating excessive reaction torques on spacecraft.
        Panels are CONNECTED to profilers from the start so locked profilers (-pi to -pi)
        combined with moderate motors keep panels near stowed position.
        Motors are adjusted by unlockSolarPanelsForDeployment() for deployment.
        """
        for i in range(len(self.panel_motors_list)):
            # MODERATE GAINS: Balance between locking panels and minimizing spacecraft torques
            # Start with small values and increase if panels still oscillate too much
            self.panel_motors_list[i].K = 0.05  # Moderate position feedback
            self.panel_motors_list[i].P = 0.1   # Moderate damping

            # Connect motor to panel state (sensed angle/rate from dynamics)
            self.panel_motors_list[i].hingedBodyStateSensedInMsg.subscribeTo(
                SimBase.DynModels.deployPanelList[i].hingedRigidBodyOutMsg
            )

            # Connect motor to profiler reference (desired angle/rate)
            self.panel_motors_list[i].hingedBodyStateReferenceInMsg.subscribeTo(
                self.profilers_list[i].hingedRigidBodyReferenceOutMsg
            )

            # CONNECT panels to profilers from the start (profilers locked at -pi during detumbling)
            # This prevents panels from drifting to 0 deg due to spring/damping dynamics
            SimBase.DynModels.deployPanelList[i].hingedRigidBodyRefMsg.subscribeTo(
                self.profilers_list[i].hingedRigidBodyReferenceOutMsg
            )

            # CONNECT motors to panels from the start (motors provide locking torque during detumbling)
            # Strong gains (K=0.1, P=0.5) resist tumbling forces
            SimBase.DynModels.deployPanelList[i].motorTorqueInMsg.subscribeTo(
                self.panel_motors_list[i].motorTorqueOutMsg
            )

    def unlockSolarPanelsForDeployment(self, currentTime_ns, SimBase):
        """Unlock solar panels and activate deployment trajectory + POWER GENERATION

        1. Activates solar panel efficiency (0% -> 29.5%) to enable power generation
        2. Reconfigures all 4 profilers from locked state (theta=-pi to theta=-pi)
           to deployment trajectory (theta=-pi to theta=-pi/2 over 60 seconds).
        3. Weakens motors (K=0.003, P=0.01) for smooth controlled deployment.

        Called by FSM event 'initiateSolarPanelDeployment'.

        Args:
            currentTime_ns: Current simulation time in nanoseconds
            SimBase: Simulation base class to access DynModels
        """
        print(f"DEBUG: unlockSolarPanelsForDeployment() CALLED at T={currentTime_ns*1e-9/3600:.2f}h")

        # STEP 1: ACTIVATE SOLAR PANEL EFFICIENCY (0% -> 29.5%)
        print(f"  ACTIVATING Solar Panel Power Generation!")
        panel_area = SimBase.DynModels.body_side_area  # 0.06 m^2
        panel_efficiency = SimBase.DynModels.nominal_panel_efficiency  # 0.295 (29.5%)
        panel_normal = [0.0, 0.0, 1.0]  # +Z body direction

        for i, panel in enumerate(SimBase.DynModels.solarPanelList):
            # Update panel efficiency from 0% to 29.5%
            panel.setPanelParameters(panel_normal, panel_area, panel_efficiency)
            print(f"    Panel {i+1}: Efficiency 0% -> {panel_efficiency*100:.1f}%")

        # STEP 2: CONFIGURE DEPLOYMENT TRAJECTORY
        for i in range(len(self.profilers_list)):
            # Reconfigure profiler for deployment trajectory
            self.profilers_list[i].startTheta = -np.pi
            self.profilers_list[i].endTheta = -np.pi/2
            # CRITICAL: Times must be relative to current simulation time!
            self.profilers_list[i].startTime = currentTime_ns
            self.profilers_list[i].endTime = currentTime_ns + mc.sec2nano(60)

            # ADJUST motors for smooth deployment
            # Lower gains allow profiler to guide movement without excessive stiffness
            self.panel_motors_list[i].K = 0.003  # Reduced from 0.005 for smoother motion
            self.panel_motors_list[i].P = 0.03   # Reduced from 0.02 for smoother motion

            print(f"  Panel {i}: startTheta={self.profilers_list[i].startTheta:.3f}, endTheta={self.profilers_list[i].endTheta:.3f}, K={self.panel_motors_list[i].K}, P={self.panel_motors_list[i].P}")

    # Global call to initialize every module
    def InitAllFSWObjects(self, SimBase):
        """Initialize all the FSW objects"""

        # note that the order in which these routines are called is important.
        # To subscribe to a message that message must already exit.

        self.SetVehicleConfiguration()
        self.SetRWConfigMsg()
        self.SetMTBConfigMsg(SimBase)  # EXCITE: Initialize MTB config first
        self.SetCSSWlsEst(SimBase)
        self.SetSunlineEphem(SimBase)
        self.SetSunSafePointGuidance(SimBase)
        self.SetPisaGS(SimBase)
        self.SetGSPointiGuidance(SimBase)
        self.SetNadirPointing(SimBase)  # EXCITE: Initialize nadir pointing for imaging mode
        self.SetAttitudeTrackingError(SimBase)
        self.SetMRPSteering()  # MODIFIED: MRP Steering (outer loop with rate limiting)
        self.SetRateServo(SimBase)  # MODIFIED: Rate Servo (inner loop for rate tracking)
        self.SetRWMotorTorque()
        self.SetLowPassFilter()
        self.SetRWNullSpace(SimBase)
        self.SetBDotController(SimBase)
        self.SetMomentumManagement(SimBase)
        self.SetTorque2Dipole(SimBase)  # Pass SimBase to access DynModels.tamComm
        self.SetDipoleMapping()
        self.SetQuestAttDet(SimBase)  # QUEST attitude determination
        self.SetSMEKF(SimBase)  # SMEKF navigation filter
        self.SetSolarPanelProfilers()  # Configure profilers first
        self.SetMotorPanels(SimBase)  # Then connect motors and panels to profilers

    def setupGatewayMsgs(self, SimBase):
        """create C-wrapped gateway messages such that different modules can write to this message
        and provide a common input msg for down-stream modules"""
        self.cmdTorqueMsg = messaging.CmdTorqueBodyMsg_C()
        self.cmdTorqueDirectMsg = messaging.CmdTorqueBodyMsg_C()
        self.attRefMsg = messaging.AttRefMsg_C()
        self.attGuidMsg = messaging.AttGuidMsg_C()
        self.cmdRwMotorMsg = messaging.ArrayMotorTorqueMsg_C()
        self.cmdTorqueFilteredMsg = messaging.CmdTorqueBodyMsg_C()

        # EXCITE: Gateway for magnetorquer dipole commands (shared between B-dot and momentum mgmt)
        self.dipoleGatewayMsg = messaging.MTBCmdMsg_C()

        # EXCITE: Create MTB config message BEFORE using it
        mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
        mtbConfigParams.numMTB = 3
        mtbConfigParams.GtMatrix_B = [
            1.0, 0.0, 0.0,  # MTB 1: X-axis
            0.0, 1.0, 0.0,  # MTB 2: Y-axis
            0.0, 0.0, 1.0   # MTB 3: Z-axis
        ]
        maxDipole = 0.8  # A*m^2 - CubeSpace CR0008 max dipole
        mtbConfigParams.maxMtbDipoles = [maxDipole] * 3
        self.mtbConfigMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)

        self.zeroGateWayMsgs()

        # connect gateway FSW effector command msgs with the dynamics
        SimBase.DynModels.extForceTorqueObject.cmdTorqueInMsg.subscribeTo(self.cmdTorqueDirectMsg)
        SimBase.DynModels.rwStateEffector.rwMotorCmdInMsg.subscribeTo(self.cmdRwMotorMsg)

        # EXCITE: Connect MTB effector to dipole gateway
        # Both B-dot controller and dipoleMapping will write to this gateway
        # The active task determines which module controls the MTBs
        SimBase.DynModels.mtbEffector.mtbCmdInMsg.subscribeTo(self.dipoleGatewayMsg)
        SimBase.DynModels.mtbEffector.mtbParamsInMsg.subscribeTo(self.mtbConfigMsg)
        SimBase.DynModels.mtbEffector.magInMsg.subscribeTo(SimBase.DynModels.magModule.envOutMsgs[0])

        # Connect all 3 MTB power modules to dipole gateway (for power consumption monitoring)
        for powerMTB in SimBase.DynModels.mtbPowerList:
            powerMTB.mtbCmdInMsg.subscribeTo(self.dipoleGatewayMsg)

    def zeroGateWayMsgs(self):
        """Zero all the FSW gateway message payloads"""
        self.cmdTorqueMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.cmdTorqueDirectMsg.write(messaging.CmdTorqueBodyMsgPayload())
        self.attRefMsg.write(messaging.AttRefMsgPayload())
        self.attGuidMsg.write(messaging.AttGuidMsgPayload())
        self.cmdRwMotorMsg.write(messaging.ArrayMotorTorqueMsgPayload())
        self.cmdTorqueFilteredMsg.write(messaging.CmdTorqueBodyMsgPayload())

        # EXCITE: Zero MTB dipole gateway
        mtbPayload = messaging.MTBCmdMsgPayload()
        mtbPayload.mtbDipoleCmds = [0.0, 0.0, 0.0]
        self.dipoleGatewayMsg.write(mtbPayload)