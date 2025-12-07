"""
EXCITE_scenario.py - Script principale per la simulazione del satellite EXCITE

Questo modulo orchestrer la simulazione completa del sistema AOCS (Attitude and Orbital Control System)
per il satellite EXCITE (CubeSat 12U). Gestisce:
- Configurazione delle condizioni iniziali orbitali e di assetto
- Loop di simulazione con gestione degli eventi FSM
- Logging dei messaggi per l'analisi dei dati
- Generazione dei grafici di performance

Architettura:
- scenario_EXCITE: Classe principale che eredita da BSKSim e BSKScenario
- runScenario(): Esegue la missione di 24 ore con transizioni automatiche FSM
- run(): Entry point per l'esecuzione della simulazione
"""

# Ottieni il percorso del file corrente
import inspect
import os
import sys

import numpy as np
from Basilisk.utilities import orbitalMotion, macros
from Basilisk import __path__

bskPath = __path__[0]
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Importa le classi master di Basilisk: classe base di simulazione e classe base dello scenario
# bskPath punta a dist3/Basilisk, risaliamo fino alla root e poi andiamo in examples
basiliskRoot = os.path.dirname(os.path.dirname(bskPath))
sys.path.append(basiliskRoot + '/examples/BskSim')
sys.path.append(basiliskRoot + '/examples/BskSim/models')
sys.path.append(basiliskRoot + '/examples/BskSim/plotting')
from BSK_masters import BSKSim, BSKScenario
# Import EXCITE modules from new structure
from excite.dynamics import spacecraft_model as EXCITE_Dynamics
from excite.fsw import fsw_model as EXCITE_Fsw
from excite.analysis import plotting as EXCITE_Plotting
import BSK_Plotting as BSK_plt

# Import EXCITE configuration
from excite.config import mission as miss_config
from excite.config import environment as env_config

# Classe scenario EXCITE (eredita da BSKSim e BSKScenario)
class scenario_EXCITE(BSKSim, BSKScenario):
    def __init__(self):
        """
        Inizializzazione dello scenario EXCITE

        Configura i rate di esecuzione, le variabili di stato FSM e i modelli dinamici/FSW.
        IMPORTANTE: fswRate deve essere <= dynRate per garantire che i messaggi siano
        scritti dalla Dinamica PRIMA di essere letti dall'FSW.
        """
        # CRITICAL: Imposta rate FSW uguale al rate Dynamics (entrambi 0.1s = 10Hz)
        # Questo garantisce che Dynamics scriva i messaggi PRIMA che FSW li legga
        # fswRate = 0.1s (10Hz), dynRate = 0.1s (10Hz)
        super(scenario_EXCITE, self).__init__(fswRate=0.1, dynRate=0.1)
        self.name = 'scenario_EXCITE'

        # Configurazione specifica EXCITE
        # CRITICAL: Timeout di backup per la fine del detumbling (FSM event autoTransitionToSunSafe)
        self.detumblingEndTime = macros.hour2nano(12)  # 12 ore fase di detumbling (timeout di backup)

        # Inizializza la richiesta di modo FSM (usata dagli eventi in EXCITE_Fsw.py)
        self.modeRequest = 'detumbling'  # Inizia direttamente in modo detumbling
        self.currentMode = None  # Modo attualmente attivo (impostato dagli eventi di attivazione)

        # Variabili di stato per le transizioni FSM basate su eventi
        self.omegaNorm = 0.0  # Magnitudine velocità angolare corrente [rad/s]
        self.detumblingComplete = False  # Flag: detumbling completato quando omega < soglia
        self.deploymentStartTime_ns = None  # [ns] Tempo effettivo di inizio dispiegamento pannelli solari

        # Variabili di stato per la gestione delle eclissi
        self.shadowFactor = 1.0  # Fattore d'ombra corrente (0=eclissi totale, 1=pieno sole)
        self.previousMode = None  # Modo a cui tornare dopo l'eclissi
        self.inEclipse = False  # Flag: attualmente in modo eclissi
        self.eclipseCounter = 0  # Contatore: numero di ingressi in eclissi durante la missione
        self.eclipseEntryTimes = []  # [ns] Lista dei tempi di ingresso in eclissi
        self.eclipseExitTimes = []  # [ns] Lista dei tempi di uscita da eclissi
        self.currentEclipseEntryTime = None  # [ns] Tempo di ingresso in eclissi corrente (temporaneo)

        # Variabili di stato della Ground Station (sistema basato su FSM)
        self.gsElevation = 0.0  # [deg] Angolo di elevazione GS corrente
        self.gsRange = 99999.0  # [km] Distanza GS corrente
        self.gsHasAccess = False  # Flag: GS attualmente visibile

        # Stato della generazione comandi random (basato su FSM)
        self.commandStartTime = 0  # [ns] Tempo di inizio comando corrente (per tracking timeout)
        self.missionOperationsStarted = False  # Flag: True dopo prima transizione a missionOperations

        # Timestamp di esecuzione comandi per plotting (registrati all'inizio dei comandi)
        self.commandExecutionTimes = {}  # Dict: {nome_comando: tempo_inizio_ns}

        # Timestamp di attivazione modi (registrati quando i modi sono effettivamente attivati, non solo richiesti)
        self.sunSafeActivationTime_ns = None
        self.gsPointingActivationTime_ns = None
        self.payloadAActivationTime_ns = None
        self.payloadBActivationTime_ns = None
        self.imagingActivationTime_ns = None
        self.navigationTaskActivationTime_ns = None  # Tempo di attivazione QUEST (all'inizio deployment)

        # Dizionario per i recorder dei messaggi
        self.msgRecList = {}

        # Imposta i modelli di dinamica e FSW per EXCITE
        self.set_DynModel(EXCITE_Dynamics)
        self.set_FswModel(EXCITE_Fsw)

        # Configura le condizioni iniziali orbitali e di assetto
        self.configure_initial_conditions()
        # Configura il logging dei messaggi per l'analisi
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following lines
        # DynModels = self.get_DynModel()
        # vizSupport.enableUnityVisualization(self, DynModels.taskName, DynModels.scObject
        #                                     , saveFile=__file__
        #                                     , rwEffectorList=DynModels.rwStateEffector
        #                                     , mtbList=DynModels.mtbEffector
        #                                     )

    def updateFSMStateVariables(self):
        """Aggiorna tutte le variabili di stato FSM dai messaggi

        Questa funzione legge i messaggi e aggiorna:
        - omegaNorm: Magnitudine velocità angolare per transizioni detumbling
        - shadowFactor: Fattore d'ombra eclissi per modo eclissi automatico
        - batterySOC: Stato di carica batteria (%) per transizioni di ricarica
        - gsElevation: Angolo di elevazione ground station per rilevamento contatto
        - gsHasAccess: Flag visibilità ground station
        """
        try:
            DynModel = self.get_DynModel()
            FswModel = self.get_FswModel()
        except Exception as e:
            print(f'[ERROR] updateFSMStateVariables failed at setup: {e}')
            import traceback
            traceback.print_exc()
            return True

        # Eclipse state tracking is handled by FSM events in EXCITE_Fsw.py

        try:
            # Aggiorna la magnitudine della velocità angolare
            scStateMsg = DynModel.scObject.scStateOutMsg.read()
            omega_BN_B = scStateMsg.omega_BN_B
            self.omegaNorm = np.linalg.norm(omega_BN_B)

            # Aggiorna il fattore d'ombra eclissi (0.0 = eclissi totale, 1.0 = pieno sole)
            eclipseMsg = DynModel.eclipseObject.eclipseOutMsgs[0].read()
            self.shadowFactor = eclipseMsg.shadowFactor

            # Aggiorna lo stato di carica della batteria (SOC in %)
            batteryMsg = DynModel.battery.batPowerOutMsg.read()
            if batteryMsg.storageCapacity > 0:
                self.batterySOC = (batteryMsg.storageLevel / batteryMsg.storageCapacity) * 100.0
            else:
                self.batterySOC = 0.0

            # Aggiorna lo stato di contatto con la ground station
            gsAccessMsg = FswModel.PisaGroundStation.accessOutMsgs[0].read()
            self.gsHasAccess = (gsAccessMsg.hasAccess == 1)
            self.gsElevation = np.degrees(gsAccessMsg.elevation)

            # Il tracking delle eclissi è ora gestito dagli eventi FSM in EXCITE_Fsw.py
            # (gli eventi enterEclipse e exitEclipse aggiornano eclipseEntryTimes/eclipseExitTimes)

        except Exception as e:
            print("ERROR in updateFSM:", str(e))
            import traceback
            traceback.print_exc()

        return True  # Always return True so it doesn't block the condition

    def configure_initial_conditions(self):
        """
        Configura le condizioni iniziali orbitali e di assetto

        Imposta un'orbita eliosincrona a 500 km con RAAN ottimizzato per visibilità da Pisa.
        Condizioni iniziali di assetto: piccola rotazione e velocità angolare non nulla
        per testare il detumbling.
        """
        # Configura le condizioni iniziali della dinamica
        oe = orbitalMotion.ClassicElements()
        oe.a = 6878.137 * 1000  # m - semiasse maggiore (raggio Terra + 500 km altitudine)
        oe.e = 0.01  # eccentricità
        oe.i = 97.99 * macros.D2R  # rad - inclinazione per orbita eliosincrona
        oe.Omega = 70 * macros.D2R  # rad - RAAN ottimizzato per visibilità da Pisa
        oe.omega = 0 * macros.D2R  # rad - argomento del periasse
        oe.f = 0 * macros.D2R      # rad - anomalia vera

        DynModels = self.get_DynModel()
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - posizione iniziale
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - velocità iniziale
        DynModels.scObject.hub.sigma_BNInit = [[0.0], [0.2], [-0.3]]  # MRP assetto iniziale
        DynModels.scObject.hub.omega_BN_BInit = [[0.4], [-0.4], [0.5]]  # rad/s - velocità angolare iniziale

    def log_outputs(self):
        """Configura il logging di tutti gli output necessari per l'analisi del controllo d'assetto EXCITE"""
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        samplingTime = FswModel.processTasksTimeStep

        # IMPORTANTE: Aggiungi ephemConverter al DynProcess così gira dall'inizio
        # Questo garantisce che QUEST abbia dati di effemeridi del sole validi anche prima che navigationTask sia abilitato
        self.AddModelToTask(DynModel.taskName, FswModel.ephemConverter, 108)
        print("[SETUP] ephemConverter aggiunto al DynProcess (sempre attivo)")

        # 1. Errore di tracking dell'assetto (sigma_BR, omega_BR_B)
        self.msgRecList['attGuidMsg'] = FswModel.attGuidMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['attGuidMsg'])

        # 2. Assetto spacecraft e velocità angolari (omega_BN_B per analisi detumbling)
        self.msgRecList['simpleNavMsg'] = DynModel.simpleNavObject.attOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['simpleNavMsg'])

        # 3. Coppia comandata (da rateServo, prima del lpFilter)
        self.msgRecList['cmdTorqueMsg'] = FswModel.cmdTorqueMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['cmdTorqueMsg'])

        # 4. Coppie effettive e velocità delle RW (dalla dinamica)
        self.msgRecList['rwSpeedMsg'] = DynModel.rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['rwSpeedMsg'])

        # Individual RW torques (4 wheels)
        for i in range(4):
            msgName = f'rw{i+1}Msg'
            self.msgRecList[msgName] = DynModel.rwStateEffector.rwOutMsgs[i].recorder(samplingTime)
            self.AddModelToTask(DynModel.taskName, self.msgRecList[msgName])

        # 5. MTB commanded dipoles (from FSW gateway)
        self.msgRecList['mtbCmdMsg'] = FswModel.dipoleGatewayMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['mtbCmdMsg'])

        # 6. Total disturbance torques (individual disturbances)
        # Gravity gradient torque
        self.msgRecList['ggTorqueMsg'] = DynModel.ggEff.gravityGradientOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['ggTorqueMsg'])

        # SRP torque
        self.msgRecList['srpTorqueLog'] = DynModel.SRPEffector.logger("torqueExternalPntB_B", samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['srpTorqueLog'])

        # Drag torque
        self.msgRecList['dragTorqueLog'] = DynModel.dragEffector.logger("torqueExternalPntB_B", samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['dragTorqueLog'])

        # Magnetic disturbance torque
        self.msgRecList['magDistTorqueLog'] = DynModel.magDistTorque.logger("torqueExternalPntB_B", samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['magDistTorqueLog'])

        # 7. Battery state of charge
        self.msgRecList['batteryMsg'] = DynModel.battery.batPowerOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['batteryMsg'])

        # 8. MTB power consumption (3 magnetorquers)
        for i in range(3):
            msgName = f'mtbPower{i+1}Msg'
            self.msgRecList[msgName] = DynModel.mtbPowerList[i].nodePowerOutMsg.recorder(samplingTime)
            self.AddModelToTask(DynModel.taskName, self.msgRecList[msgName])

        # 9. Eclipse status (for solar panel power generation analysis)
        # The eclipse module has multiple output messages (one per celestial body)
        # Index 0 is typically Earth (the first body added after the sun)
        self.msgRecList['eclipseMsg'] = DynModel.eclipseObject.eclipseOutMsgs[0].recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['eclipseMsg'])

        # 10. Solar panel deployment angles (4 panels)
        for i in range(4):
            msgName = f'panel{i+1}DeployMsg'
            self.msgRecList[msgName] = DynModel.deployPanelList[i].hingedRigidBodyOutMsg.recorder(samplingTime)
            self.AddModelToTask(DynModel.taskName, self.msgRecList[msgName])

        # 11. Solar panel power generation (4 panels)
        for i in range(4):
            msgName = f'panel{i+1}PowerMsg'
            self.msgRecList[msgName] = DynModel.solarPanelList[i].nodePowerOutMsg.recorder(samplingTime)
            self.AddModelToTask(DynModel.taskName, self.msgRecList[msgName])

        # NOTE: GS access monitoring is handled by FSM events
        # No need to record GS messages (avoids uninitialized message warnings)

        # 12. IMU Custom sensor data (gyro measurements + bias)
        self.msgRecList['imuSensorMsg'] = DynModel.IMUCustom.sensorOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['imuSensorMsg'])

        self.msgRecList['imuBiasMsg'] = DynModel.IMUCustom.biasOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['imuBiasMsg'])

        # 13. QUEST attitude determination output
        self.msgRecList['questAttMsg'] = FswModel.questModule.navStateOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['questAttMsg'])

        # NOTE: sunlineEphem recorder removed - QUEST now uses simpleNavObject.attOutMsg.vehSunPntBdy directly

        # 13b. EphemConverter sun output (for QUEST debug - sun position in inertial)
        self.msgRecList['sunEphemMsg'] = FswModel.ephemConverter.ephemOutMsgs[0].recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['sunEphemMsg'])

        # 13c. TAM magnetometer output (for QUEST debug - magnetic field in body)
        self.msgRecList['tamMsg'] = DynModel.tamComm.tamOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['tamMsg'])

        # 13d. WMM magnetic field reference (for QUEST debug - magnetic field in inertial)
        self.msgRecList['magFieldMsg'] = DynModel.magModule.envOutMsgs[0].recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['magFieldMsg'])

        # 13e. SC position (for QUEST debug - to compute sun direction as QUEST does)
        self.msgRecList['scStateMsg'] = DynModel.scObject.scStateOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['scStateMsg'])

        # 14. SMEKF navigation filter output - DISABLED FOR DEBUG
        # self.msgRecList['smekfAttMsg'] = FswModel.smekfModule.navStateOutMsg.recorder(samplingTime)
        # self.AddModelToTask(DynModel.taskName, self.msgRecList['smekfAttMsg'])

        # self.msgRecList['smekfFiltMsg'] = FswModel.smekfModule.filtDataOutMsg.recorder(samplingTime)
        # self.AddModelToTask(DynModel.taskName, self.msgRecList['smekfFiltMsg'])

        # 15. Star Tracker output (for comparison with SMEKF)
        self.msgRecList['starTrackerMsg'] = DynModel.StarTracker.sensorOutMsg.recorder(samplingTime)
        self.AddModelToTask(DynModel.taskName, self.msgRecList['starTrackerMsg'])

        return

    def pull_outputs(self, showPlots):
        """Extract and plot all EXCITE attitude control data"""

        # Eclipse statistics analysis (from EXCITE_Analysis module) - DISABLED FOR QUEST DEBUG
        # eclipseStats = EXCITE_Analysis.analyze_eclipse_statistics(self)

        print("Processing simulation data and generating plots...")

        # Extract time array (convert to hours for readability)
        timeData = self.msgRecList['attGuidMsg'].times() * macros.NANO2HOUR

        # Prepare eclipse shading data for plots
        eclipseData = {
            'entry_times_hours': [t * macros.NANO2HOUR for t in self.eclipseEntryTimes],
            'exit_times_hours': [t * macros.NANO2HOUR for t in self.eclipseExitTimes]
        }

        # Extract all logged data
        # 1. Attitude tracking errors
        attGuidRec = self.msgRecList['attGuidMsg']
        sigma_BR = attGuidRec.sigma_BR
        omega_BR_B = attGuidRec.omega_BR_B

        # 2. Spacecraft body rates (for detumbling analysis)
        navRec = self.msgRecList['simpleNavMsg']
        omega_BN_B = navRec.omega_BN_B

        # 3. RW commanded vs actual torques
        # Get commanded torque before lpFilter (from rateServo output)
        cmdTorqueRec = self.msgRecList['cmdTorqueMsg']
        cmdTorque3D = cmdTorqueRec.torqueRequestBody  # [Nm] - 3D body torque

        # Convert 3D body torque to RW individual torques using Gs matrix
        # RW pyramid configuration (26.57° elevation, 45/135/225/315° azimuth)
        import Basilisk.utilities.RigidBodyKinematics as rbk
        rwElAngle = np.array([26.57, 26.57, 26.57, 26.57]) * macros.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * macros.D2R

        # Build Gs matrix (3x4) - each column is a RW spin axis
        Gs = np.zeros((3, 4))
        for i in range(4):
            gsHat = (rbk.Mi(-rwAzimuthAngle[i], 3).dot(rbk.Mi(rwElAngle[i], 2))).dot(np.array([1, 0, 0]))
            Gs[:, i] = gsHat

        # Convert 3D torque to RW torques: u_rw = Gs^+ @ u_body (pseudo-inverse)
        Gs_pinv = np.linalg.pinv(Gs)  # 4x3 pseudo-inverse
        rwTorqueCmd = np.zeros((len(timeData), 4))
        for j in range(len(timeData)):
            rwTorqueCmd[j, :] = Gs_pinv @ cmdTorque3D[j, :]

        rwTorqueActual = np.zeros((len(timeData), 4))
        for i in range(4):
            rwRec = self.msgRecList[f'rw{i+1}Msg']
            # u_erogated = u_current + frictionTorque (effective torque applied to dynamics)
            rwTorqueActual[:, i] = rwRec.u_current + rwRec.frictionTorque

        # 4. RW speeds
        rwSpeedRec = self.msgRecList['rwSpeedMsg']
        rwSpeeds = rwSpeedRec.wheelSpeeds  # [rad/s]

        # 5. MTB commanded dipoles (actual = commanded for MTB effector)
        mtbCmdRec = self.msgRecList['mtbCmdMsg']
        mtbDipoleCmd = mtbCmdRec.mtbDipoleCmds
        mtbDipoleActual = mtbDipoleCmd

        # 6. Total disturbance torques (sum of all environmental torques)
        ggRec = self.msgRecList['ggTorqueMsg']
        srpRec = self.msgRecList['srpTorqueLog']
        dragRec = self.msgRecList['dragTorqueLog']
        magDistRec = self.msgRecList['magDistTorqueLog']

        # 7. MTB power consumption
        mtbPower = np.zeros((len(timeData), 3))
        for i in range(3):
            mtbPowerRec = self.msgRecList[f'mtbPower{i+1}Msg']
            mtbPower[:, i] = -mtbPowerRec.netPower  # Negative because consumption is stored as negative

        # Sum all disturbance torques
        totalDisturbance = (ggRec.gravityGradientTorque_B +
                           srpRec.torqueExternalPntB_B +
                           dragRec.torqueExternalPntB_B +
                           magDistRec.torqueExternalPntB_B)

        # Individual disturbances for plotting
        gg_torque = ggRec.gravityGradientTorque_B
        srp_torque = srpRec.torqueExternalPntB_B
        drag_torque = dragRec.torqueExternalPntB_B
        magDist_torque = magDistRec.torqueExternalPntB_B

        # Calculate norms for easier visualization
        totalDistNorm = np.linalg.norm(totalDisturbance, axis=1)
        omegaNorm = np.linalg.norm(omega_BN_B, axis=1)

        # Clear previous plots
        BSK_plt.clear_all_plots()

        # Prepare phase transition times for plot markers
        if self.deploymentStartTime_ns is not None:
            deploymentStartTime = self.deploymentStartTime_ns * macros.NANO2HOUR
            print(f">>> Deployment started at: {deploymentStartTime:.2f} hours (event-driven)")
        else:
            deploymentStartTime = 12.0  # Fallback value if event didn't set it
            print(f">>> WARNING: Deployment start time not recorded, using default: {deploymentStartTime} hours")

        # Build phase markers from ACTUAL activation times
        phaseMarkers = {
            'deployment': deploymentStartTime,
        }

        # Add sun-safe pointing time (if recorded by FSM)
        if self.sunSafeActivationTime_ns is not None:
            phaseMarkers['sunPointing'] = self.sunSafeActivationTime_ns * macros.NANO2HOUR
            print(f">>> Sun-Safe activated at: {phaseMarkers['sunPointing']:.2f} hours (FSM-driven)")
        else:
            phaseMarkers['sunPointing'] = 14.0  # Fallback if not recorded
            print(f">>> WARNING: sunSafe time not recorded, using default: 14.0 hours")

        # Add GS command execution times (if recorded)
        if hasattr(self, 'commandExecutionTimes'):
            command_name_mapping = {
                'gs_communication': 'gsPointing',
                'reconfant_experiment': 'payloadA',
                'gpu_experiment': 'payloadB',
                'imaging': 'imaging'
            }

            for cmd_name, marker_name in command_name_mapping.items():
                if cmd_name in self.commandExecutionTimes:
                    phaseMarkers[marker_name] = self.commandExecutionTimes[cmd_name] * macros.NANO2HOUR
                    print(f">>> {marker_name} started at: {phaseMarkers[marker_name]:.2f} hours (event-driven)")

        # Use actual activation times (recorded by FSM events) if available
        if 'gsPointing' not in phaseMarkers:
            if self.gsPointingActivationTime_ns is not None:
                phaseMarkers['gsPointing'] = self.gsPointingActivationTime_ns * macros.NANO2HOUR
            else:
                phaseMarkers['gsPointing'] = 17.0

        if 'payloadA' not in phaseMarkers:
            if self.payloadAActivationTime_ns is not None:
                phaseMarkers['payloadA'] = self.payloadAActivationTime_ns * macros.NANO2HOUR
            else:
                phaseMarkers['payloadA'] = 19.0

        if 'payloadB' not in phaseMarkers:
            if self.payloadBActivationTime_ns is not None:
                phaseMarkers['payloadB'] = self.payloadBActivationTime_ns * macros.NANO2HOUR
            else:
                phaseMarkers['payloadB'] = 20.5

        if 'imaging' not in phaseMarkers:
            if self.imagingActivationTime_ns is not None:
                phaseMarkers['imaging'] = self.imagingActivationTime_ns * macros.NANO2HOUR
            else:
                phaseMarkers['imaging'] = 22.0

        # Convert MRP errors to Euler angle errors (needed for analysis)
        from Basilisk.utilities import RigidBodyKinematics as rbk
        euler_errors = np.zeros((len(sigma_BR), 3))  # [roll, pitch, yaw] error in radians
        for i in range(len(sigma_BR)):
            dcm = rbk.MRP2C(sigma_BR[i, :])
            euler = rbk.C2Euler321(dcm)  # Returns [yaw, pitch, roll]
            euler_errors[i, 0] = euler[2]  # roll error
            euler_errors[i, 1] = euler[1]  # pitch error
            euler_errors[i, 2] = euler[0]  # yaw error

        # Calculate settling time (needed for Figure 2 marker)

        # ============================================================================
        # Generate all plots using EXCITE_Plotting module
        # ============================================================================

        # FIGURE 1: Total Disturbance Torques - DISABLED (function missing)
        # EXCITE_Plotting.plot_total_disturbance_torques(timeData, totalDisturbance, totalDistNorm, phaseMarkers)

        # FIGURE 2: Attitude Tracking Errors (with settling time marker)
        EXCITE_Plotting.plot_attitude_tracking_errors(timeData, sigma_BR, phaseMarkers, eclipseData, settling_idx)

        # FIGURE 3: Angular Rate Errors and Body Rates - DISABLED (function missing)
        # EXCITE_Plotting.plot_angular_rates(timeData, omega_BR_B, omega_BN_B, omegaNorm, phaseMarkers)

        # FIGURE 4: RW Torques
        EXCITE_Plotting.plot_rw_torques(timeData, rwTorqueCmd, rwTorqueActual, phaseMarkers)

        # FIGURE 5: RW Speeds - REMOVED (replaced by FIGURE 11 with individual subplots)
        # EXCITE_Plotting.plot_rw_speeds(timeData, rwSpeeds, phaseMarkers, eclipseData)

        # FIGURE 6: MTB Dipoles
        EXCITE_Plotting.plot_mtb_dipoles(timeData, mtbDipoleCmd, mtbDipoleActual, phaseMarkers)

        # FIGURE 7: Disturbance Comparison - DISABLED (function missing)
        # EXCITE_Plotting.plot_disturbance_comparison(timeData, gg_torque, srp_torque, drag_torque,
        #                                              magDist_torque, totalDistNorm, phaseMarkers)

        # FIGURE 8: Battery SoC
        batteryRec = self.msgRecList['batteryMsg']
        storageLevel = batteryRec.storageLevel
        storageCapacity = batteryRec.storageCapacity


        # EXCITE_Plotting.plot_battery_soc(timeData, storageLevel, storageCapacity, phaseMarkers, eclipseData)  # DISABLED (function missing)

        # FIGURE 9: MTB Power Debug - DISABLED (function missing)
        # EXCITE_Plotting.plot_mtb_power_debug(timeData, mtbDipoleCmd, mtbPower, phaseMarkers)

        # FIGURE 10: Solar Panel Angles
        panelThetaList = []
        for i in range(4):
            panelDeployRec = self.msgRecList[f'panel{i+1}DeployMsg']
            panelThetaList.append(panelDeployRec.theta)
        EXCITE_Plotting.plot_solar_panel_angles(timeData, panelThetaList, phaseMarkers)

        # FIGURE 11: Reaction Wheel Angular Velocities
        EXCITE_Plotting.plot_rw_angular_velocities(timeData, rwSpeeds, phaseMarkers, eclipseData)

        # FIGURE 12: Solar Panel Power
        panelPowerTimesList = []
        panelPowerList = []
        for i in range(4):
            panelPowerRec = self.msgRecList[f'panel{i+1}PowerMsg']
            panelPowerTimesList.append(panelPowerRec.times() * macros.NANO2HOUR)
            panelPowerList.append(panelPowerRec.netPower)
        # EXCITE_Plotting.plot_solar_panel_power(panelPowerTimesList, panelPowerList, phaseMarkers, eclipseData)  # DISABLED (function missing)

        # ============================================================================
        # NAVIGATION FILTER PLOTS (QUEST + SMEKF)
        # ============================================================================

        # Extract true attitude from simpleNav (sigma_BN)
        sigma_BN_true = navRec.sigma_BN  # MRP attitude truth

        # Convert true MRP to Euler angles (roll, pitch, yaw)
        euler_true = np.zeros((len(sigma_BN_true), 3))
        for i in range(len(sigma_BN_true)):
            dcm = rbk.MRP2C(sigma_BN_true[i, :])
            euler = rbk.C2Euler321(dcm)  # Returns [yaw, pitch, roll]
            euler_true[i, 0] = euler[2]  # roll
            euler_true[i, 1] = euler[1]  # pitch
            euler_true[i, 2] = euler[0]  # yaw

        # Extract SMEKF estimated attitude - DISABLED FOR DEBUG
        # smekfAttRec = self.msgRecList['smekfAttMsg']
        # sigma_BN_smekf = smekfAttRec.sigma_BN

        # Convert SMEKF MRP to Euler angles - DISABLED FOR DEBUG
        # euler_smekf = np.zeros((len(sigma_BN_smekf), 3))
        # for i in range(len(sigma_BN_smekf)):
        #     dcm = rbk.MRP2C(sigma_BN_smekf[i, :])
        #     euler = rbk.C2Euler321(dcm)
        #     euler_smekf[i, 0] = euler[2]  # roll
        #     euler_smekf[i, 1] = euler[1]  # pitch
        #     euler_smekf[i, 2] = euler[0]  # yaw

        # Extract QUEST estimated attitude
        questAttRec = self.msgRecList['questAttMsg']
        sigma_BN_quest = questAttRec.sigma_BN

        # Calculate QUEST start time (used for debug and plots)
        questStartTime_hours = None
        if self.navigationTaskActivationTime_ns is not None:
            questStartTime_hours = self.navigationTaskActivationTime_ns * macros.NANO2HOUR

        # DEBUG: Check simpleNavObject sun direction (used by QUEST)
        simpleNav_vehSunPntBdy = navRec.vehSunPntBdy
        print("\n" + "="*70)
        print("DEBUG: QUEST Input Analysis")
        print("="*70)

        # Find first valid sun direction
        nonzero_idx = np.where(np.linalg.norm(simpleNav_vehSunPntBdy, axis=1) > 0.1)[0]
        if len(nonzero_idx) > 0:
            first_valid = nonzero_idx[0]
            print(f"\n1) Sun direction in BODY frame (from simpleNavObject):")
            print(f"   First valid at index {first_valid} (t={timeData[first_valid]:.2f}h)")
            print(f"   vehSunPntBdy = {simpleNav_vehSunPntBdy[first_valid]}")
            print(f"   |vehSunPntBdy| = {np.linalg.norm(simpleNav_vehSunPntBdy[first_valid]):.6f}")

            # Check at multiple time points after QUEST activation
            if questStartTime_hours is not None:
                # Check 1 hour AFTER activation to ensure QUEST has been running
                check_time_target = questStartTime_hours + 1.0
                check_idx = np.searchsorted(timeData, check_time_target)
            else:
                check_idx = len(timeData)//2
            if check_idx < len(simpleNav_vehSunPntBdy):
                check_time = timeData[check_idx]
                sun_B = simpleNav_vehSunPntBdy[check_idx]
                sigma_true = sigma_BN_true[check_idx]
                sigma_quest = sigma_BN_quest[check_idx]

                print(f"\n2) Detailed check at t={check_time:.2f}h:")
                print(f"   vehSunPntBdy (body) = {sun_B}")
                print(f"   |vehSunPntBdy| = {np.linalg.norm(sun_B):.6f}")
                print(f"   sigma_BN_true = {sigma_true}")
                print(f"   sigma_BN_quest = {sigma_quest}")
                print(f"   |sigma_quest| = {np.linalg.norm(sigma_quest):.2e}")

                # Compute what vehSunPntBdy SHOULD be if rotated correctly
                # If attitude is sigma_BN, then sun_B = DCM(sigma_BN) * sun_N
                # So sun_N = DCM(sigma_BN)^T * sun_B
                dcm_BN = rbk.MRP2C(sigma_true)
                sun_N_computed = dcm_BN.T @ sun_B  # Rotate body back to inertial
                print(f"\n3) Sun direction in INERTIAL (computed from body + true attitude):")
                print(f"   sun_N = DCM^T * sun_B = {sun_N_computed}")
                print(f"   |sun_N| = {np.linalg.norm(sun_N_computed):.6f}")

                # Check if body and inertial are the same (would cause identity output)
                diff = np.linalg.norm(sun_B - sun_N_computed)
                print(f"\n4) Sanity check:")
                print(f"   |sun_B - sun_N| = {diff:.6f}")
                if diff < 0.01:
                    print("   WARNING: Body and Inertial sun vectors are nearly identical!")
                    print("   This would cause QUEST to output identity attitude!")

        else:
            print("WARNING: No valid sun direction data found (all zeros)!")
            print("Check if simpleNavObject.sunStateInMsg is connected to SPICE sun message")
            print(f"First 5 values: {simpleNav_vehSunPntBdy[:5]}")

        # Check magnetometer data (second vector for QUEST)
        if 'tamMsg' in self.msgRecList and 'magFieldMsg' in self.msgRecList:
            tamRec = self.msgRecList['tamMsg']
            magFieldRec = self.msgRecList['magFieldMsg']
            tam_B = tamRec.tam_B  # Magnetic field in body frame [T]
            mag_N = magFieldRec.magField_N  # Magnetic field in inertial frame [T]

            print(f"\n6) Magnetometer data (second QUEST vector):")
            # Check at same time point as sun analysis
            if questStartTime_hours is not None:
                mag_idx = np.searchsorted(timeData, questStartTime_hours + 1.0)
            else:
                mag_idx = len(timeData)//2
            if mag_idx < len(tam_B):
                tam_body = tam_B[mag_idx]
                mag_inertial = mag_N[mag_idx]
                print(f"   At t={timeData[mag_idx]:.2f}h:")
                print(f"   TAM (body) = {tam_body} [T]")
                print(f"   |TAM| = {np.linalg.norm(tam_body):.2e} T")
                print(f"   WMM (inertial) = {mag_inertial} [T]")
                print(f"   |WMM| = {np.linalg.norm(mag_inertial):.2e} T")
                # Compare normalized vectors
                if np.linalg.norm(tam_body) > 1e-9 and np.linalg.norm(mag_inertial) > 1e-9:
                    tam_hat = tam_body / np.linalg.norm(tam_body)
                    mag_hat = mag_inertial / np.linalg.norm(mag_inertial)
                    diff_mag = np.linalg.norm(tam_hat - mag_hat)
                    print(f"   |mag_B_hat - mag_N_hat| = {diff_mag:.6f}")

        # Check ephemConverter output (sun position in inertial - used by QUEST internally)
        if 'sunEphemMsg' in self.msgRecList:
            sunEphemRec = self.msgRecList['sunEphemMsg']
            r_Sun_N = sunEphemRec.r_BdyZero_N  # Sun position in inertial [m]
            print(f"\n5) EphemConverter output (sun position for QUEST):")
            # Find first non-zero
            nonzero_ephem = np.where(np.linalg.norm(r_Sun_N, axis=1) > 1e6)[0]
            if len(nonzero_ephem) > 0:
                idx = nonzero_ephem[0]
                print(f"   First valid at t={timeData[idx]:.2f}h:")
                print(f"   r_Sun_N = {r_Sun_N[idx]} [m]")
                print(f"   |r_Sun_N| = {np.linalg.norm(r_Sun_N[idx]):.3e} m ({np.linalg.norm(r_Sun_N[idx])/1e9:.3f} million km)")
                # Check at quest activation
                if questStartTime_hours is not None:
                    quest_idx = np.searchsorted(timeData, questStartTime_hours)
                    if quest_idx < len(r_Sun_N):
                        print(f"   At QUEST activation (t={questStartTime_hours:.2f}h):")
                        print(f"   r_Sun_N = {r_Sun_N[quest_idx]} [m]")
                        print(f"   |r_Sun_N| = {np.linalg.norm(r_Sun_N[quest_idx]):.3e} m")
            else:
                print("   WARNING: No valid sun ephemeris data!")
                print("   ephemConverter may not be running. Check if navigationTask is enabled.")

        print("="*70 + "\n")

        # Convert QUEST MRP to Euler angles
        euler_quest = np.zeros((len(sigma_BN_quest), 3))
        for i in range(len(sigma_BN_quest)):
            dcm = rbk.MRP2C(sigma_BN_quest[i, :])
            euler = rbk.C2Euler321(dcm)
            euler_quest[i, 0] = euler[2]  # roll
            euler_quest[i, 1] = euler[1]  # pitch
            euler_quest[i, 2] = euler[0]  # yaw

        # Extract angular velocities - DISABLED FOR DEBUG
        # omega_true = omega_BN_B  # Already extracted above (from simpleNav)
        # omega_smekf = smekfAttRec.omega_BN_B

        # Extract gyro bias: true (from IMU) and estimated (from SMEKF) - DISABLED FOR DEBUG
        # imuBiasRec = self.msgRecList['imuBiasMsg']
        # bias_true = imuBiasRec.bias

        # smekfFiltRec = self.msgRecList['smekfFiltMsg']
        # # SMEKF state vector is [delta_theta(3), bias(3)] - bias is indices 3:6
        # smekf_state = smekfFiltRec.state
        # bias_smekf = smekf_state[:, 3:6] if smekf_state.shape[1] >= 6 else np.zeros_like(bias_true)

        # FIGURE 13: SMEKF Attitude Estimation Error - DISABLED FOR DEBUG
        # EXCITE_Plotting.plot_attitude_estimation_error_smekf(
        #     timeData, euler_true, euler_smekf, phaseMarkers, figNum=13
        # )

        # FIGURE 14: QUEST Attitude Estimation Error in MRP (from navigationTask activation)
        if questStartTime_hours is not None:
            print(f">>> QUEST plot starting at: {questStartTime_hours:.2f} hours (navigationTask activation)")
        else:
            print(f">>> WARNING: navigationTask activation time not recorded, showing full timeline")

        # Pass MRP directly (sigma_BN_true and sigma_BN_quest) instead of Euler angles
        # Include phaseMarkers for attitude mode transitions and eclipseData for eclipse shading
        EXCITE_Plotting.plot_attitude_estimation_error_quest(
            timeData, sigma_BN_true, sigma_BN_quest, phaseMarkers, figNum=14,
            startTime_hours=questStartTime_hours, eclipseData=eclipseData
        )

        # FIGURE 15: QUEST Estimated Attitude Angles (Roll, Pitch, Yaw)
        EXCITE_Plotting.plot_quest_attitude_angles(
            timeData, sigma_BN_quest, phaseMarkers, figNum=15,
            startTime_hours=questStartTime_hours, eclipseData=eclipseData
        )

        # FIGURE 16: SMEKF Angular Velocity Estimation Error - DISABLED FOR DEBUG
        # EXCITE_Plotting.plot_omega_estimation_error_smekf(
        #     timeData, omega_true, omega_smekf, phaseMarkers, figNum=16
        # )

        # FIGURE 16: Gyro Bias Estimation (True vs SMEKF) - DISABLED FOR DEBUG
        # EXCITE_Plotting.plot_gyro_bias_estimation(
        #     timeData, bias_true, bias_smekf, phaseMarkers, figNum=16
        # )

        print("Navigation filter plots generated (QUEST only - SMEKF disabled for debug)")

        # ============================================================================
        # MTB Power Analysis (from EXCITE_Analysis module) - DISABLED FOR QUEST DEBUG
        # ============================================================================
        # mtbPowerStats = EXCITE_Analysis.analyze_mtb_power(timeData, mtbDipoleCmd, mtbPower)

        # ============================================================================
        # Oscillation Analysis (from EXCITE_Analysis module) - DISABLED FOR QUEST DEBUG
        # ============================================================================
        # euler_errors_gs = euler_errors[settling_idx:gs_end_idx, :]
        # time_gs = timeData[settling_idx:gs_end_idx]
        # oscillationResults = EXCITE_Analysis.analyze_oscillations(euler_errors_gs, time_gs)

        # ============================================================================
        # RMS Metrics Analysis (from EXCITE_Analysis module) - DISABLED FOR QUEST DEBUG
        # ============================================================================
        # omega_BR_gs = omega_BR_B[settling_idx:gs_end_idx, :]
        # rmsResults = EXCITE_Analysis.analyze_rms_metrics(euler_errors_gs, omega_BR_gs, time_gs)

        # ============================================================================
        # Show or save plots
        # ============================================================================
        figureList = {}
        if showPlots:
            print("Displaying plots...")
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["totalDisturbance", "attitudeError", "angularRates",
                          "rwTorques", "rwSpeeds", "mtbDipoles", "disturbanceComparison",
                          "batterySoC", "mtbPowerDebug", "solarPanelAngles", "solarPanelRates", "solarPanelPower"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)
            print(f"Plots saved with prefix: {fileName}")

        print("Data processing complete.\n")

        return figureList


def runScenario(scenario):
    """
    Execute EXCITE mission scenario with FSM-driven autonomous operation:

    Mission phases (event-driven transitions):
    - Phase 1 (0-~12h): Detumbling using B-dot controller with magnetorquers
                        (ends when omega < 1e-2 rad/s)
    - Phase 2 (~12h-60s): Solar panel deployment with continued detumbling
    - Phase 3 (post-deploy, 1h): Post-deployment detumbling stabilization
                                 (ends when time > deployEnd+1h AND omega < 1e-2)
    - Phase 4 (14-17h): Sun-safe pointing with RW + MTB momentum management
    - Phase 5 (17-19h): Ground station pointing (S-band Y-axis) with RW + MTB
    - Phase 6 (19-20.5h): Payload Mode A - ReconfANT experiment (X-axis pointing)
    - Phase 7 (20.5-22h): Payload Mode B - GPU IoT ISM experiment (-X-axis pointing)
    - Phase 8 (22-24h): Imaging Mode - Nadir pointing for IM200 camera (Y-axis pointing)

    Eclipse management: Automatically disables guidance during eclipse (shadowFactor < 0.1),
                        restores previous mode when exiting eclipse (shadowFactor > 0.9).
    """
    simulationTime = macros.hour2nano(25)  # 25 hours for full mission

    print("\n" + "="*70)
    print("EXCITE ATTITUDE CONTROL SIMULATION - FSM EVENT-DRIVEN")
    print("="*70)
    print(f"Total simulation time: 24 hours")
    print(f"Mission phases (automatic FSM event transitions):")
    print(f"  1. Detumbling (ends when omega < 1e-2 rad/s)")
    print(f"  2. Solar panel deployment (60 seconds, detumbling continues)")
    print(f"  3. Post-deployment detumbling (1h after deployment, omega < 1e-2)")
    print(f"  4. Sun-safe pointing (14-17h)")
    print(f"  5. GS pointing Y-axis/S-band (17-19h)")
    print(f"  6. Payload Mode A X-axis/ReconfANT (19-20.5h)")
    print(f"  7. Payload Mode B -X-axis/GPU IoT (20.5-22h)")
    print(f"  8. Imaging Mode Y-axis/nadir (22-24h)")
    print(f"  Eclipse: Auto-disable guidance (shadow < 0.1), restore on exit (shadow > 0.9)")
    print("="*70 + "\n")

    scenario.InitializeSimulation()

    # Enable progress bar
    scenario.showProgressBar = True

    print(">>> INITIAL STATE: DETUMBLING MODE")
    print("    - B-dot controller active")
    print("    - Magnetorquers for rate damping")
    print("    - Solar panels locked at stowed position")
    print("    - RWs inactive")
    print("    - FSM will transition to deployment when omega < 1e-2 rad/s\n")

    # FSM events will handle all task activation automatically
    # modeRequest is set to 'detumbling' in __init__, triggering the activateDetumbling event

    print(">>> STARTING FSM-CONTROLLED MISSION")
    print("="*70)
    print("    All transitions handled by FSM events")
    print("    - Physical constraints: omega, shadowFactor")
    print("    - Time-based transitions: 14h, 17h, 19h, 20.5h, 22h")
    print("    - Eclipse detection and mode restoration")
    print("="*70 + "\n")

    # Run complete mission - FSM events handle all transitions
    scenario.ConfigureStopTime(simulationTime)
    scenario.ExecuteSimulation()

    print("\n>>> SIMULATION COMPLETE at t = 24h")
    print("="*70)
    print("All FSM transitions executed successfully")
    print("Eclipse management handled automatically")
    print("="*70 + "\n")

    return


def run(showPlots):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    scenario = scenario_EXCITE()
    runScenario(scenario)
    figureList = scenario.pull_outputs(showPlots)
    return figureList

if __name__ == "__main__":
    run(True)