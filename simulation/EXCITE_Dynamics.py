"""
EXCITE_Dynamics.py - Modello dinamico del satellite EXCITE

Questo modulo implementa tutta la fisica e l'ambiente della simulazione:
- Dinamica dello spacecraft (massa, inerzia, equazioni del moto)
- Attuatori: 4 Ruote di Reazione (RW), 3 Magnetorquer (MTB), Propulsore chimico H2O2
- Sensori: Star Tracker, CSS, Magnetometro (TAM), IMU custom, GPS/GNSS
- Ambiente spaziale: Gravità (Terra/Sole/Luna), Gradiente gravitazionale, Drag atmosferico,
  Pressione radiazione solare (SRP), Disturbo magnetico residuo, Eclissi
- Sottosistema elettrico: 4 Pannelli solari dispiegabili, Batteria, Power sinks

Classe principale:
- BSKDynamicModels: Configura e inizializza tutti i moduli dinamici
"""

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
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]

class BSKDynamicModels():
    """
    Classe che gestisce tutti i modelli dinamici della simulazione EXCITE

    Inizializza e configura:
    - Spacecraft hub (massa, inerzia, geometria)
    - Attuatori (RW, MTB, thruster)
    - Sensori (Star Tracker, IMU, TAM, CSS)
    - Ambiente (gravità, atmosfera, campo magnetico, eclissi)
    - Sottosistema elettrico (pannelli solari, batteria)
    """
    def __init__(self, SimBase, dynRate):
        """
        Inizializza i modelli dinamici

        Args:
            SimBase: Riferimento alla simulazione base
            dynRate: Frequenza di aggiornamento dinamica [s]
        """
        # Memorizza riferimento SimBase per uso in altri metodi
        self.SimBase = SimBase

        # Definisce variabili di classe vuote (inizializzate dopo)
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None
        self.RW1 = None
        self.RW2 = None
        self.RW3 = None
        self.RW4 = None

        # Definisce nome processo, nome task e time-step
        self.processName = SimBase.DynamicsProcessName

        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Crea il task principale della dinamica
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # ===== Istanzia i moduli dinamici come oggetti =====

        # Spacecraft e ambiente
        self.scObject = spacecraft.Spacecraft()  # Oggetto centrale dello spacecraft
        self.gravFactory = simIncludeGravBody.gravBodyFactory()  # Factory per corpi gravitazionali
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()  # Convertitore effemeridi Terra
        self.atmosphereObject = exponentialAtmosphere.ExponentialAtmosphere()  # Modello atmosfera esponenziale
        self.magModule = magneticFieldWMM.MagneticFieldWMM()  # Campo magnetico WMM (World Magnetic Model)
        self.ggEff = GravityGradientEffector.GravityGradientEffector()  # Effettore gradiente gravitazionale
        self.SRPEffector = facetSRPDynamicEffector.FacetSRPDynamicEffector()  # Pressione radiazione solare (SRP)
        self.dragEffector = facetDragDynamicEffector.FacetDragDynamicEffector()  # Drag atmosferico
        self.magDistTorque = magneticDisturbanceTorque.MagneticDisturbanceTorque()  # Disturbo magnetico residuo
        self.eclipseObject = eclipse.Eclipse()  # Calcolo eclissi
        self.simpleNavObject = simpleNav.SimpleNav()  # Navigazione semplificata (truth data)
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()  # Forze/coppie esterne aggiuntive

        # Attuatori
        self.rwFactory = simIncludeRW.rwFactory()  # Factory per ruote di reazione
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()  # Effettore RW
        self.thrusterStateEffector = thrusterStateEffector.ThrusterStateEffector()  # Effettore propulsore
        self.thFactory = None  # Inizializzato in SetThrusterStateEffector()
        self.mtbEffector = MtbEffector.MtbEffector()  # Effettore magnetorquer (MTB)

        # Sensori
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()  # Sensori solari grossolani (CSS)
        self.StarTracker = starTracker.StarTracker()  # Star Tracker (alta precisione)
        self.IMU = imuSensor.ImuSensor()  # IMU standard (non usato, mantenuto per compatibilità)
        self.IMUCustom = imuSensorCustom.imuSensorCustom()  # IMU custom con modello deriva bias
        self.TAM = magnetometer.Magnetometer()  # Magnetometro (TAM)
        self.tamComm = tamComm.tamComm()  # Convertitore frame TAM (sensore -> body)
        self.cameraIM200 = camera.Camera()  # Camera IM200 ClydeSpace

        # Sottosistema elettrico
        self.battery = simpleBattery.SimpleBattery()  # Batteria
        self.powersink = simplePowerSink.SimplePowerSink()  # Power sink (carichi)

        # Crea lista di 4 pannelli solari dispiegabili
        self.solarPanelList = []
        for i in range(4):
            panel = simpleSolarPanel.SimpleSolarPanel()
            panel.ModelTag = f"solarPanel_{i+1}"
            self.solarPanelList.append(panel)

        # Crea lista di 4 effettori di dispiegamento pannelli
        self.deployPanelList = []
        for i in range(4):
            deploy_panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
            deploy_panel.ModelTag = f"solarPaneldeploy_{i+1}"
            self.deployPanelList.append(deploy_panel)

        # Crea lista di 4 moduli potenza RW (uno per ruota)
        self.rwPowerList = []
        for i in range(4):
            powerRW = ReactionWheelPower.ReactionWheelPower()
            powerRW.ModelTag = f"RW{i+1}_Power"
            self.rwPowerList.append(powerRW)

        # Crea lista di 3 moduli potenza MTB (uno per asse)
        self.mtbPowerList = []
        for i in range(3):
            powerMTB = MtbPower.MtbPower()
            powerMTB.ModelTag = f"MTB{i+1}_Power"
            self.mtbPowerList.append(powerMTB)

        # Inizializza tutti i moduli e scrive i messaggi iniziali
        self.InitAllDynObjects()

        # Aggiunge i modelli al task
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 201)  # DEVE eseguire PRIMA per fornire le posizioni dei pianeti
        SimBase.AddModelToTask(self.taskName, self.scObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.atmosphereObject, 199)
        SimBase.AddModelToTask(self.taskName, self.magModule, 198)  # Esegue DOPO SPICE per avere posizioni valide dei pianeti
        SimBase.AddModelToTask(self.taskName, self.ggEff, 197)
        SimBase.AddModelToTask(self.taskName, self.SRPEffector, 197)
        SimBase.AddModelToTask(self.taskName, self.dragEffector, 197)
        SimBase.AddModelToTask(self.taskName, self.magDistTorque, 197)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 107)  # DEVE eseguire PRIMA dei CSS
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.mtbEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.thrusterStateEffector, 301)  # Propulsore chimico H2O2
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, 108)
        SimBase.AddModelToTask(self.taskName, self.StarTracker, 108)
        # SimBase.AddModelToTask(self.taskName, self.IMU, 108)  # Sostituito da IMUCustom
        SimBase.AddModelToTask(self.taskName, self.IMUCustom, 108)  # IMU custom con modello di deriva del bias
        SimBase.AddModelToTask(self.taskName, self.TAM, 196)  # TAM esegue DOPO magModule (magModule=198, TAM=196)
        SimBase.AddModelToTask(self.taskName, self.tamComm, 195)  # tamComm esegue DOPO TAM per convertire il frame del sensore in frame body
        SimBase.AddModelToTask(self.taskName, self.cameraIM200, 108)  # Sensore camera
        SimBase.AddModelToTask(self.taskName, self.battery, 195)
        SimBase.AddModelToTask(self.taskName, self.powersink, 195)
        # Aggiunge tutti i 4 effettori di dispiegamento dei pannelli solari PRIMA (priorità più alta) in modo che scrivano i messaggi prima che i pannelli solari li leggano
        for deploy_panel in self.deployPanelList:
            SimBase.AddModelToTask(self.taskName, deploy_panel, 196)  # DEVE eseguire PRIMA dei pannelli solari
        # Aggiunge tutti i 4 pannelli solari al task principale (generazione di potenza disabilitata inizialmente via efficiency=0)
        for panel in self.solarPanelList:
            SimBase.AddModelToTask(self.taskName, panel, 194)  # Esegue DOPO che i pannelli di dispiegamento hanno scritto i loro messaggi
        print("[DYNAMICS] Moduli pannelli solari aggiunti al task (dispiegamento abilitato)")
        # Add all 4 RW power modules to task
        for powerRW in self.rwPowerList:
            SimBase.AddModelToTask(self.taskName, powerRW, 195)
        # Add all 3 MTB power modules to task
        for powerMTB in self.mtbPowerList:
            SimBase.AddModelToTask(self.taskName, powerMTB, 195)

        # The RWs Faults are here commented for the moment. They will be added in the future, assessing the task of the FDIR creation.

        # SimBase.createNewEvent("addOneTimeRWFault", self.processTasksTimeStep, True,
        #     ["self.TotalSim.CurrentNanos>=self.oneTimeFaultTime and self.oneTimeRWFaultFlag==1"],
        #     ["self.DynModels.AddRWFault('friction',0.05,1, self.TotalSim.CurrentNanos)", "self.oneTimeRWFaultFlag=0"])


        # SimBase.createNewEvent("addRepeatedRWFault", self.processTasksTimeStep, True,
        #     ["self.repeatRWFaultFlag==1"],
        #     ["self.DynModels.PeriodicRWFault(1./3000,'friction',0.005,1, self.TotalSim.CurrentNanos)", "self.setEventActivity('addRepeatedRWFault',True)"])

    def SetSpacecraftHub(self):
        """
        Definisce la massa e l'inerzia dello spacecraft.
        """
        self.scObject.ModelTag = "bsk-Sat"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [0.3280, -0.006412, -0.003930,
                    -0.006412, 0.3363, -0.002078,
                    -0.003930, -0.002078, 0.3494]
        
        self.scObject.hub.mHub = 14.0 # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetIntegrator(self):
        """
        Imposta l'integratore RKF45 per la simulazione.
        """
        self.integrator = svIntegrators.svIntegratorRKF45(self.scObject)
        self.scObject.setIntegrator(self.integrator)

    def SetGravityBodies(self):
        """
        Imposta i corpi gravitazionali (Terra, Sole, Luna) per la simulazione.
        """
        # Clear existing gravity bodies
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        
        # Create gravity bodies
        self.gravBodies = self.gravFactory.createBodies(['earth', 'sun', 'moon'])
        self.gravBodies['earth'].isCentralBody = True
        self.gravBodies['earth'].useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S.txt', 10)
        
        # Add gravity bodies to spacecraft
        self.gravFactory.addBodiesTo(self.scObject)
        
        # Create SPICE interface
        time = self.SimBase.timeInitString
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', time, epochInMsg=True)
        self.gravFactory.spiceObject.zeroBase = 'earth'
        
        # Define body names
        self.sun = 0
        self.earth = 1
        self.moon = 2

        # Connect epoch message
        self.epochMsg = self.gravFactory.epochMsg

    def SetEclipseObject(self):
        """
        Imposta l'oggetto Eclipse per modellare le eclissi (ombre).
        """
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])
        self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[self.moon])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetWMMObject(self):
        """
        Imposta il modello magnetico mondiale (WMM) per il campo magnetico terrestre.
        """
        self.magModule.ModelTag = "WMM"
        self.magModule.dataPath = bskPath + '/supportData/MagneticField/'
        self.magModule.epochInMsg.subscribeTo(self.gravFactory.epochMsg)
        self.magModule.addSpacecraftToModel(self.scObject.scStateOutMsg)  # This connects stateInMsg automatically

    def SetAtmosphereObject(self):
        """
        Imposta il modello atmosferico esponenziale.
        """
        self.atmosphereObject.ModelTag = "ExponentialAtmosphere"
        self.atmosphereObject.planetRadius = 6378136.6  # meters
        self.atmosphereObject.scaleHeight = 8500.0  # meters
        self.atmosphereObject.baseDensity = 1.225  # kg/m^3
        self.atmosphereObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetExternalForceTorqueObject(self):
        """Imposta l'oggetto per forze e coppie esterne."""
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetGravityGradientEffector(self):
        """Modella gli effetti del Gravity Gradient Effector sullo spacecraft EXCITE"""
        self.ggEff.ModelTag = "gravityGradient"
        self.ggEff.addPlanetName(self.gravBodies['earth'].planetName)  # Sorgente primaria di gradiente di gravità
        self.ggEff.addPlanetName(self.gravBodies['moon'].planetName)   # Perturbazione secondaria
        self.ggEff.addPlanetName(self.gravBodies['sun'].planetName)    # Perturbazione terziaria
        # Disturbo da gradiente di gravità ABILITATO (~222 nNm a 500km)
        self.scObject.addDynamicEffector(self.ggEff)

    def SetSRPEffector(self):
        """Modella gli effetti della Pressione di Radiazione Solare (SRP) sullo spacecraft EXCITE"""
        self.ModelTag = "SRPEffector"
        self.SRPEffector.setNumFacets(14)
        self.SRPEffector.setNumArticulatedFacets(0) # Impostato a 0 per ora, sarà cambiato a 4 quando verrà simulato
                                                    # il dispiegamento dei pannelli solari
        self.width = 0.2  # m 
        self.length = 0.2 # m
        self.height = 0.3 # m 
        self.panel_width = 0.05 # m
        self.body_side_area = self.length * self.height       # m^2 - facce laterali rettangolari del CubeSat
        self.body_top_bottom_area = self.length * self.width  # m^2 - facce superiore e inferiore quadrate del CubeSat

        self.facetAreaList = [self.body_top_bottom_area,  # +Z - Area pannello direzione asse Z positivo del CubeSat ("pannello body mounted v1:1" nel file CAD)
                              self.body_side_area,        # +X - Area pannello direzione asse X positivo del Cubesat ("lateral panel 3 v1:1" nel file CAD)
                              self.body_side_area,        # +Y - Area pannello direzione asse Y positivo del Cubesat ("lateral panel 2 v1:1" nel file CAD)
                              self.body_side_area,        # -Y - Area pannello direzione asse Y negativo del Cubesat ("lateral panel 1 v1:1" nel file CAD)
                              self.body_side_area,        # +X - Area pannello direzione asse X negativo del CubeSat ("lateral panel 4 v1:1" nel file CAD)
                              self.body_top_bottom_area,  # -Z - Area pannello direzione asse Z negativo del Cubesat (non specificato nel file CAD, c'è un foro per il propulsore)
                              self.body_side_area,        # +Z - Direzione asse Z positivo della prima area pannello solare del CubeSat ("deployable solar panel v1:1" nel file CAD)
                              self.body_side_area,        # -Z - Direzione asse Z negativo della prima area pannello solare del CubeSat ("deployable solar panel v1:1" nel file CAD)
                              self.body_side_area,        # +Z - Direzione asse Z positivo della seconda area pannello del CubeSat ("deployable solar panel v1:2" nel file CAD)
                              self.body_side_area,        # -Z - Direzione asse Z negativo della seconda area pannello del CubeSat ("deployable solar panel v1:2" nel file CAD)
                              self.body_side_area,        # +Z - Direzione asse Z positivo della terza area pannello solare del CubeSat ("deployable solar panel v1:3" nel file CAD)
                              self.body_side_area,        # -Z - Direzione asse Z negativo della terza area pannello solare del CubeSat ("deployable solar panel v1:3" nel file CAD)
                              self.body_side_area,        # +Z - Direzione asse Z positivo della quarta area pannello solare del CubeSat ("deployable solar panel v1:4" nel file CAD)
                              self.body_side_area,        # -Z - Direzione asse Z negativo della quarta area pannello solare del CubeSat ("deployable solar panel v1:4" nel file CAD)
                              self.body_side_area]        # -Z - Direzione asse Z negativo della quarta area pannello solare del CubeSat ("deployable solar panel v1:4" nel file CAD)

       # I dati seguenti devono essere controllati di nuovo: in particolare dobbiamo valutare a cosa servono queste matrici e cosa cambia tra
       # i lati superiore e inferiore dei pannelli solari

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
        self.facetRotHat_FList = [np.array([1.0, 0.0, 0.0]) for _ in range(14)]  # Asse di rotazione per faccette articolate (non usato per faccette fisse)

        # Posizione del centro di pressione di ogni faccetta rispetto al frame del corpo. Sono valori approssimati, puntano al centro della faccetta.
        # Sono nello stesso ordine dei pannelli presentati sopra.

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
        # Disturbo Pressione Radiazione Solare ABILITATO (~48 nNm, 14 faccette + pannelli dispiegabili)
        self.scObject.addDynamicEffector(self.SRPEffector)

    def SetMagDistEffector(self):
        """Modella gli effetti del Disturbo Magnetico sullo spacecraft EXCITE."""
        self.magDistTorque.ModelTag = "magneticDisturbanceTorque"
        self.spacecraft_dipole = [0.035, 0.035, 0.02]
        self.magDistTorque.spacecraftMagneticDipole = [[self.spacecraft_dipole[0]], [self.spacecraft_dipole[1]], [self.spacecraft_dipole[2]]]
        self.magDistTorque.magFieldInMsg.subscribeTo(self.magModule.envOutMsgs[0])
        # Disturbo magnetico ABILITATO (~3057 nNm - disturbo DOMINANTE con IGRF)
        self.scObject.addDynamicEffector(self.magDistTorque)

    def SetDragDistEffector(self):
        """Modella gli effetti della Resistenza Atmosferica (Drag) sullo spacecraft EXCITE"""
        self.dragEffector.ModelTag = "dragEffector"

        self.dragEffector.atmoDensInMsg.subscribeTo(self.atmosphereObject.envOutMsgs[0])

        # useremo qui la stessa geometria delle faccette usata per SRP, ma con coefficienti di resistenza invece di quelli solari

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
        # Disturbo resistenza atmosferica ABILITATO con ExponentialAtmosphere
        # Passato da MSIS a ExponentialAtmosphere per evitare instabilità numeriche
        # A 550 km, coppia di resistenza ~0.02 nNm (minore rispetto a magnetica ~3057 nNm)
        self.scObject.addDynamicEffector(self.dragEffector)

    def SetSimpleNavObject(self):
        """Imposta l'oggetto sensore di navigazione."""
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        # Connette lo stato del sole per il calcolo della direzione del sole (vehSunPntBdy)
        # Questo abilita simpleNavObject a calcolare la direzione del sole nel frame del corpo con rumore
        self.simpleNavObject.sunStateInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[0])  # Sole

        print("[NAVIGATION SETUP] Configurazione errori di navigazione realistici:")

        # ERRORI NAV REALISTICI - GPS/GNSS + Star Tracker + IMU + CSS
        # Limiti 3-sigma per simpleNavObject (limiti random walk)
        pos_3sigma = 5.0      # m - Accuratezza posizione GPS/GNSS in LEO
        vel_3sigma = 0.1      # m/s - Accuratezza velocità GPS/GNSS
        att_3sigma_deg = 0.07  # deg - Accuratezza Star Tracker (alta qualità)
        rate_3sigma = 0.05 * mc.D2R  # 0.05 deg/s - Accuratezza IMU/giroscopio
        sun_3sigma_deg = 1.0  # deg - Accuratezza direzione sole CSS

        # Converte in radianti dove necessario
        att_3sigma_rad = att_3sigma_deg * mc.D2R
        sun_3sigma_rad = sun_3sigma_deg * mc.D2R

        # Calcola deviazioni standard (1-sigma = 3-sigma / 3)
        pos_std = pos_3sigma / 3.0         # m
        vel_std = vel_3sigma / 3.0         # m/s
        att_std = att_3sigma_rad / 3.0     # rad
        rate_std = rate_3sigma / 3.0       # rad/s
        sun_std = sun_3sigma_rad / 3.0     # rad

        # PMatrix: Matrice di covarianza (deviazioni standard 1-sigma)
        self.simpleNavObject.PMatrix = np.diag([
            pos_std, pos_std, pos_std,       # ~1.67 m (GPS/GNSS)
            vel_std, vel_std, vel_std,       # ~0.033 m/s (GPS/GNSS)
            att_std, att_std, att_std,       # ~0.033 deg (Star Tracker)
            rate_std, rate_std, rate_std,    # ~0.017 deg/s (IMU)
            sun_std, sun_std, sun_std,       # ~0.33 deg (CSS)
            0.0, 0.0, 0.0
        ])

        # walkBounds: Limiti random walk (valori 3-sigma)
        self.simpleNavObject.walkBounds = [
            pos_3sigma, pos_3sigma, pos_3sigma,        # 5.0 m
            vel_3sigma, vel_3sigma, vel_3sigma,        # 0.1 m/s
            att_3sigma_rad, att_3sigma_rad, att_3sigma_rad,  # 0.1 deg
            rate_3sigma, rate_3sigma, rate_3sigma,     # 0.05 deg/s
            sun_3sigma_rad, sun_3sigma_rad, sun_3sigma_rad,  # 1.0 deg
            0.0, 0.0, 0.0
        ]

        # Stampa riepilogo configurazione
        print(f"  Position (3σ): {pos_3sigma} m (GPS/GNSS)")
        print(f"  Velocity (3σ): {vel_3sigma} m/s (GPS/GNSS)")
        print(f"  Attitude (3σ): {att_3sigma_deg} deg (Star Tracker)")
        print(f"  Rate (3σ): {rate_3sigma*mc.R2D:.3f} deg/s (IMU)")
        print(f"  Sun direction (3σ): {sun_3sigma_deg} deg (CSS)")

    def SetReactionWheelDynEffector(self):
        """Imposta i 4 dispositivi reaction wheel (CW0162 da CubeSpace)"""
        # specifica capacità momento angolare RW
        maxRWMomentum = 16.2e-3  # Nms
        rwMaxSpeed = 10000.0 #8000.0  # RPM
        rwInertia = 25997e-9 #9.3805e-4 #1.946e-4  # kg*m^2
        rwMass = 0. #0.144 # kg
        rwMaxTorque = 7.0e-3 #37.0e-3 #20.0e-3  # N*m
        # Definisce piramide RW ortogonale
        # -- Direzioni di puntamento
        rwElAngle = np.array([26.57, 26.57, 26.57, 26.57])*mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
        # il vettore posizione di ogni reaction wheel qui sotto non è corretto affatto: dobbiamo controllare il datasheet di CubeSpace per valutare
        # dove ogni reaction wheel è posizionata all'interno del box RW.
        rwPosVector = [[0.023, 0.023, 0.046],
                       [0.023, -0.023, 0.046],
                       [-0.023, -0.023, 0.046],
                       [-0.023, 0.023, 0.046]
                       ]
        
            # Crea la configurazione a piramide di 4 ruote
        numRW = 4
        for rw in range(numRW):
            # Calcola il vettore direzione asse di spin (gsHat) per ogni ruota
            # Usa matrici di rotazione per convertire angoli di elevazione e azimuth in vettore unitario 3D
            # IMPORTANTE: da riguardare
            gsHat = (rbk.Mi(-rwAzimuthAngle[rw], 3).dot(rbk.Mi(rwElAngle[rw], 2))).dot(np.array([1, 0, 0]))
            #print(f"  Allocation Matrix gsHat = {gsHat} ")
                    # Crea reaction wheel individuale con configurazione personalizzata usando specifiche ESATTE CW0162
            self.rwFactory.create('custom',
                            gsHat,  # Vettore direzione asse di spin nel frame del corpo
                            Omega_max=rwMaxSpeed,  # Valore RPM (10000) - Basilisk converte internamente
                            u_max=rwMaxTorque,  # Coppia massima in uscita [N*m]
                            # maxMomentum calcolato automaticamente da Js * Omega_max
                            rwMass = rwMass, # massa delle reaction wheel, espressa in kg. Non sicuro se questo metodo ha l'attributo massa.
                            rWB_B = [0, 0, 0], #rwPosVector[rw], 
                            #rWB_B=rwPosVector[rw], # +rwCenterOfMassLocation,  # Vettore posizione dal CM dello spacecraft [m]
                            Js=rwInertia,  # Inerzia volano da datasheet [kg*m^2]
                            Omega=0.0)  # Velocità iniziale ruota (zero all'avvio)

        self.rwStateEffector.ModelTag = "RW_CW0162_Cluster"
        self.rwFactory.addToSpacecraft("RWA", self.rwStateEffector, self.scObject)

        # Aggiunge parametri di attrito realistici alle reaction wheel CW0162
        # Questi valori modellano la risposta di coppia non ideale delle reaction wheel fisiche
        friction_coulomb = 0.8795e-3    # [N·m] Coppia attrito di Coulomb (τc)
        friction_static = 0.9055e-3     # [N·m] Attrito statico (stiction, τst = τc + δ)
        friction_viscous = 5.16e-6      # [N·m·s/rad] Coefficiente attrito viscoso (cv)
        beta_stribeck = 0.5             # [rad⁻¹] Coefficiente attrito di Stribeck (βst)

        for i in range(1, numRW + 1):
            rwName = f"RW{i}"
            self.rwFactory.rwList[rwName].fCoulomb = friction_coulomb
            self.rwFactory.rwList[rwName].fStatic = friction_static
            self.rwFactory.rwList[rwName].cViscous = friction_viscous
            self.rwFactory.rwList[rwName].betaStatic = beta_stribeck

    def SetMagnetorquersEffector(self):
        """Impostazione dei 3 Magnetorquer per lo spacecraft EXCITE"""

        self.mtbEffector.ModelTag = "MTBEffector"

        # FIX CRITICO: NON configurare parametri MTB o connettere messaggi qui!
        # TUTTA la configurazione magnetorquer e connessioni messaggi sono gestite dal FSW
        # in setupGatewayMsgs (EXCITE_Fsw.py:474-497) per evitare interferenze
        #
        # FSW gestisce:
        #   1. Configurazione MTB (GtMatrix, maxDipoles, ecc.)
        #   2. creazione dipoleGatewayMsg e inizializzazione a zero
        #   3. Tutte le sottoscrizioni messaggi (mtbCmdInMsg, magInMsg, mtbParamsInMsg)
        #
        # Dynamics aggiunge SOLO l'effettore vuoto allo spacecraft

        self.scObject.addDynamicEffector(self.mtbEffector)

    def SetThrusterStateEffector(self):
        """
        Configura propulsore chimico H2O2 per manovre orbitali (esperimento IOD UniPi).

        Specifiche Propulsore:
        - Tipo: monopropellente H2O2 (catalizzato)
        - Spinta (BoL): 0.5 N
        - Impulso Specifico: 165 s
        - Posizione: [-0.10128, -0.10045, -0.02950] m (frame corpo)
        - Direzione: [0, 0, -1] (accende in direzione -Z)

        Note Configurazione:
        - Il propulsore singolo richiede puntamento velocità ADCS per allineamento corretto
        - Offset dal CM (~0.216 m) genera coppia di disturbo ~50 mNm
        - RW + MTB devono compensare coppia di accoppiamento durante accensione
        - Compatibile con integratore a passo variabile RKF45
        """
        # Crea una nuova istanza factory propulsore (critico per esecuzioni multiple)
        self.thFactory = simIncludeThruster.thrusterFactory()

        # Parametri propulsore H2O2 (specifiche IOD UniPi)
        MaxThrust_N = 0.5           # [N] Spinta NOMINALE (propulsore monopropellente H2O2)
        steadyIsp_s = 165.0         # [s] Impulso specifico per monopropellente H2O2
        cutoffFrequency = 0.2       # [rad/s] Tempo di risposta ~15s (ramp-up lento per spool-up termico realistico)
        areaNozzle_m2 = 33.183e-6   # [m^2] Area uscita ugello = 33.183 mm^2
        MinOnTime_s = 0.05          # [s] Durata minima impulso on-time

        # Posizione propulsore nel frame corpo [m]
        # I test mostrano coppia = 0 a [0,0,0] ma il tumbling avviene comunque!
        # Il problema NON è l'offset - qualcos'altro nel modulo propulsore causa tumbling
        thruster_position_B = [0.0, 0.0, 0.0]

        # Direzione accensione propulsore (vettore unitario nel frame corpo)
        # Accende in direzione -Z per manovre allineate alla velocità
        thruster_direction_B = [0.0, 0.0, -1.0]

        # Crea propulsore usando factory
        self.thFactory.create(
            'Blank_Thruster',           # Tipo propulsore personalizzato
            thruster_position_B,        # Posizione nel frame corpo
            thruster_direction_B,       # Direzione spinta
            MaxThrust=MaxThrust_N,
            steadyIsp=steadyIsp_s,
            MinOnTime=MinOnTime_s,
            cutoffFrequency=cutoffFrequency,
            areaNozzle=areaNozzle_m2
        )

        # Aggiunge propulsore allo spacecraft
        thrModelTag = "H2O2_UniPi_Thruster"
        self.thFactory.addToSpacecraft(thrModelTag,
                                      self.thrusterStateEffector,
                                      self.scObject)

        print(f"[EXCITE Dynamics] H2O2 thruster configured:")
        print(f"  Thrust: {MaxThrust_N} N")
        print(f"  Isp: {steadyIsp_s} s")
        print(f"  Min On-Time: {MinOnTime_s} s")
        print(f"  Nozzle area: {areaNozzle_m2*1e6:.3f} mm^2")

        # DEBUG: Stampa COM effettivo spacecraft dopo aggiunta tutti componenti
        print(f"\n[COM DEBUG] Spacecraft Center of Mass Analysis:")
        # r_CN_NInit è la posizione iniziale nel frame inerziale, non COM
        # r_BcB_B è il COM dell'hub nel frame corpo
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            print(f"  Hub COM in body frame: {self.scObject.hub.r_BcB_B}")
        else:
            print(f"  Hub COM not accessible via r_BcB_B")

        # Prova modi diversi per ottenere COM
        actual_com = [0.0, 0.0, 0.0]
        com_found = False

        # Prova metodo 1: r_BcB_B
        if hasattr(self.scObject.hub, 'r_BcB_B'):
            raw_com = self.scObject.hub.r_BcB_B
            # Converte da [[x], [y], [z]] a [x, y, z] se necessario
            if isinstance(raw_com, list) and len(raw_com) == 3:
                if isinstance(raw_com[0], list):
                    actual_com = [raw_com[0][0], raw_com[1][0], raw_com[2][0]]
                else:
                    actual_com = raw_com
            else:
                actual_com = list(raw_com)
            com_found = True
            print(f"  Using r_BcB_B for COM: {actual_com}")
        # Prova metodo 2: c_B
        elif hasattr(self.scObject, 'c_B'):
            actual_com = self.scObject.c_B
            com_found = True
            print(f"  Using c_B for COM: {actual_com}")
        # Prova metodo 3: Calcola da proprietà di massa
        elif hasattr(self.scObject.hub, 'mHub') and hasattr(self.scObject.hub, 'r_BP_B'):
            # Questo sarebbe il calcolo del COM dalle proprietà di massa
            print(f"  Trying to calculate COM from mass properties...")
            print(f"    Hub mass: {self.scObject.hub.mHub if hasattr(self.scObject.hub, 'mHub') else 'N/A'}")

        if not com_found:
            print(f"  WARNING: Could not determine actual COM, assuming [0,0,0]")
            # Stampiamo attributi disponibili per debug
            print(f"  Available hub attributes: {[attr for attr in dir(self.scObject.hub) if not attr.startswith('_')][:10]}")

        # Calcola offset da posizione propulsore a COM effettivo
        thruster_to_com = [actual_com[i] - thruster_position_B[i] for i in range(3)]
        offset_magnitude = np.linalg.norm(thruster_to_com)

        print(f"  Thruster position: {thruster_position_B}")
        print(f"  Offset from thruster to COM: {thruster_to_com}")
        print(f"  Offset magnitude: {offset_magnitude*1000:.2f} mm")

        # Calcola coppia attesa
        expected_torque = np.cross(thruster_to_com, [0, 0, -MaxThrust_N])
        expected_torque_mag = np.linalg.norm(expected_torque) * 1000  # mNm
        print(f"  Expected torque from offset: {expected_torque_mag:.2f} mNm")
        print(f"  Expected torque components [mNm]: [{expected_torque[0]*1000:.2f}, {expected_torque[1]*1000:.2f}, {expected_torque[2]*1000:.2f}]")
        print(f"  Position: {thruster_position_B} m")
        print(f"  Direction: {thruster_direction_B}")

        # Calcola e mostra coppia di disturbo
        r_CM = np.array([-0.001377, 0.00177, 0.18619])  # From SetSpacecraftHub()
        r_thruster = np.array(thruster_position_B)
        r_offset = r_thruster - r_CM
        F_thrust = np.array(thruster_direction_B) * MaxThrust_N
        M_disturbance = np.cross(r_offset, F_thrust)
        print(f"  Offset from CM: {np.linalg.norm(r_offset)*1000:.1f} mm")
        print(f"  Disturbance torque: [{M_disturbance[0]*1000:.1f}, {M_disturbance[1]*1000:.1f}, {M_disturbance[2]*1000:.1f}] mNm")
        print(f"  WARNING: Disturbance torque magnitude ({np.linalg.norm(M_disturbance)*1000:.1f} mNm) requires RW+MTB compensation")

    # PLACEHOLDER_FOR_PART_3