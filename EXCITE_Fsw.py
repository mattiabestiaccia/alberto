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