/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

/*
 * SMEKF.c - Sequential Multiplicative Extended Kalman Filter per determinazione assetto
 *
 * Questo modulo implementa un filtro di Kalman esteso moltiplicativo sequenziale per la stima
 * dell'assetto di un satellite. Il filtro fonde misure da:
 * - IMU (giroscopi) per la propagazione dello stato
 * - Star Tracker (misure ad alta precisione dell'assetto)
 * - QUEST (determinazione assetto da vettori Sole + Magnetometro)
 *
 * Vettore di stato (6 elementi):
 * - delta_theta (3): Errore di assetto in parametri MRP
 * - bias (3): Bias dei giroscopi
 *
 * Il filtro usa una formulazione moltiplicativa per l'assetto (quaternioni) e
 * una formulazione additiva per il bias.
 *
 * Algoritmo:
 * 1. Propagazione temporale con IMU (predizione)
 * 2. Update sequenziale con misure disponibili (correzione)
 * 3. Reset della parte moltiplicativa (proiezione su manifold)
 */

#include "fswAlgorithms/attDetermination/SMEKF/SMEKF.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>
#include <math.h>

/*! Inizializza i messaggi di output per il filtro SMEKF.
    @param configData Dati di configurazione del stimatore SMEKF
    @param moduleID Identificatore del modulo
 */
void SelfInit_SMEKF(SMEKFConfig *configData, int64_t moduleID)
{
    NavAttMsg_C_init(&configData->navStateOutMsg);
    InertialFilterMsg_C_init(&configData->filtDataOutMsg);
}

/*! Resetta il filtro SMEKF a uno stato iniziale e inizializza le matrici interne di stima.
    @param configData Dati di configurazione del stimatore SMEKF
    @param callTime Tempo della chiamata in nanosecondi
    @param moduleID Identificatore del modulo
 */
void Reset_SMEKF(SMEKFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Verifica che i messaggi di input richiesti siano connessi */
    if (!IMUSensorMsg_C_isLinked(&configData->imuSensorInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: SMEKF.imuSensorInMsg wasn't connected.");
    }

    /* Inizializza il timing */
    configData->timeTag = callTime * NANO2SEC;
    configData->firstRun = 1;

    /* Inizializza output diagnostici */
    configData->numMeas = 0;
    configData->numIterations = 0;

    /* Inizializza vettori di stato a zero */
    v4SetZero(configData->quat_BN);
    v3SetZero(configData->bias);
    v3SetZero(configData->omega_BN_B);
    mSetZero(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES);

    /* Copia le condizioni iniziali se fornite, altrimenti usa i default */
    if (v4Norm(configData->quat_BN_init) > 0.1) {
        v4Copy(configData->quat_BN_init, configData->quat_BN);
        SMEKF_normalizeQuaternion(configData->quat_BN);
    } else {
        /* Default: quaternione identità */
        configData->quat_BN[0] = 1.0;
        configData->quat_BN[1] = 0.0;
        configData->quat_BN[2] = 0.0;
        configData->quat_BN[3] = 0.0;
    }

    vCopy(configData->bias_init, SMEKF_N_BIAS_STATES, configData->bias);
    mCopy(configData->covar_init, SMEKF_N_STATES, SMEKF_N_STATES, configData->covar);

    /* Initialize previous state variables */
    v4Copy(configData->quat_BN, configData->quat_BN_prev);
    vCopy(configData->bias, SMEKF_N_BIAS_STATES, configData->bias_prev);
    mCopy(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES, configData->covar_prev);

    /* Write initial output message to avoid "not properly initialized" warnings */
    /* This ensures the message is valid even if Update is never called */
    {
        NavAttMsgPayload navMsgOut;
        InertialFilterMsgPayload filtMsgOut;
        double sigma_BN[3];

        /* Initialize output structures */
        navMsgOut = NavAttMsg_C_zeroMsgPayload();
        filtMsgOut = InertialFilterMsg_C_zeroMsgPayload();

        /* Convert quaternion to MRP for NavAttMsg */
        EP2MRP(configData->quat_BN, sigma_BN);
        v3Copy(sigma_BN, navMsgOut.sigma_BN);
        v3Copy(configData->omega_BN_B, navMsgOut.omega_BN_B);
        navMsgOut.timeTag = configData->timeTag;

        /* Populate filter diagnostics */
        filtMsgOut.timeTag = configData->timeTag;
        filtMsgOut.numObs = 0;
        vCopy(configData->bias, 3, filtMsgOut.state + 3);  /* Bias in state[3:6] */
        mCopy(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES, filtMsgOut.covar);

        /* Write initial messages */
        NavAttMsg_C_write(&navMsgOut, &configData->navStateOutMsg, moduleID, callTime);
        InertialFilterMsg_C_write(&filtMsgOut, &configData->filtDataOutMsg, moduleID, callTime);
    }

    /* Set default noise parameters if not set */
    if (configData->sigma_v <= 0.0) {
        configData->sigma_v = 1e-4;  /* Default angular velocity noise [rad/s/sqrt(s)] */
    }
    if (configData->sigma_u <= 0.0) {
        configData->sigma_u = 1e-6;  /* Default bias drift noise [rad/s^2/sqrt(s)] */
    }

    /* Set default measurement noise covariances if not set */
    if (mIsZero(configData->R_quest, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, 1e-10)) {
        mSetIdentity(configData->R_quest, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES);
        mScale(1e-6, configData->R_quest, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, configData->R_quest);
    }
    if (mIsZero(configData->R_st, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, 1e-10)) {
        mSetIdentity(configData->R_st, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES);
        mScale(1e-6, configData->R_st, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, configData->R_st);
    }

    /* Enable both measurements by default */
    if (configData->useQuest == 0 && configData->useST == 0) {
        configData->useQuest = 1;
        configData->useST = 1;
    }

    /* Set default propagationOnly flag (0 = full filter with measurements) */
    if (configData->propagationOnly != 1) {
        configData->propagationOnly = 0;
    }

    return;
}

/*! Implementa il loop principale di aggiornamento del filtro SMEKF.
    Esegue la propagazione temporale usando i dati IMU e gli update sequenziali delle misure usando QUEST e Star Tracker.
    @param configData Dati di configurazione del stimatore SMEKF
    @param callTime Tempo della chiamata in nanosecondi
    @param moduleID Identificatore del modulo
 */
void Update_SMEKF(SMEKFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    IMUSensorMsgPayload imuSensorBuffer;
    NavAttMsgPayload questAttBuffer;
    STAttMsgPayload stAttBuffer;
    STSensorMsgPayload stSensorBuffer;
    NavAttMsgPayload navAttOutBuffer;
    InertialFilterMsgPayload filtDataOutBuffer;

    double newTimeTag;
    double dt;
    double w_meas[3];
    double q_meas[4];
    double q_meas_MRP[3];
    double K_out[SMEKF_N_STATES * SMEKF_N_ATT_STATES];
    int numMeasAvailable = 0;

    /* Compute time step based on task rate (callTime) */
    newTimeTag = callTime * NANO2SEC;
    dt = newTimeTag - configData->timeTag;

    if (dt <= 0.0) {
        /* No time has passed, write previous state and return */
        goto writeOutput;
    }

    /* Read IMU sensor data */
    imuSensorBuffer = IMUSensorMsg_C_read(&configData->imuSensorInMsg);
    int imuIsWritten = IMUSensorMsg_C_isWritten(&configData->imuSensorInMsg);

    if (!imuIsWritten) {
        /* No IMU data available, write previous state and return */
        goto writeOutput;
    }

    /* Extract angular velocity measurement from IMU */
    v3Copy(imuSensorBuffer.AngVelPlatform, w_meas);

    /* INITIALIZATION: On first run, initialize from Star Tracker if available */
    if (configData->firstRun) {
        configData->firstRun = 0;

        /* Try to get initial attitude from Star Tracker */
        if (STSensorMsg_C_isLinked(&configData->stSensorInMsg) &&
            STSensorMsg_C_isWritten(&configData->stSensorInMsg)) {
            stSensorBuffer = STSensorMsg_C_read(&configData->stSensorInMsg);
            /* IMPORTANT: qInrtl2Case is q_NB (Inertial to Body), but filter needs q_BN (Body to Inertial) */
            SMEKF_quaternionInverse(stSensorBuffer.qInrtl2Case, configData->quat_BN);
            SMEKF_normalizeQuaternion(configData->quat_BN);
        } else if (STAttMsg_C_isLinked(&configData->stAttInMsg) &&
                   STAttMsg_C_isWritten(&configData->stAttInMsg)) {
            stAttBuffer = STAttMsg_C_read(&configData->stAttInMsg);
            MRP2EP(stAttBuffer.MRP_BdyInrtl, configData->quat_BN);
            SMEKF_normalizeQuaternion(configData->quat_BN);
        } else if (NavAttMsg_C_isLinked(&configData->questAttInMsg) &&
                   NavAttMsg_C_isWritten(&configData->questAttInMsg)) {
            questAttBuffer = NavAttMsg_C_read(&configData->questAttInMsg);
            MRP2EP(questAttBuffer.sigma_BN, configData->quat_BN);
            SMEKF_normalizeQuaternion(configData->quat_BN);
        }

        /* Initialize previous state */
        v4Copy(configData->quat_BN, configData->quat_BN_prev);
    }

    /* TIME UPDATE: Propagate state and covariance */
    SMEKF_timeUpdate(configData, w_meas, dt);

    /* Update time tag */
    configData->timeTag = newTimeTag;

    /* MEASUREMENT UPDATE: Sequential updates with available measurements */
    /* Skip this section if propagationOnly flag is set (debug mode) */
    if (!configData->propagationOnly) {
        configData->numMeas = 0;
        double *R_last = NULL;  /* Track R from last measurement for covariance update */

        /* Check QUEST measurement */
        if (configData->useQuest && NavAttMsg_C_isLinked(&configData->questAttInMsg)) {
            if (NavAttMsg_C_isWritten(&configData->questAttInMsg)) {
                questAttBuffer = NavAttMsg_C_read(&configData->questAttInMsg);
                v3Copy(questAttBuffer.sigma_BN, q_meas_MRP);
                MRP2EP(q_meas_MRP, q_meas);
                SMEKF_normalizeQuaternion(q_meas);

                SMEKF_measurementUpdate(configData, q_meas, configData->R_quest, K_out);
                configData->numMeas++;
                numMeasAvailable = 1;
                R_last = configData->R_quest;  /* Save R for Joseph form update */
            }
        }

        /* Check Star Tracker measurement (quaternion format from sensor) */
        if (configData->useST && STSensorMsg_C_isLinked(&configData->stSensorInMsg)) {
            if (STSensorMsg_C_isWritten(&configData->stSensorInMsg)) {
                stSensorBuffer = STSensorMsg_C_read(&configData->stSensorInMsg);
                /* IMPORTANT: qInrtl2Case is q_NB (Inertial to Body), but filter needs q_BN (Body to Inertial) */
                SMEKF_quaternionInverse(stSensorBuffer.qInrtl2Case, q_meas);
                SMEKF_normalizeQuaternion(q_meas);

                SMEKF_measurementUpdate(configData, q_meas, configData->R_st, K_out);
                configData->numMeas++;
                numMeasAvailable = 1;
                R_last = configData->R_st;  /* Save R for Joseph form update */
            }
        }
        /* Alternative: Check Star Tracker MRP format */
        else if (configData->useST && STAttMsg_C_isLinked(&configData->stAttInMsg)) {
            if (STAttMsg_C_isWritten(&configData->stAttInMsg)) {
                stAttBuffer = STAttMsg_C_read(&configData->stAttInMsg);
                v3Copy(stAttBuffer.MRP_BdyInrtl, q_meas_MRP);
                MRP2EP(q_meas_MRP, q_meas);
                SMEKF_normalizeQuaternion(q_meas);

                SMEKF_measurementUpdate(configData, q_meas, configData->R_st, K_out);
                configData->numMeas++;
                numMeasAvailable = 1;
                R_last = configData->R_st;  /* Save R for Joseph form update */
            }
        }

        /* Update covariance after all sequential measurement updates using Joseph form */
        /* Joseph form: P = (I - KH)P(I - KH)^T + KRK^T */
        if (numMeasAvailable && configData->numMeas > 0 && R_last != NULL) {
            double I_minus_KH[SMEKF_N_STATES * SMEKF_N_STATES];
            double H_k[SMEKF_N_ATT_STATES * SMEKF_N_STATES];
            double KH[SMEKF_N_STATES * SMEKF_N_STATES];
            double P_temp1[SMEKF_N_STATES * SMEKF_N_STATES];
            double P_temp2[SMEKF_N_STATES * SMEKF_N_STATES];
            double I_minus_KH_T[SMEKF_N_STATES * SMEKF_N_STATES];
            double KR[SMEKF_N_STATES * SMEKF_N_ATT_STATES];
            double KRK_T[SMEKF_N_STATES * SMEKF_N_STATES];
            double K_T[SMEKF_N_ATT_STATES * SMEKF_N_STATES];

            /* H_k = [I(3x3), 0(3x3)] */
            mSetZero(H_k, SMEKF_N_ATT_STATES, SMEKF_N_STATES);
            mSetIdentity(H_k, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES);

            /* Compute K*H */
            mMultM(K_out, SMEKF_N_STATES, SMEKF_N_ATT_STATES,
                   H_k, SMEKF_N_ATT_STATES, SMEKF_N_STATES, KH);

            /* I - K*H */
            mSetIdentity(I_minus_KH, SMEKF_N_STATES, SMEKF_N_STATES);
            mSubtract(I_minus_KH, SMEKF_N_STATES, SMEKF_N_STATES, KH, I_minus_KH);

            /* (I - K*H)^T */
            mTranspose(I_minus_KH, SMEKF_N_STATES, SMEKF_N_STATES, I_minus_KH_T);

            /* First term: (I - K*H) * P * (I - K*H)^T */
            mMultM(I_minus_KH, SMEKF_N_STATES, SMEKF_N_STATES,
                   configData->covar, SMEKF_N_STATES, SMEKF_N_STATES, P_temp1);
            mMultM(P_temp1, SMEKF_N_STATES, SMEKF_N_STATES,
                   I_minus_KH_T, SMEKF_N_STATES, SMEKF_N_STATES, P_temp2);

            /* Second term: K * R * K^T */
            /* K * R */
            mMultM(K_out, SMEKF_N_STATES, SMEKF_N_ATT_STATES,
                   R_last, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, KR);

            /* K^T */
            mTranspose(K_out, SMEKF_N_STATES, SMEKF_N_ATT_STATES, K_T);

            /* K * R * K^T */
            mMultM(KR, SMEKF_N_STATES, SMEKF_N_ATT_STATES,
                   K_T, SMEKF_N_ATT_STATES, SMEKF_N_STATES, KRK_T);

            /* P = (I - K*H)P(I - K*H)^T + KRK^T */
            mAdd(P_temp2, SMEKF_N_STATES, SMEKF_N_STATES, KRK_T, configData->covar);
        }
    } else {
        /* Propagation-only mode: set numMeas to 0 for output diagnostics */
        configData->numMeas = 0;
    }

    /* Store previous states for next iteration */
    v4Copy(configData->quat_BN, configData->quat_BN_prev);
    vCopy(configData->bias, SMEKF_N_BIAS_STATES, configData->bias_prev);
    mCopy(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES, configData->covar_prev);

writeOutput:
    /* Write navigation output message (MRP format) */
    navAttOutBuffer = NavAttMsg_C_zeroMsgPayload();
    EP2MRP(configData->quat_BN, navAttOutBuffer.sigma_BN);
    v3Copy(configData->omega_BN_B, navAttOutBuffer.omega_BN_B);
    navAttOutBuffer.timeTag = configData->timeTag;
    NavAttMsg_C_write(&navAttOutBuffer, &configData->navStateOutMsg, moduleID, callTime);

    /* Write filter data output message */
    filtDataOutBuffer = InertialFilterMsg_C_zeroMsgPayload();
    filtDataOutBuffer.timeTag = configData->timeTag;
    filtDataOutBuffer.numObs = (int32_t)configData->numMeas;
    memmove(filtDataOutBuffer.covar, configData->covar,
            SMEKF_N_STATES * SMEKF_N_STATES * sizeof(double));
    /* Pack state: first 3 elements are MRP from quaternion, last 3 are bias */
    EP2MRP(configData->quat_BN, filtDataOutBuffer.state);
    memmove(filtDataOutBuffer.state + 3, configData->bias, 3 * sizeof(double));
    InertialFilterMsg_C_write(&filtDataOutBuffer, &configData->filtDataOutMsg, moduleID, callTime);

    return;
}

/*! Propagation step: updates quaternion, bias, and covariance matrix
    @param configData The configuration data
    @param w_meas Measured angular velocity [rad/s] (3x1)
    @param dt Time step [s]
 */
void SMEKF_timeUpdate(SMEKFConfig *configData, double w_meas[3], double dt)
{
    double w_hat[3];
    double PHI[SMEKF_N_STATES * SMEKF_N_STATES];
    double Q_d[SMEKF_N_STATES * SMEKF_N_STATES];
    double PHI_T[SMEKF_N_STATES * SMEKF_N_STATES];
    double P_PHI_T[SMEKF_N_STATES * SMEKF_N_STATES];
    double PHI_P_PHI_T[SMEKF_N_STATES * SMEKF_N_STATES];
    double q_before[4];

    /* DIAGNOSTIC: Store quaternion before propagation */
    v4Copy(configData->quat_BN, q_before);

    /* Compute estimated angular velocity: w_hat = w_meas - bias */
    v3Subtract(w_meas, configData->bias, w_hat);
    v3Copy(w_hat, configData->omega_BN_B);

    /* DIAGNOSTIC: Print propagation input */
    printf("\n--- SMEKF Time Update [t=%.2f, dt=%.3f] ---\n", configData->timeTag, dt);
    printf("  w_meas = [%+.6e, %+.6e, %+.6e] rad/s (%.4f, %.4f, %.4f) deg/s\n",
           w_meas[0], w_meas[1], w_meas[2],
           w_meas[0]*180.0/M_PI, w_meas[1]*180.0/M_PI, w_meas[2]*180.0/M_PI);
    printf("  bias   = [%+.6e, %+.6e, %+.6e] rad/s (%.4f, %.4f, %.4f) deg/s\n",
           configData->bias[0], configData->bias[1], configData->bias[2],
           configData->bias[0]*180.0/M_PI, configData->bias[1]*180.0/M_PI, configData->bias[2]*180.0/M_PI);
    printf("  w_hat  = [%+.6e, %+.6e, %+.6e] rad/s (%.4f, %.4f, %.4f) deg/s\n",
           w_hat[0], w_hat[1], w_hat[2],
           w_hat[0]*180.0/M_PI, w_hat[1]*180.0/M_PI, w_hat[2]*180.0/M_PI);
    printf("  q_before = [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           q_before[0], q_before[1], q_before[2], q_before[3]);

    /* Propagate quaternion */
    SMEKF_propagateQuaternion(configData->quat_BN, w_hat, dt, configData->quat_BN);

    printf("  q_after  = [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           configData->quat_BN[0], configData->quat_BN[1], configData->quat_BN[2], configData->quat_BN[3]);

    /* Bias remains constant in propagation (bias_k+1 = bias_k) */
    /* No change needed for configData->bias */

    /* Compute state transition matrix PHI */
    SMEKF_computePropagationMatrix(w_hat, dt, PHI);

    /* Compute discrete process noise Q_d */
    SMEKF_computeProcessNoise(configData->sigma_v, configData->sigma_u, dt, Q_d);

    /* Propagate covariance: P = PHI * P * PHI^T + Q_d */
    mTranspose(PHI, SMEKF_N_STATES, SMEKF_N_STATES, PHI_T);
    mMultM(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES,
           PHI_T, SMEKF_N_STATES, SMEKF_N_STATES, P_PHI_T);
    mMultM(PHI, SMEKF_N_STATES, SMEKF_N_STATES,
           P_PHI_T, SMEKF_N_STATES, SMEKF_N_STATES, PHI_P_PHI_T);
    mAdd(PHI_P_PHI_T, SMEKF_N_STATES, SMEKF_N_STATES, Q_d, configData->covar);

    return;
}

/*! Computes the state transition matrix PHI (6x6) for MEKF error state

    MEKF Error State Dynamics:
    F = [-[w_hat]×   -I₃]
        [0₃          0₃ ]

    Discrete State Transition (first-order):
    PHI = I + F·dt = [I₃ - [w_hat]×·dt   -I₃·dt]
                      [0₃                 I₃    ]

    @param w_hat Estimated angular velocity [rad/s] (3x1)
    @param dt Time step [s]
    @param PHI Output state transition matrix (6x6)
 */
void SMEKF_computePropagationMatrix(double w_hat[3], double dt, double PHI[SMEKF_N_STATES * SMEKF_N_STATES])
{
    double w_tilde[3][3];
    double w_tilde_sq[3][3];
    double PHI_11[3][3], PHI_12[3][3];
    double w_norm, w_norm_sq, w_norm_cube;
    double theta;  /* theta = ||w|| * dt */
    double sin_theta, cos_theta;
    double coeff1_11, coeff2_11;  /* Coefficients for PHI_11 */
    double coeff1_12, coeff2_12;  /* Coefficients for PHI_12 */
    double I3[3][3];
    double temp1[3][3], temp2[3][3];
    int i, j;

    /* Initialize PHI to identity */
    mSetIdentity(PHI, SMEKF_N_STATES, SMEKF_N_STATES);

    /* Compute skew-symmetric matrix [w_hat x] */
    v3Tilde(w_hat, w_tilde);

    /* Compute ||w_hat|| */
    w_norm = v3Norm(w_hat);
    theta = w_norm * dt;

    /* Check for small angle (use linear approximation) */
    if (theta < 1e-8) {
        /* Linear approximation (for numerical stability when ||w|| ≈ 0):
           PHI_11 = I - [w]× * dt
           PHI_12 = -I * dt */

        m33Scale(-dt, w_tilde, temp1);
        mSetIdentity(PHI_11, 3, 3);
        m33Add(PHI_11, temp1, PHI_11);

        /* PHI_12 = -I * dt */
        mSetIdentity(PHI_12, 3, 3);
        m33Scale(-dt, PHI_12, PHI_12);

    } else {
        /* Exact Rodrigues formula for large angles:

           PHI_11 = I - [w]×·sin(θ)/||w|| + [w]×²·(1-cos(θ))/||w||²

           PHI_12 = [w]×·(1-cos(θ))/||w||² - dt·I - [w]×²·(θ - sin(θ))/||w||³

           where θ = ||w|| * dt
        */

        w_norm_sq = w_norm * w_norm;
        w_norm_cube = w_norm_sq * w_norm;

        sin_theta = sin(theta);
        cos_theta = cos(theta);

        /* Compute [w_hat]×² = [w_hat]× * [w_hat]× */
        m33MultM33(w_tilde, w_tilde, w_tilde_sq);

        /* Create identity matrix */
        mSetIdentity(I3, 3, 3);

        /* Compute PHI_11 = I - [w]×·sin(θ)/||w|| + [w]×²·(1-cos(θ))/||w||² */
        coeff1_11 = -sin_theta / w_norm;
        coeff2_11 = (1.0 - cos_theta) / w_norm_sq;

        m33Scale(coeff1_11, w_tilde, temp1);
        m33Scale(coeff2_11, w_tilde_sq, temp2);

        m33Copy(I3, PHI_11);
        m33Add(PHI_11, temp1, PHI_11);
        m33Add(PHI_11, temp2, PHI_11);

        /* Compute PHI_12 = [w]×·(1-cos(θ))/||w||² - dt·I - [w]×²·(θ - sin(θ))/||w||³ */
        coeff1_12 = (1.0 - cos_theta) / w_norm_sq;
        coeff2_12 = -(theta - sin_theta) / w_norm_cube;

        m33Scale(coeff1_12, w_tilde, temp1);
        m33Scale(-dt, I3, temp2);

        m33Add(temp1, temp2, PHI_12);  /* PHI_12 = [w]×·coeff1 - dt·I */

        m33Scale(coeff2_12, w_tilde_sq, temp1);
        m33Add(PHI_12, temp1, PHI_12);  /* PHI_12 += [w]×²·coeff2 */
    }

    /* Assemble full PHI matrix = [PHI_11  PHI_12]
                                    [0       I    ] */
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            /* PHI_11 block */
            PHI[MXINDEX(SMEKF_N_STATES, i, j)] = PHI_11[i][j];

            /* PHI_12 block */
            PHI[MXINDEX(SMEKF_N_STATES, i, j + 3)] = PHI_12[i][j];

            /* PHI_21 block: 0 */
            PHI[MXINDEX(SMEKF_N_STATES, i + 3, j)] = 0.0;

            /* PHI_22 block: I (already set by mSetIdentity) */
        }
    }

    /* DEBUG: Print PHI matrix values during critical periods */
    if (theta > 0.001) {  /* Only print when angular motion is significant */
        printf("\n--- PHI Matrix [||w||=%.6f rad/s, dt=%.3f s, theta=%.6f rad (%.2f deg)] ---\n",
               w_norm, dt, theta, theta * 180.0 / M_PI);
        printf("  Small angle approx: %s\n", (theta < 1e-8) ? "YES" : "NO");

        if (theta >= 1e-8) {
            printf("  PHI_11 coefficients: -sin(θ)/||w||=%+.6e, (1-cos(θ))/||w||²=%+.6e\n",
                   coeff1_11, coeff2_11);
            printf("  PHI_12 coefficients: (1-cos(θ))/||w||²=%+.6e, -(θ-sin(θ))/||w||³=%+.6e\n",
                   coeff1_12, coeff2_12);
        }

        printf("  PHI_11 diagonal: [%.6f, %.6f, %.6f]\n", PHI_11[0][0], PHI_11[1][1], PHI_11[2][2]);
        printf("  PHI_12[0,:] = [%+.6e, %+.6e, %+.6e]\n", PHI_12[0][0], PHI_12[0][1], PHI_12[0][2]);
    }

    return;
}

/*! Computes the discrete process noise covariance Q_d (6x6)
    Simplified diagonal form: Q_d = G * Q_c * G^T * dt
    @param sigma_v Angular velocity white noise std dev [rad/s/sqrt(s)]
    @param sigma_u Gyro bias random walk std dev [rad/s^2/sqrt(s)]
    @param dt Time step [s]
    @param Q_d Output process noise covariance (6x6)
 */
void SMEKF_computeProcessNoise(double sigma_v, double sigma_u, double dt,
                               double Q_d[SMEKF_N_STATES * SMEKF_N_STATES])
{
    double sigma_v_sq = sigma_v * sigma_v;
    double sigma_u_sq = sigma_u * sigma_u;
    double Q_11_scale, Q_22_scale;
    int i, j;

    /* Initialize to zero */
    mSetZero(Q_d, SMEKF_N_STATES, SMEKF_N_STATES);

    /* Simplified diagonal Q_d (Euler discretization):
       Q_d = [sigma_v^2*dt*I,     0;
              0,                  sigma_u^2*dt*I] */

    Q_11_scale = sigma_v_sq * dt;
    Q_22_scale = sigma_u_sq * dt;

    /* Fill diagonal blocks only (no cross-coupling terms) */
    for (i = 0; i < SMEKF_N_STATES; i++) {
        for (j = 0; j < SMEKF_N_STATES; j++) {
            if (i < 3 && j < 3) {
                /* Q_11 block: attitude noise */
                Q_d[MXINDEX(SMEKF_N_STATES, i, j)] = (i == j) ? Q_11_scale : 0.0;
            } else if (i >= 3 && j >= 3) {
                /* Q_22 block: bias drift noise */
                Q_d[MXINDEX(SMEKF_N_STATES, i, j)] = (i == j) ? Q_22_scale : 0.0;
            }
            /* Q_12 and Q_21 blocks remain zero (no cross-coupling) */
        }
    }

    return;
}

/*! Propagates quaternion forward using RK4 integration with Basilisk dEP function

    Uses 4th-order Runge-Kutta integration of quaternion kinematic equation:
    dQ/dt = (1/2) * B(Q) * omega

    where B(Q) is the quaternion kinematic matrix (implemented in Basilisk's dEP).

    RK4 stages:
    k1 = f(t, q)              = dEP(q, w)
    k2 = f(t+dt/2, q+dt/2*k1) = dEP(q+dt/2*k1, w)
    k3 = f(t+dt/2, q+dt/2*k2) = dEP(q+dt/2*k2, w)
    k4 = f(t+dt, q+dt*k3)     = dEP(q+dt*k3, w)

    q(t+dt) = q(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

    @param q_in Input quaternion (scalar-first: [q0, q1, q2, q3])
    @param w_hat Estimated angular velocity [rad/s] (3x1) - body frame
    @param dt Time step [s]
    @param q_out Output quaternion (scalar-first: [q0, q1, q2, q3])
 */
void SMEKF_propagateQuaternion(double q_in[4], double w_hat[3], double dt, double q_out[4])
{
    double k1[4], k2[4], k3[4], k4[4];
    double q_temp[4];
    int i;

    /* Stage 1: k1 = dEP(q, w) at t */
    dEP(q_in, w_hat, k1);

    /* Stage 2: k2 = dEP(q + dt/2 * k1, w) at t + dt/2 */
    for (i = 0; i < 4; i++) {
        q_temp[i] = q_in[i] + 0.5 * dt * k1[i];
    }
    dEP(q_temp, w_hat, k2);

    /* Stage 3: k3 = dEP(q + dt/2 * k2, w) at t + dt/2 */
    for (i = 0; i < 4; i++) {
        q_temp[i] = q_in[i] + 0.5 * dt * k2[i];
    }
    dEP(q_temp, w_hat, k3);

    /* Stage 4: k4 = dEP(q + dt * k3, w) at t + dt */
    for (i = 0; i < 4; i++) {
        q_temp[i] = q_in[i] + dt * k3[i];
    }
    dEP(q_temp, w_hat, k4);

    /* Final RK4 update: q(t+dt) = q(t) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4) */
    for (i = 0; i < 4; i++) {
        q_out[i] = q_in[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    /* Normalize to maintain unit quaternion constraint */
    SMEKF_normalizeQuaternion(q_out);

    return;
}

/*! Performs sequential measurement update with a single quaternion measurement
    @param configData The configuration data
    @param q_meas Measured quaternion (scalar-first: [q0, q1, q2, q3])
    @param R Measurement noise covariance (3x3)
    @param K_out Output Kalman gain (6x3) - stored for covariance update
 */
void SMEKF_measurementUpdate(SMEKFConfig *configData, double q_meas[4],
                             double R[SMEKF_N_ATT_STATES * SMEKF_N_ATT_STATES],
                             double K_out[SMEKF_N_STATES * SMEKF_N_ATT_STATES])
{
    double q_meas_inv[4];
    double delta_q[4];
    double delta_z[3];
    double H_k[SMEKF_N_ATT_STATES * SMEKF_N_STATES];
    double P_H_T[SMEKF_N_STATES * SMEKF_N_ATT_STATES];
    double H_P_H_T[SMEKF_N_ATT_STATES * SMEKF_N_ATT_STATES];
    double S[SMEKF_N_ATT_STATES * SMEKF_N_ATT_STATES];
    double S_inv[SMEKF_N_ATT_STATES * SMEKF_N_ATT_STATES];
    double K_k[SMEKF_N_STATES * SMEKF_N_ATT_STATES];
    double delta_x[SMEKF_N_STATES];
    double q_update[4];
    double H_T[SMEKF_N_STATES * SMEKF_N_ATT_STATES];
    double delta_alpha[3];  /* Attitude error vector for exact quaternion update */
    double theta_norm;      /* Norm of attitude error */

    /* DIAGNOSTIC: Print quaternions before computing innovation */
    printf("\n=== SMEKF Measurement Update [t=%.2f] ===\n", configData->timeTag);
    printf("  q_meas (measurement) = [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           q_meas[0], q_meas[1], q_meas[2], q_meas[3]);
    printf("  q_hat  (propagated)  = [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           configData->quat_BN[0], configData->quat_BN[1], configData->quat_BN[2], configData->quat_BN[3]);

    /* Compute delta_q = q_meas * q_hat^-1 */
    SMEKF_quaternionInverse(configData->quat_BN, q_meas_inv);
    SMEKF_quaternionMultiply(q_meas, q_meas_inv, delta_q);
    SMEKF_normalizeQuaternion(delta_q);

    printf("  delta_q = [%+.6f, %+.6f, %+.6f, %+.6f] (q_meas ⊗ q_hat^-1)\n",
           delta_q[0], delta_q[1], delta_q[2], delta_q[3]);

    /* Innovation: delta_z = 2 * [delta_q1; delta_q2; delta_q3] */
    delta_z[0] = 2.0 * delta_q[1];
    delta_z[1] = 2.0 * delta_q[2];
    delta_z[2] = 2.0 * delta_q[3];

    /* Measurement matrix: H_k = [I(3x3), 0(3x3)] */
    mSetZero(H_k, SMEKF_N_ATT_STATES, SMEKF_N_STATES);
    mSetIdentity(H_k, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES);

    /* Compute Kalman gain: K = P * H^T * (H * P * H^T + R)^-1 */
    mTranspose(H_k, SMEKF_N_ATT_STATES, SMEKF_N_STATES, H_T);
    mMultM(configData->covar, SMEKF_N_STATES, SMEKF_N_STATES,
           H_T, SMEKF_N_STATES, SMEKF_N_ATT_STATES, P_H_T);
    mMultM(H_k, SMEKF_N_ATT_STATES, SMEKF_N_STATES,
           P_H_T, SMEKF_N_STATES, SMEKF_N_ATT_STATES, H_P_H_T);
    mAdd(H_P_H_T, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, R, S);

    if (mInverse(S, SMEKF_N_ATT_STATES, S_inv) < 0) {
        /* Singular matrix, skip this measurement */
        _bskLog(configData->bskLogger, BSK_WARNING, "SMEKF: Singular innovation covariance, skipping measurement");
        return;
    }

    mMultM(P_H_T, SMEKF_N_STATES, SMEKF_N_ATT_STATES,
           S_inv, SMEKF_N_ATT_STATES, SMEKF_N_ATT_STATES, K_k);

    /* Store Kalman gain for covariance update */
    mCopy(K_k, SMEKF_N_STATES, SMEKF_N_ATT_STATES, K_out);

    /* State update: delta_x = K * delta_z */
    mMultV(K_k, SMEKF_N_STATES, SMEKF_N_ATT_STATES, delta_z, delta_x);

    /* DIAGNOSTIC LOGGING */
    {
        double delta_z_norm = sqrt(delta_z[0]*delta_z[0] + delta_z[1]*delta_z[1] + delta_z[2]*delta_z[2]);
        double delta_z_deg = delta_z_norm * 180.0 / M_PI;
        double delta_x_att_norm = sqrt(delta_x[0]*delta_x[0] + delta_x[1]*delta_x[1] + delta_x[2]*delta_x[2]);
        double delta_x_att_deg = delta_x_att_norm * 180.0 / M_PI;

        printf("SMEKF DEBUG [t=%.2f]:\n", configData->timeTag);
        printf("  Innovation delta_z = [%+.6f, %+.6f, %+.6f] rad\n", delta_z[0], delta_z[1], delta_z[2]);
        printf("  |delta_z| = %.6f rad (%.2f deg)\n", delta_z_norm, delta_z_deg);
        printf("  State update delta_x:\n");
        printf("    Attitude: [%+.6e, %+.6e, %+.6e] rad (|δα| = %.4f deg)\n",
               delta_x[0], delta_x[1], delta_x[2], delta_x_att_deg);
        printf("    Bias:     [%+.6e, %+.6e, %+.6e] rad/s\n", delta_x[3], delta_x[4], delta_x[5]);
        printf("  Bias evolution:\n");
        printf("    Before: [%+.6e, %+.6e, %+.6e] rad/s\n",
               configData->bias[0], configData->bias[1], configData->bias[2]);
        printf("    After:  [%+.6e, %+.6e, %+.6e] rad/s\n",
               configData->bias[0] + delta_x[3],
               configData->bias[1] + delta_x[4],
               configData->bias[2] + delta_x[5]);
        printf("    (%.2f, %.2f, %.2f) deg/hr\n",
               (configData->bias[0] + delta_x[3]) * 180.0/M_PI * 3600.0,
               (configData->bias[1] + delta_x[4]) * 180.0/M_PI * 3600.0,
               (configData->bias[2] + delta_x[5]) * 180.0/M_PI * 3600.0);
    }

    /* Update quaternion using exact formula (not linear approximation)
       Build delta_q from attitude error delta_x[0:2], then multiply:
       q_new = delta_q ⊗ q_old */

    /* Extract attitude error components */
    delta_alpha[0] = delta_x[0];
    delta_alpha[1] = delta_x[1];
    delta_alpha[2] = delta_x[2];

    /* Compute norm of attitude error */
    theta_norm = sqrt(delta_alpha[0]*delta_alpha[0] +
                      delta_alpha[1]*delta_alpha[1] +
                      delta_alpha[2]*delta_alpha[2]);

    /* Build error quaternion delta_q from attitude error vector */
    if (theta_norm > 1e-8) {
        /* Exact formula for larger errors:
           delta_q = [cos(|δα|/2), sin(|δα|/2) * δα/|δα|] */
        double half_theta = theta_norm / 2.0;
        double sin_half = sin(half_theta);
        double cos_half = cos(half_theta);

        delta_q[0] = cos_half;  /* scalar part */
        delta_q[1] = sin_half * delta_alpha[0] / theta_norm;
        delta_q[2] = sin_half * delta_alpha[1] / theta_norm;
        delta_q[3] = sin_half * delta_alpha[2] / theta_norm;
    } else {
        /* Small-angle approximation: delta_q ≈ [1, δα/2] */
        delta_q[0] = 1.0;
        delta_q[1] = delta_alpha[0] / 2.0;
        delta_q[2] = delta_alpha[1] / 2.0;
        delta_q[3] = delta_alpha[2] / 2.0;
    }

    /* Normalize delta_q to ensure it's a unit quaternion */
    SMEKF_normalizeQuaternion(delta_q);

    /* Quaternion multiplication: q_new = delta_q ⊗ q_old */
    SMEKF_quaternionMultiply(delta_q, configData->quat_BN, q_update);

    /* Normalize result (should already be normalized, but for numerical safety) */
    SMEKF_normalizeQuaternion(q_update);
    v4Copy(q_update, configData->quat_BN);

    /* Update bias: bias = bias + delta_x[3:5] */
    configData->bias[0] += delta_x[3];
    configData->bias[1] += delta_x[4];
    configData->bias[2] += delta_x[5];

    return;
}

/*! Computes the THETA matrix (4x3) for quaternion error state update
    THETA(q) = [-q1, -q2, -q3;
                 q0, -q3,  q2;
                 q3,  q0, -q1;
                -q2,  q1,  q0]
    @param q Input quaternion (scalar-first: [q0, q1, q2, q3])
    @param THETA Output matrix (4x3, stored row-major)
 */
void SMEKF_computeThetaMatrix(double q[4], double THETA[4 * SMEKF_N_ATT_STATES])
{
    double q0 = q[0];
    double q1 = q[1];
    double q2 = q[2];
    double q3 = q[3];

    /* Row 0: [-q1, -q2, -q3] */
    THETA[0] = -q1;  THETA[1] = -q2;  THETA[2] = -q3;

    /* Row 1: [q0, -q3, q2] */
    THETA[3] = q0;   THETA[4] = -q3;  THETA[5] = q2;

    /* Row 2: [q3, q0, -q1] */
    THETA[6] = q3;   THETA[7] = q0;   THETA[8] = -q1;

    /* Row 3: [-q2, q1, q0] */
    THETA[9] = -q2;  THETA[10] = q1;  THETA[11] = q0;

    return;
}

/*! Quaternion multiplication: q_out = q1 * q2
    Uses Hamilton convention with scalar-first format
    @param q1 First quaternion (scalar-first: [q0, q1, q2, q3])
    @param q2 Second quaternion (scalar-first: [q0, q1, q2, q3])
    @param q_out Output quaternion (scalar-first: [q0, q1, q2, q3])
 */
void SMEKF_quaternionMultiply(double q1[4], double q2[4], double q_out[4])
{
    double temp[4];

    /* q_out = q1 * q2 using Hamilton product */
    temp[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    temp[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    temp[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    temp[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

    v4Copy(temp, q_out);
    return;
}

/*! Quaternion inverse (conjugate for unit quaternions)
    @param q Input quaternion (scalar-first: [q0, q1, q2, q3])
    @param q_inv Output inverse quaternion (scalar-first: [q0, -q1, -q2, -q3])
 */
void SMEKF_quaternionInverse(double q[4], double q_inv[4])
{
    q_inv[0] = q[0];
    q_inv[1] = -q[1];
    q_inv[2] = -q[2];
    q_inv[3] = -q[3];
    return;
}

/*! Normalizes a quaternion to unit length
    @param q Input/output quaternion (scalar-first: [q0, q1, q2, q3])
 */
void SMEKF_normalizeQuaternion(double q[4])
{
    double norm = v4Norm(q);
    if (norm > 1e-10) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    } else {
        /* Default to identity quaternion if near-zero */
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    }
    return;
}
