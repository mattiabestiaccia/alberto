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
 * questAttDet.c - Algoritmo QUEST per determinazione assetto da osservazioni vettoriali
 *
 * Questo modulo implementa l'algoritmo QUEST (QUaternion ESTimator) per la determinazione
 * statica dell'assetto di un satellite. QUEST risolve il "problema di Wahba": data una serie
 * di vettori misurati in terna Body e i corrispondenti vettori di riferimento in terna Inerziale,
 * calcola la matrice di rotazione (e il quaternione) ottimi che minimizzano l'errore pesato.
 *
 * Input utilizzati:
 * - Direzione Sole in terna Body (da simpleNavObject.vehSunPntBdy)
 * - Direzione Sole in terna Inerziale (da ephemerisConverter - posizione Sole)
 * - Campo magnetico in terna Body (da magnetometro TAM)
 * - Campo magnetico in terna Inerziale (da modello WMM)
 *
 * Output:
 * - Quaternione ottimo q_BN (Body to Inertial)
 * - Parametri MRP sigma_BN
 *
 * Algoritmo:
 * 1. Costruisce la matrice di profilo attitudinale B = sum(w_i * v_i * w_i^T)
 * 2. Risolve l'equazione caratteristica per trovare l'autovalore massimo lambda_max
 * 3. Calcola il quaternione ottimo come autovettore corrispondente a lambda_max
 *
 * Nota: QUEST è sensibile alla qualità dei vettori di input. Se i due vettori (Sole e Mag)
 * sono troppo allineati (quasi paralleli), la soluzione diventa instabile.
 */

#include "questAttDet.h"
#include <string.h>
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/* Le funzioni helper sono dichiarate nel file header */

/*!
 \verbatim embed:rst
    Inizializza il messaggio di output del modulo di tipo :ref:`NavAttMsgPayload`
 \endverbatim

 @param configData Dati di configurazione del modulo
 @param moduleID Identificatore del modulo
 */
void SelfInit_questAttDet(questAttDetConfig *configData, int64_t moduleID)
{
    NavAttMsg_C_init(&configData->navStateOutMsg);
}


/*! Esegue un reset completo del modulo. Le variabili locali che mantengono stati variabili
 nel tempo tra le chiamate vengono resettate ai loro valori di default.

 @param configData Dati di configurazione del modulo
 @param callTime Tempo della chiamata in nanosecondi
 @param moduleID Identificatore del modulo
 */
void Reset_questAttDet(questAttDetConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Imposta i parametri di default se non già configurati */
    /* Nota: lambda0 = 0.0 significa usare metodo adattivo con quaternione precedente */
    if (configData->sunWeight == 0.0) {
        configData->sunWeight = 1.0;
    }
    if (configData->magWeight == 0.0) {
        configData->magWeight = 1.0;
    }
    if (configData->maxIterations == 0) {
        configData->maxIterations = QUEST_MAX_ITERATIONS;
    }
    if (configData->tolerance == 0.0) {
        configData->tolerance = QUEST_TOLERANCE;
    }
    if (configData->useSunVector == 0 && configData->useMagVector == 0) {
        configData->useSunVector = 1;
        configData->useMagVector = 1;
    }

    /* Reset internal variables */
    configData->numVectors = 0;
    configData->lambda_max = 0.0;
    configData->lambda_initial = 0.0;
    configData->numIterations = 0;

    /* Initialize previous quaternion to identity [1, 0, 0, 0] */
    configData->quat_BN_prev[0] = 1.0;
    configData->quat_BN_prev[1] = 0.0;
    configData->quat_BN_prev[2] = 0.0;
    configData->quat_BN_prev[3] = 0.0;
    configData->firstRun = 1;

    /* Initialize current quaternion to identity as well */
    configData->quat_BN[0] = 1.0;
    configData->quat_BN[1] = 0.0;
    configData->quat_BN[2] = 0.0;
    configData->quat_BN[3] = 0.0;

    /* Write initial output message to avoid "not properly initialized" warnings */
    /* This ensures the message is valid even if Update is never called */
    {
        NavAttMsgPayload navMsgOut;
        double sigma_BN[3];

        /* Initialize output structure */
        navMsgOut = NavAttMsg_C_zeroMsgPayload();

        /* Convert identity quaternion to MRP (will be [0,0,0]) */
        EP2MRP(configData->quat_BN, sigma_BN);
        v3Copy(sigma_BN, navMsgOut.sigma_BN);
        v3SetZero(navMsgOut.omega_BN_B);  /* Zero angular velocity */
        navMsgOut.timeTag = callTime * NANO2SEC;

        /* Write initial message */
        NavAttMsg_C_write(&navMsgOut, &configData->navStateOutMsg, moduleID, callTime);
    }
}

/*! Implementa l'algoritmo QUEST per la determinazione dell'assetto da osservazioni vettoriali.
 Calcola il quaternione ottimo che minimizza la funzione di loss di Wahba.

 @param configData Dati di configurazione del modulo
 @param callTime Tempo della chiamata in nanosecondi
 @param moduleID Identificatore del modulo
 */
void Update_questAttDet(questAttDetConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* Osservazioni vettoriali in terna Body */
    double v[QUEST_MAX_VECTORS][3];
    /* Vettori di riferimento in terna Inerziale */
    double w[QUEST_MAX_VECTORS][3];
    /* Pesi per ogni osservazione */
    double weights[QUEST_MAX_VECTORS];

    /* Matrici e parametri dell'algoritmo QUEST */
    double B[3][3];                 /* Matrice di profilo attitudinale */
    double S[3][3];                 /* S = B + B^T */
    double z[3];                    /* z = sum(a_i * (v_i x w_i)) */
    double sigma, kappa, delta;     /* Parametri della matrice K */
    double a, b, c, d;              /* Coefficienti equazione caratteristica */
    double lambda_max;              /* Autovalore massimo */
    double q_BN[4];                 /* Quaternione ottimo (formato scalar-first) */

    NavAttMsgPayload outputAtt;
    NavAttMsgPayload sunlineBuffer;
    EphemerisMsgPayload sunEphemBuffer;
    SCStatesMsgPayload scPosBuffer;
    TAMSensorBodyMsgPayload tamBuffer;
    MagneticFieldMsgPayload magFieldBuffer;

    int vecIdx = 0;
    int iterations = 0;

    /* Initialize output message */
    outputAtt = NavAttMsg_C_zeroMsgPayload();

    /* Collect vector observations */
    configData->numVectors = 0;

    /* Sun vector observation */
    if (configData->useSunVector &&
        NavAttMsg_C_isLinked(&configData->sunlineInMsg) &&
        EphemerisMsg_C_isLinked(&configData->sunEphemerisInMsg) &&
        SCStatesMsg_C_isLinked(&configData->scPositionInMsg))
    {
        double r_SB_N[3];  /* Vector from spacecraft to sun in inertial frame */

        sunlineBuffer = NavAttMsg_C_read(&configData->sunlineInMsg);
        sunEphemBuffer = EphemerisMsg_C_read(&configData->sunEphemerisInMsg);
        scPosBuffer = SCStatesMsg_C_read(&configData->scPositionInMsg);

        /* Body frame measurement (from CSS or other sun sensor) - normalized */
        v3Normalize(sunlineBuffer.vehSunPntBdy, v[vecIdx]);

        /* Inertial frame reference: r_SB_N = r_Sun_N - r_SC_N, then normalize */
        v3Subtract(sunEphemBuffer.r_BdyZero_N, scPosBuffer.r_BN_N, r_SB_N);
        v3Normalize(r_SB_N, w[vecIdx]);

        /* Weight */
        weights[vecIdx] = configData->sunWeight;

        vecIdx++;
        configData->numVectors++;
    }

    /* Magnetometer vector observation */
    if (configData->useMagVector &&
        TAMSensorBodyMsg_C_isLinked(&configData->tamSensorInMsg) &&
        MagneticFieldMsg_C_isLinked(&configData->magFieldInMsg))
    {
        tamBuffer = TAMSensorBodyMsg_C_read(&configData->tamSensorInMsg);
        magFieldBuffer = MagneticFieldMsg_C_read(&configData->magFieldInMsg);

        /* Body frame measurement (normalized) */
        v3Normalize(tamBuffer.tam_B, v[vecIdx]);

        /* Inertial frame reference (normalized) */
        v3Normalize(magFieldBuffer.magField_N, w[vecIdx]);

        /* Weight */
        weights[vecIdx] = configData->magWeight;

        vecIdx++;
        configData->numVectors++;
    }

    /* Check if we have enough observations */
    if (configData->numVectors < 2) {
        _bskLog(configData->bskLogger, BSK_WARNING,
                "questAttDet: Insufficient vector observations. Need at least 2 vectors for attitude determination.");
        return;
    }

    /* Step 1: Compute attitude profile matrix B */
    questAttDet_computeAttitudeProfile(weights, v, w, configData->numVectors, B);

    /* Step 2: Compute K-matrix components */
    questAttDet_computeKMatrix(B, S, z, &sigma, &kappa, &delta);

    /* Step 2b: Compute initial eigenvalue estimate lambda_0 */
    double lambda_initial;
    if (configData->lambda0 == 0.0) {
        /* Use adaptive method with previous quaternion */
        if (configData->firstRun == 1) {
            /* First run: use default value */
            lambda_initial = 1.0;
        } else {
            /* Compute lambda_0 = q_prev^T @ K @ q_prev
               where K is the 4x4 QUEST K-matrix:
               K = [S - sigma*I_3,  z  ]
                   [z^T,            sigma]
            */
            double q_prev[4] = {configData->quat_BN_prev[0],
                                configData->quat_BN_prev[1],
                                configData->quat_BN_prev[2],
                                configData->quat_BN_prev[3]};

            /* Build K matrix (4x4) */
            double K[4][4];

            /* Upper-left 3x3 block: S - sigma*I */
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    K[i][j] = S[i][j];
                    if (i == j) {
                        K[i][j] -= sigma;
                    }
                }
            }

            /* Upper-right column: z */
            K[0][3] = z[0];
            K[1][3] = z[1];
            K[2][3] = z[2];

            /* Lower-left row: z^T */
            K[3][0] = z[0];
            K[3][1] = z[1];
            K[3][2] = z[2];

            /* Lower-right element: sigma */
            K[3][3] = sigma;

            /* Compute K * q_prev */
            double Kq[4];
            for (int i = 0; i < 4; i++) {
                Kq[i] = 0.0;
                for (int j = 0; j < 4; j++) {
                    Kq[i] += K[i][j] * q_prev[j];
                }
            }

            /* Compute lambda_0 = q_prev^T * Kq */
            lambda_initial = 0.0;
            for (int i = 0; i < 4; i++) {
                lambda_initial += q_prev[i] * Kq[i];
            }
        }
    } else {
        /* Use user-specified value */
        lambda_initial = configData->lambda0;
    }

    /* Step 3: Compute characteristic equation coefficients */
    double zTz = v3Dot(z, z);
    double zTSz, zTS[3], S2z[3], zTS2z;
    m33MultV3(S, z, zTS);
    zTSz = v3Dot(z, zTS);
    m33MultV3(S, zTS, S2z);
    zTS2z = v3Dot(z, S2z);

    a = sigma * sigma - kappa;
    b = sigma * sigma + zTz;
    c = delta + zTSz;
    d = zTS2z;

    /* Step 4: Solve characteristic equation using Newton-Raphson */
    lambda_max = questAttDet_solveCharacteristicEq(a, b, c, d, sigma,
                                                    lambda_initial,
                                                    configData->tolerance,
                                                    configData->maxIterations,
                                                    &iterations);

    configData->lambda_max = lambda_max;
    configData->lambda_initial = lambda_initial;
    configData->numIterations = iterations;

    /* Step 5: Compute optimal quaternion */
    questAttDet_computeOptimalQuaternion(S, z, lambda_max, sigma, q_BN);

    /* Store quaternion in module variable for logging */
    v4Copy(q_BN, configData->quat_BN);

    /* Save current quaternion for next iteration's adaptive lambda_0 */
    v4Copy(q_BN, configData->quat_BN_prev);
    configData->firstRun = 0;

    /* Convert quaternion to MRP for output message (NavAttMsg uses MRP format) */
    double sigma_BN[3];
    EP2MRP(q_BN, sigma_BN);
    v3Copy(sigma_BN, outputAtt.sigma_BN);

    /* Write output message */
    NavAttMsg_C_write(&outputAtt, &configData->navStateOutMsg, moduleID, callTime);
}

/*! Computes the attitude profile matrix B = sum(a_i * v_i * w_i^T)

 @param weights Array of weights for each observation
 @param v Array of body frame observation vectors
 @param w Array of inertial frame reference vectors
 @param numVec Number of vector observations
 @param B Output 3x3 attitude profile matrix
 */
void questAttDet_computeAttitudeProfile(double weights[], double v[][3], double w[][3],
                                        int numVec, double B[3][3])
{
    double temp[3][3];
    int i;

    m33SetZero(B);

    for (i = 0; i < numVec; i++) {
        /* temp = v_i * w_i^T */
        v3OuterProduct(v[i], w[i], temp);

        /* B += a_i * temp */
        m33Scale(weights[i], temp, temp);
        m33Add(B, temp, B);
    }
}

/*! Computes K-matrix components: S, z, sigma, kappa, delta

 @param B Attitude profile matrix
 @param S Output: S = B + B^T
 @param z Output: z vector (sum of cross products)
 @param sigma Output: trace of B
 @param kappa Output: trace of adjoint of S
 @param delta Output: determinant of S
 */
void questAttDet_computeKMatrix(double B[3][3], double S[3][3], double z[3],
                                double *sigma, double *kappa, double *delta)
{
    double BT[3][3];

    /* S = B + B^T */
    m33Transpose(B, BT);
    m33Add(B, BT, S);

    /* z = [B23 - B32, B31 - B13, B12 - B21] */
    z[0] = B[1][2] - B[2][1];
    z[1] = B[2][0] - B[0][2];
    z[2] = B[0][1] - B[1][0];

    /* sigma = trace(B) */
    *sigma = m33Trace(B);

    /* kappa = trace(adj(S)) = S11*S22 + S11*S33 + S22*S33 - S12^2 - S13^2 - S23^2 */
    *kappa = S[0][0] * S[1][1] + S[0][0] * S[2][2] + S[1][1] * S[2][2]
           - S[0][1] * S[0][1] - S[0][2] * S[0][2] - S[1][2] * S[1][2];

    /* delta = det(S) */
    *delta = m33Determinant(S);
}

/*! Solves the characteristic equation using Newton-Raphson iteration:
 lambda^4 - (a+b)*lambda^2 - c*lambda + (ab + c*sigma - d) = 0

 @param a Coefficient a
 @param b Coefficient b
 @param c Coefficient c
 @param d Coefficient d
 @param sigma Sigma parameter
 @param lambda0 Initial eigenvalue estimate
 @param tolerance Convergence tolerance
 @param maxIter Maximum number of iterations
 @param iterations Output: actual number of iterations performed
 @return Maximum eigenvalue lambda_max
 */
double questAttDet_solveCharacteristicEq(double a, double b, double c, double d, double sigma,
                                         double lambda0, double tolerance, int maxIter, int *iterations)
{
    double lambda = lambda0;
    double lambda_new;
    double f, df;
    int iter;

    for (iter = 0; iter < maxIter; iter++) {
        /* f(lambda) = lambda^4 - (a+b)*lambda^2 - c*lambda + (ab + c*sigma - d) */
        f = lambda * lambda * lambda * lambda
          - (a + b) * lambda * lambda
          - c * lambda
          + (a * b + c * sigma - d);

        /* f'(lambda) = 4*lambda^3 - 2*(a+b)*lambda - c */
        df = 4.0 * lambda * lambda * lambda
           - 2.0 * (a + b) * lambda
           - c;

        /* Newton-Raphson update */
        lambda_new = lambda - f / df;

        /* Check convergence */
        if (fabs(lambda_new - lambda) < tolerance) {
            *iterations = iter + 1;
            return lambda_new;
        }

        lambda = lambda_new;
    }

    /* Maximum iterations reached */
    *iterations = maxIter;
    return lambda;
}

/*! Computes the optimal quaternion from the maximum eigenvalue

 @param S S matrix (S = B + B^T)
 @param z z vector
 @param lambda_max Maximum eigenvalue
 @param sigma Sigma parameter
 @param q Output quaternion in scalar-first format [q0, q1, q2, q3]
 */
void questAttDet_computeOptimalQuaternion(double S[3][3], double z[3], double lambda_max,
                                          double sigma, double q[4])
{
    double alpha;              /* Normalization factor */
    double gamma;              /* Scalar part before normalization */
    double X[3];               /* Vector part before normalization */
    double alpha_I[3][3];      /* (lambda_max + sigma) * I */
    double temp[3][3];         /* (lambda_max + sigma) * I - S */
    double temp_inv[3][3];     /* Inverse of temp */
    int success;

    /* gamma = lambda_max - sigma */
    gamma = lambda_max - sigma;

    /* Compute X = [(lambda_max + sigma)*I - S]^(-1) * z */
    /* First: temp = (lambda_max + sigma)*I - S */
    m33SetIdentity(alpha_I);
    m33Scale(lambda_max + sigma, alpha_I, alpha_I);
    m33Subtract(alpha_I, S, temp);

    /* Invert temp matrix */
    success = m33Inverse(temp, temp_inv);
    if (!success) {
        /* Matrix is singular, use alternative formulation */
        /* X = z / (lambda_max - sigma) when matrix is near-singular */
        v3Scale(1.0 / (lambda_max - sigma), z, X);
    } else {
        /* X = temp_inv * z */
        m33MultV3(temp_inv, z, X);
    }

    /* Compute normalization factor: alpha = sqrt(X^T * X + gamma^2) */
    alpha = sqrt(v3Dot(X, X) + gamma * gamma);

    /* Normalized quaternion (scalar-first format for Basilisk) */
    q[0] = gamma / alpha;    /* Scalar part */
    q[1] = X[0] / alpha;     /* Vector part x */
    q[2] = X[1] / alpha;     /* Vector part y */
    q[3] = X[2] / alpha;     /* Vector part z */
}
