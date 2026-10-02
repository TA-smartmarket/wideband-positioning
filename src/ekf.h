// ============================================================================
//  ekf.h — 2D Extended Kalman Filter for tag tracking
//
//  State      x = [ px, py, vx, vy ]^T          (metres, metres/second)
//  Motion     constant velocity + white acceleration noise (discrete model)
//  Measurement range to a known anchor i:
//
//                 h_i(x) = sqrt( (px - ax_i)^2 + (py - ay_i)^2 )
//
//  h is NON-LINEAR in the state, so the filter is "extended": the measurement
//  function is linearised around the current estimate via its Jacobian
//
//                 H_i = [ (px-ax_i)/d , (py-ay_i)/d , 0 , 0 ]
//
//  and the standard Kalman update is applied to the linearised system.
//
//  Design notes
//  ------------
//  * One range update per anchor per cycle: the anchors are processed
//    sequentially (each is an independent scalar measurement), which avoids
//    building an NxN innovation matrix and is numerically simpler.
//  * Innovation gating (chi-square style, 3-sigma) rejects outliers such as
//    NLOS reflections before they can corrupt the estimate.
//  * Process noise follows the standard constant-velocity discretisation
//      Q = sigma_a^2 * [[dt^4/4, 0, dt^3/2, 0],
//                       [0, dt^4/4, 0, dt^3/2],
//                       [dt^3/2, 0, dt^2,   0],
//                       [0, dt^3/2, 0, dt^2  ]]
//
//  The same maths is mirrored on the server (server/app.py, class TagEKF) so
//  the device and the server agree.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <math.h>

#define EKF_N 4          // px, py, vx, vy

struct Ekf {
    float    x[EKF_N];              // state estimate
    float    P[EKF_N][EKF_N];       // error covariance
    float    sigma_a;               // process noise: acceleration (m/s^2)
    float    sigma_r;               // measurement noise: range (m)
    bool     init;                  // has a valid state
    uint32_t last_ms;               // timestamp of the last predict
    uint16_t updates;               // accepted measurement count
    float    last_innov;            // last accepted innovation (m), for tuning
};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
inline void ekfMatZero(float M[EKF_N][EKF_N])
{
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) M[i][j] = 0.0f;
}

inline void ekfMatIdentity(float M[EKF_N][EKF_N])
{
    ekfMatZero(M);
    for (int i = 0; i < EKF_N; i++) M[i][i] = 1.0f;
}

// ---------------------------------------------------------------------------
// initialisation — seed the state from a first position fix
// ---------------------------------------------------------------------------
inline void ekfInit(Ekf &e, float px, float py, float sigma_a = 1.0f,
                    float sigma_r = 0.15f)
{
    for (int i = 0; i < EKF_N; i++) e.x[i] = 0.0f;
    e.x[0] = px;
    e.x[1] = py;
    e.x[2] = 0.0f;               // unknown velocity
    e.x[3] = 0.0f;

    ekfMatZero(e.P);
    e.P[0][0] = 1.0f;            // position: ~1 m uncertainty on the first fix
    e.P[1][1] = 1.0f;
    e.P[2][2] = 4.0f;            // velocity: very uncertain at start
    e.P[3][3] = 4.0f;

    e.sigma_a  = sigma_a;
    e.sigma_r  = sigma_r;
    e.init     = true;
    e.last_ms  = millis();
    e.updates  = 0;
    e.last_innov = 0.0f;
}

// ---------------------------------------------------------------------------
// prediction:  x = F x ,  P = F P F^T + Q     (F = constant velocity)
// ---------------------------------------------------------------------------
inline void ekfPredict(Ekf &e, float dt)
{
    if (!e.init || dt <= 0.0f) return;
    if (dt > 2.0f) dt = 2.0f;                 // clamp after long stalls

    // --- state ------------------------------------------------------------
    e.x[0] += e.x[2] * dt;                    // px += vx*dt
    e.x[1] += e.x[3] * dt;                    // py += vy*dt

    // --- covariance: P = F P F^T ------------------------------------------
    // F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    float FP[EKF_N][EKF_N];
    for (int j = 0; j < EKF_N; j++) {
        FP[0][j] = e.P[0][j] + dt * e.P[2][j];
        FP[1][j] = e.P[1][j] + dt * e.P[3][j];
        FP[2][j] = e.P[2][j];
        FP[3][j] = e.P[3][j];
    }
    float FPFt[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        FPFt[i][0] = FP[i][0] + dt * FP[i][2];
        FPFt[i][1] = FP[i][1] + dt * FP[i][3];
        FPFt[i][2] = FP[i][2];
        FPFt[i][3] = FP[i][3];
    }

    // --- process noise Q (constant-velocity discretisation) ---------------
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float q   = e.sigma_a * e.sigma_a;
    const float Q[EKF_N][EKF_N] = {
        { q * dt4 / 4.0f, 0.0f,           q * dt3 / 2.0f, 0.0f           },
        { 0.0f,           q * dt4 / 4.0f, 0.0f,           q * dt3 / 2.0f },
        { q * dt3 / 2.0f, 0.0f,           q * dt2,        0.0f           },
        { 0.0f,           q * dt3 / 2.0f, 0.0f,           q * dt2        },
    };

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            e.P[i][j] = FPFt[i][j] + Q[i][j];
}

// ---------------------------------------------------------------------------
// update with one range measurement to anchor (ax, ay)
//
// Returns true when the measurement was accepted, false when it was rejected
// by the innovation gate (NLOS / outlier).
// ---------------------------------------------------------------------------
inline bool ekfUpdateRange(Ekf &e, float ax, float ay, float range, float gate_sigma = 3.0f)
{
    if (!e.init || range <= 0.01f) return false;

    const float dx = e.x[0] - ax;
    const float dy = e.x[1] - ay;
    float d = sqrtf(dx * dx + dy * dy);
    if (d < 1e-3f) d = 1e-3f;

    // --- measurement Jacobian H = [dx/d, dy/d, 0, 0] ----------------------
    const float H[EKF_N] = { dx / d, dy / d, 0.0f, 0.0f };

    // --- innovation  y = z - h(x) ----------------------------------------
    const float innov = range - d;

    // --- S = H P H^T + R  (scalar) ---------------------------------------
    float PHt[EKF_N];
    for (int i = 0; i < EKF_N; i++)
        PHt[i] = e.P[i][0] * H[0] + e.P[i][1] * H[1] + e.P[i][2] * H[2] + e.P[i][3] * H[3];

    const float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + H[3] * PHt[3]
                  + e.sigma_r * e.sigma_r;
    if (S < 1e-9f) return false;

    // --- innovation gate: reject if |y| > gate_sigma * sqrt(S) -----------
    const float gate = gate_sigma * sqrtf(S);
    if (fabsf(innov) > gate) return false;

    // --- Kalman gain  K = P H^T / S --------------------------------------
    float K[EKF_N];
    for (int i = 0; i < EKF_N; i++) K[i] = PHt[i] / S;

    // --- state update ----------------------------------------------------
    for (int i = 0; i < EKF_N; i++) e.x[i] += K[i] * innov;

    // --- covariance update  P = (I - K H) P ------------------------------
    float Pn[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            Pn[i][j] = e.P[i][j] - K[i] * (H[0] * e.P[0][j] + H[1] * e.P[1][j]
                                         + H[2] * e.P[2][j] + H[3] * e.P[3][j]);

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) e.P[i][j] = Pn[i][j];

    e.updates++;
    e.last_innov = innov;
    return true;
}

// ---------------------------------------------------------------------------
// read the current estimate
// ---------------------------------------------------------------------------
inline void ekfPosition(const Ekf &e, float &px, float &py, float &vx, float &vy)
{
    px = e.x[0]; py = e.x[1];
    vx = e.x[2]; vy = e.x[3];
}

// Position 1-sigma uncertainty (sqrt of the trace of the position block).
inline float ekfPositionSigma(const Ekf &e)
{
    return sqrtf(fmaxf(e.P[0][0], 0.0f) + fmaxf(e.P[1][1], 0.0f));
}
