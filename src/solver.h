// ============================================================================
//  solver.h — 2D position solver (multilateration)
//
//  Used by the device for standalone operation and mirrored on the server
//  (server/app.py, same maths). See docs/API.md section 7.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <math.h>

struct AnchorFix {
    float x, y;
    float range;
    bool  valid;
};

struct Position {
    bool  valid;
    float x, y;
    float confidence;
    bool  ambiguous;   // 2 anchors only, mirror solution could not be resolved
};

// Solve for (x, y) given N anchor observations.
//
//  - 0/1 anchors  -> invalid
//  - 2 anchors    -> circle intersection; of the two candidates pick the one
//                    inside the room. If that is inconclusive the result is
//                    flagged `ambiguous` and the candidate nearest the room
//                    centre is returned.
//  - 3+ anchors   -> linearised least squares (linear least squares on the
//                    difference of range equations), then one Gauss-Newton
//                    refinement step.
//
// room_w/room_h may be 0 to disable the room test.
inline Position solvePosition(const AnchorFix *a, uint8_t n, float room_w, float room_h)
{
    Position r = {false, 0.0f, 0.0f, 0.0f, false};

    // compact valid fixes
    AnchorFix f[MAX_ANCHORS];
    uint8_t m = 0;
    for (uint8_t i = 0; i < n && m < MAX_ANCHORS; i++)
        if (a[i].valid && a[i].range > 0.01f) f[m++] = a[i];

    if (m < 2) return r;

    const bool use_room = (room_w > 0.0f && room_h > 0.0f);

    if (m == 2) {
        // --- circle intersection -------------------------------------------
        const float x1 = f[0].x, y1 = f[0].y, r1 = f[0].range;
        const float x2 = f[1].x, y2 = f[1].y, r2 = f[1].range;

        const float dx = x2 - x1, dy = y2 - y1;
        const float d  = sqrtf(dx * dx + dy * dy);
        if (d < 1e-4f) return r;                       // same point
        if (d > r1 + r2 || d < fabsf(r1 - r2)) {        // no intersection
            r.valid = true;                             // best effort: midpoint
            r.x = (x1 + x2) * 0.5f;
            r.y = (y1 + y2) * 0.5f;
            r.confidence = 0.1f;
            r.ambiguous = true;
            return r;
        }

        const float aa = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
        const float hh = sqrtf(fmaxf(r1 * r1 - aa * aa, 0.0f));
        const float mx = x1 + aa * dx / d;
        const float my = y1 + aa * dy / d;

        const float c1x = mx + hh * dy / d, c1y = my - hh * dx / d;
        const float c2x = mx - hh * dy / d, c2y = my + hh * dx / d;

        bool in1 = true, in2 = true;
        if (use_room) {
            in1 = (c1x >= 0 && c1x <= room_w && c1y >= 0 && c1y <= room_h);
            in2 = (c2x >= 0 && c2x <= room_w && c2y >= 0 && c2y <= room_h);
        }

        r.valid = true;
        if (use_room && in1 != in2) {
            r.x = in1 ? c1x : c2x;
            r.y = in1 ? c1y : c2y;
            r.ambiguous = false;
        } else {
            // both inside / both outside: cannot disambiguate
            const float cx = room_w * 0.5f, cy = room_h * 0.5f;
            const float d1 = (c1x - cx) * (c1x - cx) + (c1y - cy) * (c1y - cy);
            const float d2 = (c2x - cx) * (c2x - cx) + (c2y - cy) * (c2y - cy);
            r.x = (d1 <= d2) ? c1x : c2x;
            r.y = (d1 <= d2) ? c1y : c2y;
            r.ambiguous = true;
        }
        r.confidence = 0.5f;
        return r;
    }

    // --- 3+ anchors: linearised least squares ------------------------------
    // r_i^2 = (x-xi)^2 + (y-yi)^2 ; subtract eq.0 to linearise
    const float x0 = f[0].x, y0 = f[0].y, r0 = f[0].range;

    double A[2] = {0, 0}, B[2] = {0, 0}, C = 0;   // normal equations
    for (uint8_t i = 1; i < m; i++) {
        const double ax = 2.0 * (f[i].x - x0);
        const double ay = 2.0 * (f[i].y - y0);
        const double b  = (f[i].range * f[i].range - r0 * r0)
                        - (double)f[i].x * f[i].x + (double)x0 * x0
                        - (double)f[i].y * f[i].y + (double)y0 * y0;
        A[0] += ax * ax; A[1] += ax * ay;
        B[0] += ax * b;  B[1] += ay * b;
        C    += ay * ay;
    }
    const double det = A[0] * C - A[1] * A[1];
    if (fabs(det) < 1e-9) return r;

    double px = (B[0] * C - B[1] * A[1]) / det;
    double py = (A[0] * B[1] - A[1] * B[0]) / det;

    // one Gauss-Newton refinement on the true (non-linear) residual
    for (uint8_t it = 0; it < 2; it++) {
        double j0 = 0, j1 = 0, j2 = 0, e0 = 0, e1 = 0;
        for (uint8_t i = 0; i < m; i++) {
            const double dx = px - f[i].x, dy = py - f[i].y;
            const double dist = sqrt(dx * dx + dy * dy) + 1e-9;
            const double res = dist - f[i].range;
            j0 += dx * dx / (dist * dist);
            j1 += dx * dy / (dist * dist);
            j2 += dy * dy / (dist * dist);
            e0 += dx * res / dist;
            e1 += dy * res / dist;
        }
        const double dj = j0 * j2 - j1 * j1;
        if (fabs(dj) < 1e-12) break;
        px -= (j2 * e0 - j1 * e1) / dj;
        py -= (j0 * e1 - j1 * e0) / dj;
    }

    // residual -> confidence
    double ss = 0;
    for (uint8_t i = 0; i < m; i++) {
        const double dx = px - f[i].x, dy = py - f[i].y;
        const double res = sqrt(dx * dx + dy * dy) - f[i].range;
        ss += res * res;
    }
    const double rms = sqrt(ss / m);

    r.valid = true;
    r.x = (float)px;
    r.y = (float)py;
    float conf = 1.0f - (float)(rms / 1.0f);   // 1 m RMS -> 0
    r.confidence = conf < 0.0f ? 0.0f : (conf > 1.0f ? 1.0f : conf);
    r.ambiguous = false;
    return r;
}