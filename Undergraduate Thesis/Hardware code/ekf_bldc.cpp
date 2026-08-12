#include "ekf_bldc.h"
#include <math.h>

#define PK(i,j) Pk[(i)*4+(j)]

static inline float wrapAngle(float a)
{
    float w = atan2f(sinf(a), cosf(a));
    if (w < 0.0f) w += 2.0f * (float)M_PI;
    return w;
}

void EKF_BLDC::update(float u_alpha, float u_beta,
                      float y_alpha, float y_beta, float dt)
{
    float u[2] = { u_alpha, u_beta };

    float x_bar[4];
    _predict_state(x, u, x_bar, dt);
    x_bar[3] = wrapAngle(x_bar[3]);

    x_bar_dbg[0] = x_bar[0];
    x_bar_dbg[1] = x_bar[1];

    float Fd[4][4];
    _compute_Fd(x, Fd, dt);
    _predict_cov(Fd);

    float Kk[4][2];
    _compute_gain(Kk);

    innov_dbg[0] = y_alpha - x_bar[0];
    innov_dbg[1] = y_beta  - x_bar[1];

    _correct_state(x_bar, Kk, y_alpha, y_beta);
    x[3] = wrapAngle(x[3]);

    _correct_cov(Kk);
}

float EKF_BLDC::getPredictedIalpha() const { return x_bar_dbg[0]; }
float EKF_BLDC::getPredictedIbeta()  const { return x_bar_dbg[1]; }
float EKF_BLDC::getInnovAlpha()      const { return innov_dbg[0]; }
float EKF_BLDC::getInnovBeta()       const { return innov_dbg[1]; }
float EKF_BLDC::getPk(int i, int j)  const { return Pk[i*4+j]; }

void EKF_BLDC::_predict_state(const float* xk, const float* u,
                               float* x_bar, float dt)
{
    float i_alpha = xk[0];
    float i_beta  = xk[1];
    float omega_e = xk[2];
    float theta_e = xk[3];

    float e_alpha = -sinf(theta_e);   // fα(θe)
    float e_beta  =  cosf(theta_e);   // fβ(θe)

    float f0 = (1.0f / Ls) * (u[0] - Rs * i_alpha - ke * omega_e * e_alpha);
    float f1 = (1.0f / Ls) * (u[1] - Rs * i_beta  - ke * omega_e * e_beta);
    float f2 = (P / J) * ((3.0f / 2.0f) * P * ke * (e_alpha * i_alpha + e_beta * i_beta)
                           - (F_fric / P) * omega_e - Tm);
    float f3 = omega_e;

    x_bar[0] = i_alpha + f0 * dt;
    x_bar[1] = i_beta  + f1 * dt;
    x_bar[2] = omega_e + f2 * dt;
    x_bar[3] = theta_e + f3 * dt;
}

void EKF_BLDC::_compute_Fd(const float* xk, float Fd[4][4], float dt)
{
    float i_alpha = xk[0];
    float i_beta  = xk[1];
    float omega_e = xk[2];
    float theta_e = xk[3];

    float e_alpha  = -sinf(theta_e);   // fα(θe)
    float de_alpha = -cosf(theta_e);   // f'α(θe) = d(-sinθ)/dθ = -cosθ
    float e_beta   =  cosf(theta_e);   // fβ(θe)
    float de_beta  = -sinf(theta_e);   // f'β(θe) = d(cosθ)/dθ  = -sinθ

    // Row 0
    float df0_dia = -Rs / Ls;
    float df0_dw  = -(ke / Ls) * e_alpha;
    float df0_dth = -(ke / Ls) * omega_e * de_alpha;

    // Row 1
    float df1_dib = -Rs / Ls;
    float df1_dw  = -(ke / Ls) * e_beta;
    float df1_dth = -(ke / Ls) * omega_e * de_beta;

    // the corrected inner (3/2)*P*ke torque coefficient.
    float PJ      = P / J;
    float coeff   = (3.0f / 2.0f) * P * ke;   // FIXED: was (3.0f/2.0f)*ke
    float df2_dia = PJ * coeff * e_alpha;
    float df2_dib = PJ * coeff * e_beta;
    float df2_dw  = -(F_fric / J);            // unchanged: (P/J)*(F_fric/P) = F_fric/J
    float df2_dth = PJ * coeff * (de_alpha * i_alpha + de_beta * i_beta);

    // Row 3: dθ/dt = ωe  →  df3/dωe = 1, others = 0

    Fd[0][0] = 1.0f + df0_dia * dt;
    Fd[0][1] = 0.0f;
    Fd[0][2] =        df0_dw  * dt;
    Fd[0][3] =        df0_dth * dt;

    Fd[1][0] = 0.0f;
    Fd[1][1] = 1.0f + df1_dib * dt;
    Fd[1][2] =        df1_dw  * dt;
    Fd[1][3] =        df1_dth * dt;

    Fd[2][0] =        df2_dia * dt;
    Fd[2][1] =        df2_dib * dt;
    Fd[2][2] = 1.0f + df2_dw  * dt;
    Fd[2][3] =        df2_dth * dt;

    Fd[3][0] = 0.0f;
    Fd[3][1] = 0.0f;
    Fd[3][2] =        1.0f    * dt;
    Fd[3][3] = 1.0f;
}

void EKF_BLDC::_predict_cov(const float Fd[4][4])
{
    float tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += Fd[i][k] * PK(k, j);
            tmp[i][j] = s;
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += tmp[i][k] * Fd[j][k];
            PK(i, j) = s + ((i == j) ? Q[i] : 0.0f);
        }
}

void EKF_BLDC::_compute_gain(float Kk[4][2])
{
    float S[2][2];
    S[0][0] = PK(0,0) + R[0];
    S[0][1] = PK(0,1);
    S[1][0] = PK(1,0);
    S[1][1] = PK(1,1) + R[1];

    float Sinv[2][2];
    _inv2x2(S, Sinv);

    for (int i = 0; i < 4; i++) {
        float ph0 = PK(i, 0);
        float ph1 = PK(i, 1);
        Kk[i][0] = ph0 * Sinv[0][0] + ph1 * Sinv[1][0];
        Kk[i][1] = ph0 * Sinv[0][1] + ph1 * Sinv[1][1];
    }
}

void EKF_BLDC::_correct_state(const float* x_bar, const float Kk[4][2],
                               float y_alpha, float y_beta)
{
    float innov0 = y_alpha - x_bar[0];
    float innov1 = y_beta  - x_bar[1];
    for (int i = 0; i < 4; i++)
        x[i] = x_bar[i] + Kk[i][0] * innov0 + Kk[i][1] * innov1;
}

void EKF_BLDC::_correct_cov(const float Kk[4][2])
{
    float IKH[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float KH_ij = (j == 0) ? Kk[i][0] : (j == 1) ? Kk[i][1] : 0.0f;
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH_ij;
        }

    float tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += IKH[i][k] * PK(k, j);
            tmp[i][j] = s;
        }

    float Pk_new[16];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += tmp[i][k] * IKH[j][k];
            Pk_new[i*4+j] = s;
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Pk_new[i*4+j] += Kk[i][0] * R[0] * Kk[j][0]
                            + Kk[i][1] * R[1] * Kk[j][1];

    for (int k = 0; k < 16; k++)
        Pk[k] = Pk_new[k];
}

bool EKF_BLDC::_inv2x2(const float M[2][2], float Minv[2][2])
{
    float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    if (fabsf(det) < 1e-30f) {
        Minv[0][0] = 1.0f; Minv[0][1] = 0.0f;
        Minv[1][0] = 0.0f; Minv[1][1] = 1.0f;
        return false;
    }
    float inv_det  = 1.0f / det;
    Minv[0][0] =  M[1][1] * inv_det;
    Minv[0][1] = -M[0][1] * inv_det;
    Minv[1][0] = -M[1][0] * inv_det;
    Minv[1][1] =  M[0][0] * inv_det;
    return true;
}
