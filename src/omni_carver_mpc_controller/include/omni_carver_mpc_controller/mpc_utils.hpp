#ifndef OMNI_CARVER_MPC_CONTROLLER__MPC_UTILS_HPP_
#define OMNI_CARVER_MPC_CONTROLLER__MPC_UTILS_HPP_

// mpc_utils.hpp
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace omni_carver_mpc_controller
{

/**
 * Discrete‐time linearization of
 *   x_{k+1} = f(x_k,u_k) 
 * for omni‐wheel robot:
 *
 *   x_{k+1} = x_k + dt*(cosθ*vxb - sinθ*vyb)
 *   y_{k+1} = y_k + dt*(sinθ*vxb + cosθ*vyb)
 *   θ_{k+1} = θ_k + dt*ω - where θ is between -pi to pi
 *
 * Returns A = ∂f/∂x, B = ∂f/∂u at (x0,u0).
 */
inline void linearizeDynamics(
  const Eigen::Vector3d & x0,       // [ x,   y,   θ ]ᵀ
  const Eigen::Vector3d & u0,       // [ vxb, vyb, ω ]ᵀ
  double dt,
  Eigen::Matrix3d & A,
  Eigen::Matrix3d & B)
{
  double theta = x0(2);
  double vxb   = u0(0);
  double vyb   = u0(1);

  double c = std::cos(theta);
  double s = std::sin(theta);

  // A = ∂f/∂x
  A.setIdentity();
  // ∂x_{k+1}/∂θ = -dt*(sinθ*vxb + cosθ*vyb)
  A(0,2) = -dt*( s * vxb +  c * vyb);
  // ∂y_{k+1}/∂θ =  dt*(cosθ*vxb - sinθ*vyb)
  A(1,2) =  dt*( c * vxb -  s * vyb);
  // θ update has no x,y‐dependence beyond identity(2,2)=1.

  // B = ∂f/∂u
  B.setZero();
  // Position w.r.t. vxb, vyb
  B(0,0) =  dt * c;   // ∂x_{k+1}/∂vxb
  B(0,1) = -dt * s;   // ∂x_{k+1}/∂vyb
  B(1,0) =  dt * s;   // ∂y_{k+1}/∂vxb
  B(1,1) =  dt * c;   // ∂y_{k+1}/∂vyb
  // Theta w.r.t ω
  B(2,2) =  dt;       // ∂θ_{k+1}/∂ω
}

/**
 * Build the discrete‐time prediction matrices:
 *   X_pred = A_bar * x0 + B_bar * U
 *
 * A_bar: (n*N)×n, B_bar: (n*N)×(m*N)
 *
 * @param A       The n×n state Jacobian
 * @param B       The n×m input Jacobian
 * @param N       Prediction horizon
 * @param A_bar   [out] resized to (n*N, n)
 * @param B_bar   [out] resized to (n*N, m*N)
 */
inline void buildPredictionMatrices(
  const Eigen::MatrixXd & A,
  const Eigen::MatrixXd & B,
  unsigned int N,
  Eigen::MatrixXd & A_bar,
  Eigen::MatrixXd & B_bar)
{
  const int n = A.rows();
  const int m = B.cols();

  // Resize output
  A_bar .setZero(n * N, n);
  B_bar .setZero(n * N, m * N);

  // Precompute powers of A
  std::vector<Eigen::MatrixXd> A_pow(N);
  A_pow[0] = A;
  for (unsigned int i = 1; i < N; ++i) {
    A_pow[i] = A_pow[i-1] * A;
  }

  // Fill A_bar: row‐blocks are [A; A^2; …; A^N]
  for (unsigned int i = 0; i < N; ++i) {
    A_bar.block(i*n, 0, n, n) = A_pow[i];
  }

  // Fill B_bar: each block (i,j) for j≤i is A^(i−j) * B
  for (unsigned int i = 0; i < N; ++i) {
    for (unsigned int j = 0; j <= i; ++j) {
      // block at rows [i*n..], cols [j*m..]
      B_bar.block(i*n, j*m, n, m) = A_pow[i - j] * B;
    }
  }
}

/**
 * Assemble the QP cost:
 *   J(U) = ½ Uᵀ H U + gᵀ U
 * from stage costs Q, R and reference states x_ref.
 *
 * @param Q        n×n  state weighting
 * @param R        m×m  input weighting
 * @param A_bar    (n*N)×n
 * @param B_bar    (n*N)×(m*N)
 * @param x0       n×1  current state
 * @param x_ref    n*N×1 stacked reference [x_ref₁; …; x_ref_N]
 * @param H        (m*N)×(m*N)  [out] Hessian
 * @param g        (m*N)×1      [out] gradient
 */
inline void assembleCost(
  const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & A_bar,
  const Eigen::MatrixXd & B_bar,
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & x_ref,
  Eigen::MatrixXd & H,
  Eigen::VectorXd & g)
{
  const int n = Q.rows();              // state dim
  const int m = R.rows();              // input dim
  const int N = A_bar.rows() / n;      // horizon

  // Build block‐diagonal Q_bar (nN×nN) and R_bar (mN×mN)
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(n*N, n*N);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(m*N, m*N);
  for (int i = 0; i < N; ++i) {
    Q_bar.block(i*n, i*n, n, n) = Q;
    R_bar.block(i*m, i*m, m, m) = R;
  }

  // Predicted open‐loop state: A_bar * x0
  Eigen::VectorXd x_pred0 = A_bar * x0;

  // Hessian: H = Bᵀ Q_bar B + R_bar
  H = B_bar.transpose() * Q_bar * B_bar;
  H += R_bar;

  // Gradient: g = Bᵀ Q_bar (A_bar*x0 - x_ref)
  g = B_bar.transpose() * Q_bar * (x_pred0 - x_ref);
}

}  // namespace omni_carver_mpc_controller

#endif  // OMNI_CARVER_MPC_CONTROLLER__MPC_UTILS_HPP_