/// @brief Modern C++ LINPACK subset — LU and QR decomposition
/// @author Yonglin
/// @version 0.2
/// @date 17-Apr-2026

#ifndef _MATH_LINPACK_HPP_
#define _MATH_LINPACK_HPP_

#include <vector>

namespace math {

class Linpack {
 public:
  /// LU decomposition with condition number estimate (dgeco equivalent).
  /// @param a  matrix in column-major order, size at least lda*n.
  ///           On return, contains the LU factors.
  /// @param lda leading dimension of a (>= n).
  /// @param n   order of the matrix.
  /// @param ipvt pivot indices, size n (0-based on output).
  /// @param rcond reciprocal condition estimate (output).
  /// @return true on success.
  static bool LUDecompose(std::vector<double>& a, int lda, int n,
                          std::vector<int>& ipvt, double& rcond);

  /// LU solve (dgesl equivalent).
  /// @param a   LU factors from LUDecompose.
  /// @param lda leading dimension.
  /// @param n   order.
  /// @param ipvt pivot indices from LUDecompose.
  /// @param b   right-hand side, overwritten with solution.
  /// @param job 0 -> solve A*x=B, nonzero -> solve A^T*x=B.
  /// @return true on success.
  static bool LUSolve(const std::vector<double>& a, int lda, int n,
                      const std::vector<int>& ipvt,
                      std::vector<double>& b, int job);

  /// Householder QR decomposition with column pivoting (dqrdc equivalent).
  /// @param x    n x p matrix in column-major order, size at least ldx*p.
  ///             On return, contains the QR factorization in compact form.
  /// @param ldx  leading dimension of x (>= n).
  /// @param n    number of rows.
  /// @param p    number of columns.
  /// @param qraux auxiliary output, size p.
  /// @param jpvt column pivot indices, size p (0-based on output).
  /// @param work work array, size p.
  /// @param job  0 -> no pivoting, nonzero -> pivoting.
  /// @return true on success.
  static bool QRDecompose(std::vector<double>& x, int ldx, int n, int p,
                          std::vector<double>& qraux,
                          std::vector<int>& jpvt,
                          std::vector<double>& work, int job);

  /// QR solve / projection / least squares (dqrsl equivalent).
  /// @param x     QR factors from QRDecompose.
  /// @param ldx   leading dimension.
  /// @param n     number of rows.
  /// @param k     number of columns used (<= min(n,p)).
  /// @param qraux from QRDecompose.
  /// @param y     right-hand side vector, size n.
  /// @param qy    output Q*y if requested (may be null).
  /// @param qty   output Q^T*y if requested (may be null).
  /// @param b     output least-squares solution, size k (may be null).
  /// @param rsd   output residual y - X*b, size n (may be null).
  /// @param xb    output projection X*b, size n (may be null).
  /// @param job   decimal abcde: a!=0 compute qy; b/c/d/e!=0 compute qty;
  ///              c!=0 compute b; d!=0 compute rsd; e!=0 compute xb.
  /// @param info  0 on success; if b requested and R singular, index of
  ///              first zero diagonal (1-based, as in original).
  /// @return true on success.
  static bool QRSolve(const std::vector<double>& x, int ldx, int n, int k,
                      const std::vector<double>& qraux,
                      const std::vector<double>& y,
                      std::vector<double>* qy,
                      std::vector<double>* qty,
                      std::vector<double>* b,
                      std::vector<double>* rsd,
                      std::vector<double>* xb,
                      int job, int& info);

  /// Truncated least squares with factor reuse (dqstre equivalent).
  /// @param x     n x p matrix, destroyed during factorization.
  /// @param x_fac saved factorized matrix for reuse, size at least ldx*p.
  /// @param ldx   leading dimension.
  /// @param n     number of rows.
  /// @param p     number of columns.
  /// @param y     right-hand side, size n.
  /// @param tol   tolerance for truncation (0 = full rank).
  /// @param b     solution vector, size p (output).
  /// @param rsd   residual vector, size n (output).
  /// @param k     number of columns used (output).
  /// @param jpvt  pivot indices, size p.
  /// @param qraux auxiliary array, size p.
  /// @param work  work array, size p.
  /// @param reuse if true, reuse x_fac instead of factorizing x again.
  /// @return true on success.
  static bool QRLS(std::vector<double>& x,
                   std::vector<double>& x_fac, int ldx, int n, int p,
                   const std::vector<double>& y, double tol,
                   std::vector<double>& b, std::vector<double>& rsd,
                   int& k, std::vector<int>& jpvt,
                   std::vector<double>& qraux,
                   std::vector<double>& work, bool reuse);
};

}  // namespace math

#endif  // _MATH_LINPACK_HPP_
