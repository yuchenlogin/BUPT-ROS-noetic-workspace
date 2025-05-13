#ifndef _TaskSolver_h_
#define _TaskSolver_h_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/../unsupported/Eigen/KroneckerProduct>
#include <limits.h>
#include "osqp.h"

#include <RBDyn/MultiBodyConfig.h>

namespace OSQPTasks
{

  class TaskSolver
  {
  public:
    TaskSolver();
    virtual ~TaskSolver();

    csc *EigenSparseToCSC(const Eigen::SparseMatrix<c_float> &mat);

    void update(const Eigen::SparseMatrix<double> &P, std::vector<c_float> &q,
                const Eigen::SparseMatrix<double> &A, std::vector<c_float> &l, std::vector<c_float> &u);

    void update(const Eigen::SparseMatrix<double> &P, double *q,
                const Eigen::SparseMatrix<double> &A, double *l, double *u);

    void update(const Eigen::MatrixXd &P, const Eigen::VectorXd &q,
                const Eigen::MatrixXd &A, const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    void updateData();

    Eigen::VectorXd solver();

  private:
    // Problem settings
    OSQPSettings *settings_;

    // Structures
    OSQPWorkspace *work_;
    OSQPData *data_;
  };

} // namespace OSQPTasks

#endif
