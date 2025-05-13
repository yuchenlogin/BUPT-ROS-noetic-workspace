
#include <time.h>
#include <iostream>
#include "TaskSolver.h"

using namespace OSQPTasks;

TaskSolver::TaskSolver()
{

  settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  data_ = (OSQPData *)c_malloc(sizeof(OSQPData));
}

TaskSolver::~TaskSolver()
{
}

Eigen::SparseMatrix<double> DenseToSparse(Eigen::MatrixXd A)
{

  Eigen::SparseMatrix<double> A_sparse(A.rows(), A.cols());
  std::vector<Eigen::Triplet<double>> triplets;

  for (int i = 0; i < A.rows(); i++)
  {
    for (int j = 0; j < A.cols(); j++)
    {
      if (fabs(A(i, j)) > 0.00001)
        triplets.emplace_back(i, j, A(i, j));
    }
  }
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());

  return A_sparse;
}

csc *TaskSolver::EigenSparseToCSC(const Eigen::SparseMatrix<c_float> &mat)
{
  // A csc matrix is in the compressed column major.
  c_float *values =
      static_cast<c_float *>(c_malloc(sizeof(c_float) * mat.nonZeros()));
  c_int *inner_indices =
      static_cast<c_int *>(c_malloc(sizeof(c_int) * mat.nonZeros()));
  c_int *outer_indices =
      static_cast<c_int *>(c_malloc(sizeof(c_int) * (mat.cols() + 1)));
  for (int i = 0; i < mat.nonZeros(); ++i)
  {
    values[i] = *(mat.valuePtr() + i);
    inner_indices[i] = static_cast<c_int>(*(mat.innerIndexPtr() + i));
  }
  for (int i = 0; i < mat.cols() + 1; ++i)
  {
    outer_indices[i] = static_cast<c_int>(*(mat.outerIndexPtr() + i));
  }
  return csc_matrix(mat.rows(), mat.cols(), mat.nonZeros(), values,
                    inner_indices, outer_indices);
}

void checkPositive(Eigen::MatrixXd P)
{

  // matrix positive check //osqp may still fail after positive check
  Eigen::LDLT<Eigen::MatrixXd> mLDLT(P);
  if (!mLDLT.isPositive())
    std::cout << "P is not positive semidefinite matrix" << std::endl;
}

void TaskSolver::update(
    const Eigen::SparseMatrix<double> &P, double *q,
    const Eigen::SparseMatrix<double> &A, double *l, double *u)
{

  data_->n = A.cols();
  data_->m = A.rows();
  data_->P = EigenSparseToCSC(P);
  data_->q = q;
  data_->A = EigenSparseToCSC(A);
  data_->l = l;
  data_->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings_);
  // settings_->alpha = 1.6; // Change alpha parameter

  // Setup workspace
  // work_ = osqp_setup(data_, settings_);
  int exitflag = osqp_setup(&work_, data_, settings_);
}

void TaskSolver::update(
    const Eigen::SparseMatrix<double> &P, std::vector<c_float> &q,
    const Eigen::SparseMatrix<double> &A, std::vector<c_float> &l, std::vector<c_float> &u)
{

  data_->n = A.cols();
  data_->m = A.rows();
  data_->P = EigenSparseToCSC(P);
  data_->q = q.data();
  data_->A = EigenSparseToCSC(A);
  data_->l = l.data();
  data_->u = u.data();

  // Define Solver settings as default
  osqp_set_default_settings(settings_);
  settings_->alpha = 1.6; // Change alpha parameter

  // Setup workspace
  // work_ = osqp_setup(data_, settings_);
  int exitflag = osqp_setup(&work_, data_, settings_);
}

void TaskSolver::update(
    const Eigen::MatrixXd &P, const Eigen::VectorXd &q,
    const Eigen::MatrixXd &A, const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{

  // Populate data_
  std::vector<c_float> q_vec, l_vec, u_vec;
  for (int i = 0; i < q.rows(); i++)
    q_vec.push_back(q(i));
  for (int i = 0; i < l.rows(); i++)
  {
    l_vec.push_back(l(i));
    u_vec.push_back(u(i));
  }

  //need about 150us to convert A and P to sparse
  Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  Eigen::SparseMatrix<double> P_sparse = P.sparseView();
  // Eigen::SparseMatrix<double> A_sparse = DenseToSparse(A);
  // Eigen::SparseMatrix<double> P_sparse = DenseToSparse(P);

  data_->n = A_sparse.cols();
  data_->m = A_sparse.rows();
  data_->P = EigenSparseToCSC(P_sparse);
  data_->q = q_vec.data();
  data_->A = EigenSparseToCSC(A_sparse);
  data_->l = l_vec.data();
  data_->u = u_vec.data();

  // Define Solver settings as default
  osqp_set_default_settings(settings_);
  settings_->alpha = 1.6; // Change alpha parameter

  // Setup workspace
  // work_ = osqp_setup(data_, settings_);
  int exitflag = osqp_setup(&work_, data_, settings_);
}

Eigen::VectorXd TaskSolver::solver()
{

  osqp_solve(work_);

  Eigen::VectorXd solution(data_->n);

  for (int i = 0; i < data_->n; i++)
    solution(i) = *(work_->solution->x + i);

  return solution;
}
