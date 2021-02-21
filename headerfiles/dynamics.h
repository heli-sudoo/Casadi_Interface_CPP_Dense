#ifndef DYNAMICS_DDP
#define DYNAMICS_DDP

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/unsupported/Eigen/SparseExtra>
#include <eigen3/Eigen/IterativeLinearSolvers>

#define int_T long long int

using namespace std;
using namespace std::chrono;

using Eigen::MatrixXd;
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;
typedef Eigen::Matrix<double, 14,14> StateMat;
typedef Eigen::Matrix<double, 14, 4> ContrlMat;
typedef Eigen::Matrix<double, 2, 14> ObservMat;
typedef Eigen::Matrix<double, 2, 4> DirectMat; 
typedef Eigen::Matrix<double, 14,1> StateVec;
typedef Eigen::Matrix<double, 2, 7> JacobMat;
typedef Eigen::Matrix4d ContrlVec;
typedef Eigen::Vector2d OutputVec;
typedef Eigen::Matrix<double, 7, 1> CoorVec;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DMat;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> DVec;


void dynamics(DVec, DVec, DVec&, DVec&, int);
void dynamics_par(DVec, DVec, DMat&, DMat&, DMat&, DMat&, int);
void resetmap(DVec, DVec&, DVec&, int);
void resetmap_par(DVec, DMat&, DMat&, DMat&, DMat&, int);
void FootJacobian(DVec, DMat&, DMat&, int);
void casadi_interface(vector<DMat> &ARG, vector<DMat*> RES,
                      int f(const double **, double **, int_T *, double *, int),
                      const int_T *f_sparse_out(int_T),
                      int f_work(int_T *, int_T *, int_T *, int_T *));

template <typename T>
void casadi_interface(vector<DMat> &ARG, vector<T*> RES,
                      int f(const double **, double **, int_T *, double *, int),
                      const int_T *f_sparse_out(int_T),
                      int f_work(int_T *, int_T *, int_T *, int_T *));


double _dt;
DVec _xdot;
DMat _Ac;
DMat _Bc;


#endif