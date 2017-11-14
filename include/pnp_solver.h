#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>

class PnpSolver
{
public:
  PnpSolver();
  ~PnpSolver(){};

  void set_internal_parameters(double cx,double cy,double fx,double fy);
  void set_maximum_number_of_correspondences(int n);
  void reset_number_of_correspondences();
  
  void add_correspondence(Eigen::Vector3d point3d,Eigen::Vector2d point2d);
  
  double compute_pose(Eigen::Matrix3d &R,Eigen::Vector3d &T);
private:
  void choose_control_points();
  void compute_barycentric_coordinates();
  
  inline double dot(const double *v1,const double *v2);
  void compute_L_6x10(const Eigen::Matrix<double,12,12> &Ut,Eigen::Matrix<double,6,10> &L_6x10);
  void compute_rho(Eigen::Matrix<double,6,1> &Rho);
  
  void find_betas_0(const Eigen::Matrix<double,6,10> &L_6x10,const Eigen::Matrix<double,6,1> &Rho,
		      Eigen::Vector4d &Betas);
  void find_betas_1(const Eigen::Matrix<double,6,10> &L_6x10,const Eigen::Matrix<double,6,1> &Rho,
		      Eigen::Vector4d &Betas);
  void find_betas_2(const Eigen::Matrix<double,6,10> &L_6x10,const Eigen::Matrix<double,6,1> &Rho,
		      Eigen::Vector4d &Betas);
  
  
  void compute_ccs(const Eigen::Vector4d &Betas,const Eigen::Matrix<double,12,12> &Ut);
  void compute_pcs();
  void solve_for_sign();
  void estimate_R_and_t(Eigen::Matrix3d &R,Eigen::Vector3d &T);
  double reprojection_error(const Eigen::Matrix3d &R,const Eigen::Vector3d &T);
  double compute_R_and_t(const Eigen::Matrix<double,12,12> &Ut,const Eigen::Vector4d &Betas,
		      Eigen::Matrix3d &R,Eigen::Vector3d &T);
   
  int maximum_number_of_correspondences;
  int number_of_correspondences;
  double cx,cy,fx,fy;
  Eigen::MatrixXd pws,us,alphas,pcs;
  
  Eigen::Matrix<double,4,3> cws,ccs;
  double cws_determinant;
};

#endif