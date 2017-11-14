#include "pnp_solver.h"

int main()
{
    PnpSolver solver;
    solver.set_internal_parameters(320,240,800,800);
    solver.set_maximum_number_of_correspondences(10);
    solver.add_correspondence(Eigen::Vector3d(1.0,1.0,1.0),Eigen::Vector2d(1.0,0.0));
}