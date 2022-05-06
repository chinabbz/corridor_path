#ifndef MPC_H
#define MPC_H

#include <vector>
#include <eigen3/Eigen/Core>

#define LMIN 2.0
#define ALPHAMIN 0.5 // 30度
#define KMAX 0.5     // 曲率<0.5

using namespace std;

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state);
    void Solve();
    void Solve(int index);
    vector<double> mpc_x;
    vector<double> mpc_y;
};

#endif /* MPC_H */