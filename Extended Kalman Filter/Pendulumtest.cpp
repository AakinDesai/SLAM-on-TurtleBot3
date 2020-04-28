#include"kalman.h"
#include<stdlib.h>

Eigen::MatrixXd dynamics(Eigen::MatrixXd f);
Eigen::MatrixXd jacobian(Eigen::MatrixXd g);


int main() {

    int n = 2;
    float l = 1.0;
    float g = 9.8;
    int m = 1;
    float dt = 0.2;

    Eigen::MatrixXd xb(n, 1);
    Eigen::MatrixXd P(n, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd R(m, m);
    

    P << 1, 0, 0, 1;
    Q << 0, 0.2, 0.5, 0;
    H << 1, 0;
    R << 0.1;

    int it = 0;
    float mess[100];
    float actual[100];
    float error,xi,xid,xidv;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);
    std::ofstream myfile;
    myfile.open("error.csv");

    for (it; it < 500; it++) {

        xb << rand() % 2,0;
        xi = xb(0, 0);
        xid = xb(1, 0);
        int i = 1;
        for (i; i <= 100; i++) {

            xidv = xid - ((g / l * sin(xi)) * dt);
            xi = xi + (xid * dt);
            xid = xidv;
            actual[i - 1] = xi;
            mess[i - 1] = xi + distribution(generator);
        }

        kalmanfilter kf(dt,Q, H, R);
        kf.init(xb,P);
        xb = kf.update(mess,dynamics,jacobian);
        error = xb(0, 0) - actual[99];
        myfile << error << "\n";
    }
    return 0;
}

Eigen::MatrixXd dynamics(Eigen::MatrixXd f) {

    Eigen::MatrixXd d (2,1);
    d << f(0, 0) + (f(1, 0)) * 0.2 , f(1, 0) - (9.8 / 1.0 * sin(f(0, 0)) * 0.2);
    return d;
}

Eigen::MatrixXd jacobian(Eigen::MatrixXd g) {

    Eigen::MatrixXd d(2, 2);
    d << 1, 0.2, -(9.8 / 1.0 * cos(g(0, 0)) * 0.2), 1;
    return d;
}