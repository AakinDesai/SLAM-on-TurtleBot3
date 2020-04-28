#include"kalman.h"
#include<stdlib.h>

int main() {

    int n = 1;
    int l = 1;
    int m = 1;
    float dt = 0.2;

    Eigen::MatrixXd A(n, n);
    Eigen::MatrixXd xb(n, 1);
    Eigen::MatrixXd B(n, l);
    Eigen::MatrixXd u(l, 1);
    Eigen::MatrixXd P(n, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd R(m, m);
    Eigen::MatrixXd K(n, m);

    A << 1;
    B << dt;
    u << 1;
    P << 0.5;
    Q << 0.05;
    H << 1;
    R << 0.01;

    int it = 0;
    float mess[50];
    float actual[50];
    float error;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);
    std::ofstream myfile;
    myfile.open("error.csv");

    for (it; it < 500; it++) {

        xb << rand() % 100;
        int i = 1;
        for (i; i <= 50; i++) {

            mess[i - 1] = xb(0, 0) + (u(0, 0) * i * dt) + distribution(generator);
            actual[i - 1] = xb(0, 0) + (u(0, 0) * i * dt);
        }

        kalmanfilter kf(dt, A, B, Q, H, R);
        kf.init(xb, u, P);
        xb = kf.update(mess);
        error = xb(0, 0) - actual[49];
        myfile << error << "\n";
    }
    return 0;
}