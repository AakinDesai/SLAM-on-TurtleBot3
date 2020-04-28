#include"kalman.h"
#include<stdlib.h>

int main() {

    int n = 2;
    int m = 1;
    int l = 1;
    float dt = 0.2;
    float ma = 1.0;
    float k = 10.0;
    float b = 1.0;


    Eigen::MatrixXd A(n, n);
    Eigen::MatrixXd xb(n, 1);
    Eigen::MatrixXd B(n, l);
    Eigen::MatrixXd u(l, 1);
    Eigen::MatrixXd P(n, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd R(m, m);
    Eigen::MatrixXd Z(m, 1);
    Eigen::MatrixXd K(n, m);
    Eigen::MatrixXd I(n, n);


    A << 1, dt, -(k / ma) * dt, -(b / ma) * dt + 1;
    B << 0, 0;
    u << 0;
    P << 1, 0, 0, 1;
    Q << 0.1, 0, 0, 0.1;
    H << 1, 0;
    R << 0.1;


    int it = 0;
    float mess[50];
    float actual[50];
    float error;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);
    std::ofstream myfile;
    myfile.open("error.csv");

    for (it; it < 500; it++) {

        xb << rand() % 100,0;
        float xid = 0;
        float xi = xb(0, 0);
        int i = 1;
        for (i; i <= 50; i++) {

            xid = xid - (((b * xid + k * xi) / ma) * dt);
            xi = xi + (dt * xid);
            actual[i-1] = xi;
            mess[i - 1] = actual[i-1] + distribution(generator);
        }
        
        kalmanfilter kf(dt, A, B, Q, H, R);
        kf.init(xb, u, P);
        xb = kf.update(mess);
        error = xb(0, 0) - actual[49];
        myfile <<error<< "\n";
    }
    return 0;
}