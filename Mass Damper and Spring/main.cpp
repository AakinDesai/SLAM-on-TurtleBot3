#include<iostream>
#include<Eigen/dense>
#include<random>
#include<fstream>

int main() {

    int n = 2;
    int m = 1;
    float dt = 0.2;
    float ma = 1.0;
    float k = 10.0;
    float b = 1.0;


    Eigen::MatrixXd A(n, n);
    Eigen::MatrixXd xb(n, 1);
    Eigen::MatrixXd P(n, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd R(m, m);
    Eigen::MatrixXd Z(m, 1);
    Eigen::MatrixXd K(n, m);
    Eigen::MatrixXd I(n, n);


    A << 1,dt,-( k / ma)*dt, -( b / ma) * dt + 1;
    xb << 10 , 0;
    P << 1,0,0,1;
    Q << 0.1,0,0,0.1;
    H << 1,0;
    R << 0.1;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);

    int i = 1;
    float xid = 0;
    float xi = 10;
    float actual[51];
    actual[0] = 10;

    for (i; i <= 50; i++) {

        xid = xid - (((b * xid + k * xi) / ma) * dt);
        xi = xi + (dt * xid);
        actual[i] = xi;
    }

    int it = 1;
    I.setIdentity();

    std::ofstream myfile;
    myfile.open("msd.csv");
    myfile << xb(0,0) << ',' << actual[0] << ',' << actual[0] << "\n";
    

    for (it; it <= 50; it++)
    {
        xb = A * xb ;
        P = A * P * A.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << actual[it] + distribution(generator);
        xb = xb + K * (Z - (H * xb));
        P = (I - (K * H)) * P;
        myfile << xb(0,0) << ',' << Z << ','<< actual[it] << "\n";
    }

   return 0;
}