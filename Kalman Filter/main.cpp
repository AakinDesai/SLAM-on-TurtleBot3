#include<iostream>
#include<Eigen/dense>
#include<random>
#include<fstream>

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
     Eigen::MatrixXd Z(m, 1);
     Eigen::MatrixXd K(n, m);
     Eigen::MatrixXd I(1, 1);
     

     A << 1;
     xb << 0;
     B << dt;
     u << 1;
     P << 0.5;
     Q << 0.05;
     H << 1;
     R << 0.01;

     std::default_random_engine generator;
     std::normal_distribution<double> distribution(0.0, 0.1);

     int it = 1;
     I.setIdentity();

     std::ofstream myfile;
     myfile.open("kalman.csv");
     myfile << xb << ',' << '0'<<"\n";

     for (it; it <= 50; it++)
     {
         xb = A * xb + B * u;
         P = A * P * A.transpose() + Q;
         K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
         Z << (u(0,0)*it*dt) + distribution(generator);
         xb = xb + K * (Z - (H * xb));
         P = (I - (K * H)) * P;
         std::cout << xb << std::endl;
         myfile << xb<<','<<Z << "\n";
     }


     return 0;
}