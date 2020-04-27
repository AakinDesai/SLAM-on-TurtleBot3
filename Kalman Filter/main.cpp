#include<iostream>
#include<Eigen/dense>
#include<random>
#include<fstream>

class kalmanfilter {

private :

    double dt;
    Eigen::MatrixXd A,xb,B,u,P,Q,H,R,K;
    

public :

    kalmanfilter(double t, Eigen::MatrixXd dyn, Eigen::MatrixXd con, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc);
    void init(Eigen::MatrixXd st, Eigen::MatrixXd uo, Eigen::MatrixXd cvm);
    void update(float * sensor);
};

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
     xb << 0;
     B << dt;
     u << 1;
     P << 0.5;
     Q << 0.05;
     H << 1;
     R << 0.01;

     int i = 1;
     float mess[50];
     std::default_random_engine generator;
     std::normal_distribution<double> distribution(0.0, 0.1);
     
     

     for (i; i <= 50; i++) {

         mess[i-1] = (u(0, 0) * i * dt) + distribution (generator);

     }

     kalmanfilter kf(dt,A,B,Q,H,R);
     kf.init(xb, u, P);
     kf.update(mess);

     return 0;
}

 kalmanfilter::kalmanfilter(double t, Eigen::MatrixXd dyn, Eigen::MatrixXd con, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc) {

     dt = t;
     A = dyn;
     B = con;
     Q = pc;
     H = mea;
     R = mc;

 }

void kalmanfilter :: init(Eigen::MatrixXd st, Eigen::MatrixXd uo, Eigen::MatrixXd cvm) {

    xb = st;
    u = uo;
    P = cvm;
}

void kalmanfilter::update(float *sensor) {

    int r = P.rows();
    Eigen::MatrixXd I(r, r);
    Eigen::MatrixXd Z(1, 1);
    
    I.setIdentity();
    int it = 1;
    std::ofstream myfile;
    myfile.open("kalman.csv");
    myfile << xb << ',' << '0' << ',' << '0' << "\n";

    for (it; it <= 50; it++)
    {
        xb = A * xb + B * u;
        P = A * P * A.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << sensor[it-1];
        xb = xb + K * (Z - (H * xb));
        P = (I - (K * H)) * P;
        std::cout << xb << std::endl;
        myfile << xb << ',' << sensor[it-1] << "\n";
    }

}