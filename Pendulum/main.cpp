#include<iostream>
#include<Eigen/dense>
#include<random>
#include<fstream>
#include <math.h>      

#define PI 3.14159265

class kalmanfilter {

private:

    float dt;
    Eigen::MatrixXd xb, P, Q, H, R, K;


public:

    kalmanfilter(double t, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc);
    void init(Eigen::MatrixXd st,Eigen::MatrixXd cvm);
    void update(float* sensor, float* orga, float*orgv);
};

int main() {

    int n = 2;
    int m = 1;
    float dt = 0.2;
    float g = 9.8;
    float l = 1.0;
    


    Eigen::MatrixXd A(n, n);
    Eigen::MatrixXd xb(n, 1);
    Eigen::MatrixXd P(n, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd R(m, m);
    Eigen::MatrixXd Z(m, 1);
    Eigen::MatrixXd K(n, m);
    Eigen::MatrixXd I(n, n);

    P << 1, 0, 0, 1;
    Q << 0, 0.2, 0.5, 0;
    H << 1, 0;
    R << 1;
    xb << 30, 0;
    

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1);

    int i = 1;
    float a = 30;
    float w = 0;
    float actuala[51],actualw[51],mess[50];
    actuala[0] = 30;
    actualw[0] = 0;

    for (i; i <= 50; i++) {

        actualw[i] = actualw[i - 1] - ((g * l * sin(actuala[i - 1] * PI / 180)) * dt);
        actuala[i] = actuala[i - 1] + (actualw[i - 1] * dt);
        mess[i-1] = actuala[i] + distribution(generator);
    }

    kalmanfilter kf(dt,Q, H, R);
    kf.init(xb,P);
    kf.update(mess, actuala,actualw);

    return 0;
}

kalmanfilter::kalmanfilter(double t,Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc) {

    dt = t;
    Q = pc;
    H = mea;
    R = mc;

}

void kalmanfilter::init(Eigen::MatrixXd st,Eigen::MatrixXd cvm) {

    xb = st;
    P = cvm;
}

void kalmanfilter::update(float* sensor, float* orga, float*orgv) {

    int r = P.rows();
    Eigen::MatrixXd A(r, r);
    Eigen::MatrixXd I(r, r);
    Eigen::MatrixXd Z(1, 1);

    I.setIdentity();
    int it = 1;
    std::ofstream myfile;
    myfile.open("kalman.csv");
    myfile << xb(0, 0) << ',' << orga[0] << ',' << orga[0] << "\n";

    for (it; it <= 50; it++)
    {
        xb << orga[it], orgv[it];
        A << 1, dt, (9.8 * 1 * cos(orga[it - 1] * PI / 180))* dt, 1;
        P = A * P * A.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << sensor[it - 1];
        xb = xb + K * (Z - (H * xb));
        P = (I - (K * H)) * P;
        myfile << xb(0, 0) << ',' << sensor[it - 1] << ',' << orga[it] << "\n";
    }

}