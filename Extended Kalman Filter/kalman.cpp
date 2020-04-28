#include"kalman.h"

kalmanfilter::kalmanfilter(double t, Eigen::MatrixXd dyn, Eigen::MatrixXd con, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc) {

    dt = t;
    A = dyn;
    B = con;
    Q = pc;
    H = mea;
    R = mc;

}

kalmanfilter::kalmanfilter(double t, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc) {

    dt = t;
    Q = pc;
    H = mea;
    R = mc;

}

void kalmanfilter::init(Eigen::MatrixXd st, Eigen::MatrixXd uo, Eigen::MatrixXd cvm) {

    xb = st;
    u = uo;
    P = cvm;
}

void kalmanfilter::init(Eigen::MatrixXd st, Eigen::MatrixXd cvm) {

    xb = st;
    P = cvm;
}

Eigen::MatrixXd kalmanfilter::update(float* sensor) {

    int r = P.rows();
    Eigen::MatrixXd I(r, r);
    Eigen::MatrixXd Z(1, 1);

    I.setIdentity();
    int it = 1;
    
    for (it; it <= 50; it++)
    {
        xb = A * xb + B * u;
        P = A * P * A.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << sensor[it - 1];
        xb = xb + K * (Z - (H * xb));
        P = (I - (K * H)) * P;
     }

    std::cout << P(0, 0) << "\n";
    
    return xb;

}
Eigen::MatrixXd kalmanfilter::update(float* sensor, Eigen::MatrixXd(*f)(Eigen::MatrixXd), Eigen::MatrixXd(*g)(Eigen::MatrixXd)) {

    int r = P.rows();
    Eigen::MatrixXd I(r, r);
    Eigen::MatrixXd Z(1, 1);
    
    I.setIdentity();
    int it = 1;

    for (it; it <= 100; it++)
    {
        
        xb = f(xb);
        A = g(xb);
        P = A * P * A.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Z << sensor[it - 1];
        xb = xb + K * (Z - (H * xb));
        P = (I - (K * H)) * P;
    }
     
    std::cout << P(0, 0) << "\n";

    return xb;
}