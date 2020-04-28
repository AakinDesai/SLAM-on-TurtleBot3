#pragma once

#include<iostream>
#include<Eigen/dense>
#include<random>
#include<fstream>

class kalmanfilter {

private:

    double dt;
    Eigen::MatrixXd A, xb, B, u, P, Q, H, R, K;


public:

    kalmanfilter(double t, Eigen::MatrixXd dyn, Eigen::MatrixXd con, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc);
    kalmanfilter(double t, Eigen::MatrixXd pc, Eigen::MatrixXd mea, Eigen::MatrixXd mc);
    void init(Eigen::MatrixXd st, Eigen::MatrixXd uo, Eigen::MatrixXd cvm);
    void init(Eigen::MatrixXd st, Eigen::MatrixXd cvm);
    Eigen::MatrixXd update(float* sensor);
    Eigen::MatrixXd update(float* sensor, Eigen::MatrixXd (*f)(Eigen::MatrixXd), Eigen::MatrixXd(*g)(Eigen::MatrixXd));
};
