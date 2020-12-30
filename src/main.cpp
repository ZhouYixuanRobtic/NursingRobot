#define Eigen_MKL_USE_ALL
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"
int main()
{

    Eigen::MatrixXd A(1000,1000),B(1000,1000);
    A.setRandom();B.setRandom();
    Eigen::MatrixXd C(1000,1000);
    C.setZero();
    clock_t start=clock();
    C.noalias() += A*B;
    clock_t end=clock();
    std::cout<<(double)(end-start)/CLOCKS_PER_SEC;
    std::vector<Eigen::Vector4f> a, b{};
    asm("#it begins here!");
    a = b;
    asm("#it ends here!");

}
