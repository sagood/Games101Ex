#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

int main()
{
    /*
     * PA 0
     */
    // TO DO: Define point P
    // TO DO: Define rotation matrix M
    // TO DO: M * P
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f r;
    r << std::cos(M_PI / 4), -std::sin(M_PI / 4), 0,
        std::sin(M_PI / 4), std::cos(M_PI / 4), 0,
        0, 0, 1;
    std::cout << r * p << std::endl;

    return 0;
}