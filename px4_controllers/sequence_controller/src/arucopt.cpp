#include <iostream>
#include <vector>
#include <utility>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

using namespace std;

#define invalid -99999.9
#define pi 3.141592653589
std::vector<Eigen::Vector3d> pose;
std::vector<Eigen::Vector2d> gradient;
std::vector<std::pair<Eigen::Vector4d , Eigen::Vector2i>> measurement;
bool valid(int first, int second){
    if(pose[first].norm() > 9000 || pose[second].norm() > 9000 ) return false;
    return true;  
}

double evaluate( std::vector<Eigen::Vector3d> costUp , std::vector<Eigen::Vector3d> costDown){
    double sum1 = 0, sum2 = 0;
    for (auto u: costUp){
        sum1+= u.sum();
    }
    for (auto d: costDown){
        sum2+= d.sum();
    }
    return 0.5*(sum1 + sum2);
}

std::vector<Eigen::Vector3d> evaluateCostUp(double& delta){
    std::vector<Eigen::Vector3d> cost(pose.size());
    for (auto m : measurement){
        int i1 = m.second[0];
        int i2 = m.second[1];
        if(!valid) continue;
        Eigen::Vector2d vZ(m.first[0],m.first[1]);
        Eigen::Vector2d vO(pose[i2][0]-pose[i1][0] ,pose[i2][1]-pose[i1][1]);

        Eigen::Vector2d vXu = vO - delta * Eigen::Vector2d::UnitX();
        Eigen::Vector2d vYu = vO - delta * Eigen::Vector2d::UnitY();
        

        double angle = pi /2 ;
        Eigen::Rotation2D<double> rot(angle);
        Eigen::Rotation2D<double> rotu(angle + delta);

        Eigen::Vector2d vXb = rot * vXu;
        Eigen::Vector2d vYb = rot * vYu;
        Eigen::Vector2d vOb = rotu * vO;

        double dEX = (vXb-vZ).squaredNorm();
        double dEY = (vYb-vZ).squaredNorm();
        double dEO = (vOb-vZ).squaredNorm();
        cost[i1][0] += dEX;
        cost[i1][1] += dEY;
        cost[i1][2] += dEO;
    }
    return cost;
}

std::vector<Eigen::Vector3d> evaluateCostDown(double& delta){
    double negdelta = -delta;
    return evaluateCostUp(negdelta);
}


double step(std::vector<Eigen::Vector3d>& up , std::vector<Eigen::Vector3d>& down, double learning_rate , double delta){
    double inv2delta = -0.5/delta;
    for (auto i = 0u; i < pose.size(); i++){
        pose[i] += inv2delta * (up[i] - down[i]) * learning_rate;
    }
}

int main(int argc, char** argv){
    double learning_rate = 0.001;
    double delta = 0.05; 
    for (auto batch = 0u ; batch <1000 ; batch++){
        std::vector<Eigen::Vector3d>cost_up = evaluateCostUp(delta);
        std::vector<Eigen::Vector3d>cost_down = evaluateCostDown(delta);
        step(cost_up , cost_down ,learning_rate , delta);
        std::cout << " eval : " << evaluate(cost_up,cost_down) << " at step : " << batch << std::endl;
    }  

}