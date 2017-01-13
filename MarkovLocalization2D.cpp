#include<iostream>
#include<Eigen/Dense>

using namespace std;

double sensor_right;
double sensor_wrong;
double p_move;
double p_stay;

int mod(int n, int l){
    if(n >= 0) return n % l;
    else return l + (n % l);
}

Eigen::MatrixXd sense(Eigen::MatrixXd &p, char Z, Eigen::MatrixXi &world){
    Eigen::MatrixXd q(p.rows(), p.cols());
    double sum = 0.0;

    for(int i = 0; i < p.rows(); i++){
        for(int j = 0; j < p.cols(); j++){
            double hit = (Z == world(i,j));
            q(i,j) = p(i,j) * (hit*sensor_right + (1-hit)*sensor_wrong);
            sum += q(i,j);
        }
    }

    for(int i = 0; i < q.rows(); i++){
        for(int j = 0; j < q.cols(); j++){
            q(i,j) = q(i,j)/sum;
        }
    }

    return q;
}

Eigen::MatrixXd move(Eigen::MatrixXd &p, Eigen::MatrixXi &U, int t){
    Eigen::MatrixXd q(p.rows(), p.cols());

    for(int i = 0; i < p.rows(); i++){
        for(int j = 0; j < p.cols(); j++){
            q(i,j) = (p_move*p(mod(i-U(0,t), p.rows()), mod(j-U(1,t),p.cols()))) + (p_stay*p(i,j));
        }
    }

    return q;
}


void localize(Eigen::MatrixXi &world, Eigen::VectorXi &measurements, Eigen::MatrixXi &motions, Eigen::MatrixXd &p){
    for(int i = 0; i < measurements.rows(); i++){
        p = move(p, motions, i);
        p = sense(p, measurements(i), world);
    }
}

void Run2D(){
    //--configurations
    pHit  = 0.6;
    pMiss = 0.2;
    pExact = 0.8;
    pOvershoot = 0.1;
    pUndershoot = 0.1;

    //--probability sensor is right and move probability
    sensor_right = 0.7;
    p_move = 0.8;

    sensor_wrong = 1.0 - sensor_right;
    p_stay = 1.0 - p_move;

    //--map
    Eigen::MatrixXi world(4,5);
    world << 'R', 'G', 'G', 'R', 'R',
             'R', 'R', 'G', 'R', 'R',
             'R', 'R', 'G', 'G', 'R',
             'R', 'R', 'R', 'R', 'R';

    //-- measurements
    Eigen::VectorXi measurements(5);
    measurements << 'G', 'G', 'G', 'G', 'G';

    //--motions
    // [ 0, 0] no move
    // [ 0, 1] right
    // [ 0,-1] left
    // [ 1, 0] down
    // [-1, 0] up
    Eigen::MatrixXi motions(2, 5);
    motions << 0, 0, 1, 1, 0,
               0, 1, 0, 0, 1;

    //--probabilities
    Eigen::MatrixXd p(world.rows(), world.cols());
    for(int i = 0; i < p.rows(); i++){
        for(int j = 0; j < p.cols(); j++){
            p(i,j) = (double)1.0/double(p.rows()*p.cols());
        }
    }

    //--localization
    localize(world, measurements, motions, p);

    //--print
    cout<<p<<endl;
}

int main(){
    Run2D();
}
