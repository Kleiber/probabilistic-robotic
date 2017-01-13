#include<iostream>
#include<Eigen/Dense>

using namespace std;

void KalmanFilter2D(Eigen::MatrixXd &measurements, Eigen::MatrixXd &x,Eigen::MatrixXd &P, Eigen::MatrixXd &u,
                     Eigen::MatrixXd &F, Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::MatrixXd &I){

    for(int i = 0; i < measurements.cols(); i++){
        Eigen::MatrixXd Z(1,1);
        Z << measurements(0,i);

        //--measurement update
        Eigen::MatrixXd y = Z - H*x;  //--error measurement
        Eigen::MatrixXd S = H*P*H.transpose() + R; //--error uncertainty
        Eigen::MatrixXd K = P*H.transpose()*S.inverse(); //--kalmain gain

        x = x + K*y;
        P = (I - K*H)*P;

        //--prediction
        x = F*x + u;
        P = F*P*F.transpose();
    }
}

void Run2D(){
    Eigen::MatrixXd measurements(1,3);
    measurements << 1.0, 2.0, 3.0;

    //--initial state (location and velocity)
    Eigen::MatrixXd x(2,1);
    x << 0.0, 0.0;

    //--motion vector
    Eigen::MatrixXd u(2,1);
    u << 0.0, 0.0;

    //--uncertainty covariance
    Eigen::MatrixXd P(2,2);
    P << 1000.0,    0.0,
            0.0, 1000.0;

    //--state transition matrix
    Eigen::MatrixXd F(2,2);
    F << 1.0, 1.0,
         0.0, 1.0;

    //--measurement function
    Eigen::MatrixXd H(1,2);
    H << 1.0, 0.0;

    //--measurement noise
    Eigen::MatrixXd R(1,1);
    R << 1.0;

    //--identity matrix
    Eigen::MatrixXd I(2,2);
    I << 1.0, 0.0,
         0.0, 1;0;

    KalmanFilter2D(measurements, x, P, u, F, H, R, I);

    cout<<"x = "<<x<<endl;
    cout<<"P = "<<P<<endl;
}

int main(){
    Run2D();
}
