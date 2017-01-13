#include<iostream>
#include<Eigen/Dense>

using namespace std;

void KalmanFilter4D(Eigen::MatrixXd &measurements, Eigen::MatrixXd &x,Eigen::MatrixXd &P, Eigen::MatrixXd &u,
                     Eigen::MatrixXd &F, Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::MatrixXd &I){

    for(int i = 0; i < measurements.cols(); i++){
        Eigen::MatrixXd Z(2,1);
        Z << measurements(0,i), measurements(1,i);

        //--prediction
        x = F*x + u;
        P = F*P*F.transpose();

        //--measurement update
        Eigen::MatrixXd y = Z - H*x;  //--error measurement
        Eigen::MatrixXd S = H*P*H.transpose() + R; //--error uncertainty
        Eigen::MatrixXd K = P*H.transpose()*S.inverse(); //--kalmain gain

        x = x + K*y;
        P = (I - K*H)*P;
    }
}

void Run4D(){
    double dt = 0.1;

    double initial_x =  4.0;
    double initial_y = 12.0;
    Eigen::MatrixXd measurements(2,6);
    measurements <<  5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
                    10.0, 8.0, 6.0, 4.0, 2.0,  0.0;

    /*
    double initial_x = -4.0;
    double initial_y =  8.0;
    Eigen::MatrixXd measurements(2,4);
    measurements << 1.0, 6.0, 11.0, 16.0,
                    4.0, 0.0, -4.0, -8.0;

    */
    /*
    double initial_x =  1.0;
    double initial_y = 19.0;
    Eigen::MatrixXd measurements(2,4);
    measurements <<  1.0,  1.0,  1.0,  1.0,
                    17.0, 15.0, 13.0, 11.0;

    */

    //--initial state (location and velocity)
    Eigen::MatrixXd x(4,1);
    x << initial_x, initial_y, 0.0, 0.0;

    //--motion vector
    Eigen::MatrixXd u(4,1);
    u << 0.0, 0.0, 0.0, 0.0;

    //--uncertainty covariance
    Eigen::MatrixXd P(4,4);
    P << 0.0, 0.0,    0.0,    0.0,
         0.0, 0.0,    0.0,    0.0,
         0.0, 0.0, 1000.0,    0.0,
         0.0, 0.0,    0.0, 1000.0;

    //--state transition matrix
    Eigen::MatrixXd F(4,4);
    F << 1.0, 0.0,  dt, 0.0,
         0.0, 1.0, 0.0,  dt,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    //--measurement function
    Eigen::MatrixXd H(2,4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    //--measurement noise
    Eigen::MatrixXd R(2,2);
    R << 0.1, 0.0,
         0.0, 0.1;

    //--identity matrix
    Eigen::MatrixXd I(4,4);
    I << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    KalmanFilter2D(measurements, x, P, u, F, H, R, I);

    cout<<"x = "<<x<<endl;
    cout<<"P = "<<P<<endl;
}

int main(){
    Run4D();
}

