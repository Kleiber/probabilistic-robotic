#include<stdio.h>
#include<math.h>

using namespace std;

void update(double &mean1, double &var1, double &mean2, double &var2){
    double new_mean = (var2*mean1 + var1*mean2)/(var1 + var2);
    double new_var  = 1.0/(1.0/var1 + 1.0/var2);

    mean1 = new_mean;
    var1  = new_var;
}

void predict(double &mean1, double &var1, double &mean2, double &var2){
    double new_mean = mean1 + mean2;
    double new_var  = var1 + var2;

    mean1 = new_mean;
    var1  = new_var;
}

void run1D(){
    double measurements[] = {5.0, 6.0, 7.0, 9.0, 10.0};
    double motion[] = {1.0, 1.0, 2.0, 1.0, 1.0};
    double measurement_sig = 4.0;
    double motion_sig = 2.0;
    double mu = 0.0;
    double sig = 10000.0;

    for(int i = 0; i < sizeof(measurements)/sizeof(measurements[0]); i++){
        update(mu, sig, measurements[i], measurement_sig);
        printf("update:  %.16f %.16f\n", mu, sig);
        predict(mu, sig, motion[i], motion_sig);
        printf("predict: %.16f %.16f\n", mu, sig);
    }

    printf("%.16f %.16f\n", mu, sig);
}

int main(){
    run1D();
}

