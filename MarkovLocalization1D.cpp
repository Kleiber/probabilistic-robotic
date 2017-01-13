#include<stdio.h>
#include<string>
#include<vector>

using namespace std;

double pHit;
double pMiss;
double pExact;
double pOvershoot;
double pUndershoot;

int mod(int n, int l){
    if(n >= 0) return n % l;
    else return l + (n % l);
}

vector<double> sense1D(vector<double> &p, char Z, vector<char> &world){
    vector<double> q(p.size());
    double sum = 0.0;

    for(size_t i = 0; i < p.size(); i++){
        double hit = (Z == world[i]);
        q[i] = p[i] * (hit*pHit + (1-hit)*pMiss);
	sum += q[i];
    }

    for(size_t i = 0; i < q.size(); i++) q[i] = q[i]/sum;

    return q;
}

vector<double> move1D(vector<double> &p, int U){
    vector<double> q;
    for(size_t i = 0; i < p.size(); i++){
        double sum = pExact*p[mod(i-U, p.size())];
        sum = sum + pOvershoot*p[mod(i-U-1, p.size())];
        sum = sum + pUndershoot*p[mod(i-U+1, p.size())];
        q.push_back(sum);
    }

    return q;
}

void localize1D(vector<char> &world, vector<char> &measurements, vector<int> &motions, vector<double> &p){
    for(size_t i = 0; i < measurements.size(); i++){
        p = move1D(p, motions[i]);
        p = sense1D(p, measurements[i], world);
    }
}

void run1D(){
    //--configurations
    pHit  = 0.6;
    pMiss = 0.2;
    pExact = 0.8;
    pOvershoot = 0.1;
    pUndershoot = 0.1;

    char m[] = {'G', 'R', 'R', 'G', 'G'};
    char z[] = {'R', 'R', 'G'};
    int u[] = {1, 1, 1};

    //--map
    vector<char> world(m, m + sizeof(m)/sizeof(m[0]));

    //--measurements
    std::vector<char> measurements(z, z + sizeof(z)/sizeof(z[0]));

    //--motions
    std::vector<int> motions(u, u + sizeof(u)/sizeof(u[0]));

    //--probabilities
    vector<double> p(world.size(), (double)1.0/(double)(world.size()));

    //--localization
    localize1D(world, measurements, motions, p);

    //--print
    for(size_t i = 0; i < p.size(); i++) printf("%.17f ",p[i]);
    printf("\n");
}

int main(){
    run1D();
}
