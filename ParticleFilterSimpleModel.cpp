#include<stdio.h>
#include<vector>
#include<math.h>
#include<random> //use CONFIG += c++11

using namespace std;

//random generator
std::default_random_engine generator((unsigned int)time(0));

//--world configuration
#define rows_landmarks 4
#define cols_landmarks 2
#define world_size 100.0

//--landmarks (x,y)
double landmarks[rows_landmarks][cols_landmarks] = {20.0, 20.0, 80.0, 80.0, 20.0, 80.0, 80.0, 20.0};

double mod(double n, double l){
    if(n >= 0.0) return fmod(n,l);
    else return l + fmod(n,l);
}

class robot{
  public:

    //--robot parameters
    double x;
    double y;
    double orientation;
    double forward_noise;
    double turn_noise;
    double sense_noise;

    //--create robot witn initial values
    robot(){

        std::uniform_real_distribution<> random(0, 1);
        x = random(generator) * world_size;
        y = random(generator) * world_size;
        orientation = random(generator) * 2.0 * M_PI;
        forward_noise = 0.0;
        turn_noise = 0.0;
        sense_noise = 0.0;
    }

    //--set robot values
    void set(double new_x, double new_y, double new_orientation){

        if(new_x < 0.0 || new_x >= world_size){printf("X coordinate out of bound\n"); return;}
        if(new_y < 0.0 || new_y >= world_size){printf("Y coordinate out of bound\n"); return;}
        if(new_orientation < 0.0 || new_orientation >= 2.0 * M_PI){printf("Orientation must be in [0..2pi]\n"); return;}

        x = new_x;
        y = new_y;
        orientation = new_orientation;
    }

    //--set noise values
    void set_noise(double new_f_noise, double new_t_noise, double new_s_noise){

        forward_noise = new_f_noise;
        turn_noise = new_t_noise;
        sense_noise = new_s_noise;
    }

    //--get robot sense values with noise
    vector<double> sense(){

        std::normal_distribution<double> gauss_s(0.0, sense_noise);

        vector<double> Z;
        for(int i = 0; i < rows_landmarks; i++){
            double dist = sqrtf((x-landmarks[i][0])*(x-landmarks[i][0]) + ((y-landmarks[i][1])*(y-landmarks[i][1])));
            dist += gauss_s(generator);
            Z.push_back(dist);
        }

        return Z;
    }

    //--move robot with noise
    robot* move(double turn, double forward){

        robot* res = new robot();

        if(forward < 0){printf("Robot cant move backwards\n"); return res;}
        else{
            std::normal_distribution<double> gauss_t(0.0, turn_noise);
            orientation = orientation + turn + gauss_t(generator);
            orientation = mod(orientation, 2.0*M_PI);

            std::normal_distribution<double> gauss_f(0.0, forward_noise);
            double dist = forward + gauss_f(generator);
            x = x + (cos(orientation) * dist);
            y = y + (sin(orientation) * dist);
            x = mod(x, world_size);
            y = mod(y, world_size);

            res->set(x,y,orientation);
            res->set_noise(forward_noise, turn_noise, sense_noise);

            return res;
        }
    }

    //--function gaussian
    double Gaussian(double mu, double sigma, double value){
        return expf(- ((mu - value)*(mu - value)) / (sigma *sigma) / 2.0) / sqrt(2.0 * M_PI * (sigma *sigma));
    }

    //--measurement probability
    double measurement_prob(vector<double> &measurement){
        double prob = 1.0;
        for(size_t i = 0; i < measurement.size(); i++){
            double dist = sqrtf((x-landmarks[i][0])*(x-landmarks[i][0]) + (y-landmarks[i][1])*(y-landmarks[i][1]));
            prob *= Gaussian(dist, sense_noise, measurement[i]);
        }
        return prob;
    }

    //--information
    void info(){
        printf("[x=%.25f y=%.25f heading=%.25f]\n", x, y, orientation);
    }
};

//--position comparation
double evaluation(robot* r, vector<robot*>  p){
    double sum = 0.0;
    for(size_t i = 0; i < p.size(); i++){
        double dx = mod(p[i]->x - r->x + (world_size/2.0), world_size) - (world_size/2.0);
        double dy = mod(p[i]->y - r->y + (world_size/2.0), world_size) - (world_size/2.0);
        double err = sqrtf(dx*dx + dy*dy);
        sum += err;
    }
    return sum/(double)(p.size());
}

//--particle filter
void ParticleFilter(){

    robot* myrobot = new robot();
    myrobot = myrobot->move(0.1, 5.0);
    vector<double> Z = myrobot->sense();

    int num_particles = 1000;
    int num_iterations = 10;

    //-- Make particles
    vector<robot*> Particles;

    for(int i = 0; i < num_particles; i++){
        robot* probot = new robot();
        probot->set_noise(0.05, 0.05, 5.0);
        Particles.push_back(probot);
    }

    //-- Update particles

    for(int k = 0; k < num_iterations; k++){

        myrobot = myrobot->move(0.1, 5.0);
        Z = myrobot->sense();

        //--motion update (prediction)
        vector<robot*> MoveParticles;
        for(int i = 0; i < num_particles; i++)MoveParticles.push_back(Particles[i]->move(0.1, 5.0));
        Particles = MoveParticles;

        //--measurement update
        vector<double> Weight;
        for(int i = 0; i < num_particles; i++) Weight.push_back(Particles[i]->measurement_prob(Z));

        //--resampling (wheel implementation)
        std::uniform_real_distribution<> random(0, 1);
        vector<robot*> NewParticles;

        int index = int(random(generator) * num_particles);
        double beta = 0.0;
        double maxWeight = 0.0;
        for(int i = 0; i < num_particles; i++) maxWeight = max(maxWeight, Weight[i]);

        for(int i = 0; i < num_particles; i++){
            beta +=  random(generator) * 2.0 * maxWeight;
            while(beta > Weight[index]){
                beta -= Weight[index];
                index = (index + 1) % num_particles;
            }
            NewParticles.push_back(Particles[index]);
        }

        Particles = NewParticles;
        printf("error: %.30f\n", evaluation(myrobot, Particles));
    }
}

//--main
int main(){
    ParticleFilter();
}
