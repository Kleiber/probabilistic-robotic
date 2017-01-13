#include<stdio.h>
#include<vector>
#include<math.h>
#include<random> //use CONFIG += c++11
#include<Eigen/Dense>
#include<iostream>

using namespace std;

//random generator
std::default_random_engine generator((unsigned int)time(0));

//--world configuration
#define rows_landmarks 4
#define cols_landmarks 2
#define world_size 100.0

double max_steering_angle = M_PI/ 4.0;
double bearing_noise  = 0.1;
double steering_noise = 0.1;
double distance_noise = 5.0;

double tolerance_xy = 15.0;
double tolerance_orientation = 0.25;

//--landmarks (y,x)
double landmarks[rows_landmarks][cols_landmarks] = {0.0, 100.0, 0.0, 0.0, 100.0, 0.0, 100.0, 100.0};

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
    double length;
    double bearing_noise;
    double steering_noise;
    double distance_noise;

    //--creates robot and initializes location/orientation
    robot(double _length = 20.0){

        std::uniform_real_distribution<> random(0, 1);
        x = random(generator) * world_size;
        y = random(generator) * world_size;
        orientation = random(generator) * 2.0 * M_PI;
        length = _length;
        bearing_noise  = 0.0;
        steering_noise = 0.0;
        distance_noise = 0.0;
    }

    //--sets a robot coordinate
    void set(double new_x, double new_y, double new_orientation){

        if(new_x < 0.0 || new_x >= world_size){printf("X coordinate out of bound\n"); return;}
        if(new_y < 0.0 || new_y >= world_size){printf("Y coordinate out of bound\n"); return;}
        if(new_orientation < 0.0 || new_orientation >= 2.0 * M_PI){printf("Orientation must be in [0..2pi]\n"); return;}
        x = new_x;
        y = new_y;
        orientation = new_orientation;
    }

    //--sets the noise parameters
    void set_noise(double new_b_noise, double new_s_noise, double new_d_noise){

        bearing_noise  = new_b_noise;
        steering_noise = new_s_noise;
        distance_noise = new_d_noise;
    }

    //--get robot sense values with noise(1) or without noise(0)
    vector<double> sense(bool add_noise = 1){

        vector<double> Z;
        for(int i = 0; i < rows_landmarks; i++){
            double bearing = atan2(landmarks[i][0] - y, landmarks[i][1] - x) - orientation;

            if(add_noise){
                std::normal_distribution<double> gauss_b(0.0, bearing_noise);
                bearing +=  gauss_b(generator);
            }

            bearing = mod(bearing, 2.0*M_PI);
            Z.push_back(bearing);
        }
        return Z;
    }

    //--move robot with noise
    robot* move(vector<double> motion, double tolerance = 0.001){

        robot* res = new robot();

        double steering = motion[0];
        double distance = motion[1];

        if(abs(steering) > max_steering_angle){printf("Exceeding max steerig angle\n"); return res;}
        if(distance < 0.0){printf("Moving backwards is not valid\n"); return res;}

        res->length = length;
        res->bearing_noise = bearing_noise;
        res->steering_noise = steering_noise;
        res->distance_noise = distance_noise;

        std::normal_distribution<double> gauss_s(0.0, steering_noise);
        std::normal_distribution<double> gauss_d(0.0, distance_noise);
        double steering2 = gauss_s(generator);
        double distance2 = gauss_d(generator);

        double turn = tan(steering2) * distance2/ length;

        if(abs(turn) < tolerance){
            res->x = x + (distance2 * cos(orientation));
            res->y = y + (distance2 * sin(orientation));
            res->orientation = mod(orientation + turn, 2.0*M_PI);
        }else{
            double radius = distance2/turn;
            double cx = x - (sin(orientation)*radius);
            double cy = y + (cos(orientation)*radius);
            res->orientation = mod(orientation + turn, 2.0*M_PI);
            res->x = cx + (sin(res->orientation)*radius);
            res->y = cy - (cos(res->orientation)*radius);
        }

        return res;
    }


    //--computes the probability of a measurement
    double measurement_prob(vector<double> &measurement){
        //--calculate the correct measurement
        vector<double> predicted_measurements = sense(0);

        //--compute error
        double error = 1.0;
        for(size_t i = 0; i < measurement.size(); i++){
            double error_bearing = abs(measurement[i] - predicted_measurements[i]);
            error_bearing = mod(error_bearing + M_PI, 2.0*M_PI) - M_PI;

            //--update Gaussian
            error *= (expf(- (error_bearing*error_bearing) / (bearing_noise*bearing_noise) / 2.0) / sqrtf(2.0*M_PI*bearing_noise*bearing_noise));
        }
        return error;
    }

    //--information
    void info(){
        printf("[x=%.25f y=%.25f heading=%.25f]\n", x, y, orientation);
    }
};


//--generates the measurements vector
void generate_ground_truth(Eigen::MatrixXd &motions, Eigen::VectorXd &final_robot, Eigen::MatrixXd &measurements){

    robot* myrobot = new robot();
    myrobot->set_noise(bearing_noise, steering_noise, distance_noise);

    for(int i = 0; i < motions.rows(); i++){
        vector<double> motion_i(2);
        motion_i[0] = motions(i,0);
        motion_i[1] = motions(i,1);
        myrobot = myrobot->move(motion_i);

        vector<double> Z = myrobot->sense();
        measurements(i,0) = Z[0];
        measurements(i,1) = Z[1];
        measurements(i,2) = Z[2];
        measurements(i,3) = Z[3];
    }

    final_robot(0) = myrobot->x;
    final_robot(1) = myrobot->y;
    final_robot(2) = myrobot->orientation;
}

//--extract position from a particle set
Eigen::VectorXd get_position(vector<robot*> &p){

    double x = 0.0;
    double y = 0.0;
    double orientation = 0.0;

    for(size_t i = 0; i < p.size(); i++){
        x += p[i]->x;
        y += p[i]->y;
        orientation += ((mod(p[i]->orientation - p[0]->orientation + M_PI, 2.0 * M_PI)) + (p[0]->orientation - M_PI));
    }

    double len = (double)(p.size());

    Eigen::VectorXd position(3);
    position(0) = x/len;
    position(1) = y/len;
    position(2) = orientation/len;

    return position;
}

//--checks to see if your particle filter localizes the robot to within the desired tolerances of the true position
string check_output(Eigen::VectorXd &final_robot, Eigen::VectorXd &estimated_position){

    double error_x = abs(final_robot(0) - estimated_position(0));
    double error_y = abs(final_robot(1) - estimated_position(1));
    double error_orientation = abs(final_robot(2) - estimated_position(2));
    error_orientation = mod(error_orientation + M_PI, 2.0*M_PI) - M_PI;

    bool correct = error_x < tolerance_xy && error_y < tolerance_xy && error_orientation < tolerance_orientation;

    if(correct) return "True";
    else return "False";
}

//--particle filter
Eigen::VectorXd ParticleFilter(Eigen::MatrixXd &motions, Eigen::MatrixXd &measurements, int number_of_particles = 500){

    //-- Make particles
    vector<robot*> Particles;
    for(int i = 0; i < number_of_particles; i++){
        robot* probot = new robot();
        probot->set_noise(bearing_noise, steering_noise, distance_noise);
        Particles.push_back(probot);
    }

    //-- Update particles
    for(int i = 0; i < motions.rows(); i++){

        //--motion update (prediction)
        vector<robot*> MoveParticles;
        for(int j = 0; j < number_of_particles; j++){
            vector<double> motion_i(2);
            motion_i[0] = motions(i,0);
            motion_i[1] = motions(i,1);

            MoveParticles.push_back(Particles[j]->move(motion_i));
        }
        Particles = MoveParticles;

        //--measurement update
        vector<double> Weight;
        for(int j = 0; j < number_of_particles; j++){
            vector<double> Z_i(4);
            Z_i[0] = measurements(i,0);
            Z_i[1] = measurements(i,1);
            Z_i[2] = measurements(i,2);
            Z_i[3] = measurements(i,3);

            Weight.push_back(Particles[j]->measurement_prob(Z_i));
        }

        //--resampling (wheel implementation)
        std::uniform_real_distribution<> random(0, 1);
        vector<robot*> NewParticles;

        int index = int(random(generator) * number_of_particles);
        double beta = 0.0;
        double maxWeight = 0.0;
        for(int j = 0; j < number_of_particles; j++) maxWeight = max(maxWeight, Weight[j]);

        for(int j = 0; j < number_of_particles; j++){
            beta +=  random(generator) * 2.0 * maxWeight;
            while(beta > Weight[index]){
                beta -= Weight[index];
                index = (index + 1) % number_of_particles;
            }
            NewParticles.push_back(Particles[index]);
        }

        Particles = NewParticles;
    }

    return get_position(Particles);
}

//--main
int main(){
    int number_of_iterations = 6;

    Eigen::MatrixXd motions(number_of_iterations,2);
    for(int i = 0; i < number_of_iterations; i++){
        motions(i,0) = 2.0*M_PI/20.0;
        motions(i,1) = 12.0;
    }

    Eigen::VectorXd final_robot(3);
    Eigen::MatrixXd measurements(number_of_iterations, 4);
    generate_ground_truth(motions, final_robot, measurements);

    Eigen::VectorXd estimated_position = ParticleFilter(motions, measurements);

    cout<<"Measurements: "<<endl;
    cout<<measurements<<endl;
    cout<<"Ground truth: "<<final_robot.transpose()<<endl;
    cout<<"Particle filter: "<<estimated_position.transpose()<<endl;
    cout<<"Code check: "<<check_output(final_robot, estimated_position)<<endl;
}

