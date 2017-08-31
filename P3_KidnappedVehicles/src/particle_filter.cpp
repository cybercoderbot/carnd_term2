/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

// declare a random engine to be used across multiple and various method calls
// static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
  // create normal distributions of noise for x, y, and theta
  normal_distribution<double> noise_x(x, std[0]);
  normal_distribution<double> noise_y(y, std[1]);
  normal_distribution<double> noise_theta(theta, std[2]);
  
  // declare a random engine 
  default_random_engine gen;

  // resize the vectors of particles and weights
  num_particles = 100;
  particles.resize(num_particles);
  //weights.resize(num_particles);

  // generate the particles
  for(auto& p: particles){
    p.x = noise_x(gen);
    p.y = noise_y(gen);
    p.theta = noise_theta(gen);
    p.weight = 1;
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
  default_random_engine gen;
  
  //define normal distributions for sensor noise
  normal_distribution<double> noise_x(0, std_pos[0]);
  normal_distribution<double> noise_y(0, std_pos[1]);
  normal_distribution<double> noise_theta(0, std_pos[2]);
  
  double delta_s = velocity * delta_t;
  double v_y = velocity / yaw_rate;
  double delta_yaw = yaw_rate * delta_t;

  for(auto& p: particles){

    // add measurements to each particle
    if( fabs(yaw_rate) < 0.0001){  // constant velocity
      p.x += delta_s * cos(p.theta);
      p.y += delta_s * sin(p.theta);
    } else {
      p.x += v_y * ( sin( p.theta + yaw_rate * delta_t ) - sin(p.theta) );
      p.y += v_y * ( cos( p.theta ) - cos( p.theta + yaw_rate * delta_t ) );
      p.theta += delta_yaw;
    }

    // predicted particles with added sensor noise
    p.x += noise_x(gen);
    p.y += noise_y(gen);
    p.theta += noise_theta(gen);
  }
  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
  for (auto& obs : observations) {
    // init minimum distance to maximum possible
    double min_dist = numeric_limits<double>::max();

    // init id of landmark from map placeholder to be associated with the observation
    int map_id = -1;
    
    for (auto& pred : predicted) {
      
      // get distance between current/predicted landmarks
      double cur_dist = dist(obs.x, obs.y, pred.x, pred.y);

      // find the predicted landmark nearest the current observed landmark
      if (cur_dist < min_dist) {
        min_dist = cur_dist;
        map_id = pred.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    obs.id = map_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
	vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
  double s_x = std_landmark[0];
  double s_y = std_landmark[1];
	
  // for each particle
  for (auto& p: particles) {
	// re-initialize weight
    p.weight = 1.0;
    
    // map landmark locations (within particle sensor range)
    vector<LandmarkObs> predictions;

    // for each map landmark
    for (auto& lm: map_landmarks.landmark_list){
      // step 1: collect valid landmarks, only consider landmarks within particle sensor range 
      if (fabs(lm.x_f - p.x) <= sensor_range && fabs(lm.y_f - p.y) <= sensor_range) {
        // add prediction to vector
        predictions.push_back(LandmarkObs{ lm.id_i, lm.x_f, lm.y_f });
      }
    }
    
    // step 2: convert observations coordinates from vehicle to map
    vector<LandmarkObs> transformed_os;
    for(auto& obs: observations){
      double t_x = cos(p.theta) * obs.x - sin(p.theta) * obs.y + p.x;
      double t_y = sin(p.theta) * obs.x + cos(p.theta) * obs.y + p.y;
      transformed_os.push_back(LandmarkObs{ obs.id, t_x, t_y });
    }

    // perform dataAssociation for the predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_os);

    // for (int j = 0; j < transformed_os.size(); j++) {
    for (auto& ob : transformed_os) {
      double pd_x, pd_y;

      // get the x,y coordinates of the prediction for the current observation
      for (auto& pred : predictions) {
        if (pred.id == ob.id) {
          pd_x = pred.x;
          pd_y = pred.y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double x_term = pow(pd_x - ob.x, 2) / (2*pow(s_x, 2));
      double y_term = pow(pd_y - ob.y, 2) / (2*pow(s_y, 2));

      p.weight *= ( 1/(2 * M_PI * s_x * s_y)) * exp( -(x_term + y_term) );
    }
  }
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
  default_random_engine gen;
  vector<Particle> resampled_particles;

  // get all of the current weights
  vector<double> weights;
  for (auto& p : particles) {
    weights.push_back(p.weight);
  }

  // generate random starting index for resampling wheel
  discrete_distribution<int> discrete_dist(0, num_particles-1);
  auto idx = discrete_dist(gen);

  
  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0, max_weight)
  uniform_real_distribution<double> uni_real_dist(0, max_weight);
  
  double thresh = 0;
  // resample the particles according to weights
  for (int i = 0; i < num_particles; i++) {
    thresh += uni_real_dist(gen) * 2.0;
    while (weights[idx] < thresh) {
      thresh -= weights[idx];
      idx = (idx + 1) % num_particles;
    }
    resampled_particles.push_back(particles[idx]);
  }

  particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, vector<int> associations, vector<double> sense_x, vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}


void ParticleFilter::write(string filename) {
  // You don't need to modify this file.
  ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  for (int i = 0; i < num_particles; ++i) {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}


string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
