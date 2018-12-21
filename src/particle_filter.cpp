/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Edited/finished by Andre Strobel (December 2018)
 *
 */

#define CODE_INIT
#define CODE_PREP
#define CODE_DATA
#define CODE_UPDA
#define CODE_RESA
/*
#define TEST_INIT
#define TEST_PREP
#define TEST_DATA
#define TEST_UPDA
#define TEST_RESA
#define TEST
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

using namespace std;

// define noise generator
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
#ifdef CODE_INIT
	
	// define variables
	unsigned int current_particle = 0;
	Particle particle;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_init) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: init - Start" << endl;
		cout << "  x: " << endl << x << endl << endl;
		cout << "  y: " << endl << y << endl << endl;
		cout << "  theta: " << endl << theta << endl << endl;
		cout << "  std: " << endl << createArrayString(std, 3) << endl;
	}
	
	// define number of particles
	num_particles = NUM_PARTICLES;
	
	// initialize particle vector
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		// initialize current particle
		particle.id = current_particle + 1;
		particle.x = x;
		particle.y = y;
		particle.theta = theta;
		particle.weight = INIT_WEIGHT;
		
		// add noise to current particle
		addNoise(particle, ZERO_MEAN, std);
		
		// add current particle to particle vector
		particles.push_back(particle);
	}
	
	// all particles are initialized
	is_initialized = true;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_init) {
		cout << "  num_particles: " << endl << num_particles << endl << endl;
		cout << "  particles: " << endl << createParticlesString(particles) << endl;
		cout << "  is_initialized: " << endl << is_initialized << endl << endl;
		cout << "--- PARTICLE_FILTER: init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
#endif /* CODE_INIT */
#ifdef TEST_INIT
	
  // Add random Gaussian noise to each particle.
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  double sample_x, sample_y, sample_psi;

  num_particles = 200;
  weights.resize(num_particles, 1.0f);

  for(unsigned i=0; i<num_particles; i++)
  {
    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.id = i;
    p.weight = 1.0f;
    particles.push_back(p);
  }
  is_initialized = true;
	
#endif /* TEST_INIT */
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
#ifdef CODE_PRED
	
	// define variables
	unsigned int current_particle = 0;
	double theta_0 = 0;
	double velocity_dot = 0;
	double velocity_over_yaw_rate = 0;
	double theta_dot = 0;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_prediction) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: prediction - Start" << endl;
		cout << "  delta_t: " << endl << delta_t << endl << endl;
		cout << "  std_pos: " << endl << createArrayString(std_pos, 3) << endl;
		cout << "  velocity: " << endl << velocity << endl << endl;
		cout << "  yaw_rate: " << endl << yaw_rate << endl << endl;
	}
	
	// move all particles according to measurement
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		//check whether yaw rate is zero
		if (fabs(yaw_rate) < ZERO_DETECTION) {
			
			// precalculations
			theta_0 = particles[current_particle].theta;
			velocity_dot = velocity * delta_t;
			
			// motion step
			particles[current_particle].x += velocity_dot * cos(theta_0);
			particles[current_particle].y += velocity_dot * sin(theta_0);
			
		}
		else {
			
			// precalculations
			theta_0 = particles[current_particle].theta;
			velocity_over_yaw_rate = velocity / yaw_rate;
			theta_dot = yaw_rate * delta_t;
			
			// motion step
			particles[current_particle].x += velocity_over_yaw_rate * (sin(theta_0 + theta_dot) - sin(theta_0));
			particles[current_particle].y += velocity_over_yaw_rate * (cos(theta_0) - cos(theta_0 + theta_dot));
			particles[current_particle].theta += theta_dot;
			
		}
		
		// add noise to current particle
		addNoise(particles[current_particle], ZERO_MEAN, std_pos);
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_prediction) {
		cout << "  particles: " << endl << createParticlesString(particles) << endl;
		cout << "--- PARTICLE_FILTER: prediction - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
#endif /* CODE_PRED */
#ifdef TEST_PRED
	
  // define normal distributions for sensor noise
  normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    // calculate new state
    if (fabs(yaw_rate) < 0.00001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // add noise
    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }
	
#endif /* TEST_PRED */
}

#ifdef CODE_DATA
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, std::vector<int> &associations, std::vector<double> &sense_x, std::vector<double> &sense_y) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
#endif /* CODE_DATA */
#ifdef TEST_DATA
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
#endif /* TEST_DATA */
#ifdef CODE_DATA
	
	// define variables
	unsigned int current_observation = 0;
	unsigned int current_prediction = 0;
	double min_distance = INFINITE_DISTANCE;
	double distance = 0;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_dataAssociation) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: dataAssociation - Start" << endl;
		cout << "  predicted: " << endl << createLandmarksString(predicted) << endl;
		cout << "  observations: " << endl << createLandmarksString(observations) << endl;
		cout << "  associations: " << endl << createIntegerVectorString(associations) << endl;
		cout << "  sense_x: " << endl << createDoubleVectorString(sense_x) << endl;
		cout << "  sense_y: " << endl << createDoubleVectorString(sense_y) << endl;
	}
	
	// find nearest predicted landmark for all observed landmarks
	for (current_observation = 0; current_observation < observations.size(); current_observation++) {
		
		// reinitialize minimum distance for next observation
		min_distance = INFINITE_DISTANCE;
		
		// loop through all predicted landmarks
		for (current_prediction = 0; current_prediction < predicted.size(); current_prediction++) {
			
			// calculate distance between observation and prediction
			distance = dist(observations[current_observation].x, observations[current_observation].y, predicted[current_prediction].x, predicted[current_prediction].y);
			
			// smaller distance found
			if (distance < min_distance) {
				
				// assign current prediction to current observation
				min_distance = distance;
				observations[current_observation].id = predicted[current_prediction].id;
				
			}
			
		}
		
		// save and return best fit
		associations.push_back(observations[current_observation].id);
		sense_x.push_back(observations[current_observation].x);
		sense_y.push_back(observations[current_observation].y);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_dataAssociation) {
		cout << "  observations: " << endl << createLandmarksString(observations) << endl;
		cout << "  associations: " << endl << createIntegerVectorString(associations) << endl;
		cout << "  sense_x: " << endl << createDoubleVectorString(sense_x) << endl;
		cout << "  sense_y: " << endl << createDoubleVectorString(sense_y) << endl;
		cout << "--- PARTICLE_FILTER: dataAssociation - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
#endif /* CODE_DATA */
#ifdef TEST_DATA
	
  double min_distance, dist, dx, dy;
  int min_i;

  for(unsigned obs_i = 0; obs_i < observations.size(); obs_i++)
  {
    auto obs = observations[obs_i];

    min_distance = INFINITY;
    min_i = -1;
    for(unsigned i = 0; i < predicted.size(); i++)
    {
      auto pred_lm = predicted[i];
      dx = (pred_lm.x - obs.x);
      dy = (pred_lm.y - obs.y);
      dist = dx*dx + dy*dy;
      if(dist < min_distance)
      {
        min_distance = dist;
        min_i = i;
      }
    }
    observations[obs_i].id = min_i; // Use index of landmark as the ID (rather than the id field)
  }
	
#endif /* TEST_DATA */
}

#ifdef TEST
const LandmarkObs local_to_global(const LandmarkObs& obs, const Particle& p)
{
  LandmarkObs out;

  // First rotate the local coordinates to the right orientation
  out.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
  out.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
  out.id = obs.id;
  return out;
}

inline const double gaussian_2d(const LandmarkObs& obs, const LandmarkObs &lm, const double sigma[])
{
  auto cov_x = sigma[0]*sigma[0];
  auto cov_y = sigma[1]*sigma[1];
  auto normalizer = 2.0*M_PI*sigma[0]*sigma[1];
  auto dx = (obs.x - lm.x);
  auto dy = (obs.y - lm.y);
  return exp(-(dx*dx/(2*cov_x) + dy*dy/(2*cov_y)))/normalizer;
}
#endif /* TEST */

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
#ifdef CODE_UPDA
	
	// define variables
	unsigned int current_landmark = 0;
	LandmarkObs landmark;
	double distance = 0;
	std::vector<LandmarkObs> predicted;
	unsigned int current_particle = 0;
	double part_x = 0;
	double part_y = 0;
	double part_theta = 0;
	unsigned int current_observation = 0;
	double obs_id = 0;
	double obs_x = 0;
	double obs_y = 0;
	double x_map = 0;
	double y_map = 0;
	LandmarkObs observation_map;
	std::vector<LandmarkObs> observations_map;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
	unsigned int current_observation_map = 0;
	unsigned int current_predicted = 0;
	double pred_x = 0;
	double pred_y = 0;
	double observation_weight = INIT_WEIGHT;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_updateWeights) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: updateWeights - Start" << endl;
		cout << "  sensor_range: " << endl << sensor_range << endl << endl;
		cout << "  std_landmark: " << endl << createArrayString(std_landmark, 2) << endl;
		cout << "  observations: " << endl << createLandmarksString(observations) << endl;
		cout << "  map_landmarks: " << endl << createMapString(map_landmarks) << endl;
	}
	cout << "  observations: " << endl << createLandmarksString(observations) << endl;
	cout << "  map_landmarks: " << endl << createMapString(map_landmarks) << endl;
	
	// filter for landmarks in sensor distance
	for (current_landmark = 0; current_landmark < map_landmarks.landmark_list.size(); current_landmark++) {
		
		landmark = getMapLandmark(current_landmark, map_landmarks);
		distance = dist(0, 0, landmark.x, landmark.y);
		if (distance <= sensor_range) predicted.push_back(landmark);
		
	}
	
	// remove all previous weights
	weights.clear();
	
	// update weights for all particles
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		// reset associations
		observations_map.clear();
		associations.clear();
		sense_x.clear();
		sense_y.clear();
		
		// get data of current particle
		part_x = particles[current_particle].x;
		part_y = particles[current_particle].y;
		part_theta = particles[current_particle].theta;
		
		// translate observations for particle from vehicle to map coordinates
		for (current_observation = 0; current_observation < observations.size(); current_observation++) {
			
			// get data of current observation
			obs_id = observations[current_observation].id;
			obs_x = observations[current_observation].x;
			obs_y = observations[current_observation].y;
			
			// translate observation for particle
			transformVehicle2Map(part_x, part_y, obs_x, obs_y, part_theta, x_map, y_map);
			
			// save transformed observation
			observation_map.id = obs_id;
			observation_map.x = x_map;
			observation_map.y = y_map;
			observations_map.push_back(observation_map);
			
		}
		
		// associate predicted landmarks to tranformed observations
		dataAssociation(predicted, observations_map, associations, sense_x, sense_y);
		particles[current_particle] = SetAssociations(particles[current_particle], associations, sense_x, sense_y);
		
		// initialize particle weight
		particles[current_particle].weight = INIT_WEIGHT;
		
		// calculate weights for each observation
		for (current_observation_map = 0; current_observation_map < observations_map.size(); current_observation_map++) {
			
			// search for predicted landmark with associated id to current observation
			for (current_predicted = 0; current_predicted < predicted.size(); current_predicted++) {
				
				// correct predicted landmark found
				if (predicted[current_predicted].id == associations[current_observation_map]) {
					
					// use x and y values for weight calculation
					pred_x = predicted[current_predicted].x;
					pred_y = predicted[current_predicted].y;
					
				}
				
			}
			
			// multiply multi-variate Gaussian values
			observation_weight = mvg(pred_x, pred_y, sense_x[current_observation_map], sense_y[current_observation_map], std_landmark[0], std_landmark[1]);
			// DEBUGGING ONLY - Start
			//cout << "DEBUG mvg: pred_x=" << pred_x << " pred_y=" << pred_y << " sense_x=" << sense_x[current_observation_map] << " sense_y=" << sense_y[current_observation_map] << " std0=" << std_landmark[0] << " std1=" << std_landmark[1] << endl;
			//cout << "DEBUG current_observation_map=" << current_observation_map << " observation_weight=" << observation_weight << endl;
			// DEBUGGING ONLY - End
			if (observation_weight < ZERO_DETECTION) observation_weight = ZERO_DETECTION;
			particles[current_particle].weight *= observation_weight;
			
		}
		
		// store current weight also in weight structure in particle filter object
		weights.push_back(particles[current_particle].weight);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_updateWeights) {
		cout << "  observations: " << endl << createLandmarksString(observations) << endl;
		cout << "  map_landmarks: " << endl << createMapString(map_landmarks) << endl;
		cout << "  predicted: " << endl << createLandmarksString(predicted) << endl;
		cout << "  particles: " << endl << createParticlesString(particles) << endl;
		cout << "--- PARTICLE_FILTER: updateWeights - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	cout << "  predicted: " << endl << createLandmarksString(predicted) << endl;
	
#endif /* CODE_UPDA */
#ifdef TEST_UPDA
	
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  for(unsigned p_ctr=0; p_ctr < particles.size(); p_ctr++)
  {
    auto p = particles[p_ctr];

    std::vector<LandmarkObs> predicted_landmarks;

    for(auto lm : map_landmarks.landmark_list)
    {
      LandmarkObs lm_pred;
      lm_pred.x = lm.x_f;
      lm_pred.y = lm.y_f;
      lm_pred.id = lm.id_i;
      auto dx = lm_pred.x - p.x;
      auto dy = lm_pred.y - p.y;

      // Add only if in range
      if(dx*dx + dy*dy <= sensor_range*sensor_range)
        predicted_landmarks.push_back(lm_pred);
    }
    std::vector<LandmarkObs> transformed_obs;
    double total_prob = 1.0f;

    // transform coordinates of all observations (for current particle)
    for(auto obs_lm : observations)
    {
      auto obs_global = local_to_global(obs_lm, p);
      transformed_obs.push_back(std::move(obs_global));
    }
    // Stores index of associated landmark in the observation
    dataAssociation(predicted_landmarks, transformed_obs);

    for(unsigned i=0; i < transformed_obs.size(); i++)
    {
      auto obs = transformed_obs[i];
      // Assume sorted by id and starting at 1
      auto assoc_lm = predicted_landmarks[obs.id];

      double pdf = gaussian_2d(obs, assoc_lm, sigma_landmark);
      total_prob *= pdf;
    }
    particles[p_ctr].weight = total_prob;
    weights[p_ctr] = total_prob;
  }
  std::cout<<std::endl;
	
#endif /* TEST_UPDA */
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
#ifdef CODE_RESA
	
	// define variables
	unsigned int current_particle = 0;
	std::vector<Particle> new_particles;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_resample) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: resample - Start" << endl;
		cout << "  particles: " << endl << createParticlesString(particles) << endl;
	}
	
	// define discrete distribution based on weights
	discrete_distribution<int> dd_p(weights.begin(), weights.end());
	
	// draw the same amount of new particles
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		new_particles.push_back(particles[dd_p(gen)]);
		
	}
	
	// use new particles instead of original ones
	particles = new_particles;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_resample) {
		cout << "  weights: " << endl << createDoubleVectorString(weights) << endl;
		cout << "  particles: " << endl << createParticlesString(particles) << endl;
		cout << "--- PARTICLE_FILTER: resample - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
#endif /* CODE_RESA */
#ifdef TEST_RESA
	
  std::discrete_distribution<int> d(weights.begin(), weights.end());
  std::vector<Particle> new_particles;

  for(unsigned i = 0; i < num_particles; i++)
  {
    auto ind = d(gen);
    new_particles.push_back(std::move(particles[ind]));
  }
  particles = std::move(new_particles);
	
#endif /* TEST_RESA */
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	
	// display message if required
	if (bDISPLAY && bDISPLAY_SetAssociations) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: SetAssociations - Start" << endl;
		cout << "  particle: " << endl << createParticleString(particle) << endl;
		cout << "  associations: " << endl << createIntegerVectorString(associations) << endl;
		cout << "  sense_x: " << endl << createDoubleVectorString(sense_x) << endl;
		cout << "  sense_y: " << endl << createDoubleVectorString(sense_y) << endl;
	}
	
	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_SetAssociations) {
		cout << "  particle: " << endl << createParticleString(particle) << endl;
		cout << "  associations: " << endl << createIntegerVectorString(associations) << endl;
		cout << "  sense_x: " << endl << createDoubleVectorString(sense_x) << endl;
		cout << "  sense_y: " << endl << createDoubleVectorString(sense_y) << endl;
		cout << "--- PARTICLE_FILTER: SetAssociations - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	// return output
	return particle;
	
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

void ParticleFilter::addNoise(Particle &particle, double mean[], double std[]) {
	// Function to add noise to a single particle
	
	// display message if required
	if (bDISPLAY && bDISPLAY_addNoise) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: addNoise - Start" << endl;
		cout << "  particle: " << endl << createParticleString(particle) << endl;
		cout << "  mean: " << endl << createArrayString(mean, 3) << endl;
		cout << "  std: " << endl << createArrayString(std, 3) << endl;
	}
	
	// define Gaussian distribution noise
	normal_distribution<double> nd_x(mean[0], std[0]);
	normal_distribution<double> nd_y(mean[1], std[1]);
	normal_distribution<double> nd_theta(mean[2], std[2]);
	
	// add noise to particle
	particle.x += nd_x(gen);
	particle.y += nd_y(gen);
	particle.theta += nd_theta(gen);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_addNoise) {
		cout << "  particle: " << endl << createParticleString(particle) << endl;
		cout << "--- PARTICLE_FILTER: addNoise - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

LandmarkObs ParticleFilter::getMapLandmark(unsigned int num_landmark, Map map_landmarks) {
	// Function to retrieve landmark from map
	
	// define variables
	LandmarkObs landmark;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_getMapLandmark) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: getMapLandmark - Start" << endl;
		cout << "  num_landmark: " << endl << num_landmark << endl;
		cout << "  map_landmarks: " << endl << createMapString(map_landmarks) << endl;
	}
	
	// assign data
	landmark.id = map_landmarks.landmark_list[num_landmark].id_i;
	landmark.x = (double) map_landmarks.landmark_list[num_landmark].x_f;
	landmark.y = (double) map_landmarks.landmark_list[num_landmark].y_f;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_getMapLandmark) {
		cout << "  landmark: " << endl << createLandmarkString(landmark) << endl;
		cout << "--- PARTICLE_FILTER: getMapLandmark - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	// return output
	return landmark;
	
}

void ParticleFilter::transformVehicle2Map(double x_offset_map, double y_offset_map, double x_change_relative, double y_change_relative, double alpha, double &x_map, double &y_map) {
	// Function to transform vehicle to map coordinates
	
	// display message if required
	if (bDISPLAY && bDISPLAY_transformVehicle2Map) {
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PARTICLE_FILTER: transformVehicle2Map - Start" << endl;
		cout << "  x_offset_map: " << endl << x_offset_map << endl << endl;
		cout << "  y_offset_map: " << endl << y_offset_map << endl << endl;
		cout << "  x_change_relative: " << endl << x_change_relative << endl << endl;
		cout << "  y_change_relative: " << endl << y_change_relative << endl << endl;
		cout << "  alpha: " << endl << alpha << endl << endl;
		cout << "  x_map: " << endl << x_map << endl << endl;
		cout << "  y_map: " << endl << y_map << endl << endl;
	}
	
	// transformations
	x_map = x_offset_map + (cos(alpha) * x_change_relative) - (sin(alpha) * y_change_relative);
	y_map = y_offset_map + (sin(alpha) * x_change_relative) + (cos(alpha) * y_change_relative);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_transformVehicle2Map) {
		cout << "  x_map: " << endl << x_map << endl << endl;
		cout << "  y_map: " << endl << y_map << endl << endl;
		cout << "--- PARTICLE_FILTER: transformVehicle2Map - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

std::string ParticleFilter::createDoubleVectorString(std::vector<double> double_vector) {
	// Function that creates a string for a vector of doubles
	
	//define variables
	unsigned int current_element = 0;
	std::ostringstream streamObj;
	
	// add information about all elements to string
	for (current_element = 0; current_element < double_vector.size(); current_element++) {
		
		streamObj << "Element " << current_element << ": " << double_vector[current_element] << endl;
		
	}
	
	// return output
	return streamObj.str();
	
}

std::string ParticleFilter::createIntegerVectorString(std::vector<int> int_vector) {
	// Function that creates a string for a vector of integers
	
	//define variables
	unsigned int current_element = 0;
	std::string text = "";
	
	// add information about all elements to string
	for (current_element = 0; current_element < int_vector.size(); current_element++) {
		
		text += "Element " + std::to_string(current_element) + ": " + std::to_string(int_vector[current_element]) + "\n";
		
	}
	
	// return output
	return text;
	
}

std::string ParticleFilter::createArrayString(double array[], unsigned int num_elements) {
	// Function that creates a string for an array
	
	//define variables
	unsigned int current_element = 0;
	std::ostringstream streamObj;
	
	// add information about all elements to string
	for (current_element = 0; current_element < num_elements; current_element++) {
		
		streamObj << "Element " << current_element << ": " << array[current_element] << endl;
		
	}
	
	// return output
	return streamObj.str();
	
}

std::string ParticleFilter::createParticleString(Particle particle) {
	// Function that creates a string for one particle
	
	//define variables
	std::string text = "";
	
	// add information about particle to string
	text += "id=" + std::to_string(particle.id) + " ";
	text += "x=" + std::to_string(particle.x) + " ";
	text += "y=" + std::to_string(particle.y) + " ";
	text += "theta=" + std::to_string(particle.theta) + " ";
	text += "weight=" + std::to_string(particle.weight) + "\n";
	text += "associations=\n" + createIntegerVectorString(particle.associations);
	text += "sense_x=\n" + createDoubleVectorString(particle.sense_x);
	text += "sense_y=\n" + createDoubleVectorString(particle.sense_y);
	
	// retrun output
	return text;
	
}

std::string ParticleFilter::createParticlesString(std::vector<Particle> particles) {
	// Function that creates a string for all particles
	
	//define variables
	unsigned int current_particle = 0;
	std::string text = "";
	
	// add information about all particles to string
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		text += "Particle " + std::to_string(current_particle) + ": " + createParticleString(particles[current_particle]) + "\n";
		
	}
	
	// return output
	return text;
	
}

std::string ParticleFilter::createLandmarkString(LandmarkObs landmark) {
	// Function that creates a string for one landmark
	
	//define variables
	std::string text = "";
	
	// add information about particle to string
	text += "id=" + std::to_string(landmark.id) + " ";
	text += "x=" + std::to_string(landmark.x) + " ";
	text += "y=" + std::to_string(landmark.y);
	
	// retrun output
	return text;
	
}

std::string ParticleFilter::createLandmarksString(std::vector<LandmarkObs> landmarks) {
	// Function that creates a string for all landmarks
	
	//define variables
	unsigned int current_landmark = 0;
	std::string text = "";
	
	// add information about all particles to string
	for (current_landmark = 0; current_landmark < landmarks.size(); current_landmark++) {
		
		text += "Landmark " + std::to_string(current_landmark) + ": " + createLandmarkString(landmarks[current_landmark]) + "\n";
		
	}
	
	// return output
	return text;
	
}

std::string ParticleFilter::createMapString(Map map) {
	// Function that creates a string for a map
	
	//define variables
	unsigned int current_landmark = 0;
	std::string text = "";
	
	// add information about all landmarks to string
	for (current_landmark = 0; current_landmark < map.landmark_list.size(); current_landmark++) {
		
		text += "Map landmark " + std::to_string(current_landmark) + ": ";
		text += "id_i=" + std::to_string(map.landmark_list[current_landmark].id_i) + " ";
		text += "x_f=" + std::to_string(map.landmark_list[current_landmark].x_f) + " ";
		text += "y_f=" + std::to_string(map.landmark_list[current_landmark].y_f) + "\n";
	}
	
	// return output
	return text;
	
}