/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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
	
	// define variables
	unsigned int current_particle = 0;
	
	// define number of particles
	num_particles = NUM_PARTICLES;
	
	// initialize particle vector
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		// initialize current particle
		Particle particle;
		particle.id = current_particle;
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
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// define variables
	unsigned int current_particle = 0;
	double theta_0 = 0;
	double velocity_dot = 0;
	double velocity_over_yaw_rate = 0;
	double theta_dot = 0;
	
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
	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, std::vector<int> &associations, std::vector<double> &sense_x, std::vector<double> &sense_y) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// define variables
	unsigned int current_observation = 0;
	unsigned int current_prediction = 0;
	double min_distance = INFINITE_DISTANCE;
	double distance = 0;
	
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
	
}

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
	double observation_weight = INIT_WEIGHT;
	
	// filter for landmarks in sensor distance
	for (current_landmark = 0; current_landmark < map_landmarks.landmark_list.size(); current_landmark++) {
		
		landmark = ParticleFilter::getMapLandmark(current_landmark, map_landmarks);
		distance = dist(0, 0, landmark.x, landmark.y);
		if (distance <= sensor_range) predicted.push_back(landmark);
		
	}
	
	// update weights for all particles
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
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
			ParticleFilter::transformVehicle2Map(part_x, part_y, obs_x, obs_y, part_theta, x_map, y_map);
			
			// save transformed observation
			observation_map.id = obs_id;
			observation_map.x = x_map;
			observation_map.y = y_map;
			observations_map.push_back(observation_map);
			
		}
		
		// reset associations
		associations.clear();
		sense_x.clear();
		sense_y.clear();
		
		// associate predicted landmarks to tranformed observations
		ParticleFilter::dataAssociation(predicted, observations_map, associations, sense_x, sense_y);
		particles[current_particle] = ParticleFilter::SetAssociations(particles[current_particle], associations, sense_x, sense_y);
		
		// initialize particle weight
		particles[current_particle].weight = INIT_WEIGHT;
		
		// calculate weights for each observation
		for (current_observation = 0; current_observation < observations.size(); current_observation++) {
			
			// multiply multi-variate Gaussian values
			observation_weight = mvg(part_x, part_y, sense_x[current_observation], sense_y[current_observation], std_landmark[0], std_landmark[1]);
			if (observation_weight < ZERO_DETECTION) observation_weight = ZERO_DETECTION;
			particles[current_particle].weight *= observation_weight;
			
		}
		
	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// define variables
	unsigned int current_particle = 0;
	std::vector<double> weights;
	std::vector<Particle> new_particles;
	
	// get all particle weights
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		weights.push_back(particles[current_particle].weight);
		
	}
	
	// define discrete distribution based on weights
	discrete_distribution<int> dd_p(weights.begin(), weights.end());
	
	// draw the same amount of new particles
	for (current_particle = 0; current_particle < num_particles; current_particle++) {
		
		new_particles.push_back(particles[dd_p(gen)]);
		
	}
	
	// use new particles instead of original ones
	particles = new_particles;
	
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
		
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
	
	// define Gaussian distribution noise
	normal_distribution<double> nd_x(mean[0], std[0]);
	normal_distribution<double> nd_y(mean[1], std[1]);
	normal_distribution<double> nd_theta(mean[2], std[2]);
	
	// add noise to particle
	particle.x += nd_x(gen);
	particle.y += nd_y(gen);
	particle.theta += nd_theta(gen);
	
}

LandmarkObs ParticleFilter::getMapLandmark(unsigned int num_landmark, Map map_landmarks) {
	// Function to retrieve landmark from map
	
	// define variables
	LandmarkObs landmark;
	
	// assign data
	landmark.id = map_landmarks.landmark_list[num_landmark].id_i;
	landmark.x = (double) map_landmarks.landmark_list[num_landmark].x_f;
	landmark.y = (double) map_landmarks.landmark_list[num_landmark].y_f;
	
	// return output
	return landmark;
	
}

void ParticleFilter::transformVehicle2Map(double x_offset_map, double y_offset_map, double x_change_relative, double y_change_relative, double alpha, double &x_map, double &y_map) {
	// Function to transform vehicle to map coordinates
	
	// transformations
	x_map = x_offset_map + (cos(alpha) * x_change_relative) - (sin(alpha) * y_change_relative);
	y_map = y_offset_map + (sin(alpha) * x_change_relative) + (cos(alpha) * y_change_relative);
	
}