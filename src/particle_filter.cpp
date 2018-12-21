/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Edited/finished by Andre Strobel (December 2018)
 *
 */

// #define CODE
#define TEST

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
#ifdef CODE
	
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
	
#endif /* CODE */
#ifdef TEST
	
  num_particles = 101;

  // define normal distributions for sensor noise
  normal_distribution<double> N_x_init(0, std[0]);
  normal_distribution<double> N_y_init(0, std[1]);
  normal_distribution<double> N_theta_init(0, std[2]);

  // init particles
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // add noise
    p.x += N_x_init(gen);
    p.y += N_y_init(gen);
    p.theta += N_theta_init(gen);

    particles.push_back(p);
  }

  is_initialized = true;
	
#endif /* TEST */
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
#ifdef CODE
	
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
	
#endif /* CODE */
#ifdef TEST
	
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
	
#endif /* TEST */
}

#ifdef CODE
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, std::vector<int> &associations, std::vector<double> &sense_x, std::vector<double> &sense_y) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
#endif /* CODE */
#ifdef TEST
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
#endif /* TEST */
#ifdef CODE
	
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
	
#endif /* CODE */
#ifdef TEST
	
  for (unsigned int i = 0; i < observations.size(); i++) {
    
    // grab current observation
    LandmarkObs o = observations[i];

    // init minimum distance to maximum possible
    double min_dist = numeric_limits<double>::max();

    // init id of landmark from map placeholder to be associated with the observation
    int map_id = -1;
    
    for (unsigned int j = 0; j < predicted.size(); j++) {
      // grab current prediction
      LandmarkObs p = predicted[j];
      
      // get distance between current/predicted landmarks
      double cur_dist = dist(o.x, o.y, p.x, p.y);

      // find the predicted landmark nearest the current observed landmark
      if (cur_dist < min_dist) {
        min_dist = cur_dist;
        map_id = p.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    observations[i].id = map_id;
  }
	
#endif /* TEST */
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
#ifdef CODE
	
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
	
#endif /* CODE */
#ifdef TEST
	
  // for each particle...
  for (int i = 0; i < num_particles; i++) {

    // get the particle x, y coordinates
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    // create a vector to hold the map landmark locations predicted to be within sensor range of the particle
    vector<LandmarkObs> predictions;

    // for each map landmark...
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      // get id and x,y coordinates
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;
      
      // only consider landmarks within sensor range of the particle (rather than using the "dist" method considering a circular 
      // region around the particle, this considers a rectangular region but is computationally faster)
      if (fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range) {

        // add prediction to vector
        predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
      }
    }

    // create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates
    vector<LandmarkObs> transformed_os;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
      transformed_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
    }

    // perform dataAssociation for the predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_os);

    // reinit weight
    particles[i].weight = 1.0;

    for (unsigned int j = 0; j < transformed_os.size(); j++) {
      
      // placeholders for observation and associated prediction coordinates
      double o_x, o_y, pr_x, pr_y;
      o_x = transformed_os[j].x;
      o_y = transformed_os[j].y;

      int associated_prediction = transformed_os[j].id;

      // get the x,y coordinates of the prediction associated with the current observation
      for (unsigned int k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == associated_prediction) {
          pr_x = predictions[k].x;
          pr_y = predictions[k].y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

      // product of this obersvation weight with total observations weight
      particles[i].weight *= obs_w;
    }
  }
	
#endif /* TEST */
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
#ifdef CODE
	
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
	
#endif /* CODE */
#ifdef TEST
	
  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  uniform_int_distribution<int> uniintdist(0, num_particles-1);
  auto index = uniintdist(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
	
#endif /* TEST */
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