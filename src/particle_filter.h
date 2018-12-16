/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Edited/finished by Andre Strobel (December 2018)
 *
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};



class ParticleFilter {
	
	// Number of particles to draw
	unsigned int num_particles; 
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;
	
public:
	
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param num_particles Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, std::vector<int>& associations, std::vector<double>& sense_x, std::vector<double>& sense_y);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 * @param std::vector<int> associations array containing best fit associated landmark ids
	 * @param std::vector<double> sense_x array containing best fit associated landmark x positions
	 * @param std::vector<double> sense_y array containing best fit associated landmark y positions
	 */
	void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
			const Map &map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/**
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle& particle, const std::vector<int>& associations,
		                     const std::vector<double>& sense_x, const std::vector<double>& sense_y);
	
	std::string getAssociations(Particle best);
	
	std::string getSenseX(Particle best);
	
	std::string getSenseY(Particle best);

	/**
	* initialized Returns whether particle filter is initialized yet or not.
	*/
	const bool initialized() const {
		return is_initialized;
	}

private:

	// define constants
	const int NUM_PARTICLES = 100;
	const double INIT_WEIGHT = 1;
	const double ZERO_DETECTION = 1e-6;
	double ZERO_MEAN [3] = {0, 0, 0};
	double INFINITE_DISTANCE = std::numeric_limits<double>::max();
	const bool bDISPLAY = true;
	const bool bDISPLAY_init = true;
	const bool bDISPLAY_prediction = true;
	const bool bDISPLAY_dataAssociation = true;
	const bool bDISPLAY_updateWeights = true;
	const bool bDISPLAY_resample = true;
	const bool bDISPLAY_SetAssociations = true;
	const bool bDISPLAY_addNoise = true;
	const bool bDISPLAY_getMapLandmark = true;
	const bool bDISPLAY_transformVehicle2Map = true;
	
	/**
	 * addNoise Adds noise to single particle
	 * @param particle Particle to which noise should be added
	 * @param mean[] Array of dimension 3 [mean value of x, y, yaw]
	 * @param std[] Array of dimension 3 [standard deviation of x, y, yaw]
	 */
	void addNoise(Particle& particle, double mean[], double std[]);

		/**
	 * getMapLandmark Get landmark object from map object
	 * @param num_landmark Number of the landmark
	 * @param map_landmarks Object containing array of landmark elements [landmark_list] as array of dimension 3 [id_i, x_f, y_f]
	 */
	LandmarkObs getMapLandmark(unsigned int num_landmark, Map map_landmarks);

		/**
	 * transformVehicle2Map Transform observation to particle in map coordinates
	 * @param x_offset_map Particle location in map coordinates (x direction)
	 * @param y_offset_map Particle location in map coordinates (y direction)
	 * @param x_change_relative Observation relative to vehicle coordinates (x direction)
	 * @param y_change_relative Observation relative to vehicle coordinates (y direction)
	 * @param alpha Particle angle in map coordinates
	 * @param x_map Transformed observation in map coordinates (x direction)
	 * @param y_map Transformed observation in map coordinates (y direction)
	 */
	void transformVehicle2Map(double x_offset_map, double y_offset_map, double x_change_relative, double y_change_relative, double alpha, double& x_map, double& y_map);

		/**
	 * printParticles Print information about all particles
	 * @param particles List of particle objects
	 */
	void printParticles(std::vector<Particle> particles);

};

#endif /* PARTICLE_FILTER_H_ */
