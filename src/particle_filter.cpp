/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

#define gNUM_PARTICLES 400;
using std::string;
using std::vector;

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/**
	 * TODO: Set the number of particles. Initialize all particles to 
	 *   first position (based on estimates of x, y, theta and their uncertainties
	 *   from GPS) and all weights to 1. 
	 * TODO: Add random Gaussian noise to each particle.
	 * NOTE: Consult particle_filter.h for more information about this method 
	 *   (and others in this file).
	 */
	m_numParticles = gNUM_PARTICLES;  // TODO: Set the number of particles
	default_random_engine gen;

	//Set up our Normal Distributions for Gaussian Noise
	normal_distribution<double> Nx(x, std[0]);
	normal_distribution<double> Ny(y,std[1]);
	normal_distribution<double> Ntheta(theta,std[2]);

	// Create all the particles.
	for (int i = 0; i < m_numParticles; i++)
	{
		// Initialize the particle struct
		Particle p;
		// Init all the variables of the particle.
		p.id = i;
		p.x = Nx(gen);
		p.y = Ny(gen);
		p.theta = Ntheta(gen);
		p.weight = 1; // Starting weight of the particles is 1

		// Add the newly created particle to the particle and weights vector.
		m_particles.push_back(p);
		m_weights.push_back(p.weight);
	}
	//set initialized to true, so that isInitialized() returns true now;
	m_isInitialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
																double velocity, double yaw_rate) {
	/**
	 * TODO: Add measurements to each particle and add random Gaussian noise.
	 * NOTE: When adding noise you may find std::normal_distribution 
	 *   and std::default_random_engine useful.
	 *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	 *  http://www.cplusplus.com/reference/random/default_random_engine/
	 */
	default_random_engine gen;
	// For each Particle, calculate the new x and y based on what we know about velocity and yaw rate
	// Calculate these using the formulas provided, and then add Gaussian noise to this.
	for(int i = 0; i < m_numParticles; i ++)
	{
		Particle p = m_particles[i]; // this is inefficient memory wise, but it makes the math below look prettier.
		double x, y, theta;
		
		// Calcualte the new position and angle
		if (yaw_rate == 0)
		{
			x = p.x + velocity * delta_t * cos(p.theta);
			y = p.y + velocity * delta_t * sin(p.theta);
			theta = p.theta;
		}
		else
		{
			x = p.x + velocity/yaw_rate * (sin(p.theta + (yaw_rate * delta_t)) - sin(p.theta));
			y = p.y + velocity/yaw_rate * (cos(p.theta) - cos(p.theta + (yaw_rate * delta_t)));
			theta = p.theta + (yaw_rate * delta_t);
		}

		// Add the Gaussian noise
		normal_distribution<double> Nx(x, std_pos[0]);
		normal_distribution<double> Ny(y, std_pos[1]);
		normal_distribution<double> Ntheta(theta, std_pos[2]);

		// Put these calculated numbers back into the particles vector
		m_particles[i].x = Nx(gen);
		m_particles[i].y = Ny(gen);
		m_particles[i].theta = Ntheta(gen);

	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
																		 vector<LandmarkObs>& observations) {
	/**
	 * TODO: Find the predicted measurement that is closest to each 
	 *   observed measurement and assign the observed measurement to this 
	 *   particular landmark.
	 * NOTE: this method will NOT be called by the grading code. But you will 
	 *   probably find it useful to implement this method and use it as a helper 
	 *   during the updateWeights phase.
	 */
	 return;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
																	 const vector<LandmarkObs> &observations, 
																	 const Map &map_landmarks) {
	/**
	 * TODO: Update the weights of each particle using a mult-variate Gaussian 
	 *   distribution. You can read more about this distribution here: 
	 *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	 * NOTE: The observations are given in the VEHICLE'S coordinate system. 
	 *   Your particles are located according to the MAP'S coordinate system. 
	 *   You will need to transform between the two systems. Keep in mind that
	 *   this transformation requires both rotation AND translation (but no scaling).
	 *   The following is a good resource for the theory:
	 *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	 *   and the following is a good resource for the actual equation to implement
	 *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
	 */

	//for each particle
	for (int i = 0; i < m_numParticles; ++i)
	{
		//reset the particles weight
		m_particles[i].weight = 1.0; 
		//start by converting the observations from vehicle to map coordinates
		for (int j = 0; j < observations.size(); ++j)
		{
			LandmarkObs transformedObservation;
			LandmarkObs observation = observations[j];
			// preform the space transformation from vehicle to map
			// This is derived from matix multiplication(http://planning.cs.uiuc.edu/node99.html)
			transformedObservation.x = m_particles[i].x + (observation.x * cos(m_particles[i].theta)) - (sin(m_particles[i].theta) * observation.y);
			transformedObservation.y = m_particles[i].y + (observation.x * sin(m_particles[i].theta)) + (observation.y * cos(m_particles[i].theta));

			//figure out which landmarks are in the sensor range, so we know which ones to care about
			vector<double> landmarkDists(map_landmarks.landmark_list.size());
			for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
			{
				double landmarkDistance = dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, m_particles[k].x, m_particles[k].y);
				if (landmarkDistance <= sensor_range)
				{
					landmarkDists[k] = dist(map_landmarks.landmark_list[k].x_f,  map_landmarks.landmark_list[k].y_f, transformedObservation.x, transformedObservation.y); 
				}
				else
				{
					//if the distance isn't in the sensor distance, set it to infinity so we ignore it completely(There is no nearest neighbor)
					landmarkDists[k] = std::numeric_limits<double>::infinity();
				}
			}
			// Get the nearest neighbor, by finding the minimum distance in the landmarkDists vector.
			// We can more easily do this by using the STL method distance() and min_element, which will allow
			// us to get the index of the smallest element in the array.  Once we have that, we can  
			// use that to get the landmarks x and y coordinates, since the positions in the landmark vector and the distance vectors
			// should map to the same indices.
			int nnPosition = distance(landmarkDists.begin(),min_element(landmarkDists.begin(), landmarkDists.end()));
			//From here, get the gaussian multivariate.
			m_particles[i].weight *= multiv_prob(std_landmark[0], std_landmark[1], transformedObservation.x, transformedObservation.y, map_landmarks.landmark_list[nnPosition].x_f, map_landmarks.landmark_list[nnPosition].y_f);
		}
			//update the weights 
			m_weights[i] = m_particles[i].weight;
		}
}

void ParticleFilter::resample() {
	/**
	 * TODO: Resample particles with replacement with probability proportional 
	 *   to their weight. 
	 * NOTE: You may find std::discrete_distribution helpful here.
	 *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	 */
	 default_random_engine gen;
	 discrete_distribution<int> distribution(m_weights.begin(), m_weights.end());
	 vector<Particle> resample_particles;
	 for (int i=0; i < m_numParticles; i++)
	 {
		 resample_particles.push_back(m_particles[distribution(gen)]);
	 }
	 m_particles = resample_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
																		 const vector<int>& associations, 
																		 const vector<double>& sense_x, 
																		 const vector<double>& sense_y) {
	// particle: the particle to which assign each listed association, 
	//   and association's (x,y) world coordinates mapping
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
	vector<double> v;

	if (coord == "X") {
		v = best.sense_x;
	} else {
		v = best.sense_y;
	}

	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}

// Following Function Copied from Udacity Classroom Particle Weights Solution
double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
									 double mu_x, double mu_y) {
	// calculate normalization term
	double gauss_norm;
	gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

	// calculate exponent
	double exponent;
	exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
							 + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
		
	// calculate weight using normalization terms and exponent
	double weight;
	weight = gauss_norm * exp(-exponent);
		
	return weight;
}

// a wrapper around the distance formula
double ParticleFilter::dist(double x1, double y1, double x2, double y2) 
{
 return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}