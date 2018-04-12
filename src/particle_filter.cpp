/*
 * particle_filter.cpp
 *
 *  Created on: April 12, 2018
 *      Author: Atul Singh
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
/***************************************************************
 * Set the number of particles. Initialize all particles to first position
 * (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
 * random gaussian noise is added to each particle
 ***************************************************************/
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	num_particles = 100;
	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	for (int i = 0; i < num_particles; i++)
	{
		Particle p;
		p.id = i ;
		p.x = dist_x(gen);
		p.y = dist_y(gen); 
		p.theta = dist_theta(gen);
		p.weight = 1.0 ;
		
		particles.push_back(p);
		weights.push_back(p.weight);
	}
	
	is_initialized=true;
}

/***************************************************************
 *  Add measurements to each particle and add random Gaussian noise.
 ***************************************************************/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
		
	default_random_engine gen;
	
	for(int i = 0 ; i < num_particles ; i++ )
	{
		double pred_x;
		double pred_y;
		double pred_theta;
		if(fabs(yaw_rate)< 0.0001){
		pred_x = particles[i].x + velocity*cos(particles[i].theta)*delta_t;
		pred_y = particles[i].y + velocity*sin(particles[i].theta)*delta_t;
		pred_theta =particles[i].theta;
		
		}
		else{
		pred_x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + (yaw_rate*delta_t)) - sin(particles[i].theta));
		pred_y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate*delta_t)));
		pred_theta = particles[i].theta + yaw_rate*delta_t;
		
		
		
		}
		
		
		
		normal_distribution<double> dist_x(pred_x, std_pos[0]);
		normal_distribution<double> dist_y(pred_y, std_pos[1]);
		normal_distribution<double> dist_theta(pred_theta, std_pos[2]);
		
		
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		
		
		
		
	}
	

}

/**************************************************************
 * Find the predicted measurement that is closest to each observed measurement
 * and assign the observed measurement to this particular landmark.
 ***************************************************************/
 
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

	for(int i = 0 ; i<observations.size();i++){
		double minimum = 99.9e+100;
		unsigned int min_id = -1 ;
		for (int j = 0 ; j<predicted.size() ; j++ ){
			double d = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y);//sqrt(x_diff*x_diff - y_diff*y_diff);
			if(d < minimum )
			{
				minimum = d;
				min_id = predicted[j].id;
			}
		}
		observations[i].id = min_id;
		
	}
	  

}


/***************************************************************
 *  Update the weights of each particle using a mult-variate Gaussian distribution.
 *  NOTE: The observations are given in the VEHICLE'S coordinate system. Particles are located
 *        according to the MAP'S coordinate system. So transformation is done.
 * For each particle:
 *   1. transform observations from vehicle to map coordinates assuming it's the particle observing
 *   2. find landmarks within the particle's range
 *   3. find which landmark is likely being observed based on `nearest neighbor` method
 *   4. determine the weights based on the difference particle's observation and actual observation
 ***************************************************************/
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
	
	double weight_normalizer = 0.0;
  
	for ( int i = 0 ; i < num_particles ; i++)
	{
		double px = particles[i].x;
		double py = particles[i].y;
		double ptheta = particles[i].theta;
		
		vector<LandmarkObs> map_observations;
		
		/**************************************************************
		* STEP 1:
		* transform each observations to map coordinates
		* assume observations are made in the particle's perspective
		**************************************************************/
		for( int j = 0; j<observations.size() ; j++)
		{
			double ox = observations[j].x;
			double oy = observations[j].y;
			int oid = observations[j].id;
			
			double transformed_x = px + ( cos(ptheta)*ox ) - ( sin(ptheta) * oy ) ;
			double transformed_y = py + ( sin(ptheta)*ox ) + ( cos(ptheta) * oy ) ;
			
			
			
			map_observations.push_back(LandmarkObs {oid,transformed_x,transformed_y});
			
		}
		
		vector<LandmarkObs> landmarks_in_range;
		/**************************************************************
		* STEP 2:
		* Find map landmarks within the sensor range
		**************************************************************/
		
		for(int j = 0 ; j< map_landmarks.landmark_list.size() ; j++)
		{
			int mid =  map_landmarks.landmark_list[j].id_i;
			double mx =  map_landmarks.landmark_list[j].x_f;
			double my =  map_landmarks.landmark_list[j].y_f;
			
			double dx = fabs((px - mx));
			double dy = fabs((py - my));
			
			
			if(dx <= sensor_range && dy <= sensor_range)
			{
				
				landmarks_in_range.push_back(LandmarkObs {mid,mx,my});
			}
			
		}
		/**************************************************************
	   * STEP 3:
	   * Associate landmark in range (id) to landmark observations
	   * this function modifies std::vector<LandmarkObs> observations
	   * NOTE: - all landmarks are in map coordinates
	   *       - all observations are in map coordinates
	   **************************************************************/
		dataAssociation(landmarks_in_range,map_observations);
		/**************************************************************
		* STEP 4:
		* Compare each observation (by actual vehicle) to corresponding
		* observation by the particle (landmark_in_range)
		* update the particle weight based on this
		**************************************************************/
		particles[i].weight = 1.0;
		
		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];
		double sigma_x_2 = pow(sigma_x, 2);
		double sigma_y_2 = pow(sigma_y, 2);
		double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));
		
		
		for(int j = 0; j < map_observations.size(); j++)
		{
			double ox = map_observations[j].x;
			double oy = map_observations[j].y;
			int oid = map_observations[j].id;
			
			double multi_prob = 1.0; 
			
			for(int k = 0 ; k < landmarks_in_range.size() ; k++)
			{
				double px = landmarks_in_range[k].x;
				double py = landmarks_in_range[k].y;
				int pid = landmarks_in_range[k].id;
				
				if(oid == pid )
				{
					
					multi_prob = normalizer * exp(-1.0 * ((pow((ox - px), 2)/(2.0 * sigma_x_2)) + (pow((oy - py), 2)/(2.0 * sigma_y_2))));
					
					
					particles[i].weight *= multi_prob;	
				}
				
			}
			
			
			
		}
		weight_normalizer+=particles[i].weight;
		
	}
	
	for (int i = 0; i < particles.size(); i++) {
    particles[i].weight /= weight_normalizer;
    weights[i] = particles[i].weight;
  }
	
	
}
/**************************************************************
 * Resample particles with replacement with probability proportional to their weight.
 ***************************************************************/
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> resampled_particles;

	default_random_engine gen;
	
	uniform_int_distribution<int> particle_index(0, num_particles - 1);
	
	int current_index = particle_index(gen);
	
	double beta = 0.0;
	
	double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
	
	for (int i = 0; i < particles.size(); i++) {
		uniform_real_distribution<double> random_weight(0.0, max_weight_2);
		beta += random_weight(gen);

	  while (beta > weights[current_index]) {
	    beta -= weights[current_index];
	    current_index = (current_index + 1) % num_particles;
	  }
	  resampled_particles.push_back(particles[current_index]);
	}
	particles = resampled_particles;
	
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
