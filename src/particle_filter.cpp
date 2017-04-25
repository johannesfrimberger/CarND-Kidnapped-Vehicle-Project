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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // Set number of particles
    num_particles = 1000U;
    
    // Create normal distributions for around x, y and psi.
    const double std_x = std[0];
    const double std_y = std[1];
    const double std_psi = std[2];
    
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_psi(theta, std_psi);
    
    // Create random number generator
    default_random_engine gen;
    
    particles.clear();
    
    // Set default value for weight
    const double default_weight = 1.;
    
    for(unsigned i = 0U; i < num_particles; i++)
    {
        Particle p;
        
        p.id = i;
        p.weight = default_weight;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_psi(gen);
        
        particles.push_back(p);
    }
    
    // Set flag for initialization
    is_initialized = true;
}

Particle predictParticle(const Particle& p_in, const double velocity, const double yaw_rate, const double dt)
{
    Particle p_out(p_in);
    
    // Handle special case when yaw_rate is near zero (avoid division by zero)
    if(abs(yaw_rate) > 1e-5)
    {
        // Pre-calculate
        const double velocity_by_yaw_rate = velocity/yaw_rate;
        const double yaw_rate_dt = yaw_rate * dt;
        const double theta_plus_yaw_rate_dt = p_in.theta + yaw_rate_dt;
        
        // Update x, y and theta
        p_out.x +=  velocity_by_yaw_rate * (sin(theta_plus_yaw_rate_dt) - sin(p_in.theta));
        p_out.y += velocity_by_yaw_rate * (cos(p_in.theta) - cos(theta_plus_yaw_rate_dt));
        p_out.theta += yaw_rate_dt;
        
    }
    else
    {
        // Pre-calculate
        const double velocity_dt = velocity*dt;
        
        // Update x and y position. Theta remains the same
        p_out.x += velocity_dt * cos(p_in.theta);
        p_out.y += velocity_dt * sin(p_in.theta);
    }
    
    return p_out;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // Create normal distributions for x, y and psi.
    const double std_x = std_pos[0];
    const double std_y = std_pos[1];
    const double std_psi = std_pos[2];
    
    // Create random number generator
    default_random_engine gen;
    
    for(Particle& p : particles)
    {
        const Particle predicted_p = predictParticle(p, velocity, yaw_rate, delta_t);
        
        normal_distribution<double> dist_x(predicted_p.x, std_x);
        normal_distribution<double> dist_y(predicted_p.y, std_y);
        normal_distribution<double> dist_psi(predicted_p.theta, std_psi);
        
        // Update position but do not change id or weight
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_psi(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks)
{
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html
    
    for (Particle& p : particles)
    {
    }
}

void ParticleFilter::resample()
{
    // Create discrete distribution
    std::default_random_engine gen;
    std::vector<double> weights;
    std::discrete_distribution<int> distribution {weights.begin(), weights.end()};
    
    // Sample new particles from discrete distribution
    std::vector<Particle> new_particles;
    for (int i = 0; i < num_particles; i++)
    {
        new_particles.push_back(particles[distribution(gen)]);
    }
    
    // Copy new particles to internal storage
    particles = new_particles;
}

void ParticleFilter::write(std::string filename)
{
    // You don't need to modify this file.
    std::ofstream dataFile;
    dataFile.open(filename, std::ios::app);
    
    for (int i = 0; i < num_particles; ++i) {
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    dataFile.close();
}
