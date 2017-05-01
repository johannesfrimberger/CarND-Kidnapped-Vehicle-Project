/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */


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
    
    // Set default value for weight
    const double default_weight = 1.;
    
    // Clear vectors used to store particles and weights
    particles.clear();
    weights.clear();
    
    for(unsigned i = 0U; i < num_particles; i++)
    {
        // Set-up particle
        Particle p;
        
        p.id = i;
        p.weight = default_weight;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_psi(gen);
        
        particles.push_back(p);
        weights.push_back(default_weight);
    }
    
    // Set flag for initialization
    is_initialized = true;
}

Particle predictParticle(const Particle& p_in, const double velocity, const double yaw_rate, const double dt)
{
    // Copy particle
    Particle p_out(p_in);
    
    // Handle special case when yaw_rate is near zero (avoid division by zero)
    if(abs(yaw_rate) > 1e-5)
    {
        // Pre-calculate constants
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
        // Pre-calculate constants
        const double velocity_dt = velocity * dt;
        
        // Update x and y position. Theta remains the same
        p_out.x += velocity_dt * sin(p_in.theta);
        p_out.y += velocity_dt * cos(p_in.theta);
    }
    
    return p_out;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // Create normal distributions for x, y and psi.
    const double std_x = std_pos[0];
    const double std_y = std_pos[1];
    const double std_psi = std_pos[2];
    
    for(Particle& p : particles)
    {
        // Predict position for every particle
        const Particle predicted_p = predictParticle(p, velocity, yaw_rate, delta_t);
        
        // Calculate normal distribution around predicted position (ToDo: setup distribution once before loop)
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
    // Iterate over all observations and associate closest prediction
    for (LandmarkObs& observation : observations)
    {
        // Initialize distance with infinity
        double min_d = INFINITY;
        
        // Iterate over all predicted positions
        for(const LandmarkObs& pred : predicted)
        {
            double d = dist(observation.x, observation.y, pred.x, pred.y);
            if (min_d > d)
            {
                // Calulcate index from difference in
                observation.id = &pred - &predicted[0];
                min_d = d;
            }
        }
    }
    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations, Map map_landmarks)
{
    // Clear weights vector
    weights.clear();
    
    for (Particle& particle : particles)
    {
        // Create vector of landmarks in particles sensor range
        std::vector<LandmarkObs> predicted_landmarks;
        for(const Map::single_landmark_s landmark : map_landmarks.landmark_list)
        {
            const double delta_x = (landmark.x_f - particle.x);
            const double delta_y = (landmark.y_f - particle.y);
            
            const double d = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
            
            if (d <= sensor_range)
            {
                const double new_x = delta_x * cos(particle.theta) + delta_y * sin(particle.theta);
                const double new_y = delta_y * cos(particle.theta) - delta_x * sin(particle.theta);
                
                LandmarkObs l = {landmark.id_i, new_x, new_y};
                predicted_landmarks.push_back(l);
            }
        }

        // Run data association (Nearest neighbor)
        dataAssociation(predicted_landmarks, observations);

        // Update
        double w = 1;
        for(LandmarkObs obs : observations)
        {
            const LandmarkObs& predicted_obs = predicted_landmarks[obs.id];
            
            const double delta_x = obs.x - predicted_obs.x;
            const double delta_y = obs.y - predicted_obs.y;
            
            const double exp1 = exp(-0.5 * (pow(delta_x, 2.0) * std_landmark[0] + pow(delta_y, 2.0) * std_landmark[1]));
            const double exp2 = sqrt(2.0 * M_PI * std_landmark[0] * std_landmark[1]);
            w *= exp1 / exp2;
        }
        
        // Update weight of this particle and in weights vector
        particle.weight = w;
        weights.push_back(w);
    }
}

void ParticleFilter::resample()
{
    // Create discrete distribution
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
    
    for (int i = 0; i < num_particles; ++i)
    {
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    dataFile.close();
}
