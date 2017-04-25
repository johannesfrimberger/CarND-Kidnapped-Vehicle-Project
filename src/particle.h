#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <iostream>

class Particle
{
public:
	unsigned id;
	double x;
	double y;
	double theta;
	double weight;

    /** Print current status */
    friend std::ostream& operator<<(std::ostream& os, const Particle& p)
    {
        os << "Id: " << p.id;
        os << " PosX: " << p.x;
        os << " PosY: " << p.y;
        os << " Theta: " << p.theta;
        os << " Weight: " << p.weight;
        return os;
    }
};

#endif /* PARTICLE_H_ */
