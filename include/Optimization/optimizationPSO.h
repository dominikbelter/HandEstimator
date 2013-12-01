/** @file optimizationPSO.h
 *
 * implementation - OptimizationPSO
 *
 */

#ifndef OPTIMIZATIONPSO_H_INCLUDED
#define OPTIMIZATIONPSO_H_INCLUDED

#include "optimization.h"
#include <iostream>
#include <memory>

namespace handest {
	/// create a single optimization
	Optimization* createOptimization(void);
};

using namespace handest;

	
/// dimension of the problem
const int MAX_INPUTS = 10; 
/// numbers of particles
const int MAX_PARTICLES = 500;
/// maximum velocity allowed
const float_t V_MAX = 1.5 ; 
/// number of algorithm iterations
const int MAX_EPOCHS = 1000;
/// range of the initial positions 
const float_t START_RANGE_MIN_POS = -100.0;
const float_t START_RANGE_MAX_POS = 100.0;
/// range of the initial velocities 
const float_t START_RANGE_MIN_VEL = -5.0;
const float_t START_RANGE_MAX_VEL = 5.0;
/// PSO equations parameters
const int INERTIA = 1;
const int C1 = 2;
const int C2 = 2;



class OptimizationPSO : public Optimization {
	
	public:
	/// Pointer
	typedef std::unique_ptr<OptimizationPSO> Ptr;

	/// Construction
	OptimizationPSO(void);

	/// Optimization with Particle Swarm Optimization Algorithm
	virtual void Optimize(Hand::Pose& hand, Point3D::Cloud& cloud);

	/// Save data to file
	virtual void SaveToFile(Hand::Pose& hand);


	private:
		
	class cParticle {

private:
    /// value of each optimized variable
    float_t Position[MAX_INPUTS];
    /// the optimum value of function for particle
    float_t PersBest;
    /// value of particle velocity
    float_t Velocity[MAX_INPUTS];
    float_t BestPosition[MAX_INPUTS];

public:
    cParticle();
    /// return particle coordiantes
    float_t getPosition(int index) const;
    /// set new particle coordiantes
	void  setPosition(int index, float_t value);
    /// return particle optimum
    float_t getPersBest() const;
    /// set particle optimum
	void  setPersBest(float_t value);
    /// return particle velocity vector
    float_t getVelocity(int index) const;
    /// set particle velocity vector
	void  setVelocity(int index, float_t value);
    /// set particle best coordinates
	void  setBestPosition(int index, float_t value);
    /// return particle best coordinates
	float_t getBestPosition(int index) const;

};

	cParticle particles[MAX_PARTICLES];

	/// maim algorithm loop
	void PsoAlgorithm();
	/// update velocity vector according to Kennedy&Eberhart PSO equations
	void UpdateVelocity(int gBestIndex);
	/// update position vector according to Kennedy&Eberhart PSO equations
	void UpdatePositions(int gBestIndex);
	/// initialize particle
	void InitializeParticles();
	/// return value of problem function 
	float_t GetFunctionValue(int index);
	/// return index of particle with current optimum
	int GetMinimum();
		/// get random numbers for PSO equations
	float_t GetRand();
	/// get random numbers from given range for initializing particles
	float_t GetRandomNumber(float_t LowBound, float_t UpBound);
};

#endif 
