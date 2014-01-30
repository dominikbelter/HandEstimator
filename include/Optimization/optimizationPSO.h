/** @file optimizationPSO.h
 *
 * implementation - OptimizationPSO
 *
 */

#ifndef OPTIMIZATIONPSO_H_INCLUDED
#define OPTIMIZATIONPSO_H_INCLUDED


#include "optimization.h"
#include "../include/OptimizationFunction/optimizationFunction.h"
#include "OptimizationFunction/optimizationFunctionPF.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerPCL.h"
#include "Kinematic/kinematic_liego.h"
#include "../dependencies/Eigen/Eigen"
#include <string>
#include <fstream>


#include <memory>

#define V_MAX 1.5
#define START_RANGE_MIN_VEL -5.0
#define START_RANGE_MAX_VEL 5.0
#define MAX_CONFIG_VALUE 3.14
#define MIN_CONFIG_VALUE -3.14
#define ROT_MAT_ELEMENTS 9

namespace handest {
	/// create a single optimization
	Optimization* createOptimizationPSO(void);
};

using namespace handest;
using namespace std;


//typedef handest::float_type float_t; //DB usunalem ten wiersz ze wzgledu na konflikt z cmath.h, zamiast tego zamienilem wszystkie float_t (ktore dotyczy math.h) na float_type (nasza definicja)


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

///DB wszystko ponizej powinno byc prywatne

	/// PSO defooult parameters

	/// dimension of the problem: Hand::JOINTS + wrist coordinates 
	/// xyz + orentation (euler angles)
	///static const int WRIST_COORDINATES 9;
	///static const int DIM = Hand::JOINTS + WRIST_COORDINATES; 
	static const int DIM = 32;

	/// numbers of particles
    static const int MAX_PARTICLES = 500;
	/// maximum velocity allowed
    //static const float_type V_MAX;
	/// number of algorithm iterations
    static const int MAX_EPOCHS = 100;
	/// range of the initial positions 
    static constexpr float_type START_RANGE_MIN_POS = -1.0;
    static constexpr float_type START_RANGE_MAX_POS = 1.0;
	/// range of the initial velocities 
	//static const float_t START_RANGE_MIN_VEL;
	//static const float_t START_RANGE_MAX_VEL;
	/// PSO equations parameters
	static const int INERTIA = 1;
	static const int C1 = 2;
	static const int C2 = 2;
	
	/// optimize variables 
	Hand::Pose handPSO; 
	Hand::Pose handPSODefault;
	Point3D::Cloud cloudPSO;

	enum finger {THUMB, INDEX, MIDDLE, RING, PINKY};
	

	private:
		
	class cParticle {

private:
    /// value of each optimized variable
    float_type Position[DIM];
    /// the optimum value of function for particle
    float_type PersBest;
    /// value of particle velocity
    float_type Velocity[DIM];
    float_type BestPosition[DIM];

public:
   
    /// return particle coordiantes
    float_type getPosition(int index) const
	{
		return this->Position[index];
	}
    /// set new particle coordiantes
    void  setPosition(int index, float_type value)
	{
		this->Position[index] = value;
	}
    /// return particle optimum
    float_type getPersBest() const
	{
		return this->PersBest;
	}
    /// set particle optimum
    void  setPersBest(float_type value)
	{
		this->PersBest = value;
	}
    /// return particle velocity vector
    float_type getVelocity(int index) const
	{
		return this->Velocity[index];
	}
    /// set particle velocity vector
    void  setVelocity(int index, float_type value)
	{
		this->Velocity[index] = value;
	}
    /// set particle best coordinates
    void  setBestPosition(int index, float_type value)
	{
		this->BestPosition[index] = value;
	}
    /// return particle best coordinates
    float_type getBestPosition(int index) const
	{
		return this->BestPosition[index];
	}

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
    float_type GetFunctionValue(int index);
	/// return index of particle with current optimum
	int GetMinimum();
		/// get random numbers for PSO equations
    float_type GetRand();
	/// get random numbers from given range for initializing particles
    float_type GetRandomNumber(float_type LowBound, float_type UpBound);

	Mat34 eigen_2_mat34(const Eigen::Matrix4f &trans);
	Eigen::Matrix4f mat34_2_eigen(const Mat34 &trans);
	Eigen::Vector4f vec3_2_eigen(Vec3 vec);
	Vec3 eigen_2_vec3(Eigen::Vector4f vec);
};



#endif 
