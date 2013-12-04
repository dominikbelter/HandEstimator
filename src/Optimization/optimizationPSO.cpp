#include "../include/Optimization/optimizationPSO.h"

using namespace handest;

OptimizationPSO::OptimizationPSO(void)
{
	/// initializaing floating points PSO parameters
	
	const float_t V_MAX = 1.5; 
	/// number of algorithm iterations
	const int MAX_EPOCHS = 1000;
	/// range of the initial positions 
	const float_t START_RANGE_MIN_POS = -100.0;
	const float_t START_RANGE_MAX_POS = 100.0;
	/// range of the initial velocities 
	const float_t START_RANGE_MIN_VEL = -5.0;
	const float_t START_RANGE_MAX_VEL = 5.0;	
}


void OptimizationPSO::Optimize(Hand::Pose& hand, Point3D::Cloud& cloud)
{
	
}

void OptimizationPSO::SaveToFile(Hand::Pose& hand)
{
	
}



void OptimizationPSO::PsoAlgorithm()
{
    int GlobalBest = 0;
    int Epochs = 0;

	InitializeParticles();

    
        while (Epochs < MAX_EPOCHS)
        {
            GlobalBest = GetMinimum();
            UpdateVelocity(GlobalBest);
            UpdatePositions(GlobalBest);

            Epochs += 1;
        }
       
}

void OptimizationPSO::InitializeParticles()
{
    float_t InitBestValue;
    float_t InitPos;

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        InitBestValue = 0;
        for (int j = 0; j < DIM; j++)
        {
            particles[i].setPosition(j, GetRandomNumber(START_RANGE_MIN_POS, START_RANGE_MAX_POS));
            InitPos = GetRandomNumber(START_RANGE_MIN_VEL, START_RANGE_MAX_VEL);
            particles[i].setVelocity(j,InitPos);
            particles[i].setBestPosition(j, InitPos);
        }

        InitBestValue = GetFunctionValue(i);
        particles[i].setPersBest(InitBestValue);
    }

    return;
}



int OptimizationPSO::GetMinimum()
{
    int bestParticle = 0;

        for (int i = 1; i < MAX_PARTICLES ; i++)
        {
            if (particles[i].getPersBest() < particles[bestParticle].getPersBest())

                        bestParticle = i;

        }

return bestParticle;
}

void OptimizationPSO::UpdateVelocity(int gBestIndex)
{
    float_t NewVelocity[DIM];
    float_t CurrentPos;

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        for (int j = 0; j < DIM; j++)
        {
            NewVelocity[j] = INERTIA*(particles[i].getVelocity(j) + C1*GetRand()*(particles[i].getBestPosition(j) - 
				particles[i].getPosition(j))+C2*GetRand()*(particles[gBestIndex].getBestPosition(j) - particles[i].getPosition(j)));

        /// check velocity constraints
		if (NewVelocity[j] > V_MAX)
            particles[i].setVelocity(j,V_MAX);
        else if (NewVelocity[j] < -V_MAX)
            particles[i].setVelocity(j,-V_MAX);
        else
            particles[i].setVelocity(j, NewVelocity[j]);
        }

    }
}

void OptimizationPSO::UpdatePositions(int gBestIndex)
{
    float_t  NewFunctValue, tempData;

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        for (int j = 0; j < DIM; j++)
        {
            if (i != gBestIndex)
            {
                tempData = particles[i].getPosition(j);
                particles[i].setPosition(j, tempData + (particles[i].getVelocity(j)));
            }
        }

        NewFunctValue = GetFunctionValue(i);

		/// update personal best value if neccesary
        if (NewFunctValue < particles[i].getPersBest())
        {
            particles[i].setPersBest(NewFunctValue);
        }

    }
}

float_t OptimizationPSO::GetFunctionValue(int index)
{
    float_t result;

    return result;
}

float_t OptimizationPSO::GetRandomNumber(float_t LowBound, float_t UpBound)
{
    float_t temp;
    temp = (float_t)LowBound + float_t(((UpBound-LowBound))*rand()/(RAND_MAX + 1.0));

    return temp;
}

float_t OptimizationPSO::GetRand()
{
    float_t temp;
    temp = float_t(rand()/(RAND_MAX + 1.0));
    return temp;
}