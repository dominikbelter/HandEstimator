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
	
	cloudPSO = cloud;
	handPSO = hand;
	/// perfrom PSO
	PsoAlgorithm();
	/// return hand after optimization. colud is unchanged. 
	hand = handPSO;
}

void OptimizationPSO::SaveToFile(Hand::Pose& hand)
{
    ///save to file
	//fstream plik;
	//plik.open("C:\\HandFile.txt", ios::out);
	
	//plik.close();
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
	float_t InitVel;
	//initialize particle positions
	for (int index = 0; index < MAX_PARTICLES; index++)
    {
    
	for (int i = 0; i < Hand::JOINTS ; i++)
		{
			particles[index].setPosition(i, handPSO.config.conf[i]);
			particles[index].setBestPosition(i, handPSO.config.conf[i]);	
	}

	for ( int i = 0; i < 3 ; i++)
	{
		particles[index].setPosition(Hand::JOINTS + i, handPSO.pose.p.v[i]);
		particles[index].setBestPosition(Hand::JOINTS + i, handPSO.pose.p.v[i]);
	}


	for ( int  i = 0 ; i < 3 ; i++ )
		for (int j = 0; j < 3; j++ )
		{
			particles[index].setPosition(Hand::JOINTS + 3 + i + j, handPSO.pose.R.m[i][j]);
		    particles[index].setBestPosition(Hand::JOINTS + 3 + i + j, handPSO.pose.R.m[i][j]);
		}
	}
	
	
	for (int i = 0; i < MAX_PARTICLES; i++)
    {
        InitBestValue = 0;
        for (int j = 0; j < DIM; j++)
        {
            //InitPos = GetRandomNumber(START_RANGE_MIN_POS, START_RANGE_MAX_POS);
			//particles[i].setPosition(j, InitPos);
            InitVel = GetRandomNumber(START_RANGE_MIN_VEL, START_RANGE_MAX_VEL);
            particles[i].setVelocity(j,InitVel);
            //particles[i].setBestPosition(j, InitPos);
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
    //float_t CurrentPos;

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

	for (int i = 0; i < Hand::JOINTS ; i++)
		
		/// set values of configuration
		handPSO.config.conf[i] = particles[index].getPosition(i);
		
    /// set value of xyz
	for ( int i = 0; i < 3 ; i++)
		handPSO.pose.p.v[i] = particles[index].getPosition(Hand::JOINTS + i);

	for ( int  i = 0 ; i < 3 ; i++ )
		for (int j = 0; j < 3; j++ )

			///set rotation
			handPSO.pose.R.m[i][j] = particles[index].getPosition(Hand::JOINTS + 3 + i + j);

	optimizationFunction * optimization_function = createOptimizationFunctionPF();

	result = optimization_function->FitnessValue(handPSO,cloudPSO);
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