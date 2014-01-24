#include "../include/Optimization/optimizationPSO.h"


using namespace handest;


/// A single instance of Optimization PSO
OptimizationPSO::Ptr optPSO;

OptimizationPSO::OptimizationPSO(void)
{
	/// initializaing floating points PSO parameters
	
    //const float_type V_MAX = 1.5;
	/// number of algorithm iterations
    //const int MAX_EPOCHS = 1000;//DB usuniete -- redefinicja
	/// range of the initial positions 
    //const float_type START_RANGE_MIN_POS = -5.0;//DB usuniete -- redefinicja
    //const float_type START_RANGE_MAX_POS = 5.0; //DB usuniete -- redefinicja
	/// range of the initial velocities 
    //const float_type START_RANGE_MIN_VEL = -5.0;
    //const float_type START_RANGE_MAX_VEL = 5.0;
	
}


void OptimizationPSO::Optimize(Hand::Pose& hand, Point3D::Cloud& cloud)
{

    pcl::PointXYZ pointPCL;
    //from
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPCL (new pcl::PointCloud<pcl::PointXYZ>);

    for(size_t i=0;i<cloud.size();i++)
    {
        pointPCL.x = cloud[i].position.x;
        pointPCL.y = cloud[i].position.y;
        pointPCL.z = cloud[i].position.z;

        cloudPCL->push_back(pointPCL);
}

    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // Create the filtering object
    //pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ> >sor;
    approximate_voxel_filter.setInputCloud (cloudPCL);
    approximate_voxel_filter.setLeafSize (0.01f, 0.01f, 0.01f);
    approximate_voxel_filter.filter (*cloudFilteredPCL);

    Point3D::Cloud cloudFiltered;
    Point3D point;

    for(size_t i=0;i<cloudFilteredPCL->size();i++)
            {
                point.position.v[0] = cloudFilteredPCL->at(i).x;
                point.position.v[1] = cloudFilteredPCL->at(i).y;
                point.position.v[2] = cloudFilteredPCL->at(i).z;
                cloudFiltered.push_back(point);
            }



    cloudPSO = cloudFiltered;

	// Read clouds from file
	Grabber* grabber = createGrabberKinect();

    // Palm
    grabber->LoadFromFile("../../resources/joints/palm.pcd");
	grabber->getCloud(handPSODefault.palm.surface);

    pcl::PointXYZ pointPCLPalm;
    //from
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCLPalm (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPCLPalm (new pcl::PointCloud<pcl::PointXYZ>);

    for(size_t i=0;i<handPSODefault.palm.surface.size();i++)
    {
        pointPCLPalm.x = handPSODefault.palm.surface[i].position.x;
        pointPCLPalm.y = handPSODefault.palm.surface[i].position.y;
        pointPCLPalm.z = handPSODefault.palm.surface[i].position.z;

        cloudPCLPalm->push_back(pointPCLPalm);
}

    // Create the filtering object
    //pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ> >sor;
    approximate_voxel_filter.setInputCloud (cloudPCLPalm);
    approximate_voxel_filter.setLeafSize (0.005f, 0.005f, 0.005f);
    approximate_voxel_filter.filter (*cloudFilteredPCLPalm);

    Point3D pointPalm;

    handPSODefault.palm.surface.clear();


    for(size_t i=0;i<cloudFilteredPCLPalm->size();i++)
            {
                pointPalm.position.v[0] = cloudFilteredPCLPalm->at(i).x;
                pointPalm.position.v[1] = cloudFilteredPCLPalm->at(i).y;
                pointPalm.position.v[2] = cloudFilteredPCLPalm->at(i).z;

                handPSODefault.palm.surface.push_back(pointPalm);
            }


    Visualizer* visuPCL10 = createVisualizerPCL();
    RGBA color10;
    color10.r = 255;
    color10.g = 0;
    color10.b = 0;
    color10.a = 255;
    //visuPCL1->addCloud(handPSO.palm.surface, color1);
    visuPCL10->addCloud(handPSODefault.palm.surface, color10);
    visuPCL10->show();




	// Fingers
    string fingerCloudNames[3] = { "../../resources/joints/finger_bottom.pcd",
            "../../resources/joints/finger_bottom.pcd",
            "../../resources/joints/finger_bottom.pcd" };
	
	for (int i = 0; i < 3; i++) {

		grabber->LoadFromFile(fingerCloudNames[i]);

        for (int k = 0; k < 5; k++)
        {
            grabber->getCloud(handPSODefault.fingers[THUMB + k].chain[i].surface);

            pcl::PointXYZ pointPCLFinger;
            //from
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCLFinger (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPCLFinger (new pcl::PointCloud<pcl::PointXYZ>);

            for(size_t i=0; i < handPSODefault.fingers[THUMB + k].chain[i].surface.size() ; i++)
            {
                pointPCLFinger.x = handPSODefault.fingers[THUMB + k].chain[i].surface[i].position.x;
                pointPCLFinger.y = handPSODefault.fingers[THUMB + k].chain[i].surface[i].position.y;
                pointPCLFinger.z = handPSODefault.fingers[THUMB + k].chain[i].surface[i].position.z;

                cloudPCLFinger->push_back(pointPCLFinger);
        }

            // Create the filtering object
            //pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ> >sor;
            approximate_voxel_filter.setInputCloud (cloudPCLFinger);
            approximate_voxel_filter.setLeafSize (0.005f, 0.005f, 0.005f);
            approximate_voxel_filter.filter (*cloudFilteredPCLFinger);

            Point3D pointFinger;
            handPSODefault.fingers[THUMB + k].chain[i].surface.clear();

            for(size_t i=0;i<cloudFilteredPCLFinger->size();i++)
                    {
                        pointFinger.position.v[0] = cloudFilteredPCLFinger->at(i).x;
                        pointFinger.position.v[1] = cloudFilteredPCLFinger->at(i).y;
                        pointFinger.position.v[2] = cloudFilteredPCLFinger->at(i).z;
                        handPSODefault.fingers[THUMB + k].chain[i].surface.push_back(pointFinger);
                    }


        }


    }
	// Scene
    //grabber->LoadFromFile("../resources/Hand3.pcd");
    //grabber->getCloud(cloudPSO);
	
	/// perfrom PSO
	PsoAlgorithm();
	/// return hand after optimization. colud is unchanged. 

	// set return hand as the best calcualted hand by calling GetFunctionValue once again.
	GetFunctionValue(GetMinimum());
	
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
    float_type InitBestValue;
    float_type InitPos;
    float_type InitVel;
	
	for (int i = 0; i < MAX_PARTICLES; i++)
    {
        InitBestValue = 0;
        for (int j = 0; j < DIM; j++)
        {
            //for first elements limit the value to range -1 to 1 since this is config 
			if ( j < Hand::JOINTS + ROT_MAT_ELEMENTS)
			InitPos = GetRandomNumber(MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
			else
			InitPos = GetRandomNumber(START_RANGE_MIN_POS, START_RANGE_MAX_POS);
			
			particles[i].setPosition(j, InitPos);
			// set first position as best position
			particles[i].setBestPosition(j, InitPos);
            InitVel = GetRandomNumber(START_RANGE_MIN_VEL, START_RANGE_MAX_VEL);
            particles[i].setVelocity(j,InitVel);
            
			
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
    float_type NewVelocity[DIM];
    //float_type CurrentPos;

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
    float_type  NewFunctValue, tempData;

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

float_type OptimizationPSO::GetFunctionValue(int index)
{
    float_type result;

    //for (int i = 0; i < Hand::JOINTS ; i++) //DB brakuje klamr lub ta linia jest niepotrzebna


	/* Preparing of hand to clouds comparison */
    /************************************************************************************************************/
    //ForwardKinematics *fk = new ForwardKinematicsLiego();//DB usunalem
    ForwardKinematics *fk = createForwardKinematicsLiego(); //DB korzystajmy z metod, ktore stworzylismy i unikajmy 'naked pointers'
    //DB lepiej skorzystac z auto pointers, nie musimy pamietac o zwalnianiu pamieci: http://stackoverflow.com/questions/9299489/whats-a-naked-pointer

    Hand::Config handConfig;
	/// setting config in Hand according to particle data
	for (int i=0;i<5;i++)
		for (int j=0;j<4;j++)		
			handConfig.conf[i*4+j] = particles[index].getPosition(i*4+j);

	// Lengths of joints
	for (int i=0;i<5;i++)
		for(int j=0;j<3;j++)
		handPSO.fingers[THUMB+i].chain[j].length = 4.5 / 7;
	
	fk->forward(handPSO, handConfig);
    //delete fk; //DB przy wczesniejszej deklaracji fk wykomentowanie tego nie bylo dobre
	
	// Read clouds from file
	Grabber* grabber = createGrabberKinect();

	// Palm
	handPSO.palm.surface = handPSODefault.palm.surface;
	// Cloud scaling
//	for (int j = 0; j < handPSO.palm.surface.size(); j++) {
//		for (int y = 0; y < 3; y++)
//			handPSO.palm.surface[j].position.v[y] *= 20.0;
//	}

	// Fingers
	for (int i = 0; i < 3; i++) {
		for (int k = 0; k < 5; k++) {
			handPSO.fingers[THUMB + k].chain[i].surface = handPSODefault.fingers[THUMB + k].chain[i].surface;

			// Cloud scaling for better visualization
        //	for (int j=0;j<handPSO.fingers[THUMB + k].chain[i].surface.size();j++) {
        //		for (int y=0;y<3;y++)
        //			handPSO.fingers[THUMB + k].chain[i].surface[j].position.v[y] *= 20.0;
        //	}
		}
	}
	

	// Mat34 pose of the hand
	//Mat34 hand2cs;
	//hand2cs.R.m[0][0] = hand2cs.R.m[1][1] = hand2cs.R.m[2][2] = 1.0;
	//hand.pose = hand2cs;

    Mat34 hand2cs;

	for ( int  i = 0 ; i < 3 ; i++ )
		for (int j = 0; j < 3; j++ )
			//setting hand orientation according to particle data
			hand2cs.R.m[i][j] = particles[index].getPosition(Hand::JOINTS + i*3 + j);
	
	/// set value of xyz
    for ( int i = 0; i < 3 ; i++)
        hand2cs.p.v[i] = particles[index].getPosition(Hand::JOINTS + ROT_MAT_ELEMENTS + i);

	// set final hand value 
	handPSO.pose = hand2cs;

	// Mat34 pose of the finger
	Mat34 finger2hand;
	finger2hand.R.m[0][0] = finger2hand.R.m[1][1] = finger2hand.R.m[2][2] = 1.0;
	finger2hand.p.v[0] = -1.3;
	finger2hand.p.v[1] = 0.0;
	finger2hand.p.v[2] = -0.2;
	handPSO.fingers[THUMB].pose = finger2hand;
	finger2hand.p.v[0] = -0.5;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	handPSO.fingers[INDEX].pose = finger2hand;
	finger2hand.p.v[0] = 0;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	handPSO.fingers[MIDDLE].pose = finger2hand;
	finger2hand.p.v[0] = 0.5;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	handPSO.fingers[RING].pose = finger2hand;
	finger2hand.p.v[0] = 1.0;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	handPSO.fingers[PINKY].pose = finger2hand;

	// Transforming palm
	for (int k=0;k<handPSO.palm.surface.size();k++)
	{
		Eigen::Vector4f vec = vec3_2_eigen(handPSO.palm.surface[k].position);
		vec = mat34_2_eigen(handPSO.pose) * vec;
		handPSO.palm.surface[k].position = eigen_2_vec3(vec);
	}

	// Conversion between coordinate system of clouds representing part of finger and
	// coordinate system used in forward kinematic -- it's a shame that those are different ...
	Eigen::Matrix4f fk2vis = Eigen::Matrix4f::Zero();
	fk2vis(2,0) = fk2vis(0,1) = fk2vis(1,2) = fk2vis(3,3) = 1.0;

	// Recalculation of already put clouds accordingly to the calculated fk
	// I believe it is not suppose to be done automatically inside fk as computing fk 2 times
	// would move the clouds again -> it would make visualization impossible after optimization



	for (int j=0;j<Hand::FINGERS;j++)
	{
		// Relative transformations to the absolute ones
		Eigen::Matrix4f trans[4];
		trans[0] = mat34_2_eigen(handPSO.pose) * mat34_2_eigen(handPSO.fingers[j].pose);

        trans[1] = mat34_2_eigen(handPSO.fingers[j].chain[0].poseEnd);
		trans[1] = fk2vis * trans[1] * fk2vis.inverse();
		trans[1] = trans[0] * trans[1];

		trans[2] = mat34_2_eigen(handPSO.fingers[j].chain[1].poseEnd);
		trans[2] = fk2vis * trans[2] * fk2vis.inverse();
		trans[2] = trans[1] * trans[2];

		trans[3] = mat34_2_eigen(handPSO.fingers[j].chain[2].poseEnd);
		trans[3] = fk2vis * trans[3] * fk2vis.inverse();
		trans[3] = trans[2] * trans[3];

		// For all fingers, move the clouds
		for (int i = 0; i < 3; i++)
		{
            for (int k=0;k<handPSO.fingers[j].chain[i].surface.size();k++)
			{
				Eigen::Vector4f vec = vec3_2_eigen(handPSO.fingers[j].chain[i].surface[k].position);
				vec(1) -= handPSO.fingers[j].chain[i].length;
                vec = trans[i+1] * vec;
                handPSO.fingers[j].chain[i].surface[k].position = eigen_2_vec3(vec);
            }
		}
	}

	/************************************************************************************************************/
    Visualizer* visuPCL11 = createVisualizerPCL();
    RGBA color11;
    color11.r = 255;
    color11.g = 0;
    color11.b = 0;
    color11.a = 255;
    //visuPCL1->addCloud(handPSO.palm.surface, color1);
    visuPCL11->addCloud(handPSO.palm.surface, color11);
     visuPCL11->addCloud(cloudPSO, color11);
    visuPCL11->show();



	optimizationFunction * optimization_function = createOptimizationFunctionPF();
    std::cout<<"Aktualne: "<<cloudPSO.size()<<std::endl;
	result = optimization_function->FitnessValue(handPSO,cloudPSO);
	std::cout<<"Aktualne: "<<result<<std::endl;
    return result;
}

float_type OptimizationPSO::GetRandomNumber(float_type LowBound, float_type UpBound)
{
    float_type temp;
    temp = (float_type)LowBound + float_type(((UpBound-LowBound))*rand()/(RAND_MAX + 1.0));

    return temp;
}

float_type OptimizationPSO::GetRand()
{
    float_type temp;
    temp = float_type(rand()/(RAND_MAX + 1.0));
    return temp;
}
/* Author: Smi */
Mat34 OptimizationPSO::eigen_2_mat34(const Eigen::Matrix4f &trans) {
	Mat34 pose;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose.R.m[i][j] = trans(i, j);
		}
		pose.p.v[i] = trans(i, 3);
	}
	return pose;
}
/* Author: Smi */
Eigen::Matrix4f OptimizationPSO::mat34_2_eigen(const Mat34 &trans) {
	Eigen::Matrix4f pose;
	pose.setIdentity();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose(i, j) = trans.R.m[i][j];
		}
		pose(i, 3) = trans.p.v[i];
	}
	return pose;
}
/* Author: Smi */
Eigen::Vector4f OptimizationPSO::vec3_2_eigen(Vec3 vec)
{
	Eigen::Vector4f res(vec.x, vec.y, vec.z, 1.0);
	return res;
}
/* Author: Smi */
Vec3 OptimizationPSO::eigen_2_vec3(Eigen::Vector4f vec)
{
	Vec3 res;
	res.v[0] = vec(0);
	res.v[1] = vec(1);
	res.v[2] = vec(2);
	return res;
}


handest::Optimization* handest::createOptimizationPSO(void) {
	optPSO.reset(new OptimizationPSO());
	return optPSO.get();
}
