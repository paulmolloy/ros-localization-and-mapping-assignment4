
	
#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles)
{
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++)
	{
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter()
{
	// delete particles
	for (int i = 0; i < numberOfParticles; i++)
	{
		Particle *p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles()
{
	return this->numberOfParticles;
}

std::vector<Particle *> *ParticleFilter::getParticleSet()
{
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform()
{
	//get map properties
	int mapWidth, mapHeight;
	double mapResolution;
	this->getLikelihoodField(mapWidth, mapHeight, mapResolution);
	double x, y;
	float theta;
	float weightDefault = 1.0f / numberOfParticles;

	// Initialize particles.
	// Each one at a random position uniformly distributed around the map.
	for(int i=0; i<numberOfParticles; i++){	
		Particle* p = new Particle(Util::uniformRandom(0,mapWidth*mapResolution), // sample x.
				Util::uniformRandom(0,mapHeight*mapResolution), // sample y.
				Util::normalizeTheta(Util::uniformRandom(0,2*M_PI)), // sample theta.
				weightDefault); // Sample Weight.
		this->particleSet[i] = p;	
	}	 
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y, double mean_theta,
	       	double std_xx, double std_yy, double std_tt)
{

	// Initialize particles.
	// Each one at a randomly  position distributed with gaussian noise from 
	// a position on the map.
	for(int i=0; i<numberOfParticles; i++){
		double sample_x = Util::gaussianRandom(mean_x, std_xx);
		double sample_y = Util::gaussianRandom(mean_y, std_yy);
		double sample_theta = Util::normalizeTheta(Util::gaussianRandom(mean_theta, std_tt));
		Particle* p = new Particle(sample_x, sample_y, sample_theta, 1/numberOfParticles);
		this->particleSet[i] = p;
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
	const nav_msgs::OccupancyGrid &map, double zRand, double sigmaHit)
{
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

	// calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

	// Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).
	
	ROS_INFO("...DONE creating likelihood field!");
	for(int i=0;i<map.info.width;i++){
		for(int j=0; j<map.info.height; j++){

			// For each position in the map.
			double dist = this->distMap[i + j*map.info.width];
			//		     gaussian(x, std,					 , mean);
			double p_hit = Util::gaussian(0, sigmaHit/this->likelihoodFieldResolution, dist);
			
			double prob = (1-zRand) * p_hit + zRand;
			
			// Using log probability to avoid rounding errors.		
			this->likelihoodField[i + j*map.info.width] += std::log(prob); 		
		}
	}
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid &map)
{
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++)
	{
		for (int y = 0; y < likelihoodFieldHeight; y++)
		{
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
    // TODO: Optimize; not as bad as it looks 2 for loops just iterate twice.
	for (int x = 0; x < map.info.width; x++)
	{
		for (int y = 0; y < map.info.height; y++)
		{
			if (map.data[x + y * map.info.width] >= occupiedCellProbability)
			{
				bool border = false;
				for (int i = -1; i <= 1; i++)
				{
					for (int j = -1; j <= 1; j++)
					{
						if (!border && x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
						{
							if (map.data[x + i + (y + j) * likelihoodFieldWidth] < occupiedCellProbability && map.data[x + i + (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth] = 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
    //
    // TODO: Optimize; not as bad as it looks 2 for loops just iterate twice.
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
					{
						double v = distMap[x + i + (y + j) * likelihoodFieldWidth] + ((i * j != 0) ? 1.414
																								   : 1);
                        // + sqrt(2) or 1
                        
						if (v < distMap[x + y * likelihoodFieldWidth])
						{
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
    // TODO: Optimize; not as bad as it looks 2 for loops just iterate twice.
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
					{
						double v = distMap[x + i + (y + j) * likelihoodFieldWidth] + ((i * j != 0) ? 1.414
																								   : 1);
						if (v < distMap[x + y * likelihoodFieldWidth])
						{
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double *ParticleFilter::getLikelihoodField(int &width, int &height, double &resolution)
{
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
	const sensor_msgs::LaserScanConstPtr &laserScan)
{
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
	const sensor_msgs::LaserScanConstPtr &laserScan)
{
	this->sumOfParticleWeights=0;
	double ang_min = laserScan->angle_min;
	double range_max = laserScan->range_max;
	double range_min = laserScan->range_min;
	double ang_step = this->laserSkip;
	double sum = 0;

	for(int i=0; i<this->particleSet.size(); i++){
		Particle* p = this->particleSet[i];
		double particleWeight = 0;

		// Update the particle weight based on the lazer ranges in every direction.
		for(int j = 0; j < laserScan->ranges.size(); j += ang_step){
			double distance = laserScan->ranges.at(j);	

			// Angle is based on the zero angle + the robot angle
			// + laser scan angle relative to the robot.
			double angle = ang_min + p->theta + laserScan->angle_increment*j; 
			
			// The co-ords we think the lazer hit based on 
			// where we think we are and the laser range and angle.
			int hit_x = round( (distance * cos(angle)+p->x) / this->likelihoodFieldResolution);
			int hit_y = round( (distance * sin(angle)+p->y) / this->likelihoodFieldResolution);

			int idx = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, hit_x, hit_y);
			bool isInRange = distance < range_max && distance > range_min;	
			// Add to the weight if its in range and  inside the bounds of the likelyhood field.
			if(idx >= 0 && idx < (this->likelihoodFieldWidth*this->likelihoodFieldHeight) && isInRange){
				particleWeight += this->likelihoodField[idx];
			}else{
				particleWeight += log(1E-5);      // Penalty parameter for the particles outside the bound.
			}
		}
		// Set the new particle weight and update sum.
		this->particleSet.at(i)->weight = exp(particleWeight);
		this->sumOfParticleWeights += this->particleSet.at(i)->weight;
	}


//ROS_INFO("XXXXXXXXXXX,%d", particleSet.size());

}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2, double alpha3, double alpha4)
{
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;
}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY, double oldTheta, double newX, double newY, double newTheta)
{
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY, 
		double oldTheta, double newX, double newY, double newTheta)
{
	// In Class ParticleFilterNode the parameters in the lastOdomPose is initialized with nan, 
	//which potentially causes problems here
	if(isnan(oldX) || isnan(oldY) || isnan(oldTheta)|| isnan(newTheta) || isnan(newX) || isnan(newY))
		return ;

	// Get the delta of the translation and rotations operations which 
	// resulted in the odomety values for current state compared to the old state.
	double d_trans = std::sqrt((newX-oldX)*(newX-oldX) + (newY-oldY)*(newY-oldY));
	double d_rot1 = std::atan2(newY-oldY, newX-oldX) - oldTheta;
	double d_rot2 = newTheta-oldTheta-d_rot1;
	
	// For each particle:
	for(int i=0; i< this->particleSet.size();i++){
		// Calculate a possible actual translation and rotation deltas, given our model
		// that our odometry isn't perfect and there is some amount of gaussian noise in each.
		double d_rot1_h = d_rot1 + Util::gaussianRandom(0, this->odomAlpha1*std::abs(d_rot1) + this->odomAlpha2*d_trans);
		double d_trans_h = d_trans + Util::gaussianRandom(0, this->odomAlpha2*d_trans + 
				this->odomAlpha4*std::abs(d_rot1+d_rot2));
		double d_rot2_h = d_rot2 + Util::gaussianRandom(0, this->odomAlpha1*std::abs(d_rot2) + this->odomAlpha2*d_trans);
		double x = this->particleSet[i]->x;
		double y = this->particleSet[i]->y;
		double theta = this->particleSet[i]->theta;
		// Set current x, y and theta params assuming these possible delta values.
		double x_new = x + d_trans_h * cos(theta + d_rot1_h);
		double y_new = y + d_trans_h * sin(theta + d_rot1_h);
		double theta_new = theta + d_rot1_h + d_rot2_h;
		theta_new = Util::normalizeTheta(theta_new);

		// Set the particle with this possible new robot state.
		Particle* p = new Particle(x_new, y_new, theta_new, particleSet.at(i)->weight);
		this->particleSet[i] = p;
	}
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample()
{

	std::vector<Particle*> newSet;
	double weight_sum = 0;
	int numParticles = getNumberOfParticles();
	double randNumber = Util::uniformRandom(0, 1.0f/(double)particleSet.size() );

	// Cumulative distribution function for resampling.
	double cdf[numParticles];

	cdf[0] = (this->particleSet[0]->weight / this->sumOfParticleWeights);
	// Fill it up with the sum of the normalized weights <= less then i for each i.
	for(int i=1; i < numParticles; i++){
		cdf[i] = cdf[i-1] + this->particleSet[i]->weight / this->sumOfParticleWeights;
	}

	double curThreshold = randNumber;
	for(int j=0, i=0; j < particleSet.size(); j++){

		// Skip until the next threshold reached.
		while(curThreshold > cdf[i]){
			i++;
		}
		// We didn't update the weight as 1/n, since it yields bad results
		Particle* p = new Particle(this->particleSet[i]); 
		newSet.push_back(p);

		// Increment to the next threshold.
		curThreshold += 1.0f/(double)particleSet.size();
	}

	this->particleSet = newSet;
	

	// Decide best hypothesis.
	// i.e. which particle has the heighest weight.
	int idx;
	double bestWeight=0;
	for(int i=0; i < this->particleSet.size(); i++){
		if(this->particleSet[i]->weight > bestWeight){
			idx=i;
			bestWeight = particleSet[i]->weight;
		}
	}

	this->bestHypothesis = new Particle(this->particleSet[idx]);
}


Particle *ParticleFilter::getBestHypothesis()
{
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
									int y)
{
	return x + y * width;
}

