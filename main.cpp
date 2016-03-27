/*
 * Run.cpp
 *
 *  Created on: Mar 8, 2016
 *      Author: colman
 */

#include <libplayerc++/playerc++.h>
#include <unistd.h>

using namespace PlayerCc;
using namespace std;


enum SIDE
{
	LEFT,
	RIGHT
};

class Obstecle
{
public:
	bool is_valid;
	int x;
	int y;
	SIDE side;

	Obstecle()
		: is_valid(false)
	{}

	void ToString()
	{
		if (this->is_valid)
		{
			cout << "Obstacle x: " << this->x << ", y: " << this->y << ", side: " << this->side << endl;
		}
	}
};

class SonarSensors
{
	static const int numOfSonars = 5;
	static const double obstcleDistanceTreshold = 0.4;
public:
	SonarSensors(PlayerClient& pc)
		: m_sp(&pc)
	{
		double sonarInDegs[numOfSonars] = {90, 25,0,-25,-90};

		// Initialize the array or sonar probes by radians from set degrees.
		for (int i = 0; i < numOfSonars; i++)
		{
			m_sonarLocInRad[i] = dtor(sonarInDegs[i]);
		}
	}

	/**
	 * This method gets the current map position and return
	 * if the robot has an obstecle near it
	 * An Obstecle found if the is_valid atrribute is set.
	 * */
	Obstecle GetObstecle(Position2dProxy& pp)
	{
		Obstecle obs;

		// Probe all the sonars till an obstecle is found
		for (int i = 0; i< numOfSonars; i++)
		{
			// Validate if there is an obstecle is
			// near the sonar less then the trashold
			if (m_sp[i] < 0.4)
			{
				// Set that there is an obstecle
				obs.is_valid = true;

				// Calculate the location of the obstecle
				double ang = pp.GetYaw() + m_sonarLocInRad[i];
				obs.x = m_sp[i] * cos(ang) + pp.GetXPos();
				obs.y = m_sp[i] * cos(ang) + pp.GetYPos();

				// Find wehere on what side of the robot the obstecle is
				// So we will know where to move
				if (m_sp[1] < m_sp[3])
				{
					obs.side = LEFT;
				}
				else
				{
					obs.side = RIGHT;
				}
			}
		}

		return obs;
	}


	private:
		SonarProxy m_sp;
		double m_sonarLocInRad[numOfSonars];
};

int main()
{
	// Initialzir the enviorment
	PlayerClient pc("localhost", 6665);
	Position2dProxy pp(&pc);
	pp.SetOdometry(-6.01,2.48,-3.39);

	SonarSensors sonar(pc);

	// Set initial direction of the robot.
	// Straight ahead in 0.5 mps speed.
	double speed = 0.5;
	double rotation = 0;

	pc.Read();

	while(true)
	{
		// Get an obstecle relative to the current robot position.
		Obstecle obs = sonar.GetObstecle(pp);

		// Print hte location of the found obstacle
		obs.ToString();

		if(!obs.is_valid)
		{
			// Uncrease the speed if we lowered it before to
			// avoid an obstecle
			if(speed < 0.5)
			{
				speed += 0.1;
			}

			// If there is no obstecle ahead then continue straight
			rotation = 0;
		}
		else
		{
			// Slow down and start avoiding the obstecle found
			if(speed > 0.1)
			{
				speed -= 0.1;
			}

			// If the obstecle is the nearest to
			// the the left then rotate further to the right
			// and turn left otherwise.
			if (obs.side == LEFT)
			{
				rotation += 0.5;
			}
			else
			{
				rotation -= 0.5;
			}

		}

		// Keep the understood above movment
		pp.SetSpeed(speed, rotation);

		pc.Read();
	}
}
