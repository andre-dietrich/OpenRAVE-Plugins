//#include "filter.h"
#include "occupancyGridMap.h"
#include <iostream>
#include <string>
#include <math.h>

#ifndef RANGEGRIDMAP_H
#define RANGEGRIDMAP_H

using namespace OpenRAVE;

class RangeGridMap : public OccupancyGridMap{
public:

	RangeGridMap(EnvironmentBasePtr penv, std::istream& ss) : OccupancyGridMap(penv, ss) {
		//this->generateGrid();

		UnregisterCommand("Scan");
		RegisterCommand("Scan", boost::bind(&RangeGridMap::Scan,this,_1,_2), "Makes a 2D scan of the current Environment");

	}

	virtual ~RangeGridMap() {}

	bool Scan(std::ostream& sout, std::istream& sinput)
	{
		std::vector< SensorBasePtr > sensors;
		std::string sID;

		EnvironmentBasePtr env = GetEnv()->CloneSelf(0b10001);

		env->GetSensors(sensors);
		{

			std::vector<double> ranges;
			for(unsigned int i=0; i<this->occupancyGrid.size(); i++){
				ranges.push_back(0);
			}

			for(std::vector<SensorBasePtr>::iterator iterSensor = sensors.begin();
				iterSensor != sensors.end();
				iterSensor++)
			{
				if(!(*iterSensor)->Configure(OpenRAVE::SensorBase::CC_PowerCheck, true))
					continue;

				std::stringstream ssout;
				std::stringstream ssin; ssin << "collidingbodies";

				KinBodyPtr gridPtr = RaveCreateKinBody(env);
				gridPtr->SetName("XXX");

				unsigned int iGridElement=0;
				for(std::vector<KinBody::Link::TRIMESH>::iterator triElement = meshGrid.begin();
						triElement != meshGrid.end();
						triElement++)
				{
					gridPtr->InitFromTrimesh(*triElement, false);

					gridPtr->SetTransform(m_tGridTransformation);
					env->Add(gridPtr, true);

					//(*iterSensor)->SimulationStep(1);
					env->StepSimulation(1);

					std::stringstream ssout;
					std::stringstream ssin; ssin << "collidingbodies";

					if( (*iterSensor)->SendCommand(ssout, ssin) ) {

						SensorBase::SensorDataPtr psensor = (*iterSensor)->CreateSensorData();
						(*iterSensor)->GetSensorData(psensor);

						SensorBase::SensorGeometryPtr pgeometry = (*iterSensor)->GetSensorGeometry(SensorBase::ST_Laser);
						double sensorMaximum = boost::dynamic_pointer_cast<SensorBase::LaserGeomData>(pgeometry)->max_range;

						//SensorGeometryPtr GetSensorGeometry(SensorType type)

						unsigned int iPos = 0;

						while(ssout >> sID){
							if(sID != "0"){
								if( env->GetBodyFromEnvironmentId( std::atoi( sID.c_str()) )->GetName() == "XXX"){
									double x = boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(psensor)->ranges[iPos][0];
									double y = boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(psensor)->ranges[iPos][1];
									double z = boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(psensor)->ranges[iPos][2];

									ranges[ iGridElement ] += 1-(sqrt((x*x)+(y*y)+(z*z)))/sensorMaximum;
									occupancyGrid[iGridElement] ++;
								}
							}

							iPos++;
						}

						/*
						for (std::set<std::string>::iterator itSet = set.begin(); itSet != set.end(); ++itSet)
						{
							if(env->GetBodyFromEnvironmentId( std::atoi( itSet->c_str()) )->GetName() == "XXX"){

								// search for appeareance of id ..
								this->occupancyGrid[ iGridElement ] += std::count (hits.begin(), hits.end(), *itSet);
								break;

							}
						}
						*/
					}

					std::cout << ".";

					if(iGridElement % 80 == 0){
						std::cout << " " << iGridElement << "\n";
					}

					env->Remove(gridPtr);
					iGridElement ++;
				}
			}

			//double boost::dynamic_pointer_cast<SensorBase::LaserSensorData>(psensor)->
			for(unsigned int iGrid = 0; iGrid < ranges.size(); iGrid++)
			{
				if(occupancyGrid[iGrid] != 0)
					sout << (ranges[iGrid]/occupancyGrid[iGrid]) << " ";
				else
					sout << "0.0" << " ";
			}

		}

		return true;
	}
};

#endif
