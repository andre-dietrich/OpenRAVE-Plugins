//#include "filter.h"
#include "occupancyGridMap.h"
#include <iostream>
#include <string>

#ifndef SENSORGRIDMAP_H
#define SENSORGRIDMAP_H

using namespace OpenRAVE;

class SensorGridMap : public OccupancyGridMap{
public:

	SensorGridMap(EnvironmentBasePtr penv, std::istream& ss) : OccupancyGridMap(penv, ss) {
		//this->generateGrid();

		UnregisterCommand("Scan");
		RegisterCommand("Scan", boost::bind(&SensorGridMap::Scan,this,_1,_2), "Makes a 2D scan of the current Environment");

		RegisterCommand("RenderX", boost::bind(&SensorGridMap::RenderX,this,_1,_2), "Makes a 2D scan of the current Environment");
		RegisterCommand("ScanX", boost::bind(&SensorGridMap::ScanX,this,_1,_2), "Makes a 2D scan of the current Environment");
	}

	virtual ~SensorGridMap() {}

	bool ScanX(std::ostream& sout, std::istream& sinput)
	{
		std::vector< SensorBasePtr > sensors;
		std::string sName;

		EnvironmentBasePtr env = GetEnv();//->CloneSelf(0b10001);

		env->GetSensors(sensors);
		{
			//boost::mutex::scoped_lock lock(m_mMutexdata);

			for(std::vector<SensorBasePtr>::iterator iterSensor = sensors.begin();
					iterSensor != sensors.end();
					iterSensor++)
			{
				if(!(*iterSensor)->Configure(OpenRAVE::SensorBase::CC_PowerCheck, true))
					continue;

				std::stringstream ssout;
				std::stringstream ssin; ssin << "collidingbodies";

				if( (*iterSensor)->SendCommand(ssout, ssin) ) {
					//std::cerr << gridPtr->GetName() << std::endl;
					std::string sID;

					while(ssout >> sID){
						if(sID != "0"){

							sName = env->GetBodyFromEnvironmentId( std::atoi(sID.c_str()) )->GetName();

							if(sName.rfind("olk7sh") == 0){
								sName.replace(0, 6, "");

								this->occupancyGrid[ std::atoi( sName.c_str() ) ] ++;
							}
						}
					}
				}
			}

			for(std::vector<int>::iterator itGrid = occupancyGrid.begin();
					itGrid != occupancyGrid.end();
					itGrid++)
			{
				sout << *itGrid << " ";
			}
		}

		return true;
	}

	bool Scan(std::ostream& sout, std::istream& sinput)
	{
		std::vector< SensorBasePtr > sensors;
		std::string sName;
		std::string sID;

		EnvironmentBasePtr env = GetEnv()->CloneSelf(0b10001);

		env->GetSensors(sensors);
		{
			for(std::vector<SensorBasePtr>::iterator iterSensor = sensors.begin();
				iterSensor != sensors.end();
				iterSensor++)
			{

				if(!(*iterSensor)->Configure(OpenRAVE::SensorBase::CC_PowerCheck, true))
					continue;

				try
				{

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

							std::map<std::string,unsigned int> map;

							while(ssout >> sID){
								if(sID != "0"){
									if(map.find(sID) == map.end()){
										map[sID] = 0;
									}
									map[sID]++;
								}
							}

							for(std::map<std::string,unsigned int>::const_iterator it = map.begin(); it != map.end(); it++)
							{
								std::string key = it->first;
								unsigned int value = it->second;

								if(env->GetBodyFromEnvironmentId( std::atoi( key.c_str()) )->GetName() == "XXX"){
									this->occupancyGrid[ iGridElement ] += value;
									break;
								}
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
				catch(...){
					std::cerr << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
				}
			}

			for(std::vector<int>::iterator itGrid = occupancyGrid.begin();
					itGrid != occupancyGrid.end();
					itGrid++)
			{
				sout << *itGrid << " ";
			}

		}

		return true;
	}

	bool RenderX(std::ostream& sout, std::istream& sinput)
	{
		EnvironmentBasePtr env = GetEnv();

		int iID  = 0;

		std::string input;

		sinput >> input;

		// remove grid...
		if(input == "0"){
			std::vector<KinBodyPtr> bodies;

			env->GetBodies(bodies);

			for(std::vector<KinBodyPtr>::iterator triElement = bodies.begin();
					triElement != bodies.end();
					triElement++)
			{

				input = (*triElement)->GetName();

				if(input.rfind("olk7sh") == 0){
					env->Remove(*triElement);
				}
			}
		}
		else {
			unsigned int counter = 0;
			for(std::vector<KinBody::Link::TRIMESH>::iterator triElement = meshGrid.begin();
					triElement != meshGrid.end();
					triElement++)
			{

				std::stringstream ssID;
				ssID << "olk7sh" << iID++;

				KinBodyPtr gridPtr = RaveCreateKinBody(env);
				gridPtr->InitFromTrimesh(*triElement, false);
				gridPtr->SetName(ssID.str().c_str());
				gridPtr->SetTransform(m_tGridTransformation);
				env->Add(gridPtr, true);

				if(counter % 80 == 0){
					std::cerr << " " << counter << "\n";
				}
				std::cerr << ".";
				counter++;

			}

			std::stringstream os;
			std::stringstream is;
			OccupancyGridMap::Render(os, is);


		}

		return true;

	}
};

#endif
