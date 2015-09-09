#include "filter.h"
#include <iostream>
#include "occupancyCubeMap.h"
//using namespace OpenRAVE;

#ifndef OCTOMAP_H
#define OCTOMAP_H

class Octomap : public OccupancyCubeMap
{
public:

	unsigned int m_uiSteps;

	Octomap(EnvironmentBasePtr penv, std::istream& ss) : OccupancyCubeMap(penv, ss) {

		UnregisterCommand("SetSize");
		RegisterCommand("SetSize",boost::bind(&Octomap::SetSize,this,_1,_2), "Set the size of the cubes: (int) x (double) amount, (int) y (double) resolution, (int) z (double) resolution ");

		UnregisterCommand("GetSize");
		RegisterCommand("GetSize",boost::bind(&Octomap::GetSize,this,_1,_2), "Return the current size of the grid: (int)width, (height)height, (double)resolution ");

		//UnregisterCommand("Scan");
		//RegisterCommand("Scan",boost::bind(&Octomap::Scan,this,_1,_2), "Return the current size of the grid: (int)width, (height)height, (double)resolution ");

		m_uiNumX = m_uiNumY = m_uiNumZ = 1;
		m_uiSteps = 10;
	}

	virtual ~Octomap() {}

	bool SetSize(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_dResolutionX;
		sinput >> m_dResolutionY;
		sinput >> m_dResolutionZ;
		sinput >> m_uiSteps;

		generateCubes();

		return true;
	}

	bool GetSize(std::ostream& sout, std::istream& sinput)
	{
		sout << m_dResolutionX << " " << m_dResolutionY << " " << m_dResolutionZ << " " << m_uiSteps;
		return true;
	}

	void generateCubes(){
		lCubes.clear();
		occupancyCubes.clear();

		KinBody::Link::GeometryInfo element;

		element._type = KinBody::Link::GeomBox;
		element._fTransparency = 0.5;

		unsigned int num = 1;

		for(unsigned int steps=0; steps < m_uiSteps; steps++){
			for(unsigned int x=0; x<num; x++){
				for(unsigned int y=0; y<num; y++){
					for(unsigned int z=0; z<num; z++){
						element._vGeomData = Vector(m_dResolutionX/num/2, m_dResolutionY/num/2, m_dResolutionZ/num/2);
						element._t = Transform(Vector(0,0,0,1), Vector(	x*(m_dResolutionX/num),
																		y*(m_dResolutionY/num),
																		z*(m_dResolutionZ/num)));

						lCubes.push_back(element);
						occupancyCubes.push_back(0);
					}
				}
			}

			num *= 2;
		}
	}

};

#endif
