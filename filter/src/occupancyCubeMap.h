#include "filter.h"
#include <iostream>
#include <string>
#include <math.h>

#ifndef OCCUPANCYCUBEMAP_H
#define OCCUPANCYCUBEMAP_H

using namespace OpenRAVE;

class OccupancyCubeMap : public Filter{
public:

	unsigned int m_uiNumX;
	unsigned int m_uiNumY;
	unsigned int m_uiNumZ;

	double m_dResolutionX;
	double m_dResolutionY;
	double m_dResolutionZ;

	GraphHandlePtr m_ghpGraphGeometryFree, m_ghpGraphGeometryOccupied;
	CollisionReportPtr m_cprCollistionReport;

	std::list<KinBody::GeometryInfo> lCubes;
	std::vector<int> occupancyCubes;

	OccupancyCubeMap(EnvironmentBasePtr penv, std::istream& ss) : Filter(penv, ss) {

		RegisterCommand("SetSize",boost::bind(&OccupancyCubeMap::SetSize,this,_1,_2), "Set the size of the cubes: (int) x (double) amount, (int) y (double) resolution, (int) z (double) resolution ");
		RegisterCommand("GetSize",boost::bind(&OccupancyCubeMap::GetSize,this,_1,_2), "Return the current size of the grid: (int)width, (height)height, (double)resolution ");

		RegisterCommand("Render",boost::bind(&OccupancyCubeMap::Render,this,_1,_2), "Renders the current occupancy grid");
		RegisterCommand("Scan", boost::bind(&OccupancyCubeMap::Scan,this,_1,_2), "Makes a 2D scan of the current Environment");

		m_uiNumX       = m_uiNumY       = m_uiNumZ       = 10;
		m_dResolutionX = m_dResolutionY = m_dResolutionZ = 0.1;

		m_cprCollistionReport.reset(new CollisionReport());
	}

	virtual ~OccupancyCubeMap() {}

	bool SetSize(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_uiNumX;
		sinput >> m_dResolutionX;

		sinput >> m_uiNumY;
		sinput >> m_dResolutionY;

		sinput >> m_uiNumZ;
		sinput >> m_dResolutionZ;

		generateCubes();
		return true;
	}

	bool GetSize(std::ostream& sout, std::istream& sinput)
	{
		sout << m_uiNumX << " " << m_dResolutionX << " ";
		sout << m_uiNumY << " " << m_dResolutionY << " ";
		sout << m_uiNumZ << " " << m_dResolutionZ;
		return true;
	}

	bool Scan(std::ostream& sout, std::istream& sinput)
	{
		EnvironmentBasePtr env = GetEnv()->CloneSelf(1);

		env->GetCollisionChecker()->SetCollisionOptions(CO_RayAnyHit);
		{
			if(env->GetKinBody("occupancyCube")){
				env->Remove(env->GetKinBody("occupancyCube"));
			}

			KinBodyPtr cubePtr = RaveCreateKinBody(env);

			std::vector<int>::iterator bElement = occupancyCubes.begin();
			for(std::list<KinBody::GeometryInfo>::iterator itCubes = lCubes.begin();
					itCubes != lCubes.end();
					itCubes++, bElement++)
			{
				std::list<KinBody::GeometryInfo>cube;
				cube.push_back(*itCubes);
				cubePtr->InitFromGeometries(cube);
				cubePtr->SetName("occupancyCube");
				cubePtr->SetTransform(m_tGridTransformation);
				env->Add(cubePtr, true);

				if( env->CheckCollision(KinBodyConstPtr(cubePtr), m_cprCollistionReport) ){
					*bElement = true;
					sout << true;
				} else {
					*bElement = false;
					sout << false;
				}

				env->Remove(cubePtr);
			}
		}

		env->Destroy();

		return true;
	}

	bool Render(std::ostream& sout, std::istream& sinput){

		std::list<KinBody::GeometryInfo> nCubes;
		int newGrid = 0;
		sinput >> newGrid;

		if(GetEnv()->GetKinBody("occupancyCube")){
			GetEnv()->Remove(GetEnv()->GetKinBody("occupancyCube"));
		}

		KinBodyPtr cubePtr = RaveCreateKinBody(GetEnv());
		cubePtr->SetName("occupancyCube");

		std::vector<int>::iterator bElement = occupancyCubes.begin();
		for(std::list<KinBody::GeometryInfo>::iterator itCubes = lCubes.begin();
				itCubes != lCubes.end();
				itCubes++, bElement++)
		{
			if(*bElement){
				itCubes->_fTransparency = 0.1;
				if(newGrid == 1)
					nCubes.push_back(*itCubes);
			} else {
				itCubes->_fTransparency = 0.9;
			}
		}

		if(newGrid == 1){
			if(cubePtr->InitFromGeometries(nCubes)){
				cubePtr->SetTransform(m_tGridTransformation);
				GetEnv()->AddKinBody(cubePtr);
			}
		} else {
			if(cubePtr->InitFromGeometries(lCubes)){
				cubePtr->SetTransform(m_tGridTransformation);
				GetEnv()->AddKinBody(cubePtr);
			}
		}

		return true;
	}

	void generateCubes(){
		lCubes.clear();
		occupancyCubes.clear();

		KinBody::Link::GeometryInfo element;

		element._type = KinBody::Link::GeomBox;
		element._vGeomData = Vector(m_dResolutionX/2, m_dResolutionY/2, m_dResolutionZ/2);
		element._fTransparency = 0.5;

		for(unsigned int x=0; x<m_uiNumX; x++){
			for(unsigned int y=0; y<m_uiNumY; y++){
				for(unsigned int z=0; z<m_uiNumZ; z++){
					element._t = Transform(Vector(0,0,0,1), Vector(x*m_dResolutionX, y*m_dResolutionY, z*m_dResolutionZ));

					lCubes.push_back(element);
					occupancyCubes.push_back(0);
				}
			}
		}
	}
};

#endif
