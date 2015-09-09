#include "filter.h"
#include <iostream>
//using namespace OpenRAVE;

#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

class OccupancyGridMap : public Filter
{
public:
	unsigned int m_uiWidth;
	unsigned int m_uiHeight;
	double m_dResolution;
	float m_fLineWidth;
	
	GraphHandlePtr m_ghpGraphGeometryFree, m_ghpGraphGeometryOccupied;
	CollisionReportPtr m_cprCollistionReport;
	
	// mesh...
	std::vector<int> occupancyGrid;
	std::vector<KinBody::Link::TRIMESH> meshGrid;
	
	OccupancyGridMap(EnvironmentBasePtr penv, std::istream& ss) : Filter(penv, ss) {
		RegisterCommand("SetSize",boost::bind(&OccupancyGridMap::SetSize,this,_1,_2), "Set the size of the grid: (int)width, (height)height, (double)resolution ");
		RegisterCommand("GetSize",boost::bind(&OccupancyGridMap::GetSize,this,_1,_2), "Return the current size of the grid: (int)width, (height)height, (double)resolution ");
		RegisterCommand("Scan",boost::bind(&OccupancyGridMap::Scan,this,_1,_2), "Makes a 2D scan of the current Environment");
		RegisterCommand("Render",boost::bind(&OccupancyGridMap::Render,this,_1,_2), "Renders the current occupancy grid");
		RegisterCommand("SetLineWidth",boost::bind(&OccupancyGridMap::SetLineWidth,this,_1,_2), "Set the Line Width");
		RegisterCommand("GetLineWidth",boost::bind(&OccupancyGridMap::GetLineWidth,this,_1,_2), "Get the Line Width");
		
		m_uiWidth = 10;
		m_uiHeight= 10;
		m_dResolution = 0.1;
		m_fLineWidth = 2.0;
		
		m_cprCollistionReport.reset(new CollisionReport());
		
		generateGrid();
	}
    
	virtual ~OccupancyGridMap() {}
	
	bool SetSize(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_uiWidth;
		sinput >> m_uiHeight;
		sinput >> m_dResolution;
		generateGrid();
		return true;
	}
	
	bool GetSize(std::ostream& sout, std::istream& sinput)
	{
		sout << m_uiWidth << " " << m_uiHeight << " " << m_dResolution;
		return true;
	}
	
	bool SetLineWidth(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_fLineWidth;
		return true;
	}

	bool GetLineWidth(std::ostream& sout, std::istream& sinput)
	{
		sout << m_fLineWidth;
		return true;
	}

	bool Scan(std::ostream& sout, std::istream& sinput)
	{
		EnvironmentBasePtr env = GetEnv()->CloneSelf(1);
		env->GetCollisionChecker()->SetCollisionOptions(CO_RayAnyHit);
		{
			// Lock the data mutex and fill with the range data (get all in one timestep)
			//boost::mutex::scoped_lock lock(m_mMutexdata);
			KinBodyPtr gridPtr = RaveCreateKinBody(env);

			std::vector<int>::iterator bElement = occupancyGrid.begin();
			for(std::vector<KinBody::Link::TRIMESH>::iterator triElement = meshGrid.begin();
				triElement != meshGrid.end(); triElement++, bElement++)
			{
				gridPtr->InitFromTrimesh(*triElement, false);
				gridPtr->SetName("gridElement");
				gridPtr->SetTransform(m_tGridTransformation);
				env->Add(gridPtr, true);
				if( env->CheckCollision(KinBodyConstPtr(gridPtr), m_cprCollistionReport) ){
					*bElement = true;
					sout << true;
				} else {
					*bElement = false;
					sout << false;
				}
				env->Remove(gridPtr);
			}
			env->Destroy();
		}
		
		return true;
	}
	
	bool Render(std::ostream& sout, std::istream& sinput)
	{
		std::vector<RaveVector<float> > vpointsFree;
		std::vector<RaveVector<float> > vpointsOccupied;

		std::vector<int>::iterator bElement = occupancyGrid.begin();
		for(std::vector<KinBody::Link::TRIMESH>::iterator triElement = meshGrid.begin();
			triElement != meshGrid.end();
			triElement++, bElement++)
		{
			if(!*bElement){
				vpointsFree.push_back(triElement->vertices[0]);
				vpointsFree.push_back(triElement->vertices[1]);

				vpointsFree.push_back(triElement->vertices[1]);
				vpointsFree.push_back(triElement->vertices[2]);

				vpointsFree.push_back(triElement->vertices[2]);
				vpointsFree.push_back(triElement->vertices[3]);

				vpointsFree.push_back(triElement->vertices[3]);
				vpointsFree.push_back(triElement->vertices[0]);
			} else {
				vpointsOccupied.push_back(triElement->vertices[0]);
				vpointsOccupied.push_back(triElement->vertices[1]);

				vpointsOccupied.push_back(triElement->vertices[1]);
				vpointsOccupied.push_back(triElement->vertices[2]);

				vpointsOccupied.push_back(triElement->vertices[2]);
				vpointsOccupied.push_back(triElement->vertices[3]);

				vpointsOccupied.push_back(triElement->vertices[3]);
				vpointsOccupied.push_back(triElement->vertices[0]);
			}
		}

		if( vpointsFree.size() > 0) {
			m_ghpGraphGeometryFree = GetEnv()->drawlinelist( &vpointsFree[0].x, vpointsFree.size(), sizeof(vpointsFree[0]), m_fLineWidth, Vector(0,0,0,0.2));
			if( !!m_ghpGraphGeometryFree ) {
				m_ghpGraphGeometryFree->SetTransform(m_tGridTransformation);
			}
		}

		if( vpointsOccupied.size() > 0) {
			m_ghpGraphGeometryOccupied 	= GetEnv()->drawlinelist( &vpointsOccupied[0].x, vpointsOccupied.size(), sizeof(vpointsOccupied[0]), m_fLineWidth, Vector(0,0,0,0.8));
			if( !!m_ghpGraphGeometryOccupied ) {
				m_ghpGraphGeometryOccupied->SetTransform(m_tGridTransformation);
			}
		}
		return true;
	}
	
	void printGrid(){

		for(unsigned int i=0; i<occupancyGrid.size(); i++) {
			std::cout << (occupancyGrid[i] ? 0 : 1);
			std::cout << std::endl;
		}
	}

	void generateGrid(){
		generateOccupacyGrid();
		generateMeshGrid();
	}

	void generateOccupacyGrid(){
		occupancyGrid.clear();

		for(unsigned int x=0; x<m_uiWidth; x++){
			for(unsigned int y=0; y<m_uiHeight; y++){
				occupancyGrid.push_back(0);
			}
		}
	}

	void generateMeshGrid(){
		meshGrid.clear();

		KinBody::Link::TRIMESH element;

		element.vertices.push_back(Vector(0,0,0));
		element.vertices.push_back(Vector(0,1,0));
		element.vertices.push_back(Vector(1,1,0));
		element.vertices.push_back(Vector(1,0,0));

		element.indices.push_back(0);
		element.indices.push_back(1);
		element.indices.push_back(2);
		element.indices.push_back(0);
		element.indices.push_back(2);
		element.indices.push_back(3);

		for(unsigned int x=0; x<m_uiWidth; x++){
			for(unsigned int y=0; y<m_uiHeight; y++){
				element.vertices[0] = Vector(x*m_dResolution,y*m_dResolution,0);
				element.vertices[1] = Vector(x*m_dResolution,y*m_dResolution+m_dResolution,0);
				element.vertices[2] = Vector(x*m_dResolution+m_dResolution,y*m_dResolution+m_dResolution,0);
				element.vertices[3] = Vector(x*m_dResolution+m_dResolution,y*m_dResolution,0);

				meshGrid.push_back(element);
			}
		}
	}
};


#endif
