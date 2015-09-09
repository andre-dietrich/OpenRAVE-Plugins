#include "filter.h"
#include <iostream>
//using namespace OpenRAVE;

#ifndef QUADTREE_H
#define QUADTREE_H

class Quadtree : public Filter
{
public:
	double m_dWidth;
	double m_dHeight;
	unsigned int m_uiSteps;

	float m_fLineWidth;

	GraphHandlePtr m_ghpGraphGeometryQuadtree;
	CollisionReportPtr m_cprCollistionReport;

	std::vector<KinBody::Link::TRIMESH> vQuadtree;

	Quadtree(EnvironmentBasePtr penv, std::istream& ss) : Filter(penv, ss) {
		RegisterCommand("SetSize",boost::bind(&Quadtree::SetSize,this,_1,_2), "Set the size of the grid: (float)width, (float)height, (int)steps ");
		RegisterCommand("GetSize",boost::bind(&Quadtree::GetSize,this,_1,_2), "Return the current size of the grid: (float)width, (float)height, (int)steps");
		RegisterCommand("Scan",boost::bind(&Quadtree::Scan,this,_1,_2), "Makes a 2D scan of the current Environment");
		RegisterCommand("Render",boost::bind(&Quadtree::Render,this,_1,_2), "Renders the current Quadtree");
		RegisterCommand("SetLineWidth",boost::bind(&Quadtree::SetLineWidth,this,_1,_2), "Set the Line Width");
		RegisterCommand("GetLineWidth",boost::bind(&Quadtree::GetLineWidth,this,_1,_2), "Get the Line Width");
		
		m_dWidth  = 10;
		m_dHeight = 10;
		m_uiSteps = 10;
		m_fLineWidth = 2.0;

		intiMap();

		m_cprCollistionReport.reset(new CollisionReport());
	}

	virtual ~Quadtree() {}

	void intiMap(){
		vQuadtree.clear();

		KinBody::Link::TRIMESH border;

		border.vertices.push_back(Vector(0        , 0       , 0));
		border.vertices.push_back(Vector(0        , m_dWidth, 0));
		border.vertices.push_back(Vector(m_dHeight, m_dWidth, 0));
		border.vertices.push_back(Vector(m_dHeight,        0, 0));

		border.indices.push_back(0);
		border.indices.push_back(1);
		border.indices.push_back(2);
		border.indices.push_back(0);
		border.indices.push_back(2);
		border.indices.push_back(3);

		vQuadtree.push_back(border);
	}
	
	bool SetSize(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_dWidth;
		sinput >> m_dHeight;
		sinput >> m_uiSteps;

		intiMap();

		return true;
	}
	
	bool GetSize(std::ostream& sout, std::istream& sinput)
	{
		sout << m_dWidth << " " << m_dHeight << " " << m_uiSteps;
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
		intiMap();
		EnvironmentBasePtr env = GetEnv()->CloneSelf(1);
		env->GetCollisionChecker()->SetCollisionOptions(CO_RayAnyHit);
		ScanX(vQuadtree[0], this->m_uiSteps, env, sout);
		env->Destroy();
		sout << ";";
		return true;
	}

	void ScanX(KinBody::Link::TRIMESH grid, unsigned int uiSteps, EnvironmentBasePtr env, std::ostream& sout){

		if(uiSteps == 0)
			return;

		sout << "( " ;

		Vector v[4];
		double dX = (grid.vertices[2][0] - grid.vertices[0][0]) / 2;
		double dY = (grid.vertices[2][1] - grid.vertices[0][1]) / 2;

		v[0] = grid.vertices[0];
		v[1] = Vector(v[0][0]+dX, v[0][1]   , v[0][2]);
		v[2] = Vector(v[0][0]+dX, v[0][1]+dY, v[0][2]);
		v[3] = Vector(v[0][0]   , v[0][1]+dY, v[0][2]);

		KinBodyPtr gridPtr = RaveCreateKinBody(env);
		gridPtr->SetName("gridElement");
		for(int i=0; i<4; i++){
			KinBody::Link::TRIMESH sub;

			sub.vertices.push_back(v[i]);
			sub.vertices.push_back(Vector(v[i][0]+dX, v[i][1]   , v[i][2]));
			sub.vertices.push_back(Vector(v[i][0]+dX, v[i][1]+dY, v[i][2]));
			sub.vertices.push_back(Vector(v[i][0]   , v[i][1]+dY, v[i][2]));

			sub.indices.push_back(0);
			sub.indices.push_back(1);
			sub.indices.push_back(2);
			sub.indices.push_back(0);
			sub.indices.push_back(2);
			sub.indices.push_back(3);

			sout << v[i][0]+dX/2 << "_" << v[i][1]+dY/2;

			if(i < 3)
				sout << ", " ;

			vQuadtree.push_back(sub);

			gridPtr->InitFromTrimesh(sub, false);
			gridPtr->SetTransform(m_tGridTransformation);
			env->Add(gridPtr, true);

			if( env->CheckCollision(KinBodyConstPtr(gridPtr), m_cprCollistionReport) ){
				env->Remove(gridPtr);
				ScanX(sub, uiSteps-1, env, sout);
			}
			else{
				env->Remove(gridPtr);
			}
		}

		sout << ")" ;
	}

	bool Render(std::ostream& sout, std::istream& sinput)
	{
		std::vector<RaveVector<float> > vPoints;

		for(std::vector<KinBody::Link::TRIMESH>::iterator triElement = vQuadtree.begin();
				triElement != vQuadtree.end();
				triElement++)
		{
			vPoints.push_back(triElement->vertices[0]);
			vPoints.push_back(triElement->vertices[1]);

			vPoints.push_back(triElement->vertices[1]);
			vPoints.push_back(triElement->vertices[2]);

			vPoints.push_back(triElement->vertices[2]);
			vPoints.push_back(triElement->vertices[3]);

			vPoints.push_back(triElement->vertices[3]);
			vPoints.push_back(triElement->vertices[0]);
		}

		m_ghpGraphGeometryQuadtree = GetEnv()->drawlinelist( &vPoints[0].x, vPoints.size(), sizeof(vPoints[0]), m_fLineWidth, Vector(0,0,0,0.2));
		if( !!m_ghpGraphGeometryQuadtree ) {
			m_ghpGraphGeometryQuadtree->SetTransform(m_tGridTransformation);
		}


		return true;
	}
};

#endif
