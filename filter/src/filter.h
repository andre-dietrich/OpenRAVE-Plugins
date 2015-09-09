#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include <vector>

#ifndef FILTER_H
#define FILTER_H

using namespace OpenRAVE;

class Filter : public ModuleBase
{
public:

	Transform m_tGridTransformation;
	
	boost::mutex m_mMutexdata;
	
	Filter(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
		RegisterCommand("SetTranslation",boost::bind(&Filter::SetTranslation,this,_1,_2), "");
		RegisterCommand("GetTranslation",boost::bind(&Filter::GetTranslation,this,_1,_2), "");
		
		RegisterCommand("SetRotation",boost::bind(&Filter::SetRotation,this,_1,_2), "");
		RegisterCommand("GetRotation",boost::bind(&Filter::GetRotation,this,_1,_2), "");
	}
    
	virtual ~Filter() {}
    
	bool SetTranslation(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_tGridTransformation.trans.x >> m_tGridTransformation.trans.y >> m_tGridTransformation.trans.z;
		return true;
	}
	
	bool GetTranslation(std::ostream& sout, std::istream& sinput)
	{
		sout <<" "<< m_tGridTransformation.trans.x <<" "<< m_tGridTransformation.trans.y <<" "<< m_tGridTransformation.trans.z;
		return true;
	}
	
	bool SetRotation(std::ostream& sout, std::istream& sinput)
	{
		sinput >> m_tGridTransformation.rot.x >> m_tGridTransformation.rot.y >> m_tGridTransformation.rot.z >> m_tGridTransformation.rot.w;
		return true;
	}
	
	bool GetRotation(std::ostream& sout, std::istream& sinput)
	{
		sout << m_tGridTransformation.rot.x <<" "<< m_tGridTransformation.rot.y <<" "<< m_tGridTransformation.rot.z <<" "<< m_tGridTransformation.rot.w;
		return true;
	}
};

#endif
