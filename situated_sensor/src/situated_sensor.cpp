#include "situated_distancesensor.h"
//#include "situated_camerasensor.h"


#include <list>

static std::list< boost::shared_ptr<void> >* s_listRegisteredReaders=NULL;
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
	if( !s_listRegisteredReaders ) {
		s_listRegisteredReaders = new std::list< boost::shared_ptr<void> >();
		s_listRegisteredReaders->push_back(RaveRegisterXMLReader(PT_Sensor,"sit_distance",SituatedDistanceSensor::CreateXMLReader));
//		s_listRegisteredReaders->push_back(RaveRegisterXMLReader(PT_Sensor,"sit_camera",SituatedCameraSensor::CreateXMLReader));

	}

	if( type == PT_Sensor && interfacename == "sit_distance" ) {
		return InterfaceBasePtr(new SituatedDistanceSensor(penv,sinput));
	}

//	if( type == PT_Sensor && interfacename == "sit_camera" ) {
//		return InterfaceBasePtr(new SituatedCameraSensor(penv,sinput));
//	}

	return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
	info.interfacenames[PT_Sensor].push_back("sit_distance");
//	info.interfacenames[PT_Sensor].push_back("sit_camera");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
	delete s_listRegisteredReaders;
	s_listRegisteredReaders = NULL;
}

