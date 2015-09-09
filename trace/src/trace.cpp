#include "trace.h"

static std::list< boost::shared_ptr<void> >* s_listRegisteredReaders=NULL;
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_listRegisteredReaders ) {
        s_listRegisteredReaders = new std::list< boost::shared_ptr<void> >();
        s_listRegisteredReaders->push_back(RaveRegisterXMLReader(PT_Sensor,"trace",Trace::CreateXMLReader));
    }

    if( type == PT_Sensor && interfacename == "trace" ) {
    	return InterfaceBasePtr(new Trace(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
	info.interfacenames[PT_Sensor].push_back("trace");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
	delete s_listRegisteredReaders;
	s_listRegisteredReaders = NULL;
}

