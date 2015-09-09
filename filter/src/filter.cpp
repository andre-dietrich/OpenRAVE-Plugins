#include "occupancyGridMap.h"
#include "sensorGridMap.h"
#include "rangeGridMap.h"
#include "sensorCubeMap.h"

#include "occupancyCubeMap.h"
#include "quadtree.h"
#include "octomap.h"

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
	if( type == PT_Module && interfacename == "occupancygridmap" ) {
		return InterfaceBasePtr(new OccupancyGridMap(penv,sinput));
	}

	if( type == PT_Module && interfacename == "sensorgridmap" ) {
		return InterfaceBasePtr(new SensorGridMap(penv,sinput));
	}

	if( type == PT_Module && interfacename == "sensorcubemap" ) {
		return InterfaceBasePtr(new SensorCubeMap(penv,sinput));
	}

	if( type == PT_Module && interfacename == "rangegridmap" ) {
		return InterfaceBasePtr(new RangeGridMap(penv,sinput));
	}

	if( type == PT_Module && interfacename == "occupancycubemap" ) {
		return InterfaceBasePtr(new OccupancyCubeMap(penv,sinput));
	}

	if( type == PT_Module && interfacename == "quadtree" ) {
		return InterfaceBasePtr(new Quadtree(penv,sinput));
	}

	if( type == PT_Module && interfacename == "octomap" ) {
		return InterfaceBasePtr(new Octomap(penv,sinput));
	}

	return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info) {
	info.interfacenames[PT_Module].push_back("occupancygridmap");
	info.interfacenames[PT_Module].push_back("sensorgridmap");
	info.interfacenames[PT_Module].push_back("rangegridmap");
	info.interfacenames[PT_Module].push_back("occupancycubemap");
	info.interfacenames[PT_Module].push_back("sensorcubemap");
	info.interfacenames[PT_Module].push_back("quadtree");

	info.interfacenames[PT_Module].push_back("octomap");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() {}

