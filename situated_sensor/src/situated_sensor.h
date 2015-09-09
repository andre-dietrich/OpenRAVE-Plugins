#include <openrave/plugin.h>
#include <openrave/sensor.h>
#include <boost/bind.hpp>

#include "ros/ros.h"

#include <string>
#include <vector>

using namespace OpenRAVE;

#ifndef SITUATED_SENSOR__
#define SITUATED_SENSOR__
//////////////////////////////////////////////////////////////////////////////////////
class SituatedSensor
{
public:
	class SituatedXMLReader : public virtual BaseXMLReader
	{
		public:
		SituatedXMLReader(boost::shared_ptr<SituatedSensor> psensor) : _psensor(psensor) {}

		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
			if( !!_pcurreader ) {
				if( _pcurreader->startElement(name,atts) == PE_Support )
					return PE_Support;
				return PE_Ignore;
			}

			static boost::array<std::string, 2> tags = { { "topic",	"realcolor" }};

			if( find(tags.begin(),tags.end(),name) == tags.end() ) {
				return PE_Pass;
			}
			ss.str("");
			return PE_Support;
		}

		virtual bool endElement(const std::string& name)
		{
			if( !!_pcurreader ) {
				if( _pcurreader->endElement(name) ) {
					_pcurreader.reset();
				}
				return false;
			}
			else if( name == "topic" ) {
				ss >> _psensor->m_strRosTopic;
			}
			else if( name == "realcolor" ) {
				ss >> _psensor->m_vRealColor.x >> _psensor->m_vRealColor.y >> _psensor->m_vRealColor.z >> _psensor->m_vRealColor.w;
				// ok if not everything specified
				if( !ss ) {
					ss.clear();
				}
			}
			else {
				RAVELOG_WARN(str(boost::format("bad tag: %s")%name));
			}
			if( !ss ) {
				RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
			}

			return false;
		}

		virtual void characters(const std::string& ch)
		{
			if( !!_pcurreader )
				_pcurreader->characters(ch);
			else {
				ss.clear();
				ss << ch;
			}
		}

		protected:
			BaseXMLReaderPtr _pcurreader;
			boost::shared_ptr<SituatedSensor> _psensor;
			std::stringstream ss;
		};

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	SituatedSensor(EnvironmentBasePtr penv, std::istream& ss) {// : SensorBase(penv) {

		m_vRealColor.x = 0;
		m_vRealColor.y = 1;
		m_vRealColor.z = 0;
		m_vRealColor.w = 0.25;

		int argc=0;

		ros::init(argc,NULL,"situatedsensor", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

		if( !ros::master::check() ) {
			RAVELOG_WARN("failed to create ros\n");
		}
		_ros.reset(new ros::NodeHandle());

		_threadros = boost::thread(boost::bind(&SituatedSensor::_threadrosfn, this));
	}

	virtual ~SituatedSensor() {}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual void _threadrosfn() {
		while(ros::ok()) {
			ros::spinOnce();
			usleep(10000); // query every 1ms?
		}
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
public:

	std::string m_strRosTopic;
	Vector m_vRealColor;
	ros::Subscriber m_sSubscriber;
	boost::shared_ptr<ros::NodeHandle> _ros;
	boost::thread _threadros;

	friend class SituatedSensorXMLReader;
};

#endif //SITUATED_SENSOR__
