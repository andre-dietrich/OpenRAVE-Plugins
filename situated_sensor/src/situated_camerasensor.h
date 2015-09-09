#include "situated_sensor.h"

using namespace std;

#include <openrave/plugin.h>
#include <openrave/sensor.h>
#include <basecamera.h>
#include <boost/bind.hpp>

#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"

#ifndef SITUATED_CAMERA_SENSOR__
#define SITUATED_CAMERA_SENSOR__



//////////////////////////////////////////////////////////////////////////////////////
class SituatedCameraSensor : public BaseCameraSensor, public SituatedSensor
{
public:

	class SituatedCameraXMLReader : public BaseCameraSensor::BaseCameraXMLReader
	{
	public:
		SituatedCameraXMLReader(boost::shared_ptr<SituatedCameraSensor> psensor) : BaseCameraSensor::BaseCameraXMLReader(boost::dynamic_pointer_cast<BaseCameraSensor>(psensor)) {}

		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {

			if( !!_pcurreader ) {
				if( _pcurreader->startElement(name,atts) == PE_Support )
					return PE_Support;
				return PE_Ignore;
			}

			static boost::array<std::string, 3> tags = {{"topic", "sensor", "power"}};

			if( find(tags.begin(),tags.end(),name) == tags.end() ) {
				return PE_Pass;
			}
			ss.str("");
			return PE_Support;
		}

		bool endElement(const std::string& name) {

			std::cout << "endElement " <<name  << "\n";

			if( !!_pcurreader ) {
				if( _pcurreader->endElement(name) ) {
					_pcurreader.reset();
				}
				return false;
			}

			else if( name == "sensor" ) {
				return true;
			}

			else if( name == "power" ) {
				ss >> _psensor->m_bPower;
			}

			else if( name == "topic" ) {
				ss >> _psensor->m_strRosTopic;
			}
			else {
				RAVELOG_WARN(str(boost::format("bad tag: %s")%name));
			}
			if( !ss ) {
				RAVELOG_WARN(str(boost::format("XXXXXXXXXXXXXXXXXXxerror parsing %s\n")%name));
			}

			return false;
		}

		void characters(const std::string& ch) {
			if( !!_pcurreader )
				_pcurreader->characters(ch);
			else {
				ss.clear();
				ss << ch;
			}
		}

	private:
		boost::shared_ptr<SituatedCameraSensor> _psensor;
	};


	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts) {
		return BaseXMLReaderPtr(new SituatedCameraXMLReader(boost::dynamic_pointer_cast<SituatedCameraSensor>(ptr)));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	SituatedCameraSensor(EnvironmentBasePtr penv, std::istream& ss) : BaseCameraSensor(penv), SituatedSensor(penv, ss) {
		RegisterCommand("set_real_image", boost::bind(&SituatedCameraSensor::_SetImage,this,_1,_2), "set the real camera image ... ");

		m_bPower = false;


	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetImage(std::ostream& sout, std::istream& sinput) {
		double distance;
		sinput >> distance;
		render(distance);
		return true;
	}


	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual int Configure(ConfigureCommand command, bool blocking) {
		switch(command) {
			case CC_PowerOn:
				m_sSubscriber = _ros->subscribe(this->m_strRosTopic.c_str(), 10, &SituatedCameraSensor::callback, this);
				break;
			case CC_PowerOff:
				m_sSubscriber.shutdown();
				break;
			default:
				break;
		}

		return SituatedCameraSensor::Configure(command, blocking);
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	void callback(const sensor_msgs::Range::ConstPtr& msg) {
		 render(msg->range);
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	void render(double distance){
		//if( m_bRenderData ) {
			std::cout << "render render\n";
		//}
	}

public:

    //RealMeasurement m_dRealMeasurement;
    GraphHandlePtr m_ghpGraphGeometry2;
    bool m_bPower;
    friend class SituatedCameraXMLReader;
};

#endif //SITUATED_CAMERA_SENSOR__
