#include "situated_sensor.h"

#include "../../distance_sensor/src/distancesensor.h"

#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"

#ifndef SITUATED_DISTANCE_SENSOR__
#define SITUATED_DISTANCE_SENSOR__

//////////////////////////////////////////////////////////////////////////////////////
class SituatedDistanceSensor : public DistanceSensor, public SituatedSensor
{
public:

	class SituatedDistanceXMLReader : public DistanceSensor::DistanceXMLReader, public SituatedSensor::SituatedXMLReader
	{
	public:
		SituatedDistanceXMLReader(boost::shared_ptr<SituatedDistanceSensor> psensor) : DistanceSensor::DistanceXMLReader(boost::dynamic_pointer_cast<SituatedDistanceSensor>(psensor)), SituatedSensor::SituatedXMLReader(boost::dynamic_pointer_cast<SituatedDistanceSensor>(psensor)) {
			m_usProcessing = 0;
		}

		ProcessElement startElement(const std::string& name, const AttributesList& atts) {

			if(	DistanceSensor::DistanceXMLReader::startElement(name, atts) ) {
				m_usProcessing = 1;
				return PE_Support;
			}
			else if(SituatedSensor::SituatedXMLReader::startElement(name, atts)) {
				m_usProcessing = 2;
				return PE_Support;
			}

			return PE_Ignore;
		}

		bool endElement(const std::string& name) {

			if( m_usProcessing == 1 )
				return DistanceSensor::DistanceXMLReader::endElement(name);
			else if( m_usProcessing == 2 )
				return SituatedSensor::SituatedXMLReader::endElement(name);

			return true;
		}

		void characters(const std::string& ch) {
			if( m_usProcessing == 1 )
				DistanceSensor::DistanceXMLReader::characters(ch);
			else if( m_usProcessing == 2 )
				SituatedSensor::SituatedXMLReader::characters(ch);
			//else
				//_pcurreader->characters(ch);
		}

	private:
		unsigned char m_usProcessing;
	};


	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts) {
		return BaseXMLReaderPtr(new SituatedDistanceXMLReader(boost::dynamic_pointer_cast<SituatedDistanceSensor>(ptr)));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	SituatedDistanceSensor(EnvironmentBasePtr penv, std::istream& ss) : DistanceSensor(penv, ss), SituatedSensor(penv, ss) {
		_pdata->positions.resize(2);
		_pdata->ranges.resize(2);
		_pdata->intensity.resize(2);
		
		RegisterCommand("set_real_distance", boost::bind(&SituatedDistanceSensor::_SetDistance,this,_1,_2), "set the distance of the real measurement ... ");
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetDistance(std::ostream& sout, std::istream& sinput) {
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
				m_sSubscriber = _ros->subscribe(this->m_strRosTopic.c_str(), 10, &SituatedDistanceSensor::callback, this);
				break;
			case CC_PowerOff:
				m_sSubscriber.shutdown();
				break;
			default:
				break;
		}

		return DistanceSensor::Configure(command, blocking);
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
		if( m_bRenderData ) {
			if(_pgeom->m_bMeshSensor) {
				m_ghpGraphGeometry2 = RenderMesh(scaleTriMesh(_pgeom->mesh, distance), this->m_vRealColor);
			} else {
				std::vector<RaveVector<float> > vpoints;
				vpoints.push_back(Vector(0,1,0)*_pgeom->m_dMinRange);
				vpoints.push_back(Vector(0,1,0)*distance);
				m_ghpGraphGeometry2 = RenderRay(vpoints, this->m_vRealColor);
			}

			if( !!m_ghpGraphGeometry2 ) {
				m_ghpGraphGeometry2->SetTransform(GetTransform());
			}
		}

		//_pdata->__trans = GetTransform();
		//_pdata->__stamp = msg->header.stamp;
		_pdata->positions[1] = GetTransform().trans;
		_pdata->ranges[1] = Vector(0,1,0)*distance;
		_pdata->intensity[1] = 1;
	}

public:

    //RealMeasurement m_dRealMeasurement;
    GraphHandlePtr m_ghpGraphGeometry2;

    friend class SituatedDistanceXMLReader;
};

#endif //SITUATED_DISTANCE_SENSOR__
