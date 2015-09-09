#include <openrave/plugin.h>
#include <openrave/sensor.h>
#include <boost/bind.hpp>

#include <string>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <string>
#include <iostream>
#include <math.h>

using namespace OpenRAVE;

#ifndef TRACE__
#define TRACE__

class Trace: public SensorBase
{
public:

	class OPENRAVE_API TraceGeomData : public SensorGeometry
	{
	public:
		TraceGeomData() {
			m_iLength	= 0;
			m_dTimeScan	= 0;
			m_dTimeIncrement= 0;

			m_dResolution = 0.1;
			m_dThick = 1;

			m_vColor.x = 1;
			m_vColor.y = 0;
			m_vColor.z = 0;
			m_vColor.w = 1;
		}
		virtual SensorType GetType() {
			return ST_Laser;
		}
		dReal m_iLength;
		dReal m_dTimeScan;
		dReal m_dTimeIncrement;
		dReal m_dThick;

		RaveVector<float> m_vColor;
		
		dReal m_dResolution;
	};

	class TraceXMLReader : public virtual BaseXMLReader
	{
		public:
		TraceXMLReader(boost::shared_ptr<Trace> trace) : _trace(trace) {}

		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
			if( !!_pcurreader ) {
				if( _pcurreader->startElement(name,atts) == PE_Support )
					return PE_Support;
				return PE_Ignore;
			}

			static boost::array<std::string, 9> tags = { {"sensor",
									"length",
									"color",
									"time_scan",
									"scantime",
									"resolution",
									"thick",
									"time_increment",
									"power"}};

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
			else if( name == "sensor" ) {
				return true;
			}
			else if( name == "power" ) {
				ss >> _trace->m_bPower;
			}
			else if(name == "length") {
				ss >> _trace->_pgeom->m_iLength;
			}
			else if((name == "scantime")||(name == "time_scan")) {
				ss >> _trace->_pgeom->m_dTimeScan;
			}
			else if( name == "time_increment" ) {
				ss >> _trace->_pgeom->m_dTimeIncrement;
			}
			else if(name == "resolution") {
				ss >> _trace->_pgeom->m_dResolution;
			}
			else if(name == "thick") {
				ss >> _trace->_pgeom->m_dThick;
			}
			else if( name == "color" ) {
				ss >> _trace->_pgeom->m_vColor.x >> _trace->_pgeom->m_vColor.y >> _trace->_pgeom->m_vColor.z >> _trace->_pgeom->m_vColor.w;
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
		boost::shared_ptr<Trace> _trace;
		std::stringstream ss;
	};

public:

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts) {
		return BaseXMLReaderPtr(new TraceXMLReader(boost::dynamic_pointer_cast<Trace>(ptr)));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	Trace(EnvironmentBasePtr penv, std::istream& ss) : SensorBase(penv) {
		__description = ":Interface Author: André Dietrich\n\n\
		Provides a simulated Trace sensor. Includes the following XML parameters:\n\
		* minrange - minimal sensor range\n\
		* maxrange - maximal sensor range\n\
		* mesh - file defining the sensorbeam\n\
		* resolution - defining the minimal resolution of the sensor\n\
		* scantime - \n\
		* color - \n\
		* time_scan  - \n\
		* time_increment - \n\
		* power - \n\
		* \n";

		RegisterCommand("render",boost::bind(&Trace::_Render,this,_1,_2), "draw the trace manually.");
		RegisterCommand("push_back",boost::bind(&Trace::_PushBack,this,_1,_2), "Push back a transformation.");
		RegisterCommand("erase",boost::bind(&Trace::_Erase,this,_1,_2), "Erase Element.");
		RegisterCommand("set_length",boost::bind(&Trace::_SetLength,this,_1,_2), "");
		RegisterCommand("get_length",boost::bind(&Trace::_GetLength,this,_1,_2), "");

		RegisterCommand("set_color",boost::bind(&Trace::_SetColor,this,_1,_2), "");
		RegisterCommand("get_color",boost::bind(&Trace::_GetColor,this,_1,_2), "");

		RegisterCommand("set_thick",boost::bind(&Trace::_SetThick,this,_1,_2), "");
		RegisterCommand("get_thick",boost::bind(&Trace::_GetThick,this,_1,_2), "");

		RegisterCommand("get_points",boost::bind(&Trace::_GetPoints,this,_1,_2), "");

		_pdata.reset(new LaserSensorData());
		_pgeom.reset(new TraceGeomData());

		m_dTimeToScan = _pgeom->m_dTimeScan;
		m_bPower = false;
		m_bRenderData = true;
		m_bRenderGeometry = false;
	}

	virtual ~Trace() {}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _Render(std::ostream& sout, std::istream& sinput) {

		if(m_vPoints.size()>1) {
			m_ghpGraphGeometry = GetEnv()->drawlinestrip( &m_vPoints[0].x, m_vPoints.size(), sizeof(m_vPoints[0]), _pgeom->m_dThick, _pgeom->m_vColor);
			return true;
		}

		return false;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _PushBack(std::ostream& sout, std::istream& sinput) {

		RaveVector<float> p;

		sinput >> p.x;
		sinput >> p.y;
		sinput >> p.z;

		m_vPoints.push_back(p);

		if(m_vPoints.size()> _pgeom->m_iLength){
			m_vPoints.erase( m_vPoints.begin() );
		}

		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _Erase(std::ostream& sout, std::istream& sinput) {

		std::vector<RaveVector<float> >::iterator p;
		int elem=0;

		sinput >> elem;

		p += elem;

		m_vPoints.erase( p );

		return true;
	}

	bool _GetPoints(std::ostream& sout, std::istream& sinput) {
		for(unsigned int i=0; i<m_vPoints.size(); i++){
			sout << m_vPoints[i].x << " ";
			sout << m_vPoints[i].y << " ";
			sout << m_vPoints[i].z << " ";
		}
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetLength(std::ostream& sout, std::istream& sinput) {

		sinput >> _pgeom->m_iLength;

		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _GetLength(std::ostream& sout, std::istream& sinput) {

		sout << _pgeom->m_iLength;
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetColor(std::ostream& sout, std::istream& sinput) {

		sinput >> _pgeom->m_vColor.x;
		sinput >> _pgeom->m_vColor.y;
		sinput >> _pgeom->m_vColor.z;
		sinput >> _pgeom->m_vColor.w;

		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _GetColor(std::ostream& sout, std::istream& sinput) {

		sout << _pgeom->m_vColor.x << _pgeom->m_vColor.y << _pgeom->m_vColor.z << _pgeom->m_vColor.w;
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetThick(std::ostream& sout, std::istream& sinput) {

		sinput >> _pgeom->m_dThick;
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _GetThick(std::ostream& sout, std::istream& sinput) {

		sout << _pgeom->m_dThick;
		return true;
	}

	double _distance(RaveVector<float> u, RaveVector<float> v){
		return sqrt(pow(u.x-v.x, 2) + pow(u.y-v.y, 2) + pow(u.z-v.z, 2));
	}

	bool SimulationStep(dReal dTimeElapsed)
	{
		m_dTimeToScan -= dTimeElapsed;

		if( m_bPower &&( m_dTimeToScan <= 0) ) {
			m_dTimeToScan = _pgeom->m_dTimeScan;
			Transform t = GetTransform();
			// use infinite vector
			this->m_vPoints.push_back(t.trans);

			if(m_vPoints.size()>1){
				if( _distance(m_vPoints[m_vPoints.size()-1] , m_vPoints[m_vPoints.size()-2]) <= _pgeom->m_dResolution) {
					this->m_vPoints.pop_back();
				}
			}

			if(m_vPoints.size()> _pgeom->m_iLength){
				m_vPoints.erase( m_vPoints.begin() );
			}

			m_ghpGraphGeometry = GetEnv()->drawlinestrip( &m_vPoints[0].x, m_vPoints.size(), sizeof(m_vPoints[0]), _pgeom->m_dThick, _pgeom->m_vColor);
		}

		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual int Configure(ConfigureCommand command, bool blocking) {
		switch(command) {
			case CC_PowerOn:
				m_bPower = true;
				return m_bPower;
			case CC_PowerOff:
				m_bPower = false;
				return m_bPower;
			case CC_PowerCheck:
				return m_bPower;
			case CC_RenderDataOn:
				m_bRenderData = true;
				return m_bRenderData;
			case CC_RenderDataOff: {
				boost::mutex::scoped_lock lock(m_mMutexdata);
				m_ghpGraphGeometry.reset();
				m_bRenderData = false;
				return m_bRenderData;
			}
			case CC_RenderDataCheck:
				return m_bRenderData;
			case CC_RenderGeometryOn:
			case CC_RenderGeometryOff:
			case CC_RenderGeometryCheck:
				return false;
		}
		throw openrave_exception(str(boost::format("SensorBase::Configure: unknown command 0x%x")%command));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual SensorGeometryPtr GetSensorGeometry(SensorType type) {
		if( type == ST_Invalid || type == ST_Laser ) {
			TraceGeomData* pgeom = new TraceGeomData();
			*pgeom = *_pgeom;
			return SensorGeometryPtr(pgeom);
		}
		return SensorGeometryPtr();
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual SensorDataPtr CreateSensorData(SensorType type) {
		if( type == ST_Invalid || type == ST_Laser ) {
			return SensorDataPtr(new LaserSensorData());
		}
		return SensorDataPtr();
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual bool GetSensorData(SensorDataPtr tracedata) {
		if( tracedata->GetType() == ST_Laser ) {
			boost::mutex::scoped_lock lock(m_mMutexdata);
			*boost::dynamic_pointer_cast<LaserSensorData>(tracedata) = *_pdata;
			return true;
		}
		return false;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual bool Supports(SensorType type) {
		return type == ST_Laser;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual void SetTransform(const Transform& trans) {
		m_tTransformation = trans;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	virtual Transform GetTransform() {
		return m_tTransformation;
	}

public:
	Transform m_tTransformation;
	boost::shared_ptr<TraceGeomData> _pgeom;
	boost::shared_ptr<LaserSensorData> _pdata;

	dReal m_dTimeToScan;
	boost::mutex m_mMutexdata;
	bool m_bRenderData, m_bRenderGeometry, m_bPower;
	GraphHandlePtr m_ghpGraphGeometry;
	
	std::vector<RaveVector<float> > m_vPoints;

	friend class TraceXMLReader;
};

#endif //TRACE__
