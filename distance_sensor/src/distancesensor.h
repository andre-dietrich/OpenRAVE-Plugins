#include <openrave/plugin.h>
#include <openrave/sensor.h>
#include <boost/bind.hpp>

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>
#include <iostream>
#include <math.h>

using namespace OpenRAVE;

#ifndef DISTANCE_SENSOR__
#define DISTANCE_SENSOR__

class DistanceSensor : public SensorBase
{
public:

	class OPENRAVE_API DistanceGeomData : public SensorGeometry
	{
	public:
		DistanceGeomData() {
			m_dMaxRange 	= 0;
			m_dMinRange		= 0;
			m_dTimeScan		= 0;
			m_dTimeIncrement= 0;

			m_dResolution = 0.01;
			
			m_bMeshSensor = false;
			
			m_vColor.x = 1;
			m_vColor.y = 0;
			m_vColor.z = 0;
			m_vColor.w = 0.5;
		}
		virtual SensorType GetType() {
			return ST_Laser;
		}
		dReal m_dMaxRange;
		dReal m_dMinRange;
		dReal m_dTimeScan;
		dReal m_dTimeIncrement;

		RaveVector<float> m_vColor;
		
		bool m_bMeshSensor;
		
		boost::shared_ptr< KinBody::Link::TRIMESH > mesh;
		dReal m_dResolution;
	};

	class DistanceXMLReader : public virtual BaseXMLReader
	{
		public:
		DistanceXMLReader(boost::shared_ptr<DistanceSensor> psensor) : _psensor(psensor) {}

		virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
			if( !!_pcurreader ) {
				if( _pcurreader->startElement(name,atts) == PE_Support )
					return PE_Support;
				return PE_Ignore;
			}

			static boost::array<std::string, 13> tags = { { "sensor",
									"maxrange",
									"max_range",
									"minrange",
									"min_range",
									"scantime",
									"color",
									"time_scan",
									"mesh",
									"resolution",
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
				ss >> _psensor->m_bPower;
			}
			else if((name == "maxrange")||(name == "max_range")) {
				ss >> _psensor->_pgeom->m_dMaxRange;
			}
			else if((name == "minrange")||(name == "min_range")) {
				ss >> _psensor->_pgeom->m_dMinRange;
			}
			else if((name == "scantime")||(name == "time_scan")) {
				ss >> _psensor->_pgeom->m_dTimeScan;
			}
			else if( name == "time_increment" ) {
				ss >> _psensor->_pgeom->m_dTimeIncrement;
			}
			else if(name == "mesh") {
				std::string strFilename;
				ss >> strFilename;
				_psensor->_pgeom->mesh = _psensor->GetEnv()->ReadTrimeshFile(_psensor->_pgeom->mesh, strFilename);
				_psensor->_pgeom->m_bMeshSensor = true;
			}
			else if(name == "resolution") {
				ss >> _psensor->_pgeom->m_dResolution;
			}
			else if( name == "color" ) {
				ss >> _psensor->_pgeom->m_vColor.x >> _psensor->_pgeom->m_vColor.y >> _psensor->_pgeom->m_vColor.z;
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
		boost::shared_ptr<DistanceSensor> _psensor;
		std::stringstream ss;
	};

public:

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts) {
		return BaseXMLReaderPtr(new DistanceXMLReader(boost::dynamic_pointer_cast<DistanceSensor>(ptr)));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	DistanceSensor(EnvironmentBasePtr penv, std::istream& ss) : SensorBase(penv) {
		__description = ":Interface Author: André Dietrich\n\n\
		Provides a simulated distance sensor. Includes the following XML parameters:\n\
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

		RegisterCommand("render",boost::bind(&DistanceSensor::_Render,this,_1,_2), "Set rendering of the plots (1 or 0).");
		RegisterCommand("collidingbodies",boost::bind(&DistanceSensor::_CollidingBodies,this,_1,_2), "Returns the id of the body that the beam has hit.");
		RegisterCommand("set_distance",boost::bind(&DistanceSensor::_SetDistance,this,_1,_2), "Set the distance by hand.");
		RegisterCommand("get_distance",boost::bind(&DistanceSensor::_GetDistance,this,_1,_2), "Get the distance.");

		RegisterCommand("set_intensity",boost::bind(&DistanceSensor::_SetIntensity,this,_1,_2), "Set the intensity by hand.");
		RegisterCommand("get_intensity",boost::bind(&DistanceSensor::_GetIntensity,this,_1,_2), "Get the intensity.");

		m_cprCollistionReport.reset(new CollisionReport());

		_pdata.reset(new LaserSensorData());
		_pgeom.reset(new DistanceGeomData());

		m_dTimeToScan = 0;
        m_bPower = false;
        m_bRenderData = true;
        m_bRenderGeometry = false;

        _pdata->positions.resize(1);
        _pdata->ranges.resize(1);
        _pdata->intensity.resize(1);
	}

	virtual ~DistanceSensor() {}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _Render(std::ostream& sout, std::istream& sinput) {
		sinput >> m_bRenderData;
		return !!sinput;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _CollidingBodies(std::ostream& sout, std::istream& sinput)
	{
		boost::mutex::scoped_lock lock(m_mMutexdata);
		sout << this->m_iDatabodyId << " ";
		return true;
	}
	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetDistance(std::ostream& sout, std::istream& sinput)
	{
		double distance;
		boost::mutex::scoped_lock lock(m_mMutexdata);
		sinput >> distance;

		Transform t = GetTransform();
		Vector direction(0,1,0);

		RAY ray;

		ray.pos = t.trans;
		ray.dir = t.rotate(direction) * _pgeom->m_dMaxRange;

		_pdata->__trans = GetTransform();
		_pdata->__stamp = GetEnv()->GetSimulationTime();
		_pdata->positions[0] = t.trans;

		_pdata->ranges[0] = (1/betrag(ray.dir)) * ray.dir *(distance);

		if( m_bRenderData ) {
			if(_pgeom->m_bMeshSensor) {
				KinBody::Link::TRIMESH beam;
				beam = scaleTriMesh(_pgeom->mesh, distance);
				m_ghpGraphGeometry = RenderMesh(beam, _pgeom->m_vColor);
			} else {
				std::vector<RaveVector<float> > vpoints;
				vpoints.push_back(direction*_pgeom->m_dMinRange);
				vpoints.push_back(direction*distance);
				m_ghpGraphGeometry = RenderRay(vpoints, _pgeom->m_vColor);
			}

		}

		if( m_bRenderData ) {
			if( !!m_ghpGraphGeometry ) {
				m_ghpGraphGeometry->SetTransform(t);
			}
		}

		return true;
	}
	
	bool _GetDistance(std::ostream& sout, std::istream& sinput)
	{
		boost::mutex::scoped_lock lock(m_mMutexdata);
		sout << betrag(_pdata->ranges[0]) << " ";
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _GetIntensity(std::ostream& sout, std::istream& sinput)
	{
		boost::mutex::scoped_lock lock(m_mMutexdata);
		sout << _pdata->intensity[0];
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool _SetIntensity(std::ostream& sout, std::istream& sinput)
	{
		boost::mutex::scoped_lock lock(m_mMutexdata);
		sinput >> _pdata->intensity[0];
		return true;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	GraphHandlePtr RenderRay(std::vector<RaveVector<float> > vpoints, RaveVector<float> color){
		return GetEnv()->drawlinestrip( &vpoints[0].x, vpoints.size(),sizeof(vpoints[0]),3.0f, color);
	}
	
	GraphHandlePtr RenderMesh(KinBody::Link::TRIMESH mesh, RaveVector<float> color){
		std::vector<RaveVector<float> > vpoints;

		for(std::vector<Vector>::iterator vertice = mesh.vertices.begin();
			vertice != mesh.vertices.end(); vertice++)
		{
			vpoints.push_back(*vertice);
		}

		return GetEnv()->drawtrimesh((float*)&vpoints[0], sizeof(vpoints[0]), &mesh.indices[0], mesh.indices.size()/3, color);	
	}
	
	inline double betrag(RaveVector<float> v){
		return sqrt(pow(v[0],2) + pow(v[1],2) + pow(v[2],2));
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	void SimulateRaySensor()
	{
		RAY ray;

		Transform t = GetTransform();
		Vector direction(0,1,0);

		GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Distance);
		{
			// Lock the data mutex and fill with the range data (get all in one timestep)
			boost::mutex::scoped_lock lock(m_mMutexdata);
			ray.pos = t.trans;
			ray.dir = t.rotate(direction) * _pgeom->m_dMaxRange;

			_pdata->__trans = GetTransform();
			_pdata->__stamp = GetEnv()->GetSimulationTime();
			_pdata->positions[0] = t.trans;

			if( GetEnv()->CheckCollision(ray, m_cprCollistionReport) && m_cprCollistionReport->minDistance>= _pgeom->m_dMinRange) {

				_pdata->ranges[0] = (1/betrag(ray.dir)) * ray.dir *(m_cprCollistionReport->minDistance);
				_pdata->intensity[0] = 1;

				// store the colliding bodies
				KinBody::LinkConstPtr plink = !!m_cprCollistionReport->plink1 ? m_cprCollistionReport->plink1 : m_cprCollistionReport->plink2;

				if( !!plink ) {
					m_iDatabodyId = plink->GetParent()->GetEnvironmentId();
				}

				if( m_bRenderData ) {
					std::vector<RaveVector<float> > vpoints;
					vpoints.push_back(direction*_pgeom->m_dMinRange);
					vpoints.push_back(direction*m_cprCollistionReport->minDistance);
					m_ghpGraphGeometry = RenderRay(vpoints, _pgeom->m_vColor);
				}
			} else {

				_pdata->ranges[0] = (1/betrag(ray.dir)) * ray.dir * _pgeom->m_dMaxRange;
				_pdata->intensity[0] = 0;
				m_iDatabodyId = 0;

				if( m_bRenderData ) {
					std::vector<RaveVector<float> > vpoints;
					vpoints.push_back(direction*_pgeom->m_dMinRange);
					vpoints.push_back(direction*_pgeom->m_dMaxRange);
					m_ghpGraphGeometry = RenderRay(vpoints, _pgeom->m_vColor);
				}

			}
			if( m_bRenderData ) {
				if( !!m_ghpGraphGeometry ) {
					m_ghpGraphGeometry->SetTransform(t);
				}
			}
		GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
		}
	}
	
	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	KinBody::Link::TRIMESH scaleTriMesh(boost::shared_ptr< KinBody::Link::TRIMESH > mesh, double scale){
		KinBody::Link::TRIMESH beam;

		for(std::vector<Vector>::iterator vertice = mesh->vertices.begin();
				vertice != mesh->vertices.end(); vertice++)
		{
			beam.vertices.push_back( *vertice * scale );
		}

		for(std::vector<int>::iterator indice = mesh->indices.begin();
				indice != mesh->indices.end(); indice++)
		{
			beam.indices.push_back(*indice);
		}

		return beam;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	inline double abs (double val){
		return (val < 0) ? -val : val;
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	void SimulateMeshSensor()
	{
		Transform t = GetTransform();

		GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
		{
			
			// Lock the data mutex and fill with the range data (get all in one timestep)
			boost::mutex::scoped_lock lock(m_mMutexdata);

			_pdata->__trans = GetTransform();
			_pdata->__stamp = GetEnv()->GetSimulationTime();
			_pdata->positions[0] = t.trans;

			KinBodyPtr raveBodyPtr = RaveCreateKinBody(GetEnv());
			KinBody::Link::TRIMESH beam;

			double range_min = _pgeom->m_dMinRange;
			double range_max = _pgeom->m_dMaxRange;
			double distance  = 0;
			bool collision	 = false;

			while( abs(range_max - range_min) >= _pgeom->m_dResolution){
				distance = (range_max - range_min)/2 + range_min;

				beam = scaleTriMesh(_pgeom->mesh, distance);

				raveBodyPtr->InitFromTrimesh( beam, false);
				raveBodyPtr->SetName("sensorbeam");
				raveBodyPtr->SetTransform(t);
				GetEnv()->Add(raveBodyPtr, true);
				
				if( GetEnv()->CheckCollision(KinBodyConstPtr(raveBodyPtr), m_cprCollistionReport) ) {
					range_max = distance;
					collision = true;
					// store the colliding bodies
					KinBody::LinkConstPtr plink = !!m_cprCollistionReport->plink1 ? m_cprCollistionReport->plink2 : m_cprCollistionReport->plink1;
					if( !!plink ) {
						m_iDatabodyId = plink->GetParent()->GetEnvironmentId();
					}
				}
				else {
					range_min = distance;
				}
				GetEnv()->Remove(raveBodyPtr);
			}

			if(collision) {
				_pdata->ranges[0] = (1/betrag(t.rotate(Vector(0,1,0)))) * t.rotate(Vector(0,1,0)) *(distance);
				//_pdata->ranges[0] = Vector(0,1,0) * distance;
				_pdata->intensity[0] = 1;

			} else {
				_pdata->ranges[0] = (1/betrag(t.rotate(Vector(0,1,0)))) * t.rotate(Vector(0,1,0)) *(_pgeom->m_dMaxRange);
				//_pdata->ranges[0] = Vector(0,1,0) * _pgeom->m_dMaxRange;
				_pdata->intensity[0] = 0;
				m_iDatabodyId = 0;
			}

			if( m_bRenderData ) {

				m_ghpGraphGeometry = RenderMesh(beam, _pgeom->m_vColor);

				if( !!m_ghpGraphGeometry ) {
					m_ghpGraphGeometry->SetTransform(t);
				}
			}
		GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
		}
	}

	/*************************************************************************************************************************************
	 *
	 *************************************************************************************************************************************/
	bool SimulationStep(dReal dTimeElapsed)
	{
		m_dTimeToScan -= dTimeElapsed;
		if( m_bPower &&( m_dTimeToScan <= 0) ) {
			m_dTimeToScan = _pgeom->m_dTimeScan;

			if(_pgeom->m_bMeshSensor){
				SimulateMeshSensor();
			}
			else{
				SimulateRaySensor();
			}
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
			DistanceGeomData* pgeom = new DistanceGeomData();
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
	virtual bool GetSensorData(SensorDataPtr psensordata) {
		if( psensordata->GetType() == ST_Laser ) {
			boost::mutex::scoped_lock lock(m_mMutexdata);
			*boost::dynamic_pointer_cast<LaserSensorData>(psensordata) = *_pdata;
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
	boost::shared_ptr<DistanceGeomData> _pgeom;
	boost::shared_ptr<LaserSensorData> _pdata;

	dReal m_dTimeToScan;
	boost::mutex m_mMutexdata;
	bool m_bRenderData, m_bRenderGeometry, m_bPower;
	GraphHandlePtr m_ghpGraphGeometry;
	int m_iDatabodyId;     ///< if non 0, for each point in _data, specifies the body that was hit
	CollisionReportPtr m_cprCollistionReport;

	friend class DistanceXMLReader;
};

#endif //DISTANCE_SENSOR__
