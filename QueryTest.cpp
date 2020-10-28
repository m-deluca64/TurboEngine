//EnvironmentQuery is the base class for construction EQS tests
struct EnvironmentQuery
{
  Double cost; //The estimated cost of running this test
  Double min;  //The minimum range of this test
  Double max;  //The maximum range of this test
  Double weight; //The weight to scale the final score with
  
  enum class ScoreType
  {
    Linear,
    Sqr,
    SqrRoot,
    Sigmoid,
    Sine,
    Cos,
  };
  ScoreType type;    //the type of scoring to use 
  
  EnvironmentQuery() //... an initializer ctor
  virtual ~EnvironmentQuery() = default;

  //test performs a specific test, to be implemented in a child class, e.g. DistanceTest  
  virtual bool test(TerrainNode &node, QueryOptions &options) = 0;
  
  //score simply takes the given nodes "value" and transforms it to the specified scoring function (linear, sin, sigmoid, etc)
  bool score(TerrainNode& node, QueryOptions& options);

  //helper to clamp a score to internal range
  Double clampScoreToRange(Double value);
  RTTR_ENABLE()
};

//An example (modified for readabillity) of a example test
//OtherTankDot takes in all other tanks on the map, averages their rotations, 
//then performs a dot product against the agent's rotation
bool OtherTankDotTest::test(TerrainNode& node, QueryOptions& options)
{
  //Get agent info relative to node
  Vector3 averageRotation(0.f);
  Vector3 agentToPoint = /*normalized vector of agent -> node */
  float agentToPointSqr = /*length squared of above vector (before normalizing)*/ 

  //grab a list of all available tanks currently spawned
  Array<UInt32> tanks = AISystem::instance()->get_tanks();
  if(tanks.empty())
  {
	//boolean return, this will cull this node from further testing
    return false;
  }

  //for each tank avilable
  int numTanks = 0;
  for(auto const &object : tanks)
  {
	//skip self
    if (object == options.agent){  continue;}
	
	//skip any tanks outside of the scanned area
    if((options.radius * options.radius) < (agentToPointSqr)
    {
      continue;
    }
	
	//otherwise, add their forward to the average rotation
	++numTanks;
    Vector3 forward = object->transform->transform().Forward();
    forward.Normalize();
    averageRotation += forward;
  }

  if(numTanks <= 0)
  {
    return false;
  }
  
  //average the rotation based on number of tanks sampled
  averageRotation /= static_cast<Float>(numTanks);

  //Get dot product of the average rotation, 
  //transform it from [-1,1] range to [0, 1] 
  //set as score, return true, valid node
  Float dot = agentToPoint.Dot(averageRotation);
  dot += 1.f;
  dot *= 0.5f;
  node.score = dot;

  return true;
}