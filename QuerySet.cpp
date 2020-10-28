//after tests are implemented, they are combined into layered queries
//QuerySet holds an array of tests, wrapped into a full query
struct QuerySet
{
  Array<Shared<QueryTest>> tests;
  
  //given a set of points in the world, 
  //it runs the assigned tests and outputs a "winning" point as a TerrainNode (a vector with score/meta data) 
  TerrainNode score(Array<Vector3> points, QueryOptions &options);
};

/*
At best, it runs cheap tests on each point first, sorts the points that passed the cheap test based on their max/min potential score
then iteratively runs through each test/point using a heap to determine best predicted score 

mainly relies on cheap/free tests being marked as having a 0 cost, with more expensive tests getting >0 
*/
TerrainNode QuerySet::score(Array<Vector3> points, QueryOptions &options)
{
  //sort each of the queries by cost
  std::sort(tests.begin(), tests.end(), []( Shared<QueryTest> a, Shared<QueryTest> b)->bool
    {
      return a->cost < b->cost;
    }
  );
  
  //Determine the index of the first costly test
  //These are gonna be Distance / Dot product tests 
  //that don't use outside systems (Physics) or complex logic/math
  Int32 firstCostlyTest = 0;
  Int32 finalEvalTest = tests.size() - 1;
  for(int i = 0; i < tests.size(); ++i)
  {
    if(tests[i]->cost > 0.0)
    {
      firstCostlyTest = i;
      break;
    }
  }

  //Determine the maximum score & minimum score of all tests up to the firstCostlyTest
  //Create an array of TerrainNodes and MetaPoints, which are evaluated points (from the function signature)
  Double maxOffset = maxScoreFrom(firstCostlyTest);
  Double minOffset = minScoreFrom(firstCostlyTest);
  Array<PointMetadata> nodes; //these are valid positions, which were picked from the points container
  
  //a simple compare lambda for creating a heap (priority queue) later
  auto heap_compare = [](PointMetadata const &a, PointMetadata const &b)->bool
  {
    return a.maxScore < b.maxScore;
  };
  
  //If we have "cheap" tests, perform those first
  if(firstCostlyTest != 0)
  {
	//For each point, perform each test
	//tally a running score & total weights per each test
	//if the point is still valid, create a PointMetadata from the calculated score, and add it to the nodes list
    for(auto &point : points)
    {
      Double totalScore = 0.0;
      Double weightSum = 0.0;
      TerrainNode node(point, 0.0);
      bool bValid = false;
      //perform all tests
      for(int i = 0; i < firstCostlyTest; ++i)
      {
        bValid = tests[i]->score(node, options);
        if(!bValid)
        {
          //Break on the first failure
          break;
        }
        weightSum += std::abs(tests[i]->weight);
        totalScore += node.score;
      }

      //add to the pool if it passed the tests
      if(bValid)
      {
        //final score is its sum of weighted test score, over the highest possible score
        node.score = totalScore / weightSum;
        PointMetadata data(node);
        data.evalStep = firstCostlyTest;
        data.minScore = node.score - minOffset;
        data.maxScore = node.score + maxOffset;
        nodes.push_back(data);
      }
    }
	
	//heapify nodes based on a lower max score possible. 
	//This lets us prune branches in the search for better performance
	//e.g. won't run expensive tests on "pointless" nodes
	//furthermore, there is no preference to "running all tests on a node" versus "running all nodes thru a test"
	//A min/max potential score is constantly updated on processed/valid nodes so that only the best nodes are further processed
    std::make_heap(nodes.begin(), nodes.end(), heap_compare);

	//With what nodes remain, run the remaining query tests
    while(!nodes.empty())
    {
	  //Loop checks
      PointMetadata &best = nodes[0];
      if(best.evalStep > finalEvalTest)
      {
		//this point has gone through all evaluations, and its the best possible node
        break;
      }
      if(nodes.size() == 1)
      {
        break;
      }
	  //Main body of function
      {
        //if best node's min is greater than its children's max, no need to perform test, this one wins anyway
        if( (best.minScore > nodes[1].maxScore) && (best.minScore > nodes[2].maxScore))
        {
          best.evalStep++;
          continue;
        }
        
		//otherwise, perform the test
        bool bValid = tests[best.evalStep]->score(best.node, options);
        if(!bValid)
        {
          //pop if not a valid node
          std::pop_heap(nodes.begin(), nodes.end(), heap_compare);
          nodes.pop_back();
          continue;
        }
        
		//update the nodes's best/worst score based on the recently run test
        Double normalized = best.node.score;
        Double weight = tests[best.evalStep]->weight;
        if(weight > 0)
        {
          best.minScore += (normalized / weight);
          best.maxScore -= (weight - normalized) / weight;;
        }
        else
        {
          best.minScore -= (weight - normalized) / weight;
          best.maxScore += (normalized / weight);
        }
		//also update the current evaluation step
        best.evalStep++;
		
		
        //max/min has been updated, pop/push the node to update the heap
        std::pop_heap(nodes.begin(), nodes.end(), heap_compare);
        std::push_heap(nodes.begin(), nodes.end(), heap_compare);
      }
    }
  }
  //If no cheap nodes can be determined, then we have to simply run everything
  //if we are here, costs were not properly estimated & we can't prioritize one test over the other
  
  //This is mainly a testing fallback, as in production, there was always at least 1 test marked as a 0 cost 
  else
  {
    for(auto &point : points)
    {
      Double totalScore = 0.0;
      Double weightSum = 0.0;
      TerrainNode node(point, 0.0);
      bool bValid = false;
      //perform all tests
      for(int i = 0; i < tests.size(); ++i)
      {
        bValid = tests[i]->score(node, options);
        if(!bValid)
        {
          //Break on the first failure
          break;
        }
        weightSum += std::abs(tests[i]->weight);
        totalScore += node.score;
      }

      //add to the pool if it passed the tests
      if(bValid)
      {
        //final score is its sum of weighted test score, over the highest possible score
        node.score = totalScore / weightSum;
        PointMetadata data(node);
        data.evalStep = firstCostlyTest;
        data.minScore = node.score - minOffset;
        data.maxScore = node.score + maxOffset;
        nodes.push_back(data);
      }
    }
    //sort the best at the front
    std::make_heap(nodes.begin(), nodes.end(), heap_compare);
  }

  //at this point, nodes contains the highest scoring node at the front of a heapified array
  
  Float maxWeight = maxScoreFrom(0) + std::abs(minScoreFrom(0));
  
  if(options.tickData->bDebug_)
  {
    for (auto& i : nodes)
    {
		//debug drawing
    }
  }

  return nodes.front().node;
}