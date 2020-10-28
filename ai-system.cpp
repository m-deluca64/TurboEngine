//The update tick for the AI system
void AISystem::operator()(ComponentView cv)
{
  //bRunning_ and bStepping_ are debug features
  if (bRunning_ || bStepping_)
  {
    //Main Update 
    {
      //run the trees for all brains
      for (auto s : cv)
      {
        //only continue if tree is running & its a valid tree
        if(!s.a.bTreeRunning)
        {
          continue;
        }
		
		//Grab the Tree type this brain is using
        auto tree = behaviorTreeManager_.get_tree(s.a.tree_);
        if(!tree)
        {
          continue;
        }

        //prepare the tick data
        bool bDebug = s.a.bDebug;
        if(bDrawAllDebug)
        {
          bDebug = true;
        }
		
		//Grab the BehaviorTree Instance for this entity, as well as its blackboards
        auto &instance = behaviorTreeManager_.get_instance(s.entity);
        auto &board = behaviorTreeManager_.get_black_board(s.entity);
        auto &globalBoard = behaviorTreeManager_.get_global_black_board();

		//output to log so we know which entity future AI log calls came from
        if(bDebug)
        {
          rttr::type nodeType = rttr::type::get(*this);
          std::string nodeName(nodeType.get_name());
          Log::tout << Log::Severity::attention << Log::ID(L"Behavior Trees") << "Ticking tree for entity: " << s.entity << Log::TEnd();
        }
		
		//Prepare the tick data, which includes
		//the entity id
		//the entity's brain component (container of ids for behavior tree / runtime variables)
		//instance - current state of the beahvior tree for this particular entity
		//board/globalBoard - the blackboards associated to this entity
		//a debug flag
        BehaviorTickData data(s.entity, s.a, world_, instance, board, globalBoard, bDebug);

        // Perform the update on this individual task.
        tree->tick(data);

		//closing debug 
        if(bDebug)
        {
          rttr::type nodeType = rttr::type::get(*this);
          std::string nodeName(nodeType.get_name());
          Log::tout << Log::Severity::attention << Log::ID(L"Behavior Trees") << "Finished ticking tree for entity: " << s.entity << Log::TEnd();
        }
		
        //Debug stack is a stack of the processed behavior tree nodes that occured in the previous tree->tick
        s.a.debugStack_ = data.lastNode;
		
        //if its stopped running for any reason, reset the instance data if looping
        //note, the root will always be the 0th node in the instance data
        if (instance.get_node<Behavior::NodeData>(0)->status_ != Status::Running)
        {
          //if looping this tree, then reset root, otherwise clean root
          if(s.a.bTreeLooping)
          {
            tree->reset(data);
          }
          else
          {
            s.a.bTreeRunning = false;
          }
        }
      }

    }
   
    //debug stepping
    if(++stepCount_ >= stepSize_)
    {
      bStepping_ = false;
    }
  }

  //ImGui Debug windows for AI and NavMesh systems
  debug_window(cv);
  navMesh_->debug_draw();
}