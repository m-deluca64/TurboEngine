  //=======================================================================================================================
  struct CanSeeTank: public Decorator
  {
	//
    bool bAlreadyRunning;
    Float sightCone;
    Float sightDistance;
    

    virtual Status update(BehaviorTickData d)
    {
      Transform *myTc = d.world_.comp_get<Transform>(d.entity_);
      if(!myTc)
      {
        return Status::Failed;
      }
 

	  //Number of tanks per game is dynamic, so we cast a sphere check to grab any sightable tanks
      Vector4 pos = myTc->position();
      auto manifolds = Physics::PhysicsEngine::get_instance()->cast_sphere(pos, sightDistance);
      World::Entity target = World::Entity::invalid();
      Transform *targetTC = nullptr;
	  
	  //Go through each tank found
      for(auto &manifold : manifolds)
      {
        auto *tank = d.world_.comp_get<Tank>(manifold.hitObject_);
        auto *tankTC  = d.world_.comp_get<Transform>(manifold.hitObject_);
        if (tank && tankTC)
        {
		  //skip self 
          if(tankTC == myTc)
          {
            continue;
          }

		  //Check if this tank is in our cone of vision
          Vector3 otherPos = tankTC->position();
          Vector3 forward = myTc->transform().Forward();
          forward.Normalize();
          Vector3 toOther = (otherPos - pos);
          toOther.Normalize();
          float angle = acos(forward.Dot(toOther));
          if(std::fabs(angle) > sightCone)
          {
            //not in sight, discard this one
            continue;
          }

		  //Rank the remaining tanks, based on distance^2 
          if(target !=  World::Entity::invalid())
          {
            float distanceSq1 = (targetTC->position() - myTc->position()).LengthSquared();
            float distanceSq2 = (tankTC->position() - myTc->position()).LengthSquared();
            if(distanceSq2 < distanceSq1)
            {
              target = manifold.hitObject_;
              targetTC = tankTC;
            }
          }
          else
          {
            target = manifold.hitObject_;
            targetTC = tankTC;
          }

        }
      }

	  //After choosing a target (or a null target),
	  //If an invalid target is chosen, fail and reset
      d.brain_.targetEntity = target;
      if(target == World::Entity::invalid())
      {
        if(bAlreadyRunning)
        {
          bAlreadyRunning = false;
        }
        return Status::Failed;
      }

	  //If this is the first time choosing a target, clear
	  //out any path from previous states
      if(!bAlreadyRunning)
      {
        bAlreadyRunning = true;
        d.brain_.path.clear();
        
      }
	  
	  //If we are here, we have successfully assigned a target to the blackboard 
      return get_child()->tick(d);
    }
  };