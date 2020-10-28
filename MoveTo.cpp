  //=======================================================================================================================
  //MoveTo simply steers an agent towards a point 
  //with respect to wherever the agent is currently aiming
  struct MoveTo: public Behavior
  {
    Vector3 goal; //would be in the header
    virtual Status update(BehaviorTickData d)
    {
      Gameplay::MovementComponent *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
	  PhysicsBody *physics= d.world_.comp_get<PhysicsBody>(d.entity_);
      
	  //fail if no movement || physics
      if(!mc || physics)
      {
        return Status::Failed;
      }
	  
	  //as development went on, we needed the tank's specific movement component...
	  TankMovement *tankMC = reinterpret_cast<TankMovement*>(mc->controller_.get());
	  if(!mc->controller_)
	  {
		  return Status::Failed;
      }
	  
	  //Transform garunteed if physics is available
	  //Get its position
      Transform *tc= d.world_.comp_get<Transform>(d.entity_);
      const Vector3 pos = tc->position();

	  //Get its velocity
      Vector3 velocity = physics->velocity();

      //get direction/velocity
      Float currSpeed = velocity.Length();
      Float maxSpeed = tankMC->max_speed();
      Vector3 dir = goal - pos;
	  
	  //Check if we are within 10 units of our goal (arbitrary)
      Float const range = 10.f;
      dir.y = 0;
      if (dir.LengthSquared() < range * range)
      {
		//if we are coming in hot, find a reverse thrust, and translate that to the control scheme
        if(currSpeed > maxSpeed * 0.5f)
        {
          Vector3 oppositeVel = -velocity;
          oppositeVel.Normalize();
          Vector3 control = Vector3::TransformNormal(oppositeVel, tc->transform().Invert());
		  //invert control.z as positive is downwards on a physical controller
          control.z *= -1.f;

          mc->controller_->set_input(control.x, control.z);
		  //keep "braking" until we are moving half of max speed
          return Status::Running;

        }
        return Status::Success;
      }

	  //if here, we are far enough away that 1 frame of maximum input wont reach us to our goal
	  //transform the desired dir into controller space input, and set controller
      dir.Normalize();
      Vector3 control = Vector3::TransformNormal(dir, tc->transform().Invert());
      control.Normalize();
      control.z *= -1;
      mc->controller_->set_input(control.x, control.z);

      return Status::Running;
    }
    
  };