namespace demo3
{
  //=======================================================================================================================
  struct MoveTo: public Behavior
  {
    Vector3 goal;

    virtual Status update(BehaviorTickData d)
    {
      Gameplay::MovementComponent *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
      TankMovement *tankMC = reinterpret_cast<TankMovement*>(mc->controller_.get());
      Transform *tc= d.world_.comp_get<Transform>(d.entity_);
      
      const Vector3 pos = tc->position();
      Vector4 start = pos;
      Vector4 end = pos + tc->transform().Forward() * 20.f;

      PhysicsBody *physics= d.world_.comp_get<PhysicsBody>(d.entity_);
      Vector3 velocity = physics->velocity();





      //fail if no movement
      if(!mc || !mc->controller_)
      {
        return Status::Failed;
      }
      //if we are in range, success
      Float currSpeed = velocity.Length();
      Float maxSpeed = tankMC->max_speed();

      Vector3 dir = goal - pos;
      //if theres an avoidance, increase goal radius by magnitude so point can be "reached" if obstances near by
      //this can backfire tho/could be exploited
      Float const range = 10.f;// +(avoidance.Length() * 15.f);
      dir.y = 0;
      if (dir.LengthSquared() < range * range)
      {
        if(currSpeed > maxSpeed * 0.5f)
        {
          Vector3 oppositeVel = -velocity;
          oppositeVel.Normalize();
          Vector3 control = Vector3::TransformNormal(oppositeVel, tc->transform().Invert());
          control.z *= -1.f;

          mc->controller_->set_input(control.x, control.z);
          return Status::Running;

        }

        //stop



        return Status::Success;
      }
      dir.Normalize();

      Vector3 control = Vector3::TransformNormal(dir, tc->transform().Invert());

      control.Normalize();
      control.z *= -1;


      mc->controller_->set_input(control.x, control.z);


      return Status::Running;
    }
    
  };
  //=======================================================================================================================
  struct MoveToPath: public MoveTo
  {
    bool bAimTowardsGoal;
    Weak<Timer> rotationTimer_;
    MoveToPath()
      :
      rotationTimer_(TimerSystem::instance()->add_timer((0.3f), false))
      ,bAimTowardsGoal(true)
    {
    }

    virtual ~MoveToPath() override
    {
      TimerSystem::instance()->remove_timer(rotationTimer_);
      
    }

    void pop_goal(BehaviorTickData d)
    {
      goal = d.brain_.path.front();
      d.brain_.path.erase(d.brain_.path.begin());
      d.brain_.target = goal;
      if (bAimTowardsGoal)
      {
        //Lerp rotations to face towards goal
        Transform *tc= d.world_.comp_get<Transform>(d.entity_);
        Gameplay::MovementComponent *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
        Quaternion startRotation_ = tc->rotation();
        Vector3 pos = tc->position();
        Matrix l = Matrix::CreateLookAt(pos, goal, Vector3(0.f, 1.f, 0.f));
        Quaternion endRotation_ = Quaternion::CreateFromRotationMatrix(l.Invert());
        rotationTimer_->update_func([startRotation_, endRotation_, mc](Float percentage)->void
        {
          mc->controller_->rotation(Quaternion::Slerp(startRotation_, endRotation_, percentage));
        });
        rotationTimer_->start();
      }
    }

    virtual void on_init(BehaviorTickData d) override
    {
      status_ = Status::Running;
      if(d.brain_.path.empty())
      {
        status_ = Status::Failed;
        return;
      }
      pop_goal(d);
      MoveTo::on_init(d);
    }

    Status update(BehaviorTickData d) override
    {
      int id = 550;
      auto *tc = d.world_.comp_get<Transform>(d.entity_);
      Vector3 start = tc->position();
      if(!d.brain_.path.empty())
      {
        Vector3 last = goal;
        Utils::draw_line(id++, start, last);
        for (Vector3 p : d.brain_.path)
        {
          Utils::draw_line(id++, last, p);
          last = p;
        }
      }
      else
      {
        Utils::draw_line(id++, start, goal);
      }
      Status s = MoveTo::update(d);
      if(s == Status::Success)
      {
        if(d.brain_.path.empty())
        {
          return Status::Success;
        }
        else
        {
          pop_goal(d);
          return Status::Running;
        }
      }
      return s;
    }
  };
  //=======================================================================================================================
  struct FindWalkablePoint: public Behavior
  {
    Vector3 goalSearch_;
    FindWalkablePoint(Vector3 goal = Vector3(0.f)) //default init to the ai-test start point
      : goalSearch_(goal)
    {
    }

    virtual void on_init(BehaviorTickData d) override
    {
      status_ = Status::Running;
    }
    Status update(BehaviorTickData d) override
    {
      dtNavMeshQuery query;
      auto *navmesh = AISystem::instance()->navmesh();
      if(!navmesh)
      {
        return Status::Failed;
      }
      query.init(navmesh, 2048);
      //dtPolyRef ref;
      Vector3 point;
      Vector3 extents(10.f);
      dtQueryFilter filter;
      UInt32 poly;
      //query.findRandomPoint(&filter, frand, &ref, &point.x);
      query.findNearestPoly(&goalSearch_.x, &extents.x, &filter, &poly, &point.x);

      auto *tc = d.world_.comp_get<Transform>(d.entity_);
      Vector3 pos = tc->position();
      d.brain_.path = AISystem::instance()->find_path(pos, point);
      if(d.brain_.path.size())
      {
        d.brain_.target = point;
        return Status::Success;
      }
      return Status::Failed;
    }
  };
  //=======================================================================================================================
  struct FindRandomWalkablePoint: public Behavior
  {
    FindRandomWalkablePoint()
    {
    }

    virtual void on_init(BehaviorTickData d) override
    {
      status_ = Status::Running;
    }
    Status update(BehaviorTickData d) override
    {
      dtNavMeshQuery query;
      auto *navmesh = AISystem::instance()->navmesh();
      if(!navmesh)
      {
        return Status::Failed;
      }
      query.init(navmesh, 2048);
      dtPolyRef ref;
      Vector3 point;
      //Vector3 spoint(0, 0, -80);
      Vector3 extents(10.f);
      dtQueryFilter filter;
      //UInt32 poly;
      query.findRandomPoint(&filter, frand, &ref, &point.x);
      //query.findNearestPoly(&spoint.x, &extents.x, &filter, &poly, &point.x);

      auto *tc = d.world_.comp_get<Transform>(d.entity_);
      Vector3 pos = tc->position();
      d.brain_.path = AISystem::instance()->find_path(pos, point);
      if(d.brain_.path.size())
      {
        d.brain_.target = point;
        return Status::Success;
      }
      return Status::Failed;
    }
  };
  //=======================================================================================================================
  struct FindPath: public Behavior
  {
    FindPath()
    {
    }

    virtual void on_init(BehaviorTickData d) override
    {
      status_ = Status::Running;
    }

    virtual void on_terminate(Status) override
    {
    }


    virtual Status update(BehaviorTickData d)
    {
      auto *tc = d.world_.comp_get<Transform>(d.entity_);
      Vector3 p = tc->position();
      auto path = AISystem::instance()->find_path(p, d.brain_.target);
      if(path.empty())
      {
        return Status::Failed;
      }
      d.brain_.path = path;
      return Status::Success;
    }
  };
  //=======================================================================================================================
  struct CanSeeTank: public Decorator
  {
  public:
    bool bAlreadyRunning;
    Float sightCone;
    Float sightDistance;
    CanSeeTank(Behavior::Ptr child) : Decorator(child)
      , sightCone(Camera::PI / 4.f)
      , sightDistance(80.f)
    {
    }
    virtual void on_init(BehaviorTickData) override
    {
    }

    virtual Status update(BehaviorTickData d)
    {

      //cast stuff
      //check for tanks
      //get closest tank
      auto *myTc = d.world_.comp_get<Transform>(d.entity_);
      if(!myTc)
      {
        return Status::Failed;
      }
      //Vector3 myPos = myTc->position();

      Vector4 pos = myTc->position();
      auto manifolds = Physics::PhysicsEngine::get_instance()->cast_sphere(pos, sightDistance);
      World::Entity target = World::Entity::invalid();
      Transform *targetTC = nullptr;
      for(auto &manifold : manifolds)
      {
        auto *tank = d.world_.comp_get<Tank>(manifold.hitObject_);
        auto *tankTC  = d.world_.comp_get<Transform>(manifold.hitObject_);
        if (tank && tankTC)
        {
          if(tankTC == myTc)
          {
            continue;
          }

          Vector3 otherPos = tankTC->position();
          Vector3 forward = myTc->transform().Forward();
          forward.Normalize();
          Vector3 toOther = (otherPos - pos);
          toOther.Normalize();

          float angle = acos(forward.Dot(toOther));

          if(std::fabs(angle) > sightCone)
          {
            //not in sight
            continue;
          }

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

      d.brain_.targetEntity = target;
      if(target == World::Entity::invalid())
      {
        if(bAlreadyRunning)
        {
          bAlreadyRunning = false;
        }
        return Status::Failed;
      }

      if(!bAlreadyRunning)
      {
        bAlreadyRunning = true;
        d.brain_.path.clear();
        
      }

      return get_child()->tick(d);
    }
  };

  //=======================================================================================================================
  struct FacePlayer : public Behavior
  {
    Status update(BehaviorTickData d) override
    {
      if (d.brain_.targetEntity != World::Entity::invalid())
      {
        auto *targetTC = d.world_.comp_get<Transform>(d.brain_.targetEntity);
        auto *myTC = d.world_.comp_get<Transform>(d.entity_);

        //Lerp rotations to face towards goal
        Gameplay::MovementComponent *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
        Quaternion startRotation_ = myTC->rotation();
        Vector3 pos = myTC->position();
        Vector3 goal = targetTC->position();
        Matrix l = Matrix::CreateLookAt(pos, goal, Vector3(0.f, 1.f, 0.f));
        Quaternion endRotation_ = Quaternion::CreateFromRotationMatrix(l.Invert());
        //mc->controller_->rotation(Quaternion::Slerp(startRotation_, endRotation_, percentage));
        mc->controller_->rotation(endRotation_);
        return Status::Running;
      }
      return Status::Failed;
      
    }
    void on_init(BehaviorTickData d) override
    {
      
    }
    void on_terminate(Status) override
    {
    }
  };
  //=======================================================================================================================
  struct FindPointAroundTarget: public Behavior
  {
    Status update(BehaviorTickData d) override
    {
      if (d.brain_.targetEntity != World::Entity::invalid())
      {
        dtNavMeshQuery query;
        auto *navmesh = AISystem::instance()->navmesh();
        if(!navmesh)
        {
          return Status::Failed;
        }
        query.init(navmesh, 2048);
        Vector3 point;
        Vector3 cpoint;
        Transform *targetTc = d.world_.comp_get<Transform>(d.brain_.targetEntity);
        Vector3 spoint = targetTc->position();
        Vector3 extents(10.f);
        dtPolyRef poly;
        dtPolyRef foundPoly;
        dtQueryFilter filter;
        query.findNearestPoly(&spoint.x, &extents.x, &filter, &poly, &cpoint.x);
        query.findRandomPointAroundCircle(poly, &cpoint.x, 50.f, &filter, frand, &foundPoly, &point.x);
        auto *tc = d.world_.comp_get<Transform>(d.entity_);
        Vector3 pos = tc->position();
        d.brain_.path = AISystem::instance()->find_path(pos, point);
        if(d.brain_.path.size())
        {
          d.brain_.target = point;
          return Status::Success;
        }
        return Status::Failed;
      }
      return Status::Failed;
      
    }
    void on_init(BehaviorTickData d) override
    {
      status_ = Status::Running;
      
    }
    void on_terminate(Status) override
    {
    }
  };
  //=======================================================================================================================
  struct ObstacleAvoidance : public Behavior
  {

    Status update(BehaviorTickData d) override
    {
      auto *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
      auto *physics = d.world_.comp_get<PhysicsBody>(d.entity_);
      Vector3 velocity = physics->velocity();
      auto *tc = d.world_.comp_get<Transform>(d.entity_);
      Float obstacleDistance = 20.f;
      Vector3 pos = tc->position();
      Vector3 scale = tc->scale();
      Vector3 forward = tc->transform().Forward();
      forward.Normalize();
      Vector3 velocityForward = physics->velocity();
      velocityForward.Normalize();
      Vector4 centerBox = pos + forward * (obstacleDistance * 0.5f);
      //todo this should be a obb
      World::Entity obstacles = World::Entity::invalid();
      auto manifolds = Physics::PhysicsEngine::get_instance()->cast_sphere(centerBox, obstacleDistance * 3.f);
      Float minSqr = std::numeric_limits<Float>::max();
      for(auto &manifold : manifolds)
      {
        if(manifold.hitObject_ == d.entity_)
        {
          continue;
        }
        auto *transform = d.world_.comp_get<Transform>(manifold.hitObject_);
        auto *physics = d.world_.comp_get<PhysicsBody>(manifold.hitObject_);
        if(physics->dynamic_type() != Physics::DynamicType::staticType)
        {
          Float sightCone = (Camera::PI / 2.00f);
          Vector3 mpos = transform->position();
          Vector3 toOther = (mpos - pos);
          toOther.Normalize();

          float angle = acos(velocityForward.Dot(toOther));
          Float distanceSqr = (pos - mpos).LengthSquared();
          if(manifold.hitObject_ == obstacles)
          {
            obstacles = manifold.hitObject_;
            minSqr = distanceSqr;
            if(distanceSqr > minSqr || angle > sightCone)
            {
              obstacles = World::Entity::invalid();
              continue;
            }
          }
          else if(distanceSqr < minSqr && angle < sightCone)
          {
            obstacles = manifold.hitObject_;
            minSqr = distanceSqr;
          }
        }
      }

      Vector3 avoidance(0.f);
      if(obstacles != World::Entity::invalid())
      {
        auto *transform = d.world_.comp_get<Transform>(obstacles);
        auto *physics = d.world_.comp_get<PhysicsBody>(obstacles);
        Vector3 mpos = transform->position();
        Vector3 toOther = (mpos - pos);
        toOther.Normalize();
        avoidance = velocityForward - toOther;
        avoidance.Normalize();

        //Transform avoidance into local controller space
        Vector3 desired = Vector3::TransformNormal(avoidance, tc->transform().Invert());
        desired.Normalize();
        desired.z *= -1.f;
        mc->controller_->set_input(desired);
      }
      return Status::Running;
    }

  };

};
