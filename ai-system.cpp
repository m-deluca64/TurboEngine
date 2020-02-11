namespace demo3
{
  struct MoveTo: public Behavior
  {
    Vector3 goal;

    MoveTo()
    {
    }
    ~MoveTo() override
    {
    }

    

    virtual void on_init(BehaviorTickData d) override
    {
    }

    virtual void on_terminate(Status) override
    {
    }


    virtual Status update(BehaviorTickData d)
    {
      Gameplay::MovementComponent *mc= d.world_.comp_get<Gameplay::MovementComponent>(d.entity_);
      TankMovement *tankMC = reinterpret_cast<TankMovement*>(mc->controller_.get());
      Transform *tc= d.world_.comp_get<Transform>(d.entity_);
#if 0
      PhysicsBody *physics= d.world_.comp_get<PhysicsBody>(d.entity_);
      //grab movement component of controlled entity
      const Vector3 pos = tc->position();
      //fail if no movement
      if(!mc || !mc->controller_)
      {
        return Status::Failed;
      }
      Vector3 velocity = physics->velocity();
      Float currSpeed = velocity.Length();
      Float maxSpeed = tankMC->max_speed();

      Vector3 target_offset = goal - pos;
      Float distance = target_offset.Length();
      Float ramped_speed = maxSpeed * (distance / 30.f);
      Float clipped_speed = std::min(ramped_speed, maxSpeed);
      Float ratio = clipped_speed / maxSpeed;
      Vector3 desired_velocity = (clipped_speed / distance) * target_offset;
      Vector3 steering = desired_velocity - velocity;
      if (distance < 5.f)
      {
        return Status::Success;
      }

      target_offset.y = 0;
      target_offset.Normalize();
      Vector3 control = Vector3::TransformNormal(target_offset, tc->transform().Invert()) * ratio;

      velocity.y = 0;
      velocity.Normalize();
      Vector3 percVelocity = Vector3::TransformNormal(velocity, tc->transform().Invert());

      ImGui::Begin("bob");
      ImGui::Text("control: %f %f %f", control.x, control.y, control.z);
      ImGui::Text("percVelocity: %f %f %f", percVelocity.x, percVelocity.y, percVelocity.z);
      ImGui::Text("currSpeed: %f", currSpeed);
      ImGui::Text("ratio: %f", ratio);
      ImGui::Text("ramped_speed: %f", ramped_speed);
      ImGui::Text("clipped_speed: %f", clipped_speed);
      ImGui::Text("steering: %f %f %f", steering.x, steering.y, steering.z);
      ImGui::Text("velocity: %f %f %f", velocity.x, velocity.y, velocity.z);
      ImGui::End();

      //physics->velocity(velocity + steering);
      mc->controller_->set_input(control.x, control.z);
#endif
#if 1
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

#if 0//arrival steering
      Vector3 target_offset = goal - pos;
      Float distance = target_offset.Length();
      Float ramped_speed = maxSpeed * (distance / (currSpeed ));
      Float clipped_speed = std::min(ramped_speed, maxSpeed);
      //Float ratio = (ramped_speed / maxSpeed);
      //Float magnitude = currSpeed / maxSpeed;
      Vector3 desired_velocity = (clipped_speed / distance) * target_offset;
      Vector3 steering = desired_velocity - velocity;

      Vector3 finalVel = velocity + steering;
      Float finalSpeed = finalVel.Length();
      finalVel.Normalize();

      finalVel += avoidance;
      finalVel.Normalize();


      Vector3 control2 = Vector3::TransformNormal(finalVel, tc->transform().Invert());
      control2.Normalize();
      control2.z *= -1.f;
      control2 *= ((finalSpeed / maxSpeed) * 2.f - 1.f);
#endif
      //ImGui::Begin("bob");
      //ImGui::Text("ratio: %f", ratio);
      //ImGui::Text("magnitude: %f", magnitude);
      //ImGui::Text("control: %f %f %f -- %f", control.x, control.y, control.z, control.Length());
      //ImGui::Text("control2: %f %f %f -- %f", control2.x, control2.y, control2.z, control2.Length());
      //ImGui::Text("currSpeed: %f", currSpeed);
      //ImGui::Text("finalSpeed: %f", finalSpeed);
      //ImGui::Text("ramped_speed: %f", ramped_speed);
      //ImGui::Text("clipped_speed: %f", clipped_speed);
      //ImGui::Text("velocity: %f %f %f", velocity.x, velocity.y, velocity.z);
      //ImGui::Text("finalVel: %f %f %f", finalVel.x, finalVel.y, finalVel.z);
      //ImGui::End();



      //velocity.y = 0.f;
      //steering.y = 0.f;
      //physics->velocity(velocity + steering);
      //mc->controller_->set_input(control2.x, control2.z);
      mc->controller_->set_input(control.x, control.z);
      //startRotation_ = tc->rotation();
      //Matrix l = Matrix::CreateLookAt(myPos, enemyPos, Vector3(0.f, 1.f, 0.f));
      //endRotation_ = Quaternion::CreateFromRotationMatrix(l.Invert());
      //mc->controller_->rotation(Quaternion::Slerp(startRotation_, endRotation_, time_ / timeToTurn_));
#endif



      return Status::Running;
    }
    
  };

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

std::shared_ptr<Behavior> Turbo::Gameplay::AI::MakeCombatTree()
{
  auto selector = std::make_shared<ActiveSelector>();
  auto shootAndMove= std::make_shared<Parallel>(Parallel::Policy::RequireOne, Parallel::Policy::RequireOne);
  auto persuitNavigationSequence = std::make_shared<Sequence>();
  persuitNavigationSequence->add_child(std::make_shared<demo3::FindPointAroundTarget>());
  //sequence->add_child(std::make_shared<demo3::FindWalkablePoint>(Vector3(80, 0, 80)));
  auto navmoveto = std::make_shared<demo3::MoveToPath>();
  navmoveto->bAimTowardsGoal = false;
  persuitNavigationSequence->add_child(navmoveto);
  persuitNavigationSequence->add_child(std::make_shared<Utils::Wait>(2.f));

  auto faceAndShoot = std::make_shared<Parallel>(Parallel::Policy::RequireOne, Parallel::Policy::RequireOne);
  auto navigationSequence = std::make_shared<Sequence>();
  auto shootSequence = std::make_shared<Sequence>();
  auto facePlayer = std::make_shared<demo3::FacePlayer>();
  auto randomShoot = std::make_shared<demo2::RandomShoot>();
  faceAndShoot->add_child(facePlayer);
  faceAndShoot->add_child(shootSequence);
  shootSequence->add_child(randomShoot);
  shootSequence->add_child(std::make_shared<Utils::Wait>(0.3f));

  shootAndMove->add_child(faceAndShoot);
  shootAndMove->add_child(persuitNavigationSequence);

  selector->add_child(std::make_shared<demo3::CanSeeTank>(shootAndMove));
  selector->add_child(navigationSequence);

  //Find a random point and move to it
  navigationSequence->add_child(std::make_shared<demo3::FindRandomWalkablePoint>());
  //sequence->add_child(std::make_shared<demo3::FindWalkablePoint>(Vector3(80, 0, 80)));
  navigationSequence->add_child(std::make_shared<demo3::MoveToPath>());
  navigationSequence->add_child(std::make_shared<Utils::Wait>(2.f));

  //find the starting point and return to it
  navigationSequence->add_child(std::make_shared<demo3::FindWalkablePoint>());
  navigationSequence->add_child(std::make_shared<demo3::MoveToPath>());
  navigationSequence->add_child(std::make_shared<Utils::Wait>(2.f));

  //Avoid obstacles at all times //Currently In Development, not quite working correctly
  auto alwaysAvoid = std::make_shared<Parallel>(Parallel::Policy::RequireAll, Parallel::Policy::RequireAll);
  //alwaysAvoid->add_child(std::make_shared<demo3::ObstacleAvoidance>());
  alwaysAvoid->add_child(selector);
  return alwaysAvoid;
}

Vector3 AISystem::get_poly_center(dtMeshTile const *tile, dtPoly const *poly)
{
  Vector3 center(0.f);
  for (int j = 0, nj = (int)poly->vertCount; j < nj; ++j)
  {
    Float x1 = tile->verts[poly->verts[j] * 3 + 0];
    Float y1 = tile->verts[poly->verts[j] * 3 + 1];
    Float z1 = tile->verts[poly->verts[j] * 3 + 2];
    center += Vector3(x1, y1, z1);
  }

  center /= poly->vertCount;

  return center;
}

Array<dtPolyRef> AISystem::get_neighbors(dtMeshTile const *tile, dtPoly const *poly)
{
  Array<dtPolyRef> neighbors;
  for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
  {
    
    dtPolyRef ref = tile->links[i].ref;
    if (ref)
    {
      neighbors.push_back(tile->links[i].ref);
    }
  }
  return neighbors;
}

//will read in any static entity from levelsystem and then add those meshes to the navmesh
//if the entities change at runtime, then this needs to be recalled



Array<Vector3> AISystem::find_path(const Vector3& start, const Vector3& goal)
{
  Array<Vector3> waypoints;
  dtNavMesh const * navmesh = AISystem::instance()->navmesh();
  if(!navmesh)
  {
    return waypoints;
  }
  dtNavMeshQuery query;
  query.init(AISystem::instance()->navmesh(), 2400);
  dtPoly const * startPoly;
  dtMeshTile const * startTile;
  Vector3 startPoint;
  dtPoly const * goalPoly;
  dtMeshTile const * goalTile;
  Vector3 bounds(80.f);
  Vector3 goalPoint;
  dtPolyRef startRef;
  dtPolyRef goalRef;
  //grab goal points
  //Grab the start and end points
  {
    Float box = 40.f;
    Vector3 e(box);
    dtQueryFilter filter;
    dtStatus status = query.findNearestPoly(&start.x, &e.x, &filter, &startRef, &startPoint.x);
    dtStatus status2 = query.findNearestPoly(&goal.x, &e.x, &filter, &goalRef, &goalPoint.x);
    dtStatus status3 = AISystem::instance()->navmesh()->getTileAndPolyByRef(startRef, &startTile, &startPoly);
    dtStatus status4 = AISystem::instance()->navmesh()->getTileAndPolyByRef(goalRef, &goalTile, &goalPoly);
    if( dtStatusFailed(status) || 
      dtStatusFailed(status2) || 
      dtStatusFailed(status3) || 
      dtStatusFailed(status4)
      )
    {
      return waypoints;
    }
  }

  int numPolys = 2000;
  std::map<dtPolyRef, NodeState> nodeStates;
  for(int i = 0; i < numPolys; i++)
  {
    nodeStates[i] = NodeState::Initialized;
  }
  //Uniform Cost Search
  //node -> with State = problem.InitialState, Path-Cost=0
  //openList -> priorityqueue<node> path-cost
  //closed -> empty set
  NodeQueue openlist;
  Array<NavNode> closedList;
  NavNode beginNode(startRef);
  openlist.push(beginNode);
  bool bSuccess = false;
  while(!openlist.empty())
  {
    //if Empty?(open) return fail
    //node = open.pop();
    //if node == goal, return solution(node)
    //node.state = explored
    NavNode node = openlist.top();
    openlist.pop();

    nodeStates[node.poly_] = NodeState::Explored;
    closedList.push_back(node);
    if(node.poly_ == goalRef)
    {
      bSuccess = true;
      break;
    }

    //for each action in problem.Actions(node.State) do
    dtPoly const * poly;
    dtMeshTile const * tile;
    m_navMesh->getTileAndPolyByRefUnsafe(node.poly_, &tile, &poly);
    Vector3 parentCenter = get_poly_center(tile, poly);
    Array<dtPolyRef> neighbors = get_neighbors(tile, poly);
    for(dtPolyRef neighbor : neighbors)
    {
      //child -> Child-Node(problem, node, action);
      NavNode child(neighbor);
      dtPoly const * childPoly;
      dtMeshTile const * childTile;
      m_navMesh->getTileAndPolyByRefUnsafe(child.poly_, &childTile, &childPoly);
      Vector3 childCenter = get_poly_center(childTile, childPoly);

      //h is the estimation from this node to the goal
      //g is the known cost of parent's cost + distance from parent to child
      switch(AISystem::instance()->path_algorithm())
      {
      case Heuristic::DIJKSTRA:
        child.g_ = (parentCenter - childCenter).Length() + node.total_cost();
        break;
      case Heuristic::GREEDY:
        child.h_ = (goalPoint - childCenter).Length();
        break;
      case Heuristic::ASTAR:
        child.h_ = (goalPoint - childCenter).Length();
        child.g_ = (parentCenter - childCenter).Length() + node.total_cost();
        break;
        
      }
      //child.h_ = (goalPoint - childCenter).Length();
      //child.g_ = (parentCenter - childCenter).Length() + node.total_cost();

      //if child.State is not in explored or frontier then
        //frontier -> insert(child, frontier)
      if(nodeStates[child.poly_] == NodeState::Initialized)
      {
        nodeStates[child.poly_] = NodeState::Frontier;
        child.parentPoly_ = node.poly_;
        openlist.push(child);
      }
      //else if child.State is in frontier with higher Path-Cost then
        //replace that frontier node with child 
      else if(nodeStates[child.poly_] == NodeState::Frontier)
      {
        child.parentPoly_ = node.poly_;
        openlist.replace_if_cheaper(child);
      }
    }
  }

  debug_closed_list.clear();
  debug_solution.clear();
  debug_closed_list.insert(debug_closed_list.begin(), closedList.begin(), closedList.end());

  NavNode node = closedList.back();
  closedList.pop_back();
  deque<NavNode> solution;
  while(!closedList.empty())
  {
    if(closedList.back().poly_ == node.parentPoly_)
    {
      solution.push_front(closedList.back());
      node = closedList.back();
    }
    closedList.pop_back();
  }

  debug_solution.clear();
  //debug_solution.insert(debug_solution.begin(), solution.begin(), solution.end);


  //Build the path from the solution found
  for(NavNode node : solution)
  {
    debug_solution.push_back(node);
    dtMeshTile const *tile;
    dtPoly const *poly;
    dtStatus status2 = AISystem::instance()->navmesh()->getTileAndPolyByRef(node.poly_, &tile, &poly);
    Vector3 waypoint = AISystem::instance()->get_poly_center(tile, poly);
    Vector3 hitPos;
    Vector3 hitNormal;
    Float distance;
    dtStatus stat = query.findDistanceToWall(node.poly_, &waypoint.x, tile->header->walkableRadius, 0, &distance, &hitPos.x, &hitNormal.x);
    if(dtStatusSucceed(stat))
    {
      waypoints.push_back(waypoint + (hitNormal * distance) * 0.5f);
    }

    waypoints.push_back(waypoint);
  }
  //add the goal to the end - this should be a point inside the goalRef found - or the best it could find
  //if we succeeded, then we can add the goalpoint to the waypoints - otherwise its the last poly found
  if(bSuccess && !waypoints.empty())
  {
    waypoints.erase(waypoints.begin()); //erase the beginning as we are already in that polygon
    waypoints.push_back(goalPoint);
  }

  return waypoints;
}
