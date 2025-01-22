#ifndef PLANNER_SERVER_PATH_PLANNER_HPP
#define PLANNER_SERVER_PATH_PLANNER_HPP

namespace planner_server
{

class PathPlanner
{
public:
    virtual void test() = 0;
    virtual ~PathPlanner() {}

protected:
    // pluginlib requires default constructor and destructor
    PathPlanner() {}
};

} // namespace planner_server


#endif