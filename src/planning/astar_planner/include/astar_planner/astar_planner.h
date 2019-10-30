#ifndef ASTAR_PLANNER_ASTAR_PLANNER_H_INCLUDED
#define ASTAR_PLANNER_ASTAR_PLANNER_H_INCLUDED

#include <ros/ros.h>

#include <fstream>
#include <sstream>

#include <utility>
#include <string>

//headers in Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>

class AStarPlanner
{
public:
    AStarPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AStarPlanner();
private:
    void Main_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS> Graph;
    typedef std::pair<int, int> Edge;

    enum { A, B, C, D, E, N };
    const std::string name = "ABCDE";


};

#endif  //ASTAR_PLANNER_ASTAR_PLANNER_H_INCLUDED

