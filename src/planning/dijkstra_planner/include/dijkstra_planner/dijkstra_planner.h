#ifndef DIJKSTRA_PLANNER_DIJKSTRA_PLANNER_H_INCLUDED
#define DIJKSTRA_PLANNER_DIJKSTRA_PLANNER_H_INCLUDED

#include <ros/ros.h>

#include <fstream>
#include <sstream>

#include <deque>
#include <string>


//headers in Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/graph/graphviz.hpp>

class AStarPlanner
{
public:
    AStarPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~AStarPlanner();
private:
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
    boost::no_property, boost::property<boost::edge_weight_t, int> > Graph;
    typedef std::pair<int, int>                             Edge;
    typedef boost::graph_traits<Graph>::vertex_descriptor   Vertex;

    enum { S, A, B, C, D, E, F, Z, N };
    const std::string Names = "SABCDEFZ";

    int Main_(void);
    Graph MakeGraph_(void);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string dot_filename_;

};

#endif  //DIJKSTRA_PLANNER_DIJKSTRA_PLANNER_H_INCLUDED

