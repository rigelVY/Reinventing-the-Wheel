#include <dijkstra_planner/dijkstra_planner.h>

AStarPlanner::AStarPlanner(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("dot_filename", dot_filename_, "/tmp/graph.dot");
    AStarPlanner::Main_();
}

AStarPlanner::~AStarPlanner()
{

}

AStarPlanner::Graph AStarPlanner::MakeGraph_(void)
{
    const std::vector<Edge> edges = boost::assign::list_of<Edge>
        (S, A)
        (A, B)
        (B, C)
        (B, D)
        (C, E)
        (C, F)
        (D, F)
        (E, D)
        (F, E)
        (E, Z)
        (F, Z)
    ;

    const std::vector<int> weights = boost::assign::list_of
        (3)
        (1)
        (2)
        (3)
        (7)
        (12)
        (2)
        (11)
        (3)
        (2)
        (2)
    ;

    return Graph(edges.begin(), edges.end(), weights.begin(), N);
}

int AStarPlanner::Main_(void)
{
    const Graph g = AStarPlanner::MakeGraph_();
    const Vertex from = S; // start vertex
    const Vertex to = Z; // goal vertex

    // calculate the shortest path
    std::vector<Vertex> parents(boost::num_vertices(g));
    boost::dijkstra_shortest_paths(g, from,
                boost::predecessor_map(&parents[0]));

    // when there are no path to reach the goal
    if (parents[to] == to) {
        std::cout << "no path" << std::endl;
        return 1;
    }

    // create the list of the verticies of the shortest path
    std::deque<Vertex> route;
    for (Vertex v = to; v != from; v = parents[v]) {
        route.push_front(v);
    }
    route.push_front(from);

    // output the shortest path in terminal
    for (const Vertex v : route) {
        std::cout << Names[v] << std::endl;
    }

    // save the graph structure as the image in dot format
    std::ofstream ofs(dot_filename_);
    boost::write_graphviz(ofs, g);

}
