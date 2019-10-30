#include <astar_planner/astar_planner.h>

AStarPlanner::AStarPlanner(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    AStarPlanner::Main_();
}

AStarPlanner::~AStarPlanner()
{

}

void AStarPlanner::Main_(void)
{
    Graph g;

    // 頂点を追加
    std::map<int, Graph::vertex_descriptor> desc;
    for (int i = 0; i < N; ++i) {
        desc[i] = add_vertex(g);
    }

    // 辺を追加
    add_edge(desc[A], desc[B], g);
    add_edge(desc[A], desc[C], g);
    add_edge(desc[A], desc[D], g);
    add_edge(desc[B], desc[E], g);
    add_edge(desc[C], desc[E], g);
    add_edge(desc[D], desc[E], g);

    boost::print_graph(g, name.c_str());
    
    return; 
}
