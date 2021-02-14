#include "graph.hpp"

void test01(Graph<char> g)
{
    cout << "'A' 和 'D'之间边的权重为：" << g.get_weight('A', 'D') << endl;
    g.change_weight('A', 'D', 100);
    cout << "将'A' 和 'D'之间边的权重更改为100后，其权重为：" << g.get_weight('A', 'D') << endl;
    g.remove_weight('A', 'D');
    cout << "将'A' 和 'D'之间边的权重删除后，其权重为：" << g.get_weight('A', 'D') << endl;
    cout << "将'A' 和 'D'之间边的权重重新设置为5" << endl;
    g.change_weight('A', 'D', 5);

    cout << "顶点总数：" << g.num_vertices() << endl;
    cout << "边的总数：" << g.num_edges() << endl;

    cout << "图中包含'F'吗？" << (g.contains('F') ? "包含" : "不包含") << endl;
    cout << "图中包含'G'吗？" << (g.contains('G') ? "包含" : "不包含") << endl;
    cout << "'A'和'D'相邻吗？" << (g.adjacent('A', 'D') ? "相邻" : "不相邻") << endl;
    cout << "'B'和'E'相邻吗？" << (g.adjacent('B', 'E') ? "相邻" : "不相邻") << endl;
    cout << "顶点'A'的度数为： " << g.degree('A') << endl;
    cout << "最大度数为：" << g.largest_degree() << endl;

    auto vertices = g.get_vertices();
    cout << "图中的顶点分别为：";
    for (auto u : vertices) cout << " " << u;
    cout << endl;

    map<char, int> nbrs = g.get_neighbours('F');
    cout << "顶点F的邻接顶点ID及其权重为：";
    for (auto u : nbrs) cout << " (" << u.first << ": " << u.second << ")";
    cout << endl;
}

void test02(Graph<char> g)
{
    auto dft = g.depth_first_rec('A');
    cout << "从顶点A进行深度优先遍历（递归）: {";
    for (auto u : dft) cout << u << " ";
    cout << "}" << endl;

    vector<char> dft_itr = g.depth_first_itr('A');
    cout << "从顶点A进行深度优先遍历（迭代）: {";
    for (auto u : dft_itr) cout << u << " ";
    cout << "}" << endl;

    auto bft = g.breadth_first('A');
    cout << "从顶点A进行广度优先遍历: {";
    for (auto u : bft) cout << u << " ";
    cout << "}" << endl;
}

void test03(Graph<char> g) {
    cout << "生成的最小生成树如下：" << endl;
    Graph<char> result = g.prim('A');
    result.show();
}

void test04(Graph<char> g) {
    cout << "最短路径结果如下：" << endl;
    auto dis = g.dijkstra('A');
    vector<char> vertices = g.get_vertices();
    for (auto vertex : vertices)
        if (dis[vertex] >= 0)
            cout << vertex << ": " << dis[vertex] << endl;
}

void test05(Graph<int> g) {
    vector<vector<int>> connected_components = g.get_connected_components();
    g.print_connected_components(connected_components);
}

void test06(Graph<int> g) {
    cout << "暴力求解得到的分割点为：";
    auto articulation_points2 = g.articulation_points(1);
    for (auto u : articulation_points2) cout << " " << u;
    cout << endl;

    cout << "Targan算法求得的分割点为：";
    auto articulation_points1 = g.articulation_points(2);
    for (auto u : articulation_points1) cout << " " << u;
    cout << endl;

}

int main()
{
    //Graph<char> g;
    //g.add_vertex('A');
    //g.add_vertex('B');
    //g.add_vertex('C');
    //g.add_vertex('D');
    //g.add_vertex('E');
    //g.add_vertex('F');
    //g.add_vertex('G');

    //g.add_edge('A', 'B', 7);
    //g.add_edge('A', 'D', 5);
    //g.add_edge('B', 'C', 8);
    //g.add_edge('B', 'D', 9);
    //g.add_edge('B', 'E', 7);
    //g.add_edge('C', 'E', 5);
    //g.add_edge('D', 'E', 15);
    //g.add_edge('D', 'F', 6);
    //g.add_edge('E', 'F', 8);
    //g.add_edge('E', 'G', 9);
    //g.add_edge('F', 'G', 11);

    //g.add_vertex('H');
    //g.add_edge('B', 'H', 9);
    //g.add_edge('A', 'H', 10);
    //g.add_edge('D', 'H', 11);
    //g.add_edge('A', 'H', 12);
    //g.remove_vertex('H');
    //cout << "打印图中顶点及其邻接表的详细信息如下" << endl;
    //g.show();
    //cout << endl;
    //
    // test01(g);
    // test02(g);
    // test03(g);
    // test04(g);

    Graph<int> g;

    g.add_vertex(0);
    g.add_vertex(1);
    g.add_vertex(2);
    g.add_vertex(3);
    g.add_vertex(4);
    g.add_vertex(5);
    g.add_vertex(6);
    g.add_vertex(7);
    g.add_vertex(8);
    g.add_vertex(9);
    g.add_vertex(10);
    g.add_vertex(11);
    g.add_vertex(12);

    g.add_edge(0, 1, 1);
    g.add_edge(0, 2, 1);
    g.add_edge(1, 2, 1);
    g.add_edge(1, 3, 1);
    g.add_edge(1, 4, 1);
    g.add_edge(3, 6, 1);
    g.add_edge(4, 5, 1);
    g.add_edge(7, 8, 1);
    g.add_edge(7, 9, 1);
    g.add_edge(7, 12, 1);
    g.add_edge(8, 9, 1);
    g.add_edge(8, 12, 1);
    g.add_edge(9, 10, 1);
    g.add_edge(9, 11, 1);
    g.add_edge(9, 12, 1);
    g.add_edge(10, 11, 1);
    cout << "打印图中顶点及其邻接表的详细信息如下" << endl;
    g.show();
    cout << endl;

    // test05(g);
    test06(g);
    return 0;
}

