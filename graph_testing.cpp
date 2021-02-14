#include "graph.hpp"

void test01(Graph<char> g)
{
    cout << "'A' �� 'D'֮��ߵ�Ȩ��Ϊ��" << g.get_weight('A', 'D') << endl;
    g.change_weight('A', 'D', 100);
    cout << "��'A' �� 'D'֮��ߵ�Ȩ�ظ���Ϊ100����Ȩ��Ϊ��" << g.get_weight('A', 'D') << endl;
    g.remove_weight('A', 'D');
    cout << "��'A' �� 'D'֮��ߵ�Ȩ��ɾ������Ȩ��Ϊ��" << g.get_weight('A', 'D') << endl;
    cout << "��'A' �� 'D'֮��ߵ�Ȩ����������Ϊ5" << endl;
    g.change_weight('A', 'D', 5);

    cout << "����������" << g.num_vertices() << endl;
    cout << "�ߵ�������" << g.num_edges() << endl;

    cout << "ͼ�а���'F'��" << (g.contains('F') ? "����" : "������") << endl;
    cout << "ͼ�а���'G'��" << (g.contains('G') ? "����" : "������") << endl;
    cout << "'A'��'D'������" << (g.adjacent('A', 'D') ? "����" : "������") << endl;
    cout << "'B'��'E'������" << (g.adjacent('B', 'E') ? "����" : "������") << endl;
    cout << "����'A'�Ķ���Ϊ�� " << g.degree('A') << endl;
    cout << "������Ϊ��" << g.largest_degree() << endl;

    auto vertices = g.get_vertices();
    cout << "ͼ�еĶ���ֱ�Ϊ��";
    for (auto u : vertices) cout << " " << u;
    cout << endl;

    map<char, int> nbrs = g.get_neighbours('F');
    cout << "����F���ڽӶ���ID����Ȩ��Ϊ��";
    for (auto u : nbrs) cout << " (" << u.first << ": " << u.second << ")";
    cout << endl;
}

void test02(Graph<char> g)
{
    auto dft = g.depth_first_rec('A');
    cout << "�Ӷ���A����������ȱ������ݹ飩: {";
    for (auto u : dft) cout << u << " ";
    cout << "}" << endl;

    vector<char> dft_itr = g.depth_first_itr('A');
    cout << "�Ӷ���A����������ȱ�����������: {";
    for (auto u : dft_itr) cout << u << " ";
    cout << "}" << endl;

    auto bft = g.breadth_first('A');
    cout << "�Ӷ���A���й�����ȱ���: {";
    for (auto u : bft) cout << u << " ";
    cout << "}" << endl;
}

void test03(Graph<char> g) {
    cout << "���ɵ���С���������£�" << endl;
    Graph<char> result = g.prim('A');
    result.show();
}

void test04(Graph<char> g) {
    cout << "���·��������£�" << endl;
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
    cout << "�������õ��ķָ��Ϊ��";
    auto articulation_points2 = g.articulation_points(1);
    for (auto u : articulation_points2) cout << " " << u;
    cout << endl;

    cout << "Targan�㷨��õķָ��Ϊ��";
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
    //cout << "��ӡͼ�ж��㼰���ڽӱ����ϸ��Ϣ����" << endl;
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
    cout << "��ӡͼ�ж��㼰���ڽӱ����ϸ��Ϣ����" << endl;
    g.show();
    cout << endl;

    // test05(g);
    test06(g);
    return 0;
}

