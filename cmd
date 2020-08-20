
#include<iostream>
#include<vector>
#include <utility> 
#include <map> 
#include <queue> 
#include <stack>
#include <functional>
#include <chrono>
#include <iomanip>

// C function showing how to do time delay 
#include <stdio.h> 
// To use time library of C 
#include <time.h> 

#define INF 1e9
using namespace std;
float current_x, current_y; // current pose
string map_path;

# define obscall_rang 4
# define cover_rang 5
void delay(int number_of_seconds)
{
	// Converting time into milli_seconds 
	int milli_seconds = 1000 * number_of_seconds;

	// Storing start time 
	clock_t start_time = clock();

	// looping till required time is not achieved 
	while (clock() < start_time + milli_seconds)
		;
}
vector<vector<pair<int, int>>> adj_list;
vector<pair<vector<int>, int>>path_v3;
vector <int> check(6, 0);
int find(vector<int>* p) {
	for (auto it = p->begin(); it != p->end(); it++) {
		if (*it == -1) {
			return 1;
		}
	}
	return 0;
}
bool are_they_on_same_line(pair<int, int>n1, pair<int, int>n2, pair<int, int>n3) {
	if ((n1.first == n2.first && n2.first == n3.first &&n1.first == n3.first) || (n1.second == n2.second && n2.second == n3.second && n1.second == n3.second)) {
		return true;
	}
	return false;
}
bool are_they_on_same_line2node(pair<int, int>n1, pair<int, int>n2) {
	if ((n1.first == n2.first) || (n1.second == n2.second )) {
		return true;
	}
	return false;
}

int count_paths = 0;
int count_points = 0;
map< int, pair<int, int>> m;
pair<vector<int>, vector<int> > shortest_distance(int src
) {

	// Create queue
	priority_queue<pair<int, pair<int, int> >,
		vector<pair<int, pair<int, int>> >,
		greater<pair<int, pair<int, int> > > >nodes_q;

	// Create d and p arrays
	int n = adj_list.size();
	vector<int> d(n, INF);
	vector<int> p(n, -1);

	nodes_q.push({ 0, {src, src} });

	while (!nodes_q.empty()) {
		pair<int, pair<int, int> > cur_p = nodes_q.top();
		nodes_q.pop();
		int cur_node = cur_p.second.first;
		int cur_prev_node = cur_p.second.second;
		int cur_dis = cur_p.first;

		if (d[cur_node] != INF)
			continue;

		d[cur_node] = cur_dis;
		p[cur_node] = cur_prev_node;

		// Add the nodes connected to current one
		for (int i = 0;
			i < adj_list[cur_node].size();
			i++)
		{
			int next_node = adj_list[cur_node][i].first;
			int weight = adj_list[cur_node][i].second;
			if (d[next_node] != INF)
				continue;
			nodes_q.push({ cur_dis + weight,
						  {next_node, cur_node} });
		}
	}

	return { d, p };
}

vector<int> Get_Path(int src, int des, vector<int> p) {


	stack<int> path_nodes;
	int node2 = des;

	path_nodes.push(node2);
	vector<int>path;

	while (p[node2] != node2) {
		node2 = p[node2];
		path_nodes.push(node2);
	}


	while (!path_nodes.empty()) {

		path.push_back(path_nodes.top());
		path_nodes.pop();
	}
	return path;
}

// Function to add the given edges of the graph

double distance2N(pair<double, double> node1, pair<double, double> node2) {
	return sqrt(abs(node1.first - node2.first) * abs(node1.first - node2.first) +
		abs(node1.second - node2.second) * abs(node1.second - node2.second));

}


int find_v2(vector<pair<pair<int, int>, int>> stack, int point) {
	for (int i = 0; i < stack.size(); i++) {
		if (stack[i].first.first == point) {
			return 1;
		}
	}
	return 0;
}
int find_v3(vector<pair<pair<int, int>, int>> stack, int postion) {
	for (int i = 0; i < stack.size(); i++) {
		if (stack[i].first.first == postion) {
			return i;
		}
	}
	return -1;
}
int find_v4(vector<pair<pair<int, int>, int>> stack, int point) {
	for (int i = 0; i < stack.size(); i++) {
		if (stack[i].first.first == point && stack[i].first.second == 1) {
			return 1;
		}
	}
	return 0;
}
int find_v5(pair<int, int> node) {
	for (auto i = m.begin(); i != m.end(); i++) {
		if (node.first == i->second.first && node.second == i->second.second) {
			return i->first;
		}
	}
	return -1;
}
int find_in_adj_list(int node_num, int node) {
	for (int i = 0; i < adj_list[node_num].size(); i++) {
		if (adj_list[node_num][i].first == node) {
			return i;
		}

	}
	return -1;
}
vector<vector<pair<int, int>>> matrix(6);//o(1)
bool check_point(int x, int y, int range) {
	//	ROS_INFO("chack x=%d y=%d", x, y);

	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{

				if (matrix[x + i][y + j].second == -1)
					return false;
			}

		}
	}
	return true;


}

bool is_have_visited_nighbour(int x, int y, int range) {
	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x + i >= 0 && y + j >= 0)
			{

				if (check[matrix[x + i][y + j].first] == 1)
					return true;
			}

		}
	}
	return false;


}



vector<int> Nearest(vector<int> unVisitList, int node_num) {

	pair<int, int>  coor = m[node_num];
	int x = coor.first;
	int y = coor.second;
	vector<int> distance;
	int min = INF;


	double min_distance2 = INF;
	double min_distance = INF;
	vector<int>path2;
	pair<vector<int>,vector<int>> result = shortest_distance(node_num);
	path2 = result.first;
	distance = result.second;
	vector<int> path;
	double calcu_dis;
	for (int i = 0; i < unVisitList.size(); i++) {
		if (unVisitList[i] == -1 || unVisitList[i] == 1 || unVisitList[i] == 2) {
			continue;
		}





		if (path2[i] < min_distance) {

			min_distance = path2[i];

			min = i;

		}

	}
	
	path = Get_Path(node_num, min, distance);
	return path;
}

void cover(int node_num, int range, int value) {
	pair<int, int>  coor = m[node_num];
	int x = coor.first;
	int y = coor.second;
	

	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{
				check[matrix[x + i][y + j].first] = value;
			}

		}
	}



}
void update_adj_list(int next_node) {
	pair<int, int>  coor = m[next_node];
	int x = coor.first;
	int y = coor.second;
	//ROS_INFO("update node coor x = %d ,y = %d",x,y);
	if (x + 1 != matrix.size()) {
		if (matrix[x + 1][y].second == 1 && matrix[x][y].second != -1) {
			//ROS_INFO("add node ");
			adj_list[matrix[x][y].first].push_back(matrix[x + 1][y]);
			adj_list[matrix[x + 1][y].first].push_back(matrix[x][y]);
		}
	}


	if (y + 1 != matrix[x].size()) {

		if (matrix[x][y + 1].second == 1 && matrix[x][y].second != -1) {
			//ROS_INFO("add node ");
			adj_list[matrix[x][y].first].push_back(matrix[x][y + 1]);
			adj_list[matrix[x][y + 1].first].push_back(matrix[x][y]);
		}

	}




}
void erase_from_adjs_list(int x, int y) {
	int node_num = matrix[x][y].first;
	if (adj_list[node_num].size() != 0) {
		adj_list.erase(adj_list.begin() + node_num);
		int index = find_in_adj_list(matrix[x + 1][y].first, node_num);
		if (index >= 0)
			adj_list[matrix[x + 1][y].first].erase(adj_list[matrix[x + 1][y].first].begin() + index);

		index = find_in_adj_list(matrix[x - 1][y].first, node_num);
		if (index >= 0)
			adj_list[matrix[x - 1][y].first].erase(adj_list[matrix[x - 1][y].first].begin() + index);

		index = find_in_adj_list(matrix[x][y + 1].first, node_num);
		if (index >= 0)
			adj_list[matrix[x][y + 1].first].erase(adj_list[matrix[x][y + 1].first].begin() + index);

		index = find_in_adj_list(matrix[x][y - 1].first, node_num);
		if (index >= 0)
			adj_list[matrix[x][y - 1].first].erase(adj_list[matrix[x][y - 1].first].begin() + index);


	}


}

int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;


int main(int argc, char** argv) {
	clock_t  start, end;
	int num;
	cout << "enter width:";
	cin >> num;

	cout << "size : " << num << " * " << num << endl;
	int t;

	int num2 = num;


	int num_node = 0;
	//num = 10000;
	int rt = num * num;

	matrix.resize(num);
	check.resize(rt);//o(1)

	start = clock();
	//o(n)
	cout << "map :" << endl;
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < num; j++) {
			cin >> t;
			//t = 0;
			m.insert({ num_node,{i,j} });
			if (t == 0) {
				count_points++;
				matrix[i].push_back({ num_node,1 });

			}
			else {

				check[num_node] = -1;
				matrix[i].push_back({ num_node,-1 });

			}


			num_node++;

		}

	}
	end = clock();
	double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
	//cout << "Time :" << time_taken << endl;
	int begin_row, begin_colum;
	cout << "start point: ";
	cin >> begin_row >> begin_colum;
	pair<int, int> source(begin_row, begin_colum);
	start = clock();
	int tr = num * num;
	int rowm, columm;
	vector<int>path_v4;
	vector<vector<int>>paths;
	vector<vector<int>> outputs;
	vector<int> test;
	adj_list.resize(tr);//o(1)
	int d1 = 0, d2 = 0, d3 = 2, b = 1;
	int best_script = 0;
	int best_cost = INF;
	vector<int>copy_check;



	copy_check = check;
	path_v4.clear();
	//o(n^2)

	



	
	

	

	
	






	

	

	
	start = clock();


	


	adj_list.resize(rt);//o(1)






	path_v4.clear();
	//o(n^2)
	for (int j = 0; j < matrix.size(); j++) {


		for (int i = 0; i < matrix.size(); i++) {
			if (i + 1 != matrix.size()) {
				if (matrix[i + 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i + 1][j]);
					//adj_list[matrix[i + 1][j].first].push_back(matrix[i][j]);
				}
			}

			if (i - 1 >= 0) {
				if (matrix[i - 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i - 1][j]);
					//adj_list[matrix[i + 1][j].first].push_back(matrix[i][j]);
				}
			}


			if (j + 1 != matrix[i].size()) {

				if (matrix[i][j + 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j + 1]);
					//adj_list[matrix[i][j + 1].first].push_back(matrix[i][j]);
				}

			}

			if (j - 1 >= 0) {

				if (matrix[i][j - 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j - 1]);
					//adj_list[matrix[i][j + 1].first].push_back(matrix[i][j]);
				}

			}



			
		}
	}

	int it = matrix[begin_row][begin_colum].first;







	int next_node = it;

	int flag = 0;

	

	vector<int>path_2;





	double x, pos_x, pos_y, y;
	int flag_count = 0;
	int current_point = next_node;
	vector<int> output(num * num, -1);//o(n)
	vector<vector<int>> current_grid;
	int row, col;
	row = source.first;
	col = source.second;
	//stepDir step=stepDir(0);
	int cost = 0;
	int distance = 0;
	int count_step = 0;
	vector<int> cells_nums(tr, 0);
	int count_number_turns = 0;
	int old_node;
	int old_node2;
	int first_time = 0;
	vector<int> threepoints;

	while (find(check.begin(), check.end(), 0) != check.end()) {

		if (output[next_node] == -1)
		{
			output[next_node] = count_step;
		}

		if (first_time = 0) {
			old_node2 = next_node;
			first_time++;
		}
		if (first_time == 1) {
			if(!are_they_on_same_line2node(m[old_node2],m[next_node]))
				count_number_turns++;
			
			old_node2 = next_node;
			
		}
		if (threepoints.size() !=3) {
			threepoints.push_back(next_node);
		}
		if(threepoints.size() == 3) {
			if (!are_they_on_same_line(m[threepoints[0]], m[threepoints[1]], m[threepoints[2]]))
				count_number_turns++;
		
			old_node = threepoints[1];
			threepoints.clear();
			
			threepoints.push_back(old_node);
			threepoints.push_back(next_node);
		}
		
		//			if(check[next_node]==0){
		cells_nums[next_node] = cells_nums[next_node]++;
		//cout << next_node << " ";
		path_v4.push_back(next_node);

		


		count_step++;
		cost += distance;
		check[next_node] = 1;
	

		




	
		//	path_v4.push_back(next_node);


		int x_c = m[next_node].first;
		int y_c = m[next_node].second;
		int x_o = m[next_node].first;
		int y_o = m[next_node].second;
		int node_num_c;
		int count_c = 0;
		bool found = false;
		while (!found && find(check.begin(), check.end(), 0) != check.end()) {
			node_num_c = next_node;
			
			if (check[node_num_c] != 0) {
				while (x_c + 1 < matrix.size() && check[node_num_c] == 1 && count_c < 8) {
					x_c++;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;
					
				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (x_c - 1 >= 0 && check[node_num_c] == 1 && found == false && count_c < 8) {
					x_c--;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;


				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (y_c - 1 >= 0 && check[node_num_c] == 1 && found == false && count_c < 8) {
					count_c++;
					y_c--;
					node_num_c = matrix[x_c][y_c].first;


				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (y_c + 1 < matrix.size() && check[node_num_c] == 1 && found == false && count_c < 8) {
					y_c++;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;

				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				
				
				
			


			
				if (found == false) {
					count_number_turns++;
					
					path_2 = Nearest(check, next_node);
					path_v4.insert(path_v4.end(), path_2.begin() + 1, path_2.end() - 1);//o(n)
					node_num_c = path_2[path_2.size() - 1];

					for (int i = 1; i < path_2.size() - 1; i++) {
						check[path_2[i]] = 1;
						cells_nums[path_2[i]] = cells_nums[path_2[i]]++;
						if (threepoints.size() != 3) {
							threepoints.push_back(path_2[i]);
						}
						if (threepoints.size() == 3) {
							if (!are_they_on_same_line(m[threepoints[0]], m[threepoints[1]], m[threepoints[2]]))
								count_number_turns++;

							old_node = threepoints[1];
							threepoints.clear();
							threepoints.push_back(old_node);
							threepoints.push_back(path_2[i]);
						}
						
						
						//cout << path_2[i] << " ";
					}
					distance = path_2.size() - 1;

					count_step += distance - 1;
					found = true;
				}
			}
			if (node_num_c == INF) {
				next_node++;
			}
			else
				next_node = node_num_c;


		}

	}
	
	/*cout << endl;
	cout << endl;
	int cc2 = 1;
	for (auto i = m.begin(); i != m.end(); i++) {

		cout << i->first << "\t";
		if (cc2 != 0 && cc2 % num == 0) {
			cout << endl;
		}
		cc2++;
	}
	*/
	/*outputs.push_back(output);


	cout << endl;

	

	cout << "output array:" << endl;

	int cc = 1;
	for (auto i = 0; i < outputs[0].size(); i++) {

		cout << outputs[0][i] << "\t";
		if (cc != 0 && cc % num == 0) {
			cout << endl;
		}
		cc++;
	}
	for (int i = 0; i < path_v4.size(); i++) {
		cout << path_v4[i] << " ";
	}
	cout << endl;*/
	int num_node_more_than_one = 0;
	for (int i = 0; i < cells_nums.size(); i++) {
		if (cells_nums[i] > 1) {
			num_node_more_than_one++;
		 }

	}
	cout << endl << "Number of Steps : " << count_step << endl;
	cout << "Number of nodes visited more than one : " << num_node_more_than_one << endl;
	cout << "Number of turns : " << count_number_turns<<endl;
	return 0;

}







