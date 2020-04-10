#include<iostream>
#include<vector>
#include <utility> 
#include <map> 
#include <queue> 
#include <stack>
#include <functional>
#include <dos.h> //for delay
#include<conio.h>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>


#define INF 1e9
using namespace std;
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

double distance2N(pair<int, int> node1, pair<int, int> node2) {
	return sqrt(abs(node1.first - node2.first) * abs(node1.first - node2.first) +
		abs(node1.second - node2.second) * abs(node1.second - node2.second));

}
vector<int> Nearest(pair<int, int> curPos, vector<int> unVisitList, int node_num) {

	vector<double> distanceList(unVisitList.size(), INF);//o(n)
	vector<vector<int>>paths;
	vector<int>path;
	int min = INF;
	int min_size = INF;

	//Node NearestNode = unVisitList[0];
	for (int i = 0; i < unVisitList.size(); i++) {//o(n)
		if (unVisitList[i] == -1 || unVisitList[i] == 1) {
			continue;
		}
		distanceList[i] = 1;
	}
	//FIND MIN DISTANCE NODE
	for (int i = 0; i < distanceList.size(); i++) {//o(n^3logn)
		if (distanceList[i] == INF) {
			continue;
		}
		path = Get_Path(node_num, i, shortest_distance(node_num).second);
		distanceList[i] = INF;
		if (path.size() < min_size) {
			min_size = path.size();

			paths.push_back(path);

		}




	}



	return paths[paths.size() - 1];
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
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;
pair<bool, nav_msgs::OccupancyGrid> requestMap(ros::NodeHandle& nh);
void readMap(const nav_msgs::OccupancyGrid& msg);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv) {
	ros::init(argc, argv, "navigation_goals");
	ros::init(argc, argv, "PoseUpdate");
	ros::init(argc, argv, "load_ogm");
	ros::NodeHandle nh;
	pair<bool, nav_msgs::OccupancyGrid> result = requestMap(nh);
	if (!result.first)
		exit(-1);

	ros::NodeHandle n;

	ros::Rate rate(1.0);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	int grid_x;
	int grid_y;
	float map_x, map_y;

	clock_t  start, end;
	int num= result.second.info.width;

	
	int t;

	int num2 = num;
	

	int num_node = 0;
	//num = 10000;
	int rt = num * num;

	vector<vector<pair<int, int>>> matrix(num);//o(1)
	check.resize(rt);//o(1)

	start = clock();
	//o(n)
	for (int i = 0; i < num ; i++) {
		for (int j = 0; j < num; j++) {
			
			t = grid[i][j];
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

	while (n.ok())
	{
		tf::StampedTransform transform;
		try
		{
			//ROS_INFO("Attempting to read pose...");
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			map_x = transform.getOrigin().x();
			map_y = transform.getOrigin().y();
			grid_x = (unsigned int)((map_x - result.second.info.origin.position.x) / result.second.info.resolution);
			grid_y = (unsigned int)((map_y - result.second.info.origin.position.y) / result.second.info.resolution);
			ROS_INFO("Got a index! x = %d, y = %d", grid_x, grid_y);;;

			break;

		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("Nope! %s", ex.what());
		}


		rate.sleep();

	}

	int begin_row=grid_x, begin_colum=grid_y;
	
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
	for (int j = 0; j < matrix.size(); j++) {


		for (int i = 0; i < matrix.size(); i++) {
			if (i + 1 != matrix.size()) {
				if (matrix[i + 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i + 1][j]);
					adj_list[matrix[i + 1][j].first].push_back(matrix[i][j]);
				}
			}
			if (j + 1 != matrix[i].size()) {

				if (matrix[i][j + 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j + 1]);
					adj_list[matrix[i][j + 1].first].push_back(matrix[i][j]);
				}

			}

			if (i - 1 >= 0 && j + 1 != matrix.size()) {
				if (matrix[i - 1][j + 1].second == 1 && matrix[i][j].second != -1 && (matrix[i - 1][j].second == 1 || matrix[i][j + 1].second == 1)) {

					adj_list[matrix[i][j].first].push_back(matrix[i - 1][j + 1]);
					adj_list[matrix[i - 1][j + 1].first].push_back(matrix[i][j]);


				}
			}


			if (i + 1 != matrix.size() && j + 1 != matrix.size()) {
				if (matrix[i + 1][j + 1].second == 1 && matrix[i][j].second != -1 && (matrix[i + 1][j].second == 1 || matrix[i][j + 1].second == 1)) {

					adj_list[matrix[i][j].first].push_back(matrix[i + 1][j + 1]);
					adj_list[matrix[i + 1][j + 1].first].push_back(matrix[i][j]);


				}
			}



		}
	}

	int it = find_v5(source);//o(n)

	vector <int> st;
	st.push_back(it);//o(1)



	vector<pair<pair<int, int>, int>>stack;
	int cost = 0;
	int distance = 0;
	int next_node = it;
	int next;
	int flag = 0;
	int postion = 0;

	vector<int> output(num * num, -1);//o(n)

	int count_step = 0;

	vector<int>path_2;
	int idex_t;
	end = clock();
	 time_taken = double(end - start) / double(CLOCKS_PER_SEC);
	cout << "Time :" << time_taken << endl;
	cout << "path: ";
	//o(n)
	MoveBaseClient ac("move_base", true);

	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	
	while (find(copy_check.begin(), copy_check.end(), 0) != copy_check.end()) {
		if (output[next_node] == -1)
		{
			output[next_node] = count_step;
		}
		grid_x = (unsigned int)((map_x - result.second.info.origin.position.x) / result.second.info.resolution);
		goal.target_pose.pose.position.x = (m[next_node].first * result.second.info.resolution) + result.second.info.origin.position.x;
		goal.target_pose.pose.position.y = (m[next_node].second * result.second.info.resolution) + result.second.info.origin.position.y;
		//goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		delay(10000);
		//ac.waitForResult();

		

		
		path_v4.push_back(next_node);

		if (find_v2(stack, next_node) != 1) {
			postion = path_v4.size() - 1;
			stack.push_back({ { next_node,1 },postion });

		}


		count_step++;
		cost += distance;
		copy_check[next_node] = 1;


		do {
			if (find(copy_check.begin(), copy_check.end(), 0) == copy_check.end()) {
				break;
			}
			flag = 0;

			for (int i = 0; i < adj_list[next_node].size(); i++) {
				if (find_v4(stack, adj_list[next_node][i].first) == 1) {
					continue;
				}

				flag = 1;
				distance = adj_list[next_node][i].second;
				next_node = adj_list[next_node][i].first;

				break;
			}
			if (flag == 0) {

				path_2 = Nearest(m.find(next_node)->second, copy_check, next_node);
				path_v4.insert(path_v4.end(), path_2.begin() + 1, path_2.end() - 1);//o(n)
				next_node = path_2[path_2.size() - 1];
				
				for (int i = 1; i < path_2.size() - 1; i++) {
					cout << path_2[i] << " ";
				}
				distance = path_2.size() - 1;

				count_step += distance - 1;


				flag = 1;
			}
		} while (flag == 0);

	}
	return 0;

}









pair<bool, nav_msgs::OccupancyGrid>  requestMap(ros::NodeHandle& nh) {
	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;
	while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service static_mapto become available");
	}
	ROS_INFO("Requesting the map...");
	ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
	if (mapClient.call(req, res))
	{
		readMap(res.map);
		return { true,res.map };
	}
	else {
		ROS_ERROR("Failed to call map service");
		return { false,res.map };

	}
}





void readMap(const nav_msgs::OccupancyGrid& map)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width, map.info.height, map.info.resolution);
	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;// Dynamically resize the grid
	grid.resize(rows);

	for (int i = 0; i < rows; i++)
	{
		grid[i].resize(cols);
	}
	int currCell = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++) {
			if (map.data[currCell] == 0) // unoccupied cell
				grid[i][j] = 0;
			else grid[i][j] = 1; // occupied (100) or unknown cell (-1)
			currCell++;
		}
	}
}


