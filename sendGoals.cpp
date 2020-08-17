#include<iostream>
#include<vector>
#include <utility> 
#include <map> 
#include <queue> 
#include <stack>
#include <functional>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_datatypes.h>
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include <stdio.h> 
#include <time.h> 

# define obscall_rang 4
# define cover_rang 5
#define INF 1e9
using namespace std;
string map_path;
vector<vector<pair<int, int>>> adj_list(6);
vector <int> check(6, 0);
vector<vector<pair<int, int>>> matrix(6);
map< int, pair<int, int>> gmap;
nav_msgs::OccupancyGrid result;
nav_msgs::OccupancyGrid  current_map;
void readMap(char* path);
void save_map(const nav_msgs::OccupancyGrid map);

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


double distance2N(pair<double, double> node1, pair<double, double> node2) {
	return sqrt(abs(node1.first - node2.first) * abs(node1.first - node2.first) +
		abs(node1.second - node2.second) * abs(node1.second - node2.second));

}



bool check_point(int x, int y, int range) {// check if there is obstcall in range


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



int Nearest(vector<int> unVisitList, int node_num) {// get nearest point

	pair<int, int>  coor = gmap[node_num];
	int x = coor.first;
	int y = coor.second;
	vector<int> direction_queue;
	int min = INF;


	double min_distance2 = INF;
	double min_distance = INF;
	vector<int>path2;
	path2 = shortest_distance(node_num).first;
	vector<int> path;
	double calcu_dis;
	for (int i = 0; i < unVisitList.size(); i++) {
		if (unVisitList[i] == -1 || unVisitList[i] == 1 || unVisitList[i] == 2) {
			continue;
		}


		if (!check_point(gmap[i].first, gmap[i].second, obscall_rang)) {
			check[i] = 2;
			continue;

		}



		if (path2[i] < min_distance) {

			min_distance = path2[i];

			min = i;

		}

	}
	if (min_distance != INF) {// check if there is a new update in map 
		min_distance = distance2N(gmap[min], gmap[node_num]);

		for (int i = 0; i < path2.size(); i++) {
			if (path2[i] == INF && check[i] == 0) {
				min_distance2 = distance2N(gmap[i], gmap[node_num]);
				if (min_distance2 < min_distance) {
					min = i;
					min_distance = distance2N(gmap[min], gmap[node_num]);
				}

			}


		}
	}

	if (min_distance == INF) {
		double d;
		for (int i = 0; i < unVisitList.size(); i++) {
			if (unVisitList[i] != 0)
				continue;
			d = distance2N(gmap[i], gmap[node_num]);
			if (d < min_distance) {
				min_distance = d;
				min = i;
			}
		}

	}
	if (path2[min] == INF) {// this mean we can't reach this point so ignore it
		check[min] = 1;
		return -1;
	}
	return min;
}

void cover(int node_num, int range, int value) {// cover mean assign this value to the neighbour of this point 
	pair<int, int>  coor = gmap[node_num];
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
void update_adj_list(int next_node) {//update adjust_list
	pair<int, int>  coor = gmap[next_node];
	int x = coor.first;
	int y = coor.second;
	
	if (x + 1 != matrix.size()) {
		if (matrix[x + 1][y].second == 1 && matrix[x][y].second != -1) {
			
			adj_list[matrix[x][y].first].push_back(matrix[x + 1][y]);
			adj_list[matrix[x + 1][y].first].push_back(matrix[x][y]);
		}
	}


	if (y + 1 != matrix[x].size()) {

		if (matrix[x][y + 1].second == 1 && matrix[x][y].second != -1) {
		    adj_list[matrix[x][y].first].push_back(matrix[x][y + 1]);
			adj_list[matrix[x][y + 1].first].push_back(matrix[x][y]);
		}

	}




}

void update(int next_node, nav_msgs::OccupancyGrid  current_map, int range) {// update the map in range
	int   current_rows = current_map.info.height;
	pair<int, int>  coor = gmap[next_node];
	int x = coor.first;
	int y = coor.second;


	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{
				if (current_map.data[(x + i) + (y + j) * current_rows] == 0 && matrix[x + i][y + j].second == -1) { 
					matrix[x + i][y + j].second = 1;
					if (check[matrix[x + i][y + j].first] == -1 || check[matrix[x + i][y + j].first] == 2) {
						check[matrix[x + i][y + j].first] = 0;
						cover(matrix[x + i][y + j].first, 5, 0);
					}
					if (result.data[(x + i) + (y + j) * current_rows] != 0 || result.data[(x + i) + (y + j) * current_rows] == 100) {
						check[matrix[x + i][y + j].first] = 0;
						update_adj_list(matrix[x + i][y + j].first);
					}
				}
				else if (current_map.data[(x + i) + (y + j) * current_rows] != 0 && matrix[x + i][y + j].second == 1) {
					matrix[x + i][y + j].second = -1;
					check[matrix[x + i][y + j].first] = -1;
				}
			}

		}
	}

}

int main(int argc, char** argv) {
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	float current_x, current_y; // current pose
	ros::init(argc, argv, "navigation_goals");
	ros::NodeHandle nh;
	ROS_INFO("%s", argv[1]);
	readMap(argv[1]);
	ros::Rate rate(1);
	map_path = argv[1];
	tf::TransformListener listener;
	tf::StampedTransform transform;
	int grid_x;
	int grid_y;
	float map_x, map_y;
	int width = result.info.width;
	int num_node = 0;
	int numPoints = width * width;
	check.resize(numPoints);//o(1)
	listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
	listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	map_x = transform.getOrigin().x();
	map_y = transform.getOrigin().y();
	grid_x = (unsigned int)((map_x - result.info.origin.position.x) / result.info.resolution);
	grid_y = (unsigned int)((map_y - result.info.origin.position.y) / result.info.resolution);
	double theta_begin = atan2(map_y, map_x);
	// convert angle to quarternion
	tf::Quaternion quaternion_begin;
	//tf::Quaternion quaternion = transform.getRotation(); 
	quaternion_begin = tf::createQuaternionFromYaw(theta_begin);
	geometry_msgs::Quaternion qMsg_begin;
	tf::quaternionTFToMsg(quaternion_begin, qMsg_begin);

	int begin_row = grid_x, begin_colum = grid_y;
	ROS_INFO("matrix size a %d ", matrix.size());
	pair<int, int> source(begin_row, begin_colum);
	adj_list.resize(numPoints);//o(1)

	for (int j = 0; j < matrix.size(); j++) {


		for (int i = 0; i < matrix.size(); i++) {
			if (i + 1 != matrix.size()) {
				if (matrix[i + 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i + 1][j]);
				}
			}

			if (i - 1 >= 0) {
				if (matrix[i - 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i - 1][j]);
				}
			}


			if (j + 1 != matrix[i].size()) {

				if (matrix[i][j + 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j + 1]);
				}

			}

			if (j - 1 >= 0) {

				if (matrix[i][j - 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j - 1]);
				}

			}

		}
	}

	int it = matrix[begin_row][begin_colum].first;

	int current_goal = it;
	int flag = 0;
	ros::NodeHandle n2;
	ros::Subscriber subs;

	vector<int>path_2;


	MoveBaseClient ac("move_base", true);

	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";



	double x, pos_x, pos_y, y;
	int flag_count = 0;
	nav_msgs::GetMap map_srv;

	while (find(check.begin(), check.end(), 0) != check.end()) {

		ros::ServiceClient map_fetcher = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

		if (map_fetcher.call(map_srv))
		{
			current_map = map_srv.response.map;

		}

		if (!check_point(gmap[current_goal].first, gmap[current_goal].second, obscall_rang)) {
			check[current_goal] = 2;
			ROS_INFO("check value %d", check[current_goal]);
		}
		if (flag_count >= 5)
			update(current_goal, current_map, 15);

		if (flag_count != 0 && matrix[gmap[current_goal].first][gmap[current_goal].second].second == 1 && check[current_goal] == 0) {
			listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			current_x = transform.getOrigin().x();
			current_y = transform.getOrigin().y();
			pos_x = ((gmap[current_goal].first * result.info.resolution) + result.info.origin.position.x);
			pos_y = ((gmap[current_goal].second * result.info.resolution) + result.info.origin.position.y);
			x = pos_x - current_x;
			y = pos_y - current_y;
			goal.target_pose.pose.position.x = current_x;
			goal.target_pose.pose.position.y = current_y;
			double theta = atan2(y, x);
			// convert angle to quarternion
			tf::Quaternion quaternion;
			//tf::Quaternion quaternion = transform.getRotation(); 
			quaternion = tf::createQuaternionFromYaw(theta);
			geometry_msgs::Quaternion qMsg;
			tf::quaternionTFToMsg(quaternion, qMsg);
			// set quarternion to goal
			goal.target_pose.pose.orientation = qMsg;
			goal.target_pose.header.stamp = ros::Time::now();
			ac.sendGoal(goal);
			ac.waitForResult();
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = pos_x;
			goal.target_pose.pose.position.y = pos_y;
			printf("%d", matrix[gmap[current_goal].first][gmap[current_goal].second]);
			ROS_INFO("Sending goal");
			ROS_INFO(" goal x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			ac.sendGoal(goal);
			ac.waitForResult();
		}
		cover(current_goal, cover_rang, 1);
		flag_count += 1;
		int coor_x = gmap[current_goal].first;
		int coor_y = gmap[current_goal].second;
		int x_copy = coor_x;
		int y_copy = coor_y;
		int next_goal;
		int count_c = 0;
		bool found = false;
		while (!found && find(check.begin(), check.end(), 0) != check.end()) {
			next_goal = current_goal;

			if (check[next_goal] != 0) {
				while (coor_x + 1 < matrix.size() && check[next_goal] == 1 && count_c < 10) {
					coor_x++;
					count_c++;
					next_goal = matrix[coor_x][coor_y].first;
					if (!check_point(coor_x, coor_y, obscall_rang)) {
						check[next_goal] = 2;
					}
				}
				count_c = 0;
				if (check[next_goal] == 0) {
					found = true;
				}
				else {
					coor_x = x_copy;
					coor_y = y_copy;
					next_goal = matrix[coor_x][coor_y].first;
				}
				while (coor_x - 1 >= 0 && check[next_goal] == 1 && found == false && count_c < 10) {
					coor_x--;
					count_c++;
					next_goal = matrix[coor_x][coor_y].first;
					if (!check_point(coor_x, coor_y, obscall_rang)) {
						check[next_goal] = 2;
					}

				}
				count_c = 0;
				if (check[next_goal] == 0) {
					found = true;
				}
				else {
					coor_x = x_copy;
					coor_y = y_copy;
					next_goal = matrix[coor_x][coor_y].first;
				}
				while (coor_y - 1 >= 0 && check[next_goal] == 1 && found == false && count_c < 10) {
					count_c++;
					coor_y--;
					next_goal = matrix[coor_x][coor_y].first;
					if (!check_point(coor_x, coor_y, obscall_rang)) {
						check[next_goal] = 2;
					}

				}
				count_c = 0;
				if (check[next_goal] == 0) {
					found = true;
				}
				else {
					coor_x = x_copy;
					coor_y = y_copy;
					next_goal = matrix[coor_x][coor_y].first;
				}
				while (coor_y + 1 < matrix.size() && check[next_goal] == 1 && found == false && count_c < 10) {
					coor_y++;
					count_c++;
					next_goal = matrix[coor_x][coor_y].first;
					if (!check_point(coor_x, coor_y, obscall_rang)) {
						check[next_goal] = 2;
					}
				}
				count_c = 0;
				if (check[next_goal] == 0) {
					found = true;
				}
				else {
					coor_x = x_copy;
					coor_y = y_copy;
					next_goal = matrix[coor_x][coor_y].first;
				}

				if (found == false) {
					ROS_INFO("dikstra is used");
					next_goal = Nearest(check, current_goal);
					found = true;
					if (next_goal == -1)
						found = false;

				}
			}
			if (next_goal == INF) {
				current_goal++;
			}
			else
				current_goal = next_goal;


		}

	}
	//back to initail position
	goal.target_pose.pose.position.x = map_x;
	goal.target_pose.pose.position.y = map_y;
	goal.target_pose.pose.orientation = qMsg_begin;
	goal.target_pose.header.stamp = ros::Time::now();
	ac.sendGoal(goal);
	ac.waitForResult();
	printf("finish");
	subs = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, save_map);
	ros::spinOnce();
	return 0;

}





void save_map(const nav_msgs::OccupancyGrid map) {

	rosbag::Bag map_storage;
	map_storage.open(map_path, rosbag::bagmode::Write);
	map_storage.write<nav_msgs::OccupancyGrid>("map", ros::Time::now(), map);
	map_storage.close();
	return;



}

void readMap(char* path)
{
	int rows;
	int cols;
	nav_msgs::OccupancyGrid   grid;
	nav_msgs::OccupancyGrid::ConstPtr   i;
	rosbag::Bag map_reader;
	map_reader.open(path);//absolute path to bag file
	for (rosbag::MessageInstance const m : rosbag::View(map_reader))
	{
		i = m.instantiate<nav_msgs::OccupancyGrid>();
		if (i != nullptr)
			grid = *i;
	}
	result = grid;
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n", grid.info.width, grid.info.height, grid.info.resolution);
	rows = grid.info.height;
	cols = grid.info.width;
	matrix.resize(rows);
	int currCell = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++) {
			gmap.insert({ currCell,{currCell % rows,currCell / rows} });
			if (grid.data[currCell] == 0) { // unoccupied cell

				matrix[currCell % rows].push_back({ currCell,1 });
			}
			else {

				// occupied (100) or unknown cell (-1)
				matrix[currCell % rows].push_back({ currCell,1 });
			}
			currCell++;
		}

	}

}

