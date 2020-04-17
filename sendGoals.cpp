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
// C function showing how to do time delay 
#include <stdio.h> 
// To use time library of C 
#include <time.h> 


#define INF 1e9
using namespace std;
float current_x, current_y; // current pose


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
vector<int> Nearest(vector<int> unVisitList, int node_num) {

	//vector<double> distanceList(unVisitList.size(), INF);//o(n)
	vector<vector<int>>paths;
	vector<int>path;
	vector<int>path2;
	int min = INF;
	int min_size = INF;

	//Node NearestNode = unVisitList[0];
	/*for (int i = 0; i < unVisitList.size(); i++) {//o(n)
		if (unVisitList[i] == -1 || unVisitList[i] == 1) {
			continue;
		}
		distanceList[i] = 1;
	}*/
	//FIND MIN DISTANCE NODE
	path2 = shortest_distance(node_num).second;
	for (int i = 0; i < unVisitList.size(); i++) {//o(n^3logn)
		if (unVisitList[i] == -1 || unVisitList[i] == 1) {
			continue;
		}
		if (path2[i] == -1)
			continue;
		path = Get_Path(node_num, i, path2);

		if (path.size() < min_size) {
			min_size = path.size();

			paths.push_back(path);

		}

	}

	if (paths.size() == 0) {
		return path;
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
vector<vector<pair<int, int>>> matrix(6);//o(1)
bool chech_point(int x, int y) {
	ROS_INFO("x=%d y=%d", x, y);
	for (int i = 1; i <3; i++) {
		if(x+i < matrix.size() && y+i < matrix.size())
		{if(matrix[x+i][y+i].second!=1)
		  return false ;
		}
		if(x-i >=0 && y-i >= 0)
		{if(matrix[x-i][y-i].second!=1)
		  return false ;
		}
		if(x+i < matrix.size() && y-i >= 0)
		{if(matrix[x+i][y-i].second!=1)
		  return false ;
		}
		if(x-i >= 0 && y+i < matrix.size())
		{if(matrix[x-i][y+i].second!=1)
		  return false ;
		}
		if(x-i >= 0)
		{if(matrix[x-i][y].second!=1)
		  return false ;
		}
		if(y-i >= 0)
		{if(matrix[x][y-i].second!=1)
		  return false ;
		}
		if(y+i < matrix.size())
		{if(matrix[x][y+i].second!=1)
		  return false ;
		}
		if(x+i < matrix.size())
		{if(matrix[x+i][y].second!=1)
		  return false ;
		}
		}
		return true;
		
		
	}
	int rows;
	int cols;
	double mapResolution;
	vector<vector<int> > grid;
	nav_msgs::OccupancyGrid result;
	pair<bool, nav_msgs::OccupancyGrid> requestMap(ros::NodeHandle & nh);
	void readMap();
void save_map(const nav_msgs::OccupancyGrid map);
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	int main(int argc, char** argv) {
		ros::init(argc, argv, "navigation_goals");
		
		ros::NodeHandle nh;

		
                readMap();
		
		
		ros::Rate rate(1);


		tf::TransformListener listener;
		tf::StampedTransform transform;
		int grid_x;
		int grid_y;
		float map_x, map_y;

		clock_t  start, end;
		int num = result.info.width;
		matrix.resize(num);

		int t;

		int num2 = num;


		int num_node = 0;
		
		int rt = num * num;


		check.resize(rt);//o(1)

		start = clock();
		//o(n)
		for (int i = 0; i < num; i++) {
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





		//ROS_INFO("Attempting to read pose...");
		listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		map_x = transform.getOrigin().x();
		map_y = transform.getOrigin().y();
		grid_x = (unsigned int)((map_x - result.info.origin.position.x) / result.info.resolution);
		grid_y = (unsigned int)((map_y - result.info.origin.position.y) / result.info.resolution);
		ROS_INFO("Got a index! x = %d, y = %d", grid_x, grid_y);;;


		int begin_row = grid_x, begin_colum = grid_y;

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
		ros::NodeHandle n2;



		vector<int>path_2;
		int idex_t;

		MoveBaseClient ac("move_base", true);

		while (!ac.waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the move_base action server");
		}

		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "map";


		while (find(check.begin(), check.end(), 0) != check.end()) {



			if (chech_point(m[next_node].first, m[next_node].second)) {
				

				/*calculate angle*/
				listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				current_x = transform.getOrigin().x();
				current_y = transform.getOrigin().y();
//goal.target_pose.pose.position.x =current_x;
				//goal.target_pose.pose.position.y = current_y;
				ROS_INFO(" current x=%f y=%f", current_x, current_y);

				//double x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x)- current_x;
				//double y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y)- current_y;
				//double theta = atan2(y, x);
				// convert angle to quarternion
				//tf::Quaternion quaternion;
                                tf::Quaternion quaternion = transform.getRotation(); 
				//quaternion = tf::createQuaternionFromYaw(theta);
				geometry_msgs::Quaternion qMsg;
				tf::quaternionTFToMsg(quaternion, qMsg);
				// set quarternion to goal
				goal.target_pose.pose.orientation = qMsg;


				//ROS_INFO("Sending goal");
				//ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				//ac.sendGoal(goal);
                                // ac.waitForResult();
goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x);
				goal.target_pose.pose.position.y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y);
				printf("%d", grid[m[next_node].first][m[next_node].second]);
ROS_INFO("Sending goal");
				ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				ac.sendGoal(goal);
                                 ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                  ROS_INFO("Hooray, the base moved 1 meter forward");
                                 else
                                  ROS_INFO("The base failed to move forward 1 meter for some reason");

				

			}
			path_v4.push_back(next_node);

			if (find_v2(stack, next_node) != 1) {
				postion = path_v4.size() - 1;
				stack.push_back({ { next_node,1 },postion });

			}



			check[next_node] = 1;




			flag = 0;

			for (int i = 0; i < adj_list[next_node].size(); i++) {
				if (find_v4(stack, adj_list[next_node][i].first) == 1) {
					continue;
				}

				flag = 1;

				next_node = adj_list[next_node][i].first;

				break;
			}
			if (flag == 0) {

				path_2 = Nearest(check, next_node);
				if (path_2.size() != 0) {

					for(int i=1;i<path_2.size()-1;i++){
if (chech_point(m[path_2[i]].first, m[path_2[i]].second)) {
				/*calculate angle*/
				listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
				current_x = transform.getOrigin().x();
				current_y = transform.getOrigin().y();
goal.target_pose.pose.position.x =current_x;
				goal.target_pose.pose.position.y = current_y;
				ROS_INFO(" current x=%f y=%f", current_x, current_y);

				double x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x)- current_x;
				double y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y)- current_y;
				double theta = atan2(y, x);
				// convert angle to quarternion
				tf::Quaternion quaternion;
                                //tf::Quaternion quaternion = transform.getRotation(); 
				quaternion = tf::createQuaternionFromYaw(theta);
				geometry_msgs::Quaternion qMsg;
				tf::quaternionTFToMsg(quaternion, qMsg);
				// set quarternion to goal
				goal.target_pose.pose.orientation = qMsg;


				ROS_INFO("Sending goal");
				ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				ac.sendGoal(goal);
                                 ac.waitForResult();
goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x);
				goal.target_pose.pose.position.y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y);
				printf("%d", grid[m[next_node].first][m[next_node].second]);
ROS_INFO("Sending goal");
				ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				ac.sendGoal(goal);
                                 ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                  ROS_INFO("Hooray, the base moved 1 meter forward");
                                 else
                                  ROS_INFO("The base failed to move forward 1 meter for some reason");

			}


                                               }
					next_node = path_2[path_2.size() - 1];
				}
				else {

					printf("there is stuck points");
                                        ros::Subscriber maps = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, save_map);
                
	                                	delay(1000);
		                        ros::spinOnce();
					return 0;

				}



			}
		}

printf("finish");
ros::Subscriber maps = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, save_map);
                
		delay(1000);

		ros::spinOnce();

		return 0;

	}






void save_map(const nav_msgs::OccupancyGrid map){

rosbag::Bag map_storage;
map_storage.open("/home/islam/catkin_ws2/src/roomba_nev/bag/map.bag" ,rosbag::bagmode::Write);
             map_storage.write<nav_msgs::OccupancyGrid>("map",ros::Time::now(),map );	
             map_storage.close();




}


	pair<bool, nav_msgs::OccupancyG"/home/islam/catkin_ws2/src/roomba_nev/bag/map.bag"rid>  requestMap(ros::NodeHandle & nh) {
		nav_msgs::GetMap::Request req;
		nav_msgs::GetMap::Response res;

		while (!ros::service::waitForService("map", ros::Duration(3.0)))
		{
			ROS_INFO("Waiting for service static_mapto become available");
		}
		ROS_INFO("Requesting the map...");
		ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("map");
		if (mapClient.call(req, res))
		{
			
			return { true,res.map };
		}
		else {
			ROS_ERROR("Failed to call map service");
			return { false,res.map };

		}
	}





	void readMap()
	{
nav_msgs::OccupancyGrid   map ;
nav_msgs::OccupancyGrid::ConstPtr   i; 
rosbag::Bag map_reader;
             map_reader.open("/home/islam/catkin_ws2/src/roomba_nev/bag/map.bag");//absolute path to bag file
            for(rosbag::MessageInstance const m: rosbag::View(map_reader))
    	 	{
         		  i = m.instantiate<nav_msgs::OccupancyGrid>();
         		if (i != nullptr)
           			map=*i;
     		}
		result = map;
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
				if (map.data[currCell] == 0) { // unoccupied cell
					grid[currCell % rows][currCell / rows] = 0;

				}
				else {
					grid[currCell % rows][currCell / rows] = 1; // occupied (100) or unknown cell (-1)

				}
				currCell++;
			}

		}

	}
