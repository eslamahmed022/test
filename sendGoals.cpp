
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
int find_in_adj_list(int node_num,int node){
for(int i=0;i<adj_list[node_num].size();i++){
if(adj_list[node_num][i].first==node){
return i;
}

}
return -1;
}
vector<vector<pair<int, int>>> matrix(6);//o(1)
bool check_point(int x, int y,int range) {
//	ROS_INFO("chack x=%d y=%d", x, y);
	
for (int i = -range; i <= range; i++) {
for(int j=-range;j<=range;j++){
		if (x + i < matrix.size() && y + j < matrix.size()&&x - i >= 0 && y - j >= 0)
		{
			
			if (matrix[x + i][y + j].second ==-1)
				return false;
		}
		
	}
}
	return true;


}

bool is_have_visited_nighbour(int x, int y,int range){
for (int i = -range; i <= range; i++) {
for(int j=-range;j<=range;j++){
		if (x + i < matrix.size() && y + j < matrix.size()&&x + i >= 0 && y + j >= 0)
		{
			
			if (check[matrix[x+i][y+j].first]==1)
				return true;
		}
		
	}
}
	return false;


}



int Nearest(vector<int> unVisitList, int node_num) {

	pair<int,int>  coor=m[node_num];
int x=coor.first;
int y=coor.second;
	vector<int> direction_queue;
	int min = INF;
	

        double min_distance2=INF;
	 double min_distance=INF;
vector<int>path2;
path2 = shortest_distance(node_num).first;
vector<int> path;
double calcu_dis;
	for (int i = 0; i < unVisitList.size(); i++) {
		if (unVisitList[i] == -1 || unVisitList[i] == 1||unVisitList[i] == 2) {
			continue;
		}


      if (!check_point(m[i].first, m[i].second,obscall_rang)){
check[i] = 2;
   continue; 

}

	

		if (path2[i] < min_distance) {
                      
			min_distance = path2[i];

			min=i;

		}
 
	}
min_distance=distance2N(m[min],m[node_num]);

for(int i=0;i<path2.size();i++){
if(path2[i]==INF&&check[i]==0){
min_distance2=distance2N(m[i],m[node_num]);
if(min_distance2<min_distance){
min=i;
min_distance=distance2N(m[min],m[node_num]);
}

}


}


if(min_distance==INF){
double d;
for (int i = 0; i < unVisitList.size(); i++) {
if(unVisitList[i]!=0)
  continue;
d=distance2N(m[i],m[node_num]);
if(d<min_distance){
min_distance=d;
    min=i; 
}
}

}

	return min;
}

void cover(int node_num,int range,int value) {
pair<int,int>  coor=m[node_num];
int x=coor.first;
int y=coor.second;
	ROS_INFO("x=%d y=%d", x, y);

	for (int i = -range; i <= range; i++) {
for(int j=-range;j<=range;j++){
		if (x + i < matrix.size() && y + j < matrix.size()&&x - i >= 0 && y - j >= 0)
		{
			check[matrix[x+i][y+j].first]=value;
		}
		
	}
}
	


}
nav_msgs::OccupancyGrid result;
void update_adj_list(int next_node){
pair<int,int>  coor=m[next_node];
int x=coor.first;
int y=coor.second;
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
void erase_from_adjs_list(int x,int y){
int node_num=matrix[x][y].first;
if(adj_list[node_num].size()!=0){
adj_list.erase(adj_list.begin()+node_num);
int index =find_in_adj_list(matrix[x+1][y].first,node_num);
  if(index>=0)
      adj_list[matrix[x+1][y].first].erase(adj_list[matrix[x+1][y].first].begin()+index);

index =find_in_adj_list(matrix[x-1][y].first,node_num);
  if(index>=0)
      adj_list[matrix[x-1][y].first].erase(adj_list[matrix[x-1][y].first].begin()+index);

index =find_in_adj_list(matrix[x][y+1].first,node_num);
  if(index>=0)
      adj_list[matrix[x][y+1].first].erase(adj_list[matrix[x][y+1].first].begin()+index);

index =find_in_adj_list(matrix[x][y-1].first,node_num);
  if(index>=0)
      adj_list[matrix[x][y-1].first].erase(adj_list[matrix[x][y-1].first].begin()+index);


}
   

}
void update(int next_node,nav_msgs::OccupancyGrid  current_map,int range){
  int   current_rows = current_map.info.height;
pair<int,int>  coor=m[next_node];
int x=coor.first;
int y=coor.second;


	for (int i = -range; i <= range; i++) {
              for(int j=-range;j<=range;j++){
		if (x + i < matrix.size() && y + j < matrix.size()&&x - i >= 0 && y - j >= 0)
		{
//ROS_INFO("result=%d new=%d", result.data[(x+i)+(y+j)*current_rows] , current_map.data[(x+i)+(y+j)*current_rows] );
			if (current_map.data[(x+i)+(y+j)*current_rows] == 0&&matrix[x+i][y+j].second==-1) { // unoccupied cell
                                matrix[x+i][y+j].second=1;
                             if(check[matrix[x+i][y+j].first]==-1||check[matrix[x+i][y+j].first]==2){
                                 check[matrix[x+i][y+j].first]=0;
                                 cover(matrix[x+i][y+j].first,5,0);
}
                                if(result.data[(x+i)+(y+j)*current_rows]!=0||result.data[(x+i)+(y+j)*current_rows]==100){
//ROS_INFO("node %d update",  matrix[x+i][y+j].first);
				   check[matrix[x+i][y+j].first]=0;

                             update_adj_list(matrix[x+i][y+j].first);

}
			}
			else if(current_map.data[(x+i)+(y+j)*current_rows] != 0&&matrix[x+i][y+j].second==1){
				  matrix[x+i][y+j].second=-1;
                                  check[matrix[x+i][y+j].first]=-1;
                                  cover(matrix[x+i][y+j].first,8,2);
                             // erase_from_adjs_list(x+i,y+j);

			}
		}
		
	}
}

}
nav_msgs::OccupancyGrid  current_map;
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

pair<bool, nav_msgs::OccupancyGrid> requestMap(ros::NodeHandle& nh);
void readMap(char* path);
void save_map(const nav_msgs::OccupancyGrid map);
void get_current_map(const nav_msgs::OccupancyGrid map);
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv) {
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

	clock_t  start, end;
	int num = result.info.width;
	matrix.resize(num);

	int t;

	

	int num_node = 0;

	int numPoints = num * num;


	check.resize(numPoints);//o(1)

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
	ROS_INFO("Got a index! x = %d, y = %d", grid_x, grid_y);

				double theta_begin = atan2(map_y, map_x);
				// convert angle to quarternion
				tf::Quaternion quaternion_begin;
				//tf::Quaternion quaternion = transform.getRotation(); 
				quaternion_begin = tf::createQuaternionFromYaw(theta_begin);
				geometry_msgs::Quaternion qMsg_begin;
				tf::quaternionTFToMsg(quaternion_begin, qMsg_begin);

	int begin_row = grid_x, begin_colum = grid_y;

	pair<int, int> source(begin_row, begin_colum);
	start = clock();

	
	vector<int>path_v4;
	
	
	adj_list.resize(numPoints);//o(1)
	
	




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

if (i - 1 >=0) {
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

if (j - 1 >=0) {

				if (matrix[i][j - 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j - 1]);
					//adj_list[matrix[i][j + 1].first].push_back(matrix[i][j]);
				}

			}



/*
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


*/
		}
	}

	int it = find_v5(source);//o(n)





	
	
	int next_node = it;
	
	int flag = 0;
	
	ros::NodeHandle n2;

	ros::Subscriber maps;

	vector<int>path_2;
	

	MoveBaseClient ac("move_base", true);

	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";


	
	double x, pos_x, pos_y, y;
	int flag_count=0;
int current_point=next_node;
nav_msgs::GetMap map_srv ;
vector<vector<int>> current_grid;
int row,col;
row=source.first;
col=source.second;
//stepDir step=stepDir(0);
	while (find(check.begin(), check.end(), 0) != check.end()) {
		
 ros::ServiceClient map_fetcher=nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
   
   if(map_fetcher.call(map_srv))
     {
	   current_map=map_srv.response.map;

     }
          

//			if(check[next_node]==0){

if (!check_point(m[next_node].first, m[next_node].second,obscall_rang)){
check[next_node] = 2;
		ROS_INFO("check value %d",check[next_node]);


}
			 update(next_node,current_map,15);

 if(flag_count!=0&&matrix[m[next_node].first][m[next_node].second].second==1&&check[next_node]==0) {


			listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

			current_x = transform.getOrigin().x();
			current_y = transform.getOrigin().y();
			
			x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x) - current_x;
			y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y) - current_y;
			pos_x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x);
			pos_y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y);
		
			


				/*calculate angle*/


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

				//ROS_INFO("Sending goal");
				//ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				ac.sendGoal(goal);
				ac.waitForResult();
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = pos_x;
				goal.target_pose.pose.position.y = pos_y;

				printf("%d", matrix[m[next_node].first][m[next_node].second]);
				ROS_INFO("Sending goal");
				ROS_INFO(" goal x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
				ac.sendGoal(goal);

				ac.waitForResult();
				

	

}
cover(next_node,cover_rang,1);

			

		
                    flag_count=1;     

	//	path_v4.push_back(next_node);


int x_c=m[next_node].first;
int y_c=m[next_node].second;
int x_o=m[next_node].first;
int y_o=m[next_node].second;
int node_num_c;
int count_c=0;
bool found=false;
		
while(!found&&find(check.begin(), check.end(), 0) != check.end()){
node_num_c=next_node;

if(check[node_num_c]!=0){
while(x_c+1<matrix.size()&&check[node_num_c]==1&&count_c<8){
x_c++;
count_c++;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
}
}
count_c=0;
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
while(x_c-1>=0&&check[node_num_c]==1&&found==false&&count_c<8){
x_c--;
count_c++;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
}

}
count_c=0;
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
while(y_c-1>=0&&check[node_num_c]==1&&found==false&&count_c<8){
count_c++;
y_c--;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;		
}

}
count_c=0;
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
while(y_c+1<matrix.size()&&check[node_num_c]==1&&found==false&&count_c<8){
y_c++;
count_c++;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
}
}
count_c=0;
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}


/*if(check[node_num_c]==-1||check[node_num_c]==2){
ROS_INFO("branch 2 or -1");
count_c=0;
if(!found)
do{
count_c++;
x_c++;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
		ROS_INFO("check value %d",check[next_node]);


}

}while(x_c+1<matrix.size()&&check[node_num_c]==1&&distance2N({x_c,y_c},{x_o,y_o})<10);
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
count_c=0;
ROS_INFO("found1 is %d",found);
if(!found)
do{
count_c++;
x_c--;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
		ROS_INFO("check value %d",check[next_node]);


}
}while(x_c-1>=0&&check[node_num_c]==1&&found==false&&distance2N({x_c,y_c},{x_o,y_o})<10);
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
count_c=0;
ROS_INFO("found2 is %d",found);
if(!found)
do{
count_c++;
y_c--;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
		ROS_INFO("check value %d",check[next_node]);


}
}while(y_c-1>=0&&check[node_num_c]==1&&found==false&&distance2N({x_c,y_c},{x_o,y_o})<10);
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
ROS_INFO("found3 is %d",found);
count_c=0;
if(!found)
do{
y_c++;
count_c++;
node_num_c=matrix[x_c][y_c].first;
if (!check_point(x_c, y_c,obscall_rang)){
check[node_num_c] = 2;
		ROS_INFO("check value %d",check[next_node]);


}
}while(y_c+1<matrix.size()&&check[node_num_c]==1&&found==false&&distance2N({x_c,y_c},{x_o,y_o})<10);
count_c=0;
if(check[node_num_c]==0){
found=true;
}else{
x_c=x_o;
y_c=y_o;
node_num_c=matrix[x_c][y_c].first;
}
ROS_INFO("found4 is %d",found);
}*/
if(found==false){
ROS_INFO("dikstra is used");
node_num_c = Nearest(check, next_node);
found=true;
}
}
if(node_num_c==INF){
next_node++;
}else
next_node=node_num_c;

					
}

}

				goal.target_pose.pose.position.x = map_x;
				goal.target_pose.pose.position.y = map_y;




				
				goal.target_pose.pose.orientation = qMsg_begin;
				goal.target_pose.header.stamp = ros::Time::now();

				
				ac.sendGoal(goal);
				ac.waitForResult();
	printf("finish");
	maps = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, save_map);
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
void get_current_map(const nav_msgs::OccupancyGrid map){
 current_map= map;

}

pair<bool, nav_msgs::OccupancyGrid>  requestMap(ros::NodeHandle& nh) {
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





void readMap(char* path)
{
	nav_msgs::OccupancyGrid   map;
	nav_msgs::OccupancyGrid::ConstPtr   i;
	rosbag::Bag map_reader;
	map_reader.open(path);//absolute path to bag file
	for (rosbag::MessageInstance const m : rosbag::View(map_reader))
	{
		i = m.instantiate<nav_msgs::OccupancyGrid>();
		if (i != nullptr)
			map = *i;
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

