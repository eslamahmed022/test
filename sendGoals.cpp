#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <nav_msgs/GetMap.h>
#include <vector>

using namespace std;

int rows;
int cols;
double mapResolution;
vector<vector<bool> > grid;
pair<bool,nav_msgs::OccupancyGrid> requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGrid();
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_goals");
        ros::init(argc, argv, "PoseUpdate");
ros::init(argc, argv, "load_ogm");
ros::NodeHandle nh;
pair<bool,nav_msgs::OccupancyGrid> result=requestMap(nh);
 if(!result.first)
    exit(-1);

 printGrid();
        ros::NodeHandle n;

        ros::Rate rate(1.0);
        tf::TransformListener listener;
        tf::StampedTransform transform;
int grid_x;
int grid_y;
float map_x,map_y;

   while (n.ok())
  {
    tf::StampedTransform transform;
    try
    {
  //ROS_INFO("Attempting to read pose...");
        listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
map_x=transform.getOrigin().x();
map_y=transform.getOrigin().y();
grid_x = (unsigned int)((map_x - result.second.info.origin.position.x) / result.second.info.resolution);
grid_y = (unsigned int)((map_y - result.second.info.origin.position.y) / result.second.info.resolution);
ROS_INFO("Got a index! x = %d, y = %d",grid_x,grid_y);;;
      
break;

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    } 


    rate.sleep();

  }
	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = -1.0;
	goal.target_pose.pose.position.y = -1.0;
	//goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	return 0;
}
pair<bool,nav_msgs::OccupancyGrid>  requestMap(ros::NodeHandle &nh){
nav_msgs::GetMap::Request req;
nav_msgs::GetMap::Response res;
while(!ros::service::waitForService("static_map", ros::Duration(3.0)))
 {ROS_INFO("Waiting for service static_mapto become available");
  }
ROS_INFO("Requesting the map...");
ros::ServiceClient mapClient= nh.serviceClient<nav_msgs::GetMap>("static_map");
if(mapClient.call(req, res)) 
{
readMap(res.map);
return {true,res.map};
}
else{
ROS_ERROR("Failed to call map service");
return {false,res.map};

}
}





void readMap(const nav_msgs::OccupancyGrid& map)
{ROS_INFO("Received a %d X %d map @ %.3f m/px\n",map.info.width,map.info.height,map.info.resolution);
rows = map.info.height;
cols = map.info.width;
mapResolution= map.info.resolution;// Dynamically resize the grid
grid.resize(rows);

for(int i= 0; i< rows; i++) 
{
grid[i].resize(cols);
}
int currCell= 0;
for(int i= 0; i< rows; i++) 
 {
for(int j = 0; j < cols; j++) {
if(map.data[currCell] == 0) // unoccupied cell
grid[i][j] = false;
else grid[i][j] = true; // occupied (100) or unknown cell (-1)
currCell++;
   }
 }
}

void printGrid(){printf("Grid map:\n");

printf("%d ", grid[2000][2000] ? 1 : 0);


}

