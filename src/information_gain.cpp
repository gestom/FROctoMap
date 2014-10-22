#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>
#include "CFremenGrid.h"
#include "CTimer.h"

#include <actionlib/server/simple_action_server.h>
#include <fremen/informationAction.h>


#define LIM_MIN_X -10.0
#define LIM_MIN_Y -10.0
#define LIM_MIN_Z 0.0

#define LIM_MAX_X 10.0
#define LIM_MAX_Y 10.0
#define LIM_MAX_Z 3.9999

#define RANGE_MIN 0.5
#define RANGE_MAX 4.7

#define m_colorFactor 0.8

using namespace std;
using namespace octomap;


//Parameters:
string filename;
double resolution, head_height, angular_step;
int dim_x,dim_y, dim_z;
int8_t *aux_entropy;

CFremenGrid *gridPtr;

ros::Publisher *estimate_pub_ptr, *head_pub_ptr, *rays_pub_ptr, *text_pub_ptr;

typedef actionlib::SimpleActionServer<fremen::informationAction> Server;

std_msgs::ColorRGBA heightMapColorA(double h){


    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1)) f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
        color.r = v;
        color.g = n;
        color.b = m;
        break;
    case 1:
        color.r = n;
        color.g = v;
        color.b = m;
        break;
    case 2:
        color.r = m;
        color.g = v;
        color.b = n;
        break;
    case 3:
        color.r = m;
        color.g = n;
        color.b = v;
        break;
    case 4:
        color.r = n;
        color.g = m;
        color.b = v;
        break;
    case 5:
        color.r = v;
        color.g = m;
        color.b = n;
        break;
    default:
        color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;

    }

    return color;
}


void execute(const fremen::informationGoalConstPtr& goal, Server* as)
{

    /*              Octmap Estimation and Visualization             */

    octomap_msgs::Octomap bmap_msg;
    OcTree octree (resolution);
    geometry_msgs::Point initialPt, finalPt;

    //Create pointcloud:
    octomap::Pointcloud octoCloud;
    sensor_msgs::PointCloud fremenCloud;
    float x = 0*gridPtr->positionX;
    float y = 0*gridPtr->positionY;
    geometry_msgs::Point32 test_point;
    int cnt = 0;

    int cell_x, cell_y, cell_z;
    cell_x = (int)(goal->x/resolution);
    cell_y = (int)(goal->y/resolution);
    cell_z = (int)(head_height/resolution);

    for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
        for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
            for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
                point3d ptt(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
                int s = goal->stamp;
                if(gridPtr->retrieve(cnt, goal->stamp)>0)
                {
                    //finalPt.z = (int)((w+resolution/2)/resolution)-cell_z;
                    //finalPt.y = (int)((j+resolution/2)/resolution)-cell_y;
                    //finalPt.x = (int)((i+resolution/2)/resolution)-cell_x;

                    //int cnta = ((cell_x+finalPt.x-LIM_MIN_X/resolution)*dim_y + (finalPt.y + cell_y-LIM_MIN_Y/resolution))*dim_z + (finalPt.z + cell_z-LIM_MIN_Z/resolution);
                    //ROS_INFO("something %d %d",cnt,cnta);
                    octoCloud.push_back(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
                    octree.updateNode(ptt,true,true);
                }
                cnt++;
            }
        }
    }
    //Update grid
    octree.updateInnerOccupancy();

    //init visualization markers:
    visualization_msgs::MarkerArray occupiedNodesVis;
    unsigned int m_treeDepth = octree.getTreeDepth();

    //each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(m_treeDepth + 1);
    geometry_msgs::Point cubeCenter;

    std_msgs::ColorRGBA m_color;
    m_color.r = 0.0;
    m_color.g = 0.0;
    m_color.b = 1.0;
    m_color.a = 0.5;

    for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
    {
        double size = octree.getNodeSize(i);
        occupiedNodesVis.markers[i].header.frame_id = "/map";
        occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
        occupiedNodesVis.markers[i].ns = "map";
        occupiedNodesVis.markers[i].id = i;
        occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].scale.x = size;
        occupiedNodesVis.markers[i].scale.y = size;
        occupiedNodesVis.markers[i].scale.z = size;
        occupiedNodesVis.markers[i].color = m_color;
    }

    ROS_INFO("s %i",cnt++);
    x = gridPtr->positionX;
    y = gridPtr->positionY;


    for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
    {
        if(it != NULL && octree.isNodeOccupied(*it))
        {
            unsigned idx = it.getDepth();
            cubeCenter.x = x+it.getX();
            cubeCenter.y = y+it.getY();
            cubeCenter.z = it.getZ();
            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            double minX, minY, minZ, maxX, maxY, maxZ;
            octree.getMetricMin(minX, minY, minZ);
            octree.getMetricMax(maxX, maxY, maxZ);
            double h = (1.0 - fmin(fmax((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColorA(h));
        }
    }

    /**** Robot Head Marker ****/

    //Robot Position


    visualization_msgs::Marker marker_head;
    marker_head.header.frame_id = "/map";
    marker_head.header.stamp = ros::Time();
    marker_head.ns = "my_namespace";
    marker_head.id = 1;
    marker_head.type = visualization_msgs::Marker::SPHERE;
    marker_head.action = visualization_msgs::Marker::ADD;
    marker_head.pose.position.x = goal->x;
    marker_head.pose.position.y = goal->y;
    marker_head.pose.position.z = head_height;
    marker_head.pose.orientation.x = 0.0;
    marker_head.pose.orientation.y = 0.0;
    marker_head.pose.orientation.z = 0.0;
    marker_head.pose.orientation.w = 1.0;
    marker_head.scale.x = 0.2;
    marker_head.scale.y = 0.2;
    marker_head.scale.z = 0.2;
    marker_head.color.a = 1.0;
    marker_head.color.r = 0.0;
    marker_head.color.g = 1.0;
    marker_head.color.b = 0.0;


    /****** Ray Traversal ******/

    //ROS_INFO("Robot Position (%f,%f,%f) = (%d,%d,%d)", goal->x, goal->y, head_height, cell_x, cell_y, cell_z);

    //Ray Casting (Grid Traversal - Digital Differential Analyzer)

    visualization_msgs::Marker marker_rays;
    marker_rays.header.frame_id = "/map";
    marker_rays.header.stamp = ros::Time();
    marker_rays.ns = "my_namespace";
    marker_rays.id = 2;
    marker_rays.type = visualization_msgs::Marker::LINE_LIST;
    marker_rays.action = visualization_msgs::Marker::ADD;
    marker_rays.scale.x = 0.01;


    geometry_msgs::Vector3 rayDirection, deltaT, cellIndex;

    std_msgs::ColorRGBA line_color;

    initialPt.x = goal->x;
    initialPt.y = goal->y;
    initialPt.z = head_height;

    line_color.a = 0.2;
    line_color.r = 0.0;
    line_color.g = 0.0;
    line_color.b = 1.0;

    float delta, H;
    bool free_cell;

    ROS_INFO("Performing Ray Casting...");
            H = 0;
    int gridsize = dim_x*dim_y*dim_z;
    for(float ang_v = -0.174; ang_v < 0.174; ang_v+=angular_step){
        for(float ang_h = 0; ang_h < 2*M_PI; ang_h+= angular_step){ //0 - 360 degrees
            //Initial conditions:
            rayDirection.z = sin(ang_v);//z
            rayDirection.x = cos(ang_v) * cos(ang_h);//x
            rayDirection.y = cos(ang_v) * sin(ang_h);//y

            delta = fmax(fmax(fabs(rayDirection.x), fabs(rayDirection.y)), fabs(rayDirection.z));

            deltaT.x = rayDirection.x/delta;
            deltaT.y = rayDirection.y/delta;
            deltaT.z = rayDirection.z/delta;

            free_cell = true;


            int max_it = RANGE_MAX/resolution * 2/sqrt(pow(deltaT.x,2) + pow(deltaT.y,2) + pow(deltaT.z,2));

            //    ROS_INFO("Max_it: %d %d %d", cell_x,cell_y,cell_z);

            finalPt.x = 0;
            finalPt.y = 0;
            finalPt.z = 0;

            for(int i = 0; i < max_it && free_cell; i++){
                finalPt.x += deltaT.x/2;
                finalPt.y += deltaT.y/2;
                finalPt.z += deltaT.z/2;

                //cnt?
                int cnt = ((int)((cell_x+finalPt.x-LIM_MIN_X/resolution))*dim_y + (int)(finalPt.y + cell_y-LIM_MIN_Y/resolution))*dim_z + (int)(finalPt.z + cell_z-LIM_MIN_Z/resolution);//get fremen grid index!

                //TODO
                if (cnt < 0){
                    cnt = 0;
                    free_cell = false;
                }
                if (cnt > gridsize-1){
                    cnt = gridsize-1;
                    free_cell = false;
                }
                //ROS_INFO("CNT: %d %d", cnt,gridsize);
                //ROS_INFO("DIM %d %d", dim_x, dim_z);
                if(gridPtr->retrieve(cnt, goal->stamp) > 0) free_cell = false;
                if(aux_entropy[cnt] ==0){
                    aux_entropy[cnt] = 1;
                    //float p = gridPtr->estimate(cnt,goal->stamp);
                    //h+=-p*ln(p);
                    H++;
                }
            }


            marker_rays.points.push_back(initialPt);
            marker_rays.colors.push_back(line_color);
            finalPt.x = finalPt.x*resolution+initialPt.x;
            finalPt.y = finalPt.y*resolution+initialPt.y;
            finalPt.z = finalPt.z*resolution+initialPt.z;
            marker_rays.points.push_back(finalPt);
            marker_rays.colors.push_back(line_color);

        }
    }

    //Entropy (text marker):

    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "/map";
    marker_text.header.stamp = ros::Time();
    marker_text.ns = "my_namespace";
    marker_text.id = 1;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.pose.position.x = goal->x;
    marker_text.pose.position.y = goal->y;
    marker_text.pose.position.z = head_height + 2;
    marker_text.pose.orientation.x = 0.0;
    marker_text.pose.orientation.y = 0.0;
    marker_text.pose.orientation.z = 0.0;
    marker_text.pose.orientation.w = 1.0;
    marker_text.scale.z = 0.5;
    marker_text.color.a = 1.0;
    marker_text.color.r = 0.0;
    marker_text.color.g = 1.0;
    marker_text.color.b = 0.0;
    char output[1000];
    sprintf(output,"Gain: %f",H);
    marker_text.text = output;


    //Publish Results:
    ROS_INFO("Data published!");
    head_pub_ptr->publish(marker_head);
    estimate_pub_ptr->publish(occupiedNodesVis);
    rays_pub_ptr->publish(marker_rays);
    text_pub_ptr->publish(marker_text);


    as->setSucceeded();

}


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "information_gain");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    nh.param("resolution", resolution, 0.05);
    nh.param("head_height", head_height, 2.0);
    nh.param("angular_step", angular_step, 0.01);
    nh.param("dim_x", dim_x, 1000);
    nh.param("dim_y", dim_y, 1000);
    nh.param("dim_z", dim_z, 20);
    nh.param<std::string>("filename", filename, "/home/santos/test4_room1.froctomap");


    gridPtr = NULL;
    gridPtr = new CFremenGrid(dim_x,dim_y,dim_z);
    aux_entropy= (int8_t*)calloc(dim_x * dim_y * dim_z,sizeof(int8_t));

    if(!gridPtr->load(filename.c_str())){
        ROS_INFO("Error loading the file!");
        return(0);
    }else
        ROS_INFO("Load sucessfull!");


    //Publishers:
    ros::Publisher estimate_pub = n.advertise<visualization_msgs::MarkerArray>("/estimated_grid", 100);
    estimate_pub_ptr = &estimate_pub;

    ros::Publisher head_pub = n.advertise<visualization_msgs::Marker>("/head", 100);
    head_pub_ptr = &head_pub;

    ros::Publisher rays_pub = n.advertise<visualization_msgs::Marker>("/rays", 100);
    rays_pub_ptr = &rays_pub;

    ros::Publisher text_pub = n.advertise<visualization_msgs::Marker>("/text", 100);
    text_pub_ptr = &text_pub;

    //Server:
    Server server(n, "information_server", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}

