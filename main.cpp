#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>


#define PI 3.1415926
#define ROBOT_RADIOS 10

using namespace std;
using namespace Eigen;
using namespace cv;
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> DPoint;
typedef bg::model::segment<DPoint> DSegment;

class Sensor
{
public:
    Sensor(double angle,Vector2d init_pose)
    {
        this->angle = angle;
        this->init_pose = init_pose;
    }

public:
    double angle = 0.0;
    double length = 200.0;
    Vector2d init_pose = {0,0};
    double data = 200.0;

public:
    double GetData(DSegment wall)
    {

        DSegment detect_line(DPoint(this->init_pose.x(),this->init_pose.y()),
                             DPoint(this->init_pose.x() + length * cos(angle),this->init_pose.y() + length * sin(angle)));

        std::list<DPoint> lstPoints;
        bg::intersection(detect_line, wall, lstPoints);

        if(!lstPoints.empty())
        {
            return this->data = bg::distance(lstPoints.back(), DPoint(init_pose.x(),init_pose.y()));
        }
        else
        {
            return this->data = 200.0;
        }
    }
};

class Robot
{
public:
    Vector2d center_pose = {0,0};
    double l_speed = 0.0,r_speed = 0.0;
    double direction = 0.0;                 //angle to x-axis
    vector<Sensor> sensors;
    vector<double> sensors_data;
    double map_x_max = 200 - ROBOT_RADIOS;
    double map_y_max = 100 - ROBOT_RADIOS;

private:
    double v_bound = 3.0;                   //max speed bound
    double speed_step = 0.1;               //Speed increase step
    double l = 8;                         //the distance of the two wheels
    double delta_t = 1;                   //time interval
    Vector2d ICC;                           //Instantaneou Center of Curvature
    double R = 0.0;                         //from ICC to center
    double omega = 0.0;                     //angle speed

public:
    //Receive keyboard information to control robot speed
    void SpeedControl(char order)
    {
        switch(order)
        {
            case 'W':
                l_speed = min(v_bound,l_speed + speed_step);
                break;
            case 'S':
                l_speed = max(-v_bound,l_speed - speed_step);
                break;
            case 'O':
                r_speed = min(v_bound,r_speed + speed_step);
                break;
            case 'L':
                r_speed = max(-v_bound,r_speed - speed_step);
                break;
            case 'X':
                l_speed = 0.0;
                r_speed = 0.0;
                break;
            case 'T':
                l_speed = min(v_bound,l_speed + speed_step);
                r_speed = min(v_bound,r_speed + speed_step);
                break;
            case 'G':
                l_speed = max(-v_bound,l_speed - speed_step);
                r_speed = max(-v_bound,r_speed - speed_step);
                break;
            default:
                break;
        }
    }

    //robot movement update
    void Move(vector<DSegment> virtual_wall_set)
    {
        if(l_speed == 0 && r_speed == 0)
        {
            return;
        }
        else if(l_speed + r_speed == 0)
        {
            OmegaCalculate();
            direction += omega * delta_t;
            return;
        }
        // to use center pose
        double forward_distance = (l_speed + r_speed) / 2 * delta_t;
        DPoint forward_point(center_pose.x() + forward_distance * cos(direction),center_pose.y() + forward_distance * sin(direction));
        DSegment forward_seg(DPoint(center_pose.x(),center_pose.y()),forward_point);
        double forward_angle = atan2((forward_seg.second.y() - forward_seg.first.y()),(forward_seg.second.x() - forward_seg.first.x()));
        cout<< "forward_angle: " << forward_angle <<endl;
        cout<< "sensors_data[forward_angle/PI*6]: " << sensors_data[(forward_angle - direction)/PI*6]  <<endl;

        std::list<DPoint> intersction_Points;
        double virtual_wall_angle;
        for(DSegment virtual_wall : virtual_wall_set)
        {
            virtual_wall_angle = atan((virtual_wall.second.y() - virtual_wall.first.y())/(virtual_wall.second.x() - virtual_wall.first.x()));
            bg::intersection(virtual_wall, forward_seg, intersction_Points);
            cout<<"virtual_wall\t1x : " <<virtual_wall.first.x() << "\t1y : " <<virtual_wall.first.y()
                <<"\t2x : " << virtual_wall.second.x()<<"\t2y : " << virtual_wall.second.y()<<endl;
            cout<<"forward_seg\t1x : " <<forward_seg.first.x() << "\t1y : " <<forward_seg.first.y()
                <<"\t2x : " << forward_seg.second.x()<<"\t2y : " << forward_seg.second.y()<<endl;
            if(!intersction_Points.empty())
            {
                break;
            }
        }
        //DSegment virtual_wall(DPoint(wall.first.x()-5,wall.first.y()),DPoint(wall.second.x()-5,wall.second.y()));

        if(l_speed == r_speed)
        {
            if(!intersction_Points.empty() && sensors_data[(forward_angle - direction)/PI*6] < 5 * ROBOT_RADIOS)
            {
                double collision_distance = bg::distance(intersction_Points.back(), DPoint(center_pose.x(), center_pose.y()));
                cout <<"size : "<< intersction_Points.size()<< "\tcollision_distance : " << collision_distance << endl;
                double collision_time = collision_distance / l_speed;
                double rest_time = delta_t - collision_time;
                double collision_speed = l_speed * cos(virtual_wall_angle - direction);
                center_pose = {intersction_Points.back().x(),intersction_Points.back().y()};
                Translation2d current_translation(collision_speed * rest_time * cos(virtual_wall_angle),collision_speed * rest_time * sin(virtual_wall_angle));
                center_pose = current_translation * center_pose;
                cout<< "wall_angle: " << virtual_wall_angle <<" \trest_time : "<< rest_time
                <<"\tcollision_speed : "<< collision_speed <<"\tcollision_distance : "<< collision_distance<<"\tcollision_time : "<< collision_time<<endl;
            }
            else
            {
                Translation2d current_translation(l_speed * delta_t * cos(direction),l_speed * delta_t * sin(direction));
                center_pose = current_translation * center_pose;
            }
        }
        else
        {
            if(!intersction_Points.empty() && sensors_data[(forward_angle - direction)/PI*6] < 5 * ROBOT_RADIOS)
            {
                double ave_speed = (r_speed + l_speed) / 2;
                double collision_distance = bg::distance(intersction_Points.back(), DPoint(center_pose.x(), center_pose.y()));
                cout <<"size : "<< intersction_Points.size()<< "\tcollision_distance : " << collision_distance << endl;
                double collision_time = collision_distance / ave_speed;
                direction += omega * collision_time;
                double rest_time = delta_t - collision_time;
                double collision_speed = ave_speed * cos(virtual_wall_angle - direction);
                center_pose = {intersction_Points.back().x(),intersction_Points.back().y()};
                Translation2d current_translation(collision_speed * rest_time * cos(virtual_wall_angle),collision_speed * rest_time * sin(virtual_wall_angle));
                center_pose = current_translation * center_pose;
                cout<< "wall_angle: " << virtual_wall_angle <<" \trest_time : "<< rest_time
                    <<"\tcollision_speed : "<< collision_speed <<"\tcollision_distance : "<< collision_distance<<"\tcollision_time : "<< collision_time<<endl;
            }
            else
            {
                RCalculate();
                OmegaCalculate();
                Translation2d ICC_to_center(R * cos(direction + 0.5 * PI),R * sin(direction + 0.5 * PI));
                ICC = ICC_to_center * center_pose;
                center_pose = Translation2d(ICC) * Rotation2Dd(omega * delta_t) * Translation2d(ICC).inverse() * center_pose;
                direction += omega * delta_t;
            }
        }
        if(center_pose.x() > map_x_max)
        {
            center_pose.x() =  map_x_max;
        }
        if(center_pose.x() < -map_x_max)
        {
            center_pose.x() =  -map_x_max;
        }
        if(center_pose.y() > map_y_max)
        {
            center_pose.y() =  map_y_max;
        }
        if(center_pose.y() < -map_y_max)
        {
            center_pose.y() =  -map_y_max;
        }
        if(direction > 2 * PI)
        {
            direction -= 2*PI;
        }
        if(direction >  PI)
        {
            direction -= 2*PI;
        }
        if(direction < -PI)
        {
            direction += 2*PI;
        }

    }

    //calculate the sensors data
    void GetAllData(vector<DSegment> wall_set)
    {
        for(int i = 0; i < 12; i++)
        {
            sensors.push_back(Sensor(direction + i * PI / 6,this->center_pose));
            //cout << "init_pose : "<<this->center_pose.x()<< '\t' <<this->center_pose.x() <<endl;
        }
        cout<<"the sensors' data : ";
        for(int i = 0; i < 12; i++)
        {
            double min_sensor_data = 200.0;
            for(DSegment wall : wall_set)
            {
                min_sensor_data = min(min_sensor_data,sensors[i].GetData(wall));
            }
            sensors_data.push_back(min_sensor_data);
            cout << "\t" << min_sensor_data;
        }
        cout<<endl;
    }

    //clear all data
    void ClearData()
    {
        sensors.clear();
        sensors_data.clear();
    }

private:
    //R calculate
    void RCalculate()
    {
        this->R = 0.5 * (l_speed + r_speed) / (r_speed - l_speed);
    }

    //omega calculate
    void OmegaCalculate()
    {
        this->omega = (r_speed - l_speed) / l;
    }
};

int main() {
    Robot timi;
    char order;
    DSegment virtual_wallN(DPoint(-200 ,100 - ROBOT_RADIOS),DPoint(200,100 - ROBOT_RADIOS));
    DSegment virtual_wallS(DPoint(-200,-100 + ROBOT_RADIOS),DPoint(200,-100 + ROBOT_RADIOS));
    DSegment virtual_wallW(DPoint(-200 + ROBOT_RADIOS,-100),DPoint(-200 + ROBOT_RADIOS,100));
    DSegment virtual_wallE(DPoint(200 - ROBOT_RADIOS,-100),DPoint(200 - ROBOT_RADIOS,100));
    vector<DSegment> virtual_wall_set = {virtual_wallN,virtual_wallS,virtual_wallW,virtual_wallE};

    DSegment wallN(DPoint(-200 ,100 ),DPoint(200,100 ));
    DSegment wallS(DPoint(-200,-100 ),DPoint(200,-100));
    DSegment wallW(DPoint(-200 ,-100),DPoint(-200 ,100));
    DSegment wallE(DPoint(200 ,-100),DPoint(200,100));
    vector<DSegment> wall_set = {wallN,wallS,wallW,wallE};

    while(order != 27)
    {

        //get all the sensors data
        timi.ClearData();
        timi.GetAllData(wall_set);

        //robot move control
        timi.SpeedControl(order);
        timi.Move(virtual_wall_set);
        cout << "order : " << order <<"\ncenter_pose : \n" << timi.center_pose
        << "\nl_speed : \t" << timi.l_speed << "\tr_speed : \t" << timi.r_speed<< endl;

        //visualization
        Mat img = Mat::zeros(Size(800, 600), CV_8UC3);
        img.setTo(255);
        Point p(timi.center_pose.x() + 400,timi.center_pose.y() + 300);
        circle(img, p, ROBOT_RADIOS, Scalar(0, 255, 0), -1);  //robot body
        line(img, Point(-200 + 400 , 100 + 300), Point(200 + 400 , 100 + 300), Scalar(0, 255, 255), 3);
        line(img, Point(-200 + 400 , -100 + 300), Point(200 + 400 , -100 + 300), Scalar(0, 255, 255), 3);
        line(img, Point(-200 + 400 , -100 + 300), Point(-200 + 400 , 100 + 300), Scalar(0, 255, 255), 3);
        line(img, Point(200 + 400 , -100 + 300), Point(200 + 400 , 100 + 300), Scalar(0, 255, 255), 3);
        imshow("robot simulation", img);
        order = waitKey(0);
    }

    destroyAllWindows();
    std::cout << "End Simulation!" << std::endl;
    return 0;
}
