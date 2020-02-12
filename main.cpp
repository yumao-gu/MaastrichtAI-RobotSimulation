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
//        cout << "GetData init_pose : "<<this->init_pose.x()<< '\t' <<this->init_pose.x() <<endl;
        DSegment detect_line(DPoint(this->init_pose.x(),this->init_pose.y()),
                             DPoint(this->init_pose.x() + length * cos(angle),this->init_pose.y() + length * sin(angle)));

//        cout<<"sensor start : " << detect_line.first.x() << '\t' << detect_line.first.y()<<endl;
//        cout<<"sensor end : " << detect_line.second.x() << '\t' << detect_line.second.y()<<endl;
//        cout<<"wall start : " << wall.first.x() << '\t' << wall.first.y()<<endl;
//        cout<<"wall end : " << wall.second.x() << '\t' << wall.second.y()<<endl;

        std::list<DPoint> lstPoints;
        bg::intersection(detect_line, wall, lstPoints);
        list<DPoint>::iterator it;
//        cout << "intersection: " << endl;
//        for(it = lstPoints.begin();it!=lstPoints.end();it++)
//        {
//            cout << it->x() << '\t' << it->y() <<endl;
//        }
//        cout << "end intersection: " << endl;
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
    void Move()
    {
        if(l_speed == r_speed)
        {
            Translation2d current_translation(l_speed * delta_t * cos(direction),l_speed * delta_t * sin(direction));
            center_pose = current_translation * center_pose;
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

    //calculate the sensors data
    void GetAllData(DSegment wall)
    {
        for(int i = 0; i < 12; i++)
        {
            sensors.push_back(Sensor(direction + i * PI / 6,this->center_pose));
            //cout << "init_pose : "<<this->center_pose.x()<< '\t' <<this->center_pose.x() <<endl;
        }
        cout<<"the sensors' data : ";
        for(int i = 0; i < 12; i++)
        {
            sensors_data.push_back(sensors[i].GetData(wall));
            cout<<sensors[i].GetData(wall)<<'\t';
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
    DSegment wall(DPoint(200,-100),DPoint(200,100));

    while(order != 27)
    {

        //robot move control
        timi.SpeedControl(order);
        timi.Move();
        cout << "order : " << order <<"\ncenter_pose : \n" << timi.center_pose
        << "\nl_speed : \t" << timi.l_speed << "\tr_speed : \t" << timi.r_speed<< endl;

        //get all the sensors data
        timi.ClearData();
        timi.GetAllData(wall);

        //visualization
        Mat img = Mat::zeros(Size(800, 600), CV_8UC3);
        img.setTo(255);
        Point p(timi.center_pose.x() + 400,timi.center_pose.y() + 300);
        circle(img, p, 10, Scalar(0, 255, 0), -1);  //robot body
        line(img, Point(200 + 400 , -100 + 300), Point(200 + 400 , 100 + 300), Scalar(0, 255, 255), 3);
        imshow("robot simulation", img);
        order = waitKey(0);
    }

    destroyAllWindows();
    std::cout << "End Simulation!" << std::endl;
    return 0;
}
