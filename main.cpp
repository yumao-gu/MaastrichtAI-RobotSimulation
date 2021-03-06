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
#define ROBOT_RADIOS 25

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
    Vector2d center_pose = {200,200};
    double l_speed = 0.0,r_speed = 0.0;
    double direction = 0.0;                 //angle to x-axis
    vector<Sensor> sensors;
    vector<double> sensors_data;

private:
    double v_bound = 40.0;                   //max speed bound
    double speed_step = 10.0;               //Speed increase step
    double l = 40;                         //the distance of the two wheels
    double delta_t = 0.3;                   //time interval
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
//        DPoint backward_point(center_pose.x() - 0.01 * cos(direction),center_pose.y() - 0.01 * sin(direction));
        DSegment forward_seg(DPoint(center_pose.x(),center_pose.y()),forward_point);
//        DSegment forward_seg(backward_point,forward_point);
        double forward_angle = atan2((forward_seg.second.y() - forward_seg.first.y()),(forward_seg.second.x() - forward_seg.first.x()));
        int forward_sensor_id =abs((int)((forward_angle - direction)/PI*6)%12);
        cout<< "forward_sensor_id: " << forward_sensor_id <<endl;
        std::list<DPoint> intersction_Points;
        double virtual_wall_angle;
        DSegment virtual_wall;
        for(vector<DSegment>::iterator it = virtual_wall_set.begin(); it != virtual_wall_set.end();it++)
        {
            virtual_wall = *it;
            virtual_wall_angle = atan((virtual_wall.second.y() - virtual_wall.first.y())/(virtual_wall.second.x() - virtual_wall.first.x()));
            bg::intersection(virtual_wall, forward_seg, intersction_Points);

            double ddistance = 100.0;
            ddistance = bg::distance(DPoint(center_pose.x(), center_pose.y()), virtual_wall);
            if(intersction_Points.empty() && ddistance < 0.01)
            {
                intersction_Points.push_back(DPoint(center_pose.x(), center_pose.y()));
            }

            if(!intersction_Points.empty())
            {
                virtual_wall_set.erase(it);
                break;
            }
        }


        cout<<"directoin\t"<<direction<<"\tvirtual_wall_angle\t"<<virtual_wall_angle<<endl;
        cout<<"center_pose origin on the line:\t"<<bg::distance(DPoint(center_pose.x(), center_pose.y()), virtual_wall)<<endl;
        cout<<"intersction_Points.empty()\t"<<intersction_Points.empty()<<"\tsensors_data[forward_sensor_id]\t"<<sensors_data[forward_sensor_id]<<endl;
        if(!intersction_Points.empty() && sensors_data[forward_sensor_id] < 5 * ROBOT_RADIOS)
        {
            cout<<"collision occurs"<<endl;

            double collision_distance = bg::distance(intersction_Points.back(), DPoint(center_pose.x(), center_pose.y()));

            double collision_time = collision_distance / l_speed;
            double rest_time = delta_t - collision_time;
            double collision_speed = l_speed * cos(virtual_wall_angle - direction);

            center_pose = {intersction_Points.back().x(),intersction_Points.back().y()};
            cout<<"collision_speed\t"<<collision_speed<<"\tcenter_pose\n"<<center_pose<<endl;
            cout<<"center_pose on the line:\t"<<bg::distance(DPoint(center_pose.x(), center_pose.y()), virtual_wall)<<endl;
            //Translation2d current_translation(collision_speed * rest_time * cos(virtual_wall_angle),collision_speed * rest_time * sin(virtual_wall_angle));
            //Vector2d center_pose_2nd = current_translation * center_pose;
            Vector2d center_pose_2nd = {center_pose.x() + collision_speed * rest_time * cos(virtual_wall_angle),
                                        center_pose.y() + collision_speed * rest_time * sin(virtual_wall_angle)};
//            DSegment center_pose_2 = DSegment(DPoint(center_pose_2nd.x(),center_pose_2nd.y()+1.0),DPoint(center_pose_2nd.x(),center_pose_2nd.y()-1.0));
//            std::list<DPoint> intersction2_Points;
//            bg::intersection(virtual_wall, center_pose_2, intersction2_Points);
//            center_pose_2nd = {intersction2_Points.back().x(),intersction2_Points.back().y()};
            cout<<"collision_speed\t"<<collision_speed<<"\tcenter_pose_2nd\n"<<center_pose_2nd<<endl;
            cout<<"center_pose_2nd on the line:\t"<<bg::distance(DPoint(center_pose_2nd.x(), center_pose_2nd.y()), virtual_wall)<<endl;

            if(l_speed != r_speed)
            {
                direction += omega * collision_time;
            }
            for(DSegment virtual_wall_2 : virtual_wall_set)
            {
                double virtual_wall_angle_2nd = atan((virtual_wall_2.second.y() - virtual_wall_2.first.y())/(virtual_wall_2.second.x() - virtual_wall_2.first.x()));
                std::list<DPoint> intersction_Points_2nd;
                DSegment forward_seg_2nd(DPoint(center_pose.x(),center_pose.y()),DPoint(center_pose_2nd.x(),center_pose_2nd.y()));

                bg::intersection(virtual_wall_2, forward_seg_2nd, intersction_Points_2nd);
//                cout << "forward_seg_2nd : " << "\t" << forward_seg_2nd.first.x() << "\t" << forward_seg_2nd.first.y()
//                << "\t" << forward_seg_2nd.second.x() << "\t" << forward_seg_2nd.second.y() << endl;
                cout<<"intersction_Points_2nd.empty()\t"<<intersction_Points_2nd.empty()
                    <<"\t2nd distance\t"<<bg::distance(forward_seg_2nd, virtual_wall_2)<<endl;
                if(bg::distance(forward_seg_2nd, virtual_wall_2) < 0.01)
                {
                    bg::intersection(virtual_wall_2, virtual_wall, intersction_Points_2nd);
                }

                if(intersction_Points_2nd.empty())
                {
                    continue;
                }
                else
                {
                    cout<<"collision_time_2nd occurs"<<endl;
                    double collision_distance_2nd = bg::distance(intersction_Points_2nd.back(), DPoint(center_pose.x(), center_pose.y()));
//                    double collision_time_2nd = collision_distance_2nd / collision_speed;
//                    double rest_time_2nd = rest_time - collision_time_2nd;
//                    double collision_speed_2nd = collision_speed * cos(virtual_wall_angle_2nd - virtual_wall_angle);
                    center_pose_2nd = {intersction_Points_2nd.back().x(),intersction_Points_2nd.back().y()};
                    //Translation2d current_translation_2nd(collision_speed_2nd * rest_time_2nd * cos(virtual_wall_angle_2nd),collision_speed_2nd * rest_time_2nd * sin(virtual_wall_angle_2nd));
//                    center_pose_2nd = current_translation_2nd * center_pose_2nd;
                    break;
                }
            }
            center_pose = center_pose_2nd;
        }
        else
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
    struct GridMap_Trapezoid
    {
        vector<DSegment> virtual_wall_set={
                DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,400-ROBOT_RADIOS)),
                DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,100+ROBOT_RADIOS)),
                DSegment (DPoint(300-ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,300-ROBOT_RADIOS)),
                DSegment (DPoint(0+ROBOT_RADIOS,400-ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,300-ROBOT_RADIOS))};
        vector<DSegment> wall_set ={
                DSegment (DPoint(0,0),DPoint(0,400)),
                DSegment (DPoint(0,0),DPoint(300,100)),
                DSegment (DPoint(300,100),DPoint(300,300)),
                DSegment (DPoint(0,400),DPoint(300,300))};
        void map_show(Mat& img)
        {
            line(img, Point(0 , 0), Point(0 , 400), Scalar(0, 255, 255), 3);
            line(img, Point(0 , 0), Point(300 , 100), Scalar(0, 255, 255), 3);
            line(img, Point( 300, 100), Point(300 , 300), Scalar(0, 255, 255), 3);
            line(img, Point( 0, 400), Point(300 , 300), Scalar(0, 255, 255), 3);
        }
        MatrixXd grid_map = MatrixXd::Zero(400,400);
    } A;
    DSegment virtual_wallN(DPoint(-200 + ROBOT_RADIOS,100 - ROBOT_RADIOS),DPoint(200 - ROBOT_RADIOS,100 - ROBOT_RADIOS));
    DSegment virtual_wallS(DPoint(-200 + ROBOT_RADIOS,-100 + ROBOT_RADIOS),DPoint(200 - ROBOT_RADIOS,-100 + ROBOT_RADIOS));
    DSegment virtual_wallW(DPoint(-200 + ROBOT_RADIOS,-100 + ROBOT_RADIOS),DPoint(-200 + ROBOT_RADIOS,100 - ROBOT_RADIOS));
    DSegment virtual_wallE(DPoint(200 - ROBOT_RADIOS,-100 + ROBOT_RADIOS),DPoint(200 - ROBOT_RADIOS,100 - ROBOT_RADIOS));
    vector<DSegment> virtual_wall_set = {virtual_wallN,virtual_wallS,virtual_wallW,virtual_wallE};

    DSegment wallN(DPoint(-200 ,100 ),DPoint(200,100 ));
    DSegment wallS(DPoint(-200,-100 ),DPoint(200,-100));
    DSegment wallW(DPoint(-200 ,-100),DPoint(-200 ,100));
    DSegment wallE(DPoint(200 ,-100),DPoint(200,100));
    vector<DSegment> wall_set = {wallN,wallS,wallW,wallE};

    while(order != 27)
    {
        std::cout << "BEGIN A NEW STEP !" << std::endl;
        //get all the sensors data
        timi.ClearData();
        timi.GetAllData(A.wall_set);

        //robot move control
        timi.SpeedControl(order);
        timi.Move(A.virtual_wall_set);
        cout << "order : " << order <<"\ncenter_pose : \n" << timi.center_pose
        << "\nl_speed : \t" << timi.l_speed << "\tr_speed : \t" << timi.r_speed<< endl;

        //visualization
        Mat img = Mat::zeros(Size(400, 400), CV_8UC3);
        img.setTo(255);
        Point p(timi.center_pose.x(),timi.center_pose.y());
        circle(img, p, ROBOT_RADIOS, Scalar(0, 255, 0), -1);  //robot body
        A.map_show(img);
        imshow("robot simulation", img);
        order = waitKey(0);
    }

    destroyAllWindows();
    std::cout << "End Simulation!" << std::endl;
    return 0;
}
