#include <ros/ros.h>

/////////////////////////////////////
// msgs
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

/////////////////////////////////////
// services
#include "regulator_interface/ServiceCameraImage.h"
#include "regulator_interface/ServiceLaser.h"
#include "regulator_interface/ServiceOdometry.h"

#include "garthimlib/garthim.h"

#define TEST_SERVICE_CAMERA_1 false
#define TEST_SERVICE_CAMERA_2 false
#define TEST_SERVICE_CAMERA_3 false
#define TEST_SERVICE_LASER false
#define TEST_SERVICE_ODOMETRY true

# define PI           3.14159265358979323846  /* pi */


const static double MIN_SCAN_ANGLE_RAD = -29.883638/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = 30.038820/180*M_PI;
const static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max


#define DENSIDAD_1_CAMARA 1.0 //100% de los datos
#define DENSIDAD_2_CAMARA 0.9 //90% de los datos
#define DENSIDAD_3_CAMARA 0.8 //80% de los datos
#define DENSIDAD_4_CAMARA 0.75 //75% de los datos
#define DENSIDAD_5_CAMARA 0.66 //66.6% de los datos
#define DENSIDAD_6_CAMARA 0.5 //50% de los datos
#define DENSIDAD_7_CAMARA 0.4 //40% de los datos


int main(int argc, char **argv)
{
	ros::init(argc, argv, "regulator_interfaceClient");
	ros::NodeHandle n;


	/////////////////////////////////////////

	ros::Publisher laserMio = n.advertise<sensor_msgs::LaserScan>("laserMio", 50);
	ros::Publisher imageMio = n.advertise<sensor_msgs::Image>("imageMio", 50);
	ros::Publisher odometryMio = n.advertise<nav_msgs::Odometry>("odometryMio", 50);


	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////

	int densidades=200;
	int s=20;
	int alpha=10;
	MultiDensidadStateBased md(densidades,s,alpha);




	ofstream file;
	file.open ("/home/benji/Escritorio/algoritmos/resultados/regulador/statebased/log/log-7.txt", ios::out); 

	


	ros::Rate loop_rate(10);

	int k=0;
	double t_anterior=101604.0,t, t_s;
	double t0_h,t1_h,t_h;
	double t0_hs,t1_hs,t_hs;

	//double t_d,t0_d,t1_d;
	k=80;
	for(int j=3; j<5;j++)
	{
		ofstream file;
		string name_file="/home/benji/Escritorio/algoritmos/resultados/regulador/statebased/log/log-";
		name_file = name_file + to_string(j) + ".txt";
		file.open (name_file, ios::out);
		

		while (ros::ok() && k<100)
		{
		  	//t0_d = clock();
		  	//double t_i,t0_i,t1_i;
		  	for (int i = 0; i < densidades;)
		  	{
		  		//t0_i = clock();

		  		ros::ServiceClient clientCamera1 = n.serviceClient<regulator_interface::ServiceCameraImage>("Service_Camera_GrayScale");
				regulator_interface::ServiceCameraImage srvCamera1;
				srvCamera1.request.timeout = 1000.0; //en milisegundos
				//srvCamera1.request.density = DENSIDAD_1_CAMARA;
				srvCamera1.request.density = i;

				//double t,t0,t1=0,t_s,t1_s,t0_s,t_t,t1_t,t0_t;

				//Toma de tiempos 1
				//t0_t = clock();
				

				auto start = std::chrono::system_clock::now();
				t0_h = clock();
				if (clientCamera1.call(srvCamera1))
				{
					//Toma de tiempos 2
					t1_h = clock();
					auto end = std::chrono::system_clock::now();
					auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
					//auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end-start);

					//////////////////

					start = std::chrono::system_clock::now();
					t0_hs = clock();
					//md.StateBasedAlgorithm(i,t,s,alpha);
					double value = md.updateModel(i,(double)elapsed.count()/(double)1000);
					t1_hs = clock();
					end = std::chrono::system_clock::now();
					auto elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
					//Toma tioempos final
					//t1_t = clock();

					///////////////
					t_h =  (double(t1_h-t0_h)/CLOCKS_PER_SEC);
					t_hs = (double(t1_hs-t0_hs)/CLOCKS_PER_SEC);
					t = (double)elapsed.count()/(double)1000;
					t_s = (double)elapsed_s.count();
					t_anterior += t;
					//t_s =  (double(t1_s-t0_s)/CLOCKS_PER_SEC);
					//t_t =  (double(t1_t-t0_t)/CLOCKS_PER_SEC);
					//////////////

					ROS_INFO("%f",t_s);
					//file<<i<<","<<t_anterior<<","<<t<<", "<<"statebased: "<<t_s<<",total: "<<t_t<<", nuevo medido: "<<(double)elapsed.count()/(double)1000;
					file<<i<<","<<t<<","<<(double)t_anterior<<","<<t_s<<","<<t_h<<","<<t_hs<<endl;
					//statebased.StatedBasedModelingAlgortihm(s,t,alpha);

					//ROS_INFO("i = %d, t = %f, t_s = %f, t_t = %f, nuevo = %f",i,t,t_s,t_t,(double)elapsed.count()/(double)1000);

					//////////////////
					ROS_INFO("Respuesta recibida Service_Camera_GrayScale densidad: %d - vuelta: %d",i,k);
					//sensor_msgs::PointCloud2 msg = srvCamera1.response.points;
					sensor_msgs::Image msg = srvCamera1.response.image;
					//int image = srvCamera1.response.image;
					//ROS_INFO("%u",(int) image);
					//ROS_INFO("height = %d, width = %d",msg->height, msg->width);
					//ROS_INFO("height = %d, width = %d",msg.height, msg.width);
					i++;

					



				}
				else
				{
					ROS_ERROR("Failed to call Service_Camera_GrayScale");
					//return 1;
				}


				//t1_i = clock();
				//t_i =  (double(t1_i-t0_i)/CLOCKS_PER_SEC);
				//file<<" ,total iteración: "<<t_i<<endl;
				ros::spinOnce();
		  	}

	  		k++;
	   
			
		



		    //ros::spinOnce();

		    //double t,t0,t1=0;
		    //t0 = clock();
		    //loop_rate.sleep();
		    //t1 = clock();
		    //file<<"loop_rate: "<<t<<endl;

		    //t1_d = clock();

		    //t_d =  (double(t1_d-t0_d)/CLOCKS_PER_SEC);
			//file<<" TOTAL todas las densidades: "<<t_d<<endl;
	    
		}

		file.close();
		k=0;

	}



md.PrintResults();









	/////////////////////////////////////////
	// Servicio camara 1

	
	





  return 0;
}
