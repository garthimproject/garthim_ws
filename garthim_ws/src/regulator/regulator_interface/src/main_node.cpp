#include <ros/ros.h>

/////////////////////////////////////
// msgs
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/////////////////////////////////////
// services
#include "regulator_interface/ServiceCameraImage.h"
#include "regulator_interface/ServiceLaser.h"
#include "regulator_interface/ServiceOdometry.h"

/////////////////////////////////////
// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

/////////////////////////////////////
// FUTURE
#include <future>
#include <thread>
#include <chrono>

/////////////////////////////////////
// otras
#include <iostream>
#include <vector>
#include <string>


//#include "pcl_ros/point_cloud.h"
//#include <pcl/conversions.h>
//#include <pcl/point_types.h>
//#include "tf/transform_listener.h"
//#include "laser_geometry/laser_geometry.h"


/////////////////////////////////////
/////////////////////////////////////
// DEFINE

# define PI           3.14159265358979323846  /* pi */


/*
Finalizar Tareas future
*/

//define para parar el task correspondiente con cada servicio
#define TERMINAR_SERVICIO_CAMARA_GRAYSCALE 0
#define TERMINAR_SERVICIO_CAMARA_RGB 1
#define TERMINAR_SERVICIO_LASER 3
#define TERMINAR_SERVICIO_ODOMETRIA 4

 // a esperas de como se vaya a proceder
#define TERMINAR_SERVICIO_CAMARA_3 2

//Ver como hacerlos, no aparecen en los topic
#define TERMINAR_SERVICIO_BUMPERS 5
#define TERMINAR_SERVICIO_GIROSCOPIO 6
#define TERMINAR_SERVICIO_ACELEROMETRO 7


/*
Densidades: Camara
*/
#define DENSIDAD_1_CAMARA 1.0 //100% de los datos
#define DENSIDAD_2_CAMARA 0.9 //90% de los datos
#define DENSIDAD_3_CAMARA 0.8 //80% de los datos
#define DENSIDAD_4_CAMARA 0.75 //75% de los datos
#define DENSIDAD_5_CAMARA 0.66 //66.6% de los datos
#define DENSIDAD_6_CAMARA 0.5 //50% de los datos
#define DENSIDAD_7_CAMARA 0.4 //40% de los datos

/*
Densidades: Laser
*/

#define DENSIDAD_1_LASER 1 //100% de los datos
#define DENSIDAD_2_LASER 2 //50% de los datos
#define DENSIDAD_3_LASER 3 //66.6% de los datos
#define DENSIDAD_4_LASER 4 //75% de los datos
#define DENSIDAD_5_LASER 5 //80% de los datos
#define DENSIDAD_8_LASER 8 //87.5% de los datos
#define DENSIDAD_9_LASER 1.33 //25% de los datos
#define ESCALADO 2


std::vector<std::vector<double> > tablaEscalado;


std::vector<std::vector<double> > Ordenar(std::vector<std::vector<double> > v)
{
	std::vector<std::vector<double> > ordenado = v;
	//myfile<<R.size()<<": ";
	for(int i=0; i<ordenado.size()-1;i++)
	{
		for(int j=i+1;j<ordenado.size();j++)
		{
			if(ordenado[i][0] > ordenado[j][0]){
				std::vector<double> aux = ordenado[i];
				ordenado[i] = ordenado[j];
				ordenado[j] = aux;
			}
		}
	}
	return ordenado;
}




/////////////////////////////////////
/////////////////////////////////////
// Variables Globales

//publiser para poder usarlos como depuradores en cualquier función
ros::Publisher pub_scan, pub_cloud;

//flag para imprimir por pantalla los pointclouds
bool printPoints = false;

// Buffers donde se almacena la informacion obtenida de los topic temporalmente
sensor_msgs::PointCloud2 bufferCamera;
sensor_msgs::LaserScan bufferLaser;
sensor_msgs::Image bufferImage,bufferDephImage;
nav_msgs::Odometry bufferOdometry;

//vector flags para terminar las tareas en future, van unido con los defines: Finalizar Tareas future
//posicion 0 -> servicio camara grayscale
//posicion 1 -> servicio camara rgb
//posicion 2 -> servicio camara camera3 (profundidad)
//posicion 3 -> servicio camara laser
//posicion 4 -> servicio camara odometria
//posicion 5 -> servicio camara bumpers
//posicion 6 -> servicio camara giroscopio
//posicion 7 -> servicio camara accelerometro
bool terminar[] = {false,false,false,false,false,false,false,false};





/////////////////////////////////////
/////////////////////////////////////
// Servicios


//IMAGEN en BLANCO Y NEGRO
bool service_camera_grayscale(regulator_interface::ServiceCameraImage::Request &req,
                   regulator_interface::ServiceCameraImage::Response &res)
{
	//ROS_INFO("Recibida la petición en el servicio Camera1");
	//ROS_INFO("Time-out: %u",(unsigned int)req.timeout);
	//ROS_INFO("Density data: %d",(unsigned int)req.density);
	//ROS_INFO("tamaño antes:%d",bufferImage.data.size());

	//////////////////////////
	// FUTURE

	// do something while waiting for function to set future:
	//ROS_INFO("checking, please wait");

	terminar[TERMINAR_SERVICIO_CAMARA_GRAYSCALE]=false;

	std::packaged_task<cv_bridge::CvImagePtr(float)> task([](unsigned int reduccion){ 

		cv_bridge::CvImagePtr cv_imagen;
		try
		{
			//BGR8 - MONO8


			cv::Mat imageMAT;

			//ROS_INFO("antes de convertir %s",bufferImage.encoding.c_str());			

			/*
			if(strcmp(bufferImage.encoding.c_str(), "32FC1")==0)
			{
				bufferImage.encoding = "mono16";
				if(tablaEscalado[reduccion][2]==0)
				{
					ROS_INFO("B/N ENCODING %s",bufferImage.encoding.c_str());
					cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::MONO8);
				}
				else if(tablaEscalado[reduccion][2]==1)
				{
					ROS_INFO("B/N ENCODING %s",bufferImage.encoding.c_str());
					cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::RGB8);	
				}
				
				
			}

			else
			{
				if(tablaEscalado[reduccion][2]==0 && strcmp(bufferImage.encoding.c_str(), "rgb8")==0 )
				{
					ROS_INFO("B/N ENCODING %s",bufferImage.encoding.c_str());
					cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::MONO8);	
				}
				else if(strcmp(bufferImage.encoding.c_str(), "rgb8")==0)
				{
					ROS_INFO("COLOR ENCODING %s",bufferImage.encoding.c_str());
					cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::RGB8);
				}	

			}
			*/
			if(tablaEscalado[reduccion][2]==0 && strcmp(bufferImage.encoding.c_str(), "rgb8")==0 )
			{
				//ROS_INFO("B/N ENCODING %s",bufferImage.encoding.c_str());
				cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::MONO8);	
			}
			else if(strcmp(bufferImage.encoding.c_str(), "rgb8")==0)
			{
				//ROS_INFO("COLOR ENCODING %s",bufferImage.encoding.c_str());
				cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::RGB8);
			}
			
			cv::resize(cv_imagen->image,imageMAT,cv::Size(0,0),tablaEscalado[reduccion][1],tablaEscalado[reduccion][1],cv::INTER_NEAREST);
			

			

			//ROS_INFO("datos imagen: %d %d",cv_imagen->image.rows,cv_imagen->image.cols);
			

			
			

			//ROS_INFO("Tamaño imagen original\n   filas:%d, columnas:%d",cv_imagen->image.rows,cv_imagen->image.cols);
			//ROS_INFO("Tamaño imagen reducida\n   filas:%d, columnas:%d",imageMAT.rows,imageMAT.cols);

			cv_imagen->image = imageMAT;			

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			//ROS_ERROR("cv_bridge exception: ");
		}


		//if(terminar[TERMINAR_SERVICIO_CAMARA_GRAYSCALE]) ROS_INFO("Termina forzado por pasar del timeout");
		//else ROS_INFO("Tarea terminada dentro del timeout");	

	 	return cv_imagen;//devuelvo este valor como modo de prueba,
						  //aquí hay que devolver el resultado obtenido  
	}); // wrap the function


/*    	std::future<int> future = task.get_future();  // get a future
    	std::thread(std::move(task)).detach(); // launch on a thread

	std::future_status status = future.wait_for(std::chrono::milliseconds(req.timeout));*/

	if(req.density>=0 && req.density<200) //porque no queremos el 100% de los datos
	{
		//ROS_INFO("Eso: %s",bufferImage.encoding.c_str());
		//if(strcmp(bufferImage.encoding.c_str(), "rgb8")==0) // o distinto de 32FC1 (imagen en profundidad) que el que da error en raras ocasiones
		//{

			std::future<cv_bridge::CvImagePtr> future = task.get_future();  // get a future
		    //std::thread(std::move(task)).detach(); // launch on a thread
		    std::thread(std::move(task),(unsigned int)req.density).detach(); // launch on a thread

		    std::future_status status = future.wait_for(std::chrono::milliseconds(req.timeout));

			    //en este caso, devolver datos vacios
			//en caso contrario devolver los datos obtenidos por get
			if(status != std::future_status::ready) 
			{
				terminar[TERMINAR_SERVICIO_CAMARA_GRAYSCALE]=true;
				//ROS_INFO("Timeout consumido\n");
				//res.image = bufferImage;// si queremos devolver algo
				return false; //para no devolver nada
			}
			else{
				//ROS_INFO("Terminado con éxito"); //es bloqueante
				//If the shared state is not yet ready (i.e., the provider has not yet set its value or exception), 
				//the function blocks the calling thread and waits until it is ready.
				future.get()->toImageMsg(res.image);
		
			}


		//}


		
	}
	else // 100% de los datos
	{
		res.image = bufferImage; 
	}
	
	///////////////////////////

	//ROS_INFO("Sending back response\n");	
	//ROS_INFO("tamaño despues:%d",(int)res.image.data.size());

	return true;

}


//Servicio para imgagen en Color
bool service_camera_rgb(regulator_interface::ServiceCameraImage::Request &req,
                   regulator_interface::ServiceCameraImage::Response &res)
{
	ROS_INFO("Recibida la petición en el servicio Camera1");
	ROS_INFO("Time-out: %u",req.timeout);
	ROS_INFO("Density data: %d",req.density);


	//////////////////////////
	// FUTURE

	// do something while waiting for function to set future:
	ROS_INFO("checking, please wait");

	terminar[TERMINAR_SERVICIO_CAMARA_RGB]=false;

	std::packaged_task<cv_bridge::CvImagePtr(float)> task([](float reduccion){ 

		cv_bridge::CvImagePtr cv_imagen;
		try
		{
			//BGR8/RGB8 - MONO8

			cv_imagen = cv_bridge::toCvCopy(bufferImage,sensor_msgs::image_encodings::RGB8);

			cv::Mat imageMAT;
			cv::resize(cv_imagen->image,imageMAT,cv::Size(0,0),reduccion,reduccion,CV_INTER_AREA);

			ROS_INFO("Tamaño imagen original\n   filas:%d, columnas:%d",cv_imagen->image.rows,cv_imagen->image.cols);
			ROS_INFO("Tamaño imagen reducida\n   filas:%d, columnas:%d",imageMAT.rows,imageMAT.cols);

			cv_imagen->image = imageMAT;			

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}


		if(terminar[TERMINAR_SERVICIO_CAMARA_RGB]) ROS_INFO("Termina forzado por pasar del timeout");
		else ROS_INFO("Tarea terminada dentro del timeout");	

	 	return cv_imagen;//devuelvo este valor como modo de prueba,
						  //aquí hay que devolver el resultado obtenido  
	}); // wrap the function


/*    	std::future<int> future = task.get_future();  // get a future
    	std::thread(std::move(task)).detach(); // launch on a thread

	std::future_status status = future.wait_for(std::chrono::milliseconds(req.timeout));*/

	if(req.density!=1.0) //No queremos el 100% de los datos
	{
		ROS_INFO("Eso: %s",bufferImage.encoding.c_str());
		if(strcmp(bufferImage.encoding.c_str(), "rgb8")==0) // o distinto de 32FC1 (imagen en profundidad) da error en raras ocasiones
		{

			std::future<cv_bridge::CvImagePtr> future = task.get_future();  // get a future
		    //std::thread(std::move(task)).detach(); // launch on a thread
		    std::thread(std::move(task),req.density).detach(); // launch on a thread

		    std::future_status status = future.wait_for(std::chrono::milliseconds(req.timeout));

			    //en este caso, devolver datos vacios
			//en caso contrario devolver los datos obtenidos por get
			if(status != std::future_status::ready) 
			{
				terminar[TERMINAR_SERVICIO_CAMARA_RGB]=true;
				ROS_INFO("Timeout consumido\n");
				//res.image = bufferImage;// si queremos devolver algo
				return false; //para no devolver nada
			}
			else{
				ROS_INFO("Terminado con éxito"); //es bloqueante
				//If the shared state is not yet ready (i.e., the provider has not yet set its value or exception), 
				//the function blocks the calling thread and waits until it is ready.
				future.get()->toImageMsg(res.image);
		
			}


		}


		
	}
	else //100% de los datos
	{
		res.image = bufferImage; 
	}
	
	///////////////////////////

	ROS_INFO("Sending back response\n");	

	return true;

}

/////////////////////////////////////////
/////////////////////////////////////////
//
//			LASER

// radianes to degrees
#define RAD2DEG(x) ((x)*180./M_PI)

/*const static double MIN_SCAN_ANGLE_RAD = -29.883638/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = 30.038820/180*M_PI;
const static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max*/


#define TEST_LASER_RANGES false


//Servicio para el laser
bool service_laser(regulator_interface::ServiceLaser::Request &req,
                   regulator_interface::ServiceLaser::Response &res)
{
	ROS_INFO("Recibida la petición en el servicio Laser");
	ROS_INFO("Time-out: %u",req.timeout);
	ROS_INFO("Density data: %u",req.density);



 

	//////////////////////////
	// FUTURE

	terminar[TERMINAR_SERVICIO_LASER]=false;

	//std::packaged_task<std::vector<float>()> task([]()
 	std::packaged_task<std::vector<float>(int)> task([](int densidad){ 

		

		int tam=bufferLaser.ranges.size();
		//int newtam=tam/ESCALADO*DENSIDAD_1_LASER;
		int newtam;
		if(densidad!=1)
			newtam= tam * (1 - (1/(float)densidad));
		else newtam = tam;
		
		std::vector<float>  ranges(newtam);

		ROS_INFO("Densidades: %d, Porcentaje de datos: %f, Cantidad de datos: %d",densidad,(float) (1 - (1/(float)densidad)), newtam);


		ROS_INFO("Tamaño ranges:%d, Tamaño nuevo ranges:%d, Tamaño del vector: %d",(int)tam,(int)newtam,(int)bufferLaser.ranges.size());


		int i,j;
		for (i = 0, j=0; i<newtam && j<tam && !terminar[TERMINAR_SERVICIO_LASER]; j++)
		{
			if( j%densidad != 0)
			{
				ranges[i] = bufferLaser.ranges[j];
				i++;
			}

		}

		//ROS_INFO("iteraciones i: %d",(int)i);

		
		if(TEST_LASER_RANGES){

				// ROS_INFO("Ranges original");
				// for (int i = 0; i < tam; ++i)
				// {
				// 	ROS_INFO("%d: %f",i,bufferLaser.ranges[i]);
				// }
				ROS_INFO("Ranges modificado");
				for (int i = 0; i < newtam; ++i)
				{
					ROS_INFO("%d: %f",i,ranges[i]);
				}
		}




		if(terminar[TERMINAR_SERVICIO_LASER]) ROS_INFO("Termina forzado por pasar del timeout");
		else ROS_INFO("Tarea terminada dentro del timeout");
		
	
	 	
	 	//return TERMINAR_SERVICIO_LASER; 
	 	return ranges;
	}); // wrap the function


	int densidad;
	switch ( req.density )
    {
		case DENSIDAD_1_LASER:
			densidad = DENSIDAD_1_LASER;
			break;
		case DENSIDAD_2_LASER:
			densidad = DENSIDAD_2_LASER;
			break;
		case DENSIDAD_3_LASER:
			densidad = DENSIDAD_3_LASER;
			break;
		case DENSIDAD_4_LASER:
			densidad = DENSIDAD_4_LASER;
			break;
		case DENSIDAD_5_LASER:
			densidad = DENSIDAD_5_LASER;
			break;
		case DENSIDAD_8_LASER:
			densidad = DENSIDAD_8_LASER;
			break;
		default:
			densidad = DENSIDAD_9_LASER;//para probar quitando el 75% de los datos, por defecto dejamos DENSIDAD_1_LASER
    }

    res.laser = bufferLaser;

    if(densidad!=DENSIDAD_1_LASER){
    	
    	ROS_INFO("checking, please wait");

    	std::future<std::vector<float>> future = task.get_future();  // get a future
	    //std::thread(std::move(task)).detach(); // launch on a thread
	    std::thread(std::move(task),densidad).detach(); // launch on a thread
	    

	    //std::packaged_task<int(float)> task(f);
	    //std::thread task_td(std::move(task), 2.0); // si saco la funcion fuera
	    //task_td.detach();

		std::future_status status = future.wait_for(std::chrono::milliseconds(req.timeout));
		

		//en este caso, devolver datos vacios
		//en caso contrario devolver los datos obtenidos por get
		if(status != std::future_status::ready) 
		{		
			terminar[TERMINAR_SERVICIO_LASER]=true;
			ROS_INFO("Timeout consumido\n");
			return true;
		}
		else{
			ROS_INFO("Ha terminado con éxito"); //es bloqueante
			//If the shared state is not yet ready (i.e., the provider has not yet set its value or exception), 
			//the function blocks the calling thread and waits until it is ready.
		
		}
		
		///////////////////////////
		
		//res.laser.ranges = bufferLaser.ranges;
		res.laser.ranges = future.get();
		
	    //float increment = ((bufferLaser.angle_min*-1.0) + bufferLaser.angle_max) / res.laser.ranges.size();
		res.laser.angle_increment =(float) ((bufferLaser.angle_min*-1.0) + bufferLaser.angle_max) / (float)res.laser.ranges.size();
		//ROS_INFO("Angle_increment:%f, %f, %f", (float) increment, (float) bufferLaser.angle_increment, (float) bufferLaser.angle_increment / increment);
    }

    
	ROS_INFO("Sending back response\n");

	return true;

}


/////////////////////////
// ODOMETRY

// Servicio para la odometria
bool service_odometry(regulator_interface::ServiceOdometry::Request &req,
                   regulator_interface::ServiceOdometry::Response &res)
{
	ROS_INFO("Recibida la petición en el servicio odometria");
	ROS_INFO("Time-out: %u",(unsigned int)req.timeout);

	res.odometry = bufferOdometry; 

	ROS_INFO("Sending back response\n");	

	return true;
}







/////////////////////////////////////
/////////////////////////////////////
// CALLBACK

// pointcloud
void pointsCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	//std::thread::id this_id = std::this_thread::get_id();
	//ROS_INFO("Servidor con ID threads:%d",this_id);

	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	// Do data processing here...
	output = *msg;
	  
 	// Publish the data.

	//pub.publish (output);

	//printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);	

	if(printPoints)
	{
	  for (uint j=0; j < msg->height * msg->width; j++){
	      float x = msg->data[j * msg->point_step + msg->fields[0].offset];
	      float y = msg->data[j * msg->point_step + msg->fields[1].offset];
	      float z = msg->data[j * msg->point_step + msg->fields[2].offset];
	      // Some other operations
	      printf ("\t(%f, %f, %f)\n", x, y, z);
		    
	    }
	}

	/* !!!!!!!!!!!!!!  */
	//esta operacion tiene que ser exclusiva
	bufferCamera = *msg;
	/* !!!!!!!!!!!!!!  */

	//ROS_INFO("Almacenados nuevos datos en el camera buffer.");

	pub_cloud.publish(msg);
	

	
}


// Imagen
//const sensor_msgs::ImageConstPtr& msg
//const sensor_msgs::Image::ConstPtr& msg
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	/* !!!!!!!!!!!!!!  */
	bufferImage = *msg;
	/* !!!!!!!!!!!!!!  */

	/*try
		{
			//ROS_INFO("IMAGE color");
			//BGR8 - MONO8 - RGB8
			if (msg->encoding == "32FC1"){
				//ROS_INFO("hola en image color");
						sensor_msgs::Image img;
						img.header = msg->header;
						img.height = msg->height;
						img.width = msg->width;
						img.is_bigendian = msg->is_bigendian;
						img.step = msg->step;
						img.data = msg->data;
						img.encoding = "mono16";

						cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
					}
			//cv_bridge::CvImagePtr imagen_nueva = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
			

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}*/
 
}


// Imagen profundidad
//32FC1 -> ENCODING para imagen en profundidad
void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{

	/* !!!!!!!!!!!!!!  */
	bufferDephImage = *msg;
	/* !!!!!!!!!!!!!!  */
/*
	try
		{
			
			//BGR8 - MONO8 - RGB8
			if (msg->encoding == "32FC1"){
				ROS_INFO("hola");
						sensor_msgs::Image img;
						img.header = msg->header;
						img.height = msg->height;
						img.width = msg->width;
						img.is_bigendian = msg->is_bigendian;
						img.step = msg->step;
						img.data = msg->data;
						img.encoding = "mono16";

						cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
					}
			//cv_bridge::CvImagePtr imagen_nueva = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
			

		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
 */
}

// Laser
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

	pub_scan.publish(scan);
	//printf ("LASER LASER\n");
	//ROS_INFO("Almacenados nuevos datos en el laser buffer.");
	 sensor_msgs::LaserScan laserData = *scan;
//	ROS_INFO("Range min:%f, Range max:%f", laserData.range_min, laserData.range_max);
//	ROS_INFO("Angle min:%f, Angle max:%f, Angle increment:%f, Angle increment:%f rad, Scan time:%f", laserData.angle_min*180/PI, laserData.angle_max*180/PI,laserData.angle_increment*180/PI,laserData.angle_increment,laserData.scan_time);
	
//	float angles=laserData.angle_min;
//	ROS_INFO("%f",angles);

	/*for (int i = 0; i < 24; ++i)
	{
		angles+=laserData.angle_increment;
		ROS_INFO("%f",angles*180/PI);
	}
	ROS_INFO("#############");
	*/
/*
	for (int i = 0; i < sizeof(laserData.ranges); ++i)
	{
		ROS_INFO("%f",laserData.ranges[i]);
	}

	ROS_INFO("------------");
*/
/*	
	for (int i = 0; i < sizeof(laserData.intensities); ++i)
	{
		ROS_INFO("%d",laserData.intensities[i]);
	}
	ROS_INFO("------------");
	ROS_INFO("------------");
*/





	bufferLaser = *scan;

}

// Odometria
//nav_msgs/Odometry -> tipo de mensaje
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{

	bufferOdometry = *odometryMsg;
}





int main (int argc, char** argv)
{

	/*std::future<int> future = std::async(std::launch::async, [](){ 
	std::this_thread::sleep_for(std::chrono::seconds(3));
		return 8;  
	}); 

	std::cout << "waiting...\n";
	std::future_status status;
	do {
		status = future.wait_for(std::chrono::seconds(1));
		if (status == std::future_status::deferred) {
		    std::cout << "deferred\n";
		} else if (status == std::future_status::timeout) {
		    std::cout << "timeout\n";
		} else if (status == std::future_status::ready) {
		    std::cout << "ready!\n";
		}
	} while (status != std::future_status::ready); 

	std::cout << "result is " << future.get() << '\n';
*/

	/*
	int a=10;
	int b=11;
	auto glambda = [](int a, int&& b) { return a < b; };
	bool c = glambda(10, 11); // ok
	printf("solucion a la funcion lamda: %d\n",c);
	*/

	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "regulator_interface");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	ros::Subscriber sub_points = n.subscribe("/camera/depth/points", 1000, pointsCloudCallback);
	ros::Subscriber sub_image_depth = n.subscribe("/camera/depth/image_raw", 1000, imageDepthCallback); //para profundidad
	
	ros::Subscriber sub_image = n.subscribe("/camera/rgb/image_color", 1000, imageCallback); //para RGB y MONO

	ros::Subscriber sub_laser = n.subscribe("/scan", 1000, laserCallback);

	ros::Subscriber sub_odometry = n.subscribe("/odom", 1000, odometryCallback); //Odometria


	// Publisher
	//ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scanMio", 50);


	pub_scan = n.advertise<sensor_msgs::LaserScan>("scaner/laser", 1000);
	pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/camara/profundidad/puntos/cloud", 1000);
	  
	////////////////////////////////
	// Services


	ros::ServiceServer serviceCamera1 = n.advertiseService("Service_Camera_GrayScale", service_camera_grayscale);
	ros::ServiceServer serviceCamera2 = n.advertiseService("Service_Camera_RGB", service_camera_rgb);
	//ros::ServiceServer serviceCamera3 = n.advertiseService("Service_Camera3", service_camera3);
	ros::ServiceServer serviceLaser = n.advertiseService("Service_Laser", service_laser);
	ros::ServiceServer serviceOdometry = n.advertiseService("Service_Odometry", service_odometry);

	////////////////////////////////


	//B/N
	int cantidadDensidades=100;
	int imageBytesBN = 480*640;//cantidad de pixeles
	int imageBytes = 480*640*3;//por los 3 canales
	for (int i = 1; i <= cantidadDensidades; ++i)
	{
		std::vector<double> v;
		double densidadBytes;
		densidadBytes = ((double)i/100.0)*imageBytesBN;
		v.push_back(densidadBytes);
		v.push_back(((double)i/100.0));
		v.push_back(0);
		tablaEscalado.push_back(v);
	}

	//COLOR
	for (int i = 1; i <= cantidadDensidades; ++i)
	{
		std::vector<double> v;
		double densidadBytes;
		densidadBytes = ((double)i/100.0)*imageBytes;
		v.push_back(densidadBytes);
		v.push_back(((double)i/100.0));
		v.push_back(1);
		tablaEscalado.push_back(v);
	}


	for (int i = 0; i < tablaEscalado.size(); ++i)
	{
		//cout<<tablaEscalado[i][0]<<", "<<tablaEscalado[i][1]<<", "<<tablaEscalado[i][2]<<endl;
		ROS_INFO("%d %f %f %f",i,tablaEscalado[i][0],tablaEscalado[i][1],tablaEscalado[i][2]);
	}


	tablaEscalado = Ordenar(tablaEscalado);

	for (int i = 0; i < tablaEscalado.size(); ++i)
	{
		//cout<<tablaEscalado[i][0]<<", "<<tablaEscalado[i][1]<<", "<<tablaEscalado[i][2]<<endl;
		ROS_INFO("%d %f %f %f",i,tablaEscalado[i][0],tablaEscalado[i][1],tablaEscalado[i][2]);
	}


	////////////////////////////////



	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
 	ros::spin();




	return 0;
}

/*
float* function_task_laser()
{

	    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - bufferLaser.angle_min) / bufferLaser.angle_increment);
	    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min) / bufferLaser.angle_increment);

	    //ROS_INFO("MIN_SCAN_ANGLE_RAD: %f,MAX_SCAN_ANGLE_RAD: %f, MIN_SCAN_ANGLE_RAD - bufferLaser.angle_min(%f): %f,MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min): %f",MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD,bufferLaser.angle_min , MIN_SCAN_ANGLE_RAD + bufferLaser.angle_min, MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min);

	    //ROS_INFO("Min index: %d, Max index: %d",minIndex,maxIndex);
	    float closestRange = bufferLaser.ranges[minIndex];
		for (int currIndex = minIndex; currIndex <= maxIndex; currIndex++) {
			if (bufferLaser.ranges[currIndex] < closestRange) {
				closestRange = bufferLaser.ranges[currIndex];
			}
			//ROS_INFO("-- %d, ranges[%d] = %f",currIndex,currIndex,bufferLaser.ranges[currIndex]);
		}

		int tam=maxIndex+1;
		int newtam=tam/ESCALADO*DENSIDAD_1_LASER;
		
		float ranges[newtam];


		ROS_INFO("Tamaño ranges:%d, Tamaño nuevo ranges:%d",tam,newtam);

		for (int i = 0, j=0; i<newtam && j<tam && !terminar[TERMINAR_SERVICIO_LASER]; i++,j+=2)
		{
			ranges[i] = bufferLaser.ranges[j];
		}

		
		if(TEST_LASER_RANGES){

				// ROS_INFO("Ranges original");
				// for (int i = 0; i < tam; ++i)
				// {
				// 	ROS_INFO("%d: %f",i,bufferLaser.ranges[i]);
				// }
				ROS_INFO("Ranges modificado");
				for (int i = 0; i < newtam; ++i)
				{
					ROS_INFO("%d: %f",i*2,ranges[i]);
				}
		}


		if(terminar[TERMINAR_SERVICIO_LASER]) ROS_INFO("Termina forzado por pasar del timeout");
		else ROS_INFO("Tarea terminada dentro del timeout");
		
	
	 	
	 	return ranges; 
}
*/
/*
	//	int count = bufferLaser.scan_time / bufferLaser.time_increment;
	//    ROS_INFO("I heard a laser scan %s[%d]:", bufferLaser.header.frame_id.c_str(), count);
	    // ROS_INFO("angle_range, %f, %f", RAD2DEG(bufferLaser.angle_min), RAD2DEG(bufferLaser.angle_max));
	  
	    // for(int i = 0; i < count; i++) {
	    //     float degree = RAD2DEG(bufferLaser.angle_min + bufferLaser.angle_increment * i);
	    //     ROS_INFO(": [%f, %f]", degree, bufferLaser.ranges[i]);
	    // }

		/*for(int i=0;i<5 && !terminar[TERMINAR_SERVICIO_LASER];i++){
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}* /

		//int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	    //int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - bufferLaser.angle_min) / bufferLaser.angle_increment);
	    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min) / bufferLaser.angle_increment);

	    //ROS_INFO("MIN_SCAN_ANGLE_RAD: %f,MAX_SCAN_ANGLE_RAD: %f, MIN_SCAN_ANGLE_RAD - bufferLaser.angle_min(%f): %f,MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min): %f",MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD,bufferLaser.angle_min , MIN_SCAN_ANGLE_RAD + bufferLaser.angle_min, MAX_SCAN_ANGLE_RAD - bufferLaser.angle_min);

	    //ROS_INFO("Min index: %d, Max index: %d",minIndex,maxIndex);
	    float closestRange = bufferLaser.ranges[minIndex];
		for (int currIndex = minIndex; currIndex <= maxIndex; currIndex++) {
			if (bufferLaser.ranges[currIndex] < closestRange) {
				closestRange = bufferLaser.ranges[currIndex];
			}
			//ROS_INFO("-- %d, ranges[%d] = %f",currIndex,currIndex,bufferLaser.ranges[currIndex]);
		}
*/

/*
inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}
*/
/*
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
  PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);
}
*/

	//BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	//for( pcl::PointCloud< PointT >::iterator it = msg.begin(); it<msg.end();it++)	
	//	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

	/*pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
	pcl_conversions::toPCL(*msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
	//PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);*/
