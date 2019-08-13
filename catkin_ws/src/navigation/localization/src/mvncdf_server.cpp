#include "ros/ros.h"
#include "localization/MVNCDF.h"


double mvn_cdf(double lx, double ly, double ux, double uy, double step, double mean_x, double mean_y, double (*cov)[2])
{
    double det = cov[0][0]*cov[1][1] - cov[1][0]*cov[0][1]; //determinante de la covarianza
    //Lineas para calcular inversa de la covarianza
    double icov00 =  cov[1][1];
    double icov01 = -cov[0][1];
    double icov10 = -cov[1][0];
    double icov11 =  cov[0][0];
    cov[0][0] = icov00/det;
    cov[0][1] = icov01/det;
    cov[1][0] = icov10/det;
    cov[1][1] = icov11/det;
    //denominador de la pdf
    double den = sqrt(pow(2*M_PI, 2)*det);
    double x, y;

    double cdf = 0;
    for(double x_ = lx; x_ <= ux; x_+=step)
        for(double y_ = ly; y_ <= uy; y_+=step){
            x=x_-mean_x;
            y=y_-mean_y;
            cdf+= step*step*exp(-0.5*(cov[0][0]*x*x + cov[0][1]*x*y + cov[1][0]*x*y + cov[1][1]*y*y))/den;
        }

    return cdf;
}




bool handle_mvncdf(localization::MVNCDF::Request &req,
					localization::MVNCDF::Response &res){

	double aux;
	double cov[2][2]={{req.cov00,req.cov01},{req.cov10,req.cov11}};

	//cov[0][0]=req.cov00;
	//cov[0][1]=req.cov01;
	//cov[1][0]=req.cov10;
	//cov[1][1]=req.cov11;

	/*std::cout<<"req.lx: "<<req.lx<<std::endl;
    std::cout<<"req.ly: "<<req.ly<<std::endl;
    std::cout<<"req.ux: "<<req.ux<<std::endl;
    std::cout<<"req.uy: "<<req.uy<<std::endl;
    std::cout<<"req.step: "<<req.step<<std::endl;
    std::cout<<"req.mean_x: "<<req.mean_x<<std::endl;
    std::cout<<"req.mean_y: "<<req.mean_y<<std::endl;
    for(size_t i=0;i<2;i++){
        for(size_t j=0;j<2;j++){
            std::cout<<"cov["<<i<<"]["<<j<<"]: "<<cov[i][j]<<std::endl;
        }
    }*/

	res.prob=mvn_cdf(req.lx, req.ly, req.ux, req.uy, req.step, req.mean_x, req.mean_y, cov);
    //std::cout<<"prob: "<<res.prob<<std::endl;
	return true;


}



int main(int argc, char* argv[]){

	std::cout<<"INITIALIZING MVNCDF SERVER"<<std::endl;
	ros::init(argc, argv, "mvncdf_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("/navigation/localization/mvncdf", handle_mvncdf);
    ros::spin();


	return 0;
}
