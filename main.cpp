
#include "trackcalibfuns.h"



int main()
{

	//showthemakers();
	//return 0;
	
	vector<Eigen::Matrix3f>Rs; 
	vector<Eigen::Vector3f>ts;
	vector<Eigen::Vector3f>Pi;

	//string rtfile = "rt.txt";
	//string Psfile = "Psi.txt";
	//getRTfromtxt(rtfile, Psfile, Rs, ts, Pi);


	getRtandPi(Rs,ts,Pi);
	cv::Point3f P0;
	cv::Mat Rp, tp;
	calcalib(Rs,ts,Pi,P0,Rp,tp);
	cout << "P0=  "<<P0 << endl;
	cout << "R=  " << Rp << endl << "t=  " << tp << endl;
	system("PAUSE");
	return 0;
}



