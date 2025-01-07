#include "trackcalibfuns.h"

//test knro
//cv::Mat Am = cv::Mat::eye(4, 4, CV_64FC1);
//Eigen::Matrix4d A;
//cv::cv2eigen(Am, A);
//Eigen::Matrix2d B;
//B << 1, -1,
//-1, 1;
//Eigen::MatrixXd P;
//P = Eigen::kroneckerProduct(A, B);
//cv::Mat Pm;
//cv::eigen2cv(P, Pm);

trackcalibfuns::trackcalibfuns()
{
}


trackcalibfuns::~trackcalibfuns()
{
}

void drawaxis(cv::Mat & src, cv::Mat cameramatrix, cv::Mat distmat, cv::Mat rvec, cv::Mat tvec)
{
	vector<cv::Point3f>objpts;
	vector<cv::Point2f>imgpts;
	objpts.emplace_back(0, 0, 0);
	objpts.emplace_back(0.1, 0, 0);
	objpts.emplace_back(0, 0.1, 0);
	objpts.emplace_back(0, 0, 0.1);

	cv::projectPoints(objpts, rvec, tvec, cameramatrix, distmat, imgpts);

	cv::line(src, imgpts[0], imgpts[1], cv::Scalar(0, 0, 255), 2);
	cv::line(src, imgpts[0], imgpts[2], cv::Scalar(0, 255, 0), 2);
	cv::line(src, imgpts[0], imgpts[3], cv::Scalar(255, 0, 0), 2);

}



bool getRtformtargettoglobal(vector<Eigen::Matrix3f>&Rs, vector<Eigen::Vector3f>&ts)
{


	double dx = 0.0022, dy = 0.0022;
	float foclen = 8.0;
	//float imgwidth = src.size().width, imgheight = src.size().height;
	float imgwidth = 1280.0;
	float imgheight = 960.0;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = foclen / dx;
	cameraMatrix.at<double>(0, 2) = imgwidth / 2;
	cameraMatrix.at<double>(1, 1) = foclen / dy;
	cameraMatrix.at<double>(1, 2) = imgheight / 2;
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);


	int camnum = 11;
	for (int i=0;i<camnum;++i)
	{
		cv::Mat src = cv::imread(cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\global%d.jpg", i));
		if (src.empty())
		{
			cout << "no img" << endl;
			return false;
		}
		cv::Mat outputImage = src.clone();
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		if (!detecmakers(src, outputImage,markerIds,markerCorners))
		{
			cout << "no maker" << endl;
			return false;
		}





		float makersize = 0.16;
		vector<cv::Point3f>makesilfobj;
		makesilfobj.emplace_back(-makersize / 2, makersize / 2, 0);
		makesilfobj.emplace_back(makersize / 2, makersize / 2, 0);
		makesilfobj.emplace_back(makersize / 2, -makersize / 2, 0);
		makesilfobj.emplace_back(-makersize / 2, -makersize / 2, 0);
		cv::Mat rvec, tvec;
		vector<cv::Point2f>makeimgpt;
		for (int j=0;j<markerIds.size();++j)
		{
			if (markerIds[j]==4)
			{
				makeimgpt = markerCorners[j];
				cv::solvePnP(makesilfobj, makeimgpt, cameraMatrix, distCoeffs, rvec, tvec);
				drawaxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec);
				//cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
				//cv::imshow("test", outputImage);
				//cv::waitKey(0);

				cv::Mat rotR;
				cv::Rodrigues(rvec,rotR);
				Eigen::Matrix3f eigenR;
				cv::cv2eigen(rotR,eigenR);
				Rs.push_back(eigenR);
				Eigen::Vector3f eigent;
				cv::cv2eigen(tvec, eigent);
				ts.push_back(eigent);
			}
			else continue;
		}


	}


}



bool getPiformfix2local(vector<Eigen::Vector3f>&Pi)
{
	double dx = 0.0022, dy = 0.0022;
	float foclen = 8.0;
	//float imgwidth = src.size().width, imgheight = src.size().height;
	float imgwidth = 1024.0;
	float imgheight = 768.0;
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = foclen / dx;
	cameraMatrix.at<double>(0, 2) = imgwidth / 2;
	cameraMatrix.at<double>(1, 1) = foclen / dy;
	cameraMatrix.at<double>(1, 2) = imgheight / 2;
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);

	int camnum = 11;
	for (int i=0;i<camnum;++i)
	{
		cv::Mat src = cv::imread(cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\end%d.jpg", i));
		if (src.empty())
		{
			cout << "no img" << endl;
			return false;
		}

		cv::Mat outputImage = src.clone();
		cv::threshold(src, src, 50, 255, cv::THRESH_BINARY);
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
		cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds);
		if (markerIds.size() == 0) { cout << "no maker" << endl; return false; }
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
		cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
		//cv::imshow("success", src);
		//cv::waitKey(0);


		vector<cv::Point2f>markercenters;

		for (int k= 0;k < markerCorners.size(); ++k)
		{
			cv::Point2f centsum(0, 0);
			for (int j = 0; j < markerCorners[k].size(); ++j)
			{
				centsum += markerCorners[k][j];

				//cv::circle(outputImage, markerCorners[i][j], 2, cv::Scalar(0, 0, 255), 2);
				//imshow("a", outputImage);
				//waitKey(0);
			}
			markercenters.push_back(centsum / 4.0);
		}
		for (int k = 0; k < markercenters.size(); ++k)
		{
			circle(outputImage, markercenters[k], 2, cv::Scalar(0, 255, 0), 2);
		}

		float markerleng = 0.05;
		float markdislend = markerleng + 0.01;
		float bossleng = 0.2;
		std::vector<cv::Point2f>makercenterimgpt(4);
		std::vector<cv::Point3f>makercenterobjpt(4);
		for (int k= 0; k< markerIds.size(); k++)
		{
			if (markerIds[k] == 0)
			{
				makercenterimgpt[0] = markercenters[k];
				makercenterobjpt[0] = cv::Point3f(0, 0, 0);
			}
			if (markerIds[k] == 1)
			{
				makercenterimgpt[1] = markercenters[k];
				makercenterobjpt[1] = cv::Point3f(-bossleng - markdislend, 0, 0);
			}
			if (markerIds[k] == 2)
			{
				makercenterimgpt[2] = markercenters[k];
				makercenterobjpt[2] = cv::Point3f(0, -bossleng - markdislend, 0);
			}
			if (markerIds[k] == 3)
			{
				makercenterimgpt[3] = markercenters[k];
				makercenterobjpt[3] = cv::Point3f(-bossleng - markdislend, -bossleng - markdislend, 0);
			}
		}

		cv::Mat rvec, tvec;
		solvePnP(makercenterobjpt, makercenterimgpt, cameraMatrix, distCoeffs, rvec, tvec);
		Eigen::Vector3f Pk;
		cv::cv2eigen(tvec, Pk);
		Pi.push_back(Pk);


		drawaxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec);

		cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
		cv::imshow("test", outputImage);
		cv::waitKey(0);

	}

}


bool detecmakers(cv::Mat src, cv::Mat &dst, std::vector<int> &markerIds, std::vector<std::vector<cv::Point2f>> &markerCorners)
{
	dst= src.clone();
	cv::threshold(src, src, 50, 255, cv::THRESH_BINARY);
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds);
	if (markerIds.size() == 0) { cout << "no maker" << endl; return false; }
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
	cv::aruco::drawDetectedMarkers(dst, markerCorners, markerIds);
	//cv::imshow("success", src);
	//cv::waitKey(0);
}


bool getRtandPi(vector<Eigen::Matrix3f>&Rs, vector<Eigen::Vector3f>&ts, vector<Eigen::Vector3f>&Pi)
{


	float imageResolutionX = 1280.0;
	float imageResolutionY = 960.0;
	double fovy = 38.5 * CV_PI / 180.0;
	double focalLength = (imageResolutionY / 2.0) / (tan(fovy / 2.0));
	cv::Mat cameraMatrix_g = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix_g.at<double>(0, 0) = focalLength;
	cameraMatrix_g.at<double>(0, 2) = imageResolutionX / 2.0;
	cameraMatrix_g.at<double>(1, 1) = focalLength;
	cameraMatrix_g.at<double>(1, 2) = imageResolutionY / 2.0;
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);



	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	imageResolutionX = 1024.0;
	imageResolutionY = 768.0;
	fovy = 38.5 * CV_PI/ 180.0;
	focalLength = (imageResolutionY / 2.0) / (tan(fovy / 2.0));
	cameraMatrix.at<double>(0, 0) = focalLength;
	cameraMatrix.at<double>(0, 2) = imageResolutionX / 2.0f;
	cameraMatrix.at<double>(1, 1) = focalLength;
	cameraMatrix.at<double>(1, 2) = imageResolutionY / 2.0f;

	int camnum = 11;
	for (int i = 0; i < camnum; ++i)
	{
		//cv::Mat src_g = cv::imread(cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\global%d.jpg", i));
		cv::Mat src_g = cv::imread(cv::format("imgs\\global%d.jpg", i));
		if (src_g.empty())
		{
			cout << "no img g" << endl;
			return false;
		}
		cv::Mat outputImage_g = src_g.clone();
		std::vector<int> markerIds_g;
		std::vector<std::vector<cv::Point2f>> markerCorners_g;
		if (!detecmakers(src_g, outputImage_g, markerIds_g, markerCorners_g))
		{
			cout << "no maker" << endl;
			continue;;
		}

		//cv::Mat src = cv::imread(cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\end%d.jpg", i));
		cv::Mat src = cv::imread(cv::format("imgs\\end%d.jpg", i));
		if (src.empty())
		{
			cout << "no img l" << endl;
			continue;;
		}

		cv::Mat outputImage = src_g.clone();
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		if (!detecmakers(src, outputImage, markerIds, markerCorners))
		{
			cout << "no maker l" << endl;
			continue;;
		}
		if (markerIds.size()<4)
		{
			cout << "no maker less 4" << endl;
			continue;;
		}
		//--global
		float makersize = 0.16;
		vector<cv::Point3f>makesilfobj;
		makesilfobj.emplace_back(-makersize / 2, makersize / 2, 0);
		makesilfobj.emplace_back(makersize / 2, makersize / 2, 0);
		makesilfobj.emplace_back(makersize / 2, -makersize / 2, 0);
		makesilfobj.emplace_back(-makersize / 2, -makersize / 2, 0);
		cv::Mat rvec, tvec;
		vector<cv::Point2f>makeimgpt;
		for (int j = 0; j < markerIds_g.size(); ++j)
		{
			if (markerIds_g[j] == 4)
			{
				makeimgpt = markerCorners_g[j];
				cv::solvePnP(makesilfobj, makeimgpt, cameraMatrix_g, distCoeffs, rvec, tvec);
				drawaxis(outputImage_g, cameraMatrix_g, distCoeffs, rvec, tvec);
				//cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
				//cv::imshow("test", outputImage);
				//cv::waitKey(0);

				cv::Mat rotR;
				cv::Rodrigues(rvec, rotR);
				Eigen::Matrix3f eigenR;
				cv::cv2eigen(rotR, eigenR);
				Rs.push_back(eigenR);
				Eigen::Vector3f eigent;
				cv::cv2eigen(tvec, eigent);
				ts.push_back(eigent);
			}
			else continue;
		}

		//---local

		vector<cv::Point2f>markercenters;

		for (int k = 0; k < markerCorners.size(); ++k)
		{
			cv::Point2f centsum(0, 0);
			for (int j = 0; j < markerCorners[k].size(); ++j)
			{
				centsum += markerCorners[k][j];

				//cv::circle(outputImage, markerCorners[i][j], 2, cv::Scalar(0, 0, 255), 2);
				//imshow("a", outputImage);
				//waitKey(0);
			}
			markercenters.push_back(centsum / 4.0);
		}
		for (int k = 0; k < markercenters.size(); ++k)
		{
			circle(outputImage, markercenters[k], 2, cv::Scalar(0, 255, 0), 2);
		}

		//float markerleng = 50;
		//float markdislend = markerleng + 10;
		//float bossleng = 200;
		float markerleng = 0.05;
		float markdislend = markerleng + 0.01;
		float bossleng = 0.2;
		std::vector<cv::Point2f>makercenterimgpt(4);
		std::vector<cv::Point3f>makercenterobjpt(4);
		for (int k = 0; k < markerIds.size(); k++)
		{
			if (markerIds[k] == 0)
			{
				makercenterimgpt[0] = markercenters[k];
				makercenterobjpt[0] = cv::Point3f(bossleng/2.0+markdislend/2.0, bossleng / 2.0 + markdislend/2.0, 0);
			}
			if (markerIds[k] == 1)
			{
				makercenterimgpt[1] = markercenters[k];
				makercenterobjpt[1] = cv::Point3f(-bossleng/2.0 - markdislend/2.0, bossleng / 2.0 + markdislend / 2.0, 0);
			}
			if (markerIds[k] == 2)
			{
				makercenterimgpt[2] = markercenters[k];
				makercenterobjpt[2] = cv::Point3f(bossleng / 2.0 + markdislend / 2.0, -bossleng/2.0 - markdislend/2.0, 0);
			}
			if (markerIds[k] == 3)
			{
				makercenterimgpt[3] = markercenters[k];
				makercenterobjpt[3] = cv::Point3f(-bossleng / 2.0 - markdislend / 2.0, -bossleng / 2.0 - markdislend / 2.0, 0);
			}
		}

	
		solvePnP(makercenterobjpt, makercenterimgpt, cameraMatrix, distCoeffs, rvec, tvec);
		Eigen::Vector3f Pk;
		cv::cv2eigen(tvec, Pk);
		Pi.push_back(Pk);


		drawaxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec);

		cv::namedWindow("testg", cv::WINDOW_AUTOSIZE);
		cv::imshow("testg", outputImage_g);
		cv::namedWindow("testl", cv::WINDOW_AUTOSIZE);
		cv::imshow("testl", outputImage);
		cv::waitKey(0);


	}

}

bool calcalib(vector<Eigen::Matrix3f>Rs, vector<Eigen::Vector3f>ts, vector<Eigen::Vector3f>Pi,cv::Point3f &P0,cv::Mat &Rp,cv::Mat &tp)
{
	if (Rs.size()!=Pi.size())
	{
		cout << "false data" << endl;
	}
	int ng = 3*Rs.size();
	cv::Mat1f  A=cv::Mat::ones(ng,15,CV_32FC1);
	cv::Mat1f b = cv::Mat::ones(ng,1,CV_32FC1);
	for (int i=0;i<Rs.size();++i)
	{
		Eigen::Matrix<float, 3, 9>Knro_eigen;
		Knro_eigen = Eigen::kroneckerProduct(Pi[i].transpose(), Rs[i]);
		cv::Mat Knro_mat,Ri_mat;
		cv::eigen2cv(Knro_eigen,Knro_mat);
		cv::eigen2cv(Rs[i], Ri_mat);
		cv::Mat I3 = -1*cv::Mat::eye(3,3,CV_32FC1);

		Knro_mat.copyTo(A(cv::Rect(0,3*i,9,3)));
		Ri_mat.copyTo(A(cv::Rect(9,3*i, 3, 3)));
		I3.copyTo(A(cv::Rect(12,3*i,3,3)));
		cv::Mat ti;
		cv::eigen2cv(ts[i],ti);
		//cout << i << "    " << ti.t() << endl;
		ti.copyTo(b(cv::Rect(0,3*i,1,3)));		
	}
	
	cv::Mat x = cv::Mat(15,1,CV_32FC1);
	cv::solve(A, -b,x, cv::DECOMP_SVD);

	Rp=cv::Mat::ones(3, 3,CV_32FC1);
	tp=cv::Mat::ones(3, 1,CV_32FC1);
	

		Rp.at<float>(0, 0) = x.at<float>(0, 0);
		Rp.at<float>(1, 0) = x.at<float>(1, 0);
		Rp.at<float>(2, 0) = x.at<float>(2, 0);
		Rp.at<float>(0, 1) = x.at<float>(3, 0);
		Rp.at<float>(1, 1) = x.at<float>(4, 0);
		Rp.at<float>(2, 1) = x.at<float>(5, 0);
		Rp.at<float>(0, 2) = x.at<float>(6, 0);
		Rp.at<float>(1, 2) = x.at<float>(7, 0);
		Rp.at<float>(2, 2) = x.at<float>(8, 0);
		tp.at<float>(0,0)= x.at<float>(9, 0);
		tp.at<float>(1, 0) = x.at<float>(10, 0);
		tp.at<float>(2, 0) = x.at<float>(11, 0);
		P0.x= x.at<float>(12, 0);
		P0.y = x.at<float>(13, 0);
		P0.z = x.at<float>(14, 0);

		Rp = Rp.inv();
		tp = -Rp.inv()*tp;

	return true;
}

bool getRTfromtxt(string rtfile, string Psifile, vector<Eigen::Matrix3f>&Rs, vector<Eigen::Vector3f>&ts, vector<Eigen::Vector3f>&Ps)
{
	ifstream infile(rtfile);
	if (!infile.is_open())
	{
		cout << "no rt " << endl;
		return false;
	}
	int num = 0;

	while (!infile.eof())
	{
		string thisline;
		getline(infile, thisline);
		if (thisline=="")
		{
			continue;
		}
		stringstream inf(thisline);
		float x, y, z, m;
		inf >> x >> y >> z >> m;
		Eigen::Matrix3f Ri;
		Eigen::Vector3f ti;
		if (x==0.0)
		{
			continue;
		}
		if (x!=0.0 && y!=0.0 && z!=0.0 && m!=0.0)
		{
			Ri(num, 0) = x;
			Ri(num, 1) = y;
			Ri(num, 2) = z;
			ti(num, 0) = m;
			num++;
		}
		if (num==3)
		{
			Rs.push_back(Ri);
			ts.push_back(ti);
			num = 0;
		}

	}
	infile.close();
	num = 0;
	ifstream inps(Psifile);
	if (!inps.is_open())
	{
		cout << "no ps file" << endl;
		return false;
	}
	while (!inps.eof())
	{
		string thisline;
		getline(inps, thisline);
		if (thisline == "")
		{
			continue;
		}
		stringstream inf(thisline);
		float x;
		inf >> x ;
		Eigen::Vector3f Pi;
		if (x == 1.0)
		{
			continue;
		}
		if (x != 0.0)
		{
			Pi(num, 0) = x;
			num++;
		}
		if (num == 3)
		{
			Ps.push_back(Pi);
			num = 0;
		}
	}
	inps.close();
	cout << Ps.size();
}

//单目棋盘标定
void ChessSignalCalib()
{

	//    if (ChessDialog->ShortSize==0 || ChessDialog->LongSize==0 || ChessDialog->BordeLength==0)
	//    {QMessageBox::information(this,"information","参数设置错误呀，返回！");return;}
	//     if(!m_GrabberL->pGrabber->isDevValid())
	//     {QMessageBox::information(this,"information","相机没有打开，返回！");return;}
	//      cv::Size ImageSize=cv::Size(camW,camH);  /* 图像的尺寸 */
	//       QString strint;
	//       std::string Limgpath;
	//       cv:: Mat srcL=cv::Mat::zeros(ImageSize, CV_8UC1);
	//        for (int i=0;i<ImagNumTotal;i++)
	//        {
	//           strint.sprintf("the %d-th image",i+1);
	//            QMessageBox::information(this,"information",strint);
	//            m_GrabberL->pHanderSink->snapImages(1, 2000);
	//            srcL.data = m_GrabberL->pHanderSink->getLastAcqMemBuffer()->getPtr();
	//            Limgpath=CalibImgDir.toStdString()+ cv::format("/L%d.bmp",i+1);
	//            cv::imwrite(Limgpath,srcL);
	//        }

	int  ImagNum = 11;
	cv::Size boardSize = cv::Size(6, 9);//自己画的标定板 标定板上每行、列的角点数

	int boardWidth = boardSize.width, boardHeight = boardSize.height;
	float squareSize = 45;  //自己画的标定板 标定板黑白格子的大小 单位mm

	std::vector<cv::Point2f> cornerL; /* 缓存每幅图像上检测到的角点 */
	std::vector<std::vector<cv::Point2f>> imagePointL; /* 保存检测到的所有角点 */
	int goodFrameCount = 0;
	std::vector<std::vector<cv::Point3f>>objRealPoint;
	cv::Mat rgbImageL, grayImageL;

	std::string filenameL;
	std::string  Limgpathout;
	std::vector<cv::Mat> imgvec;
	for (int i = 0; i < ImagNum; i++)
	{
		/*读取左边的图像*/
		filenameL = cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\auto%d.jpg", i);
		Limgpathout = cv::format("E:\\projects\\project_20191209cmk2buildrw\\src\\imgs\\auto-d%d.jpg", i);
		grayImageL = cv::imread(filenameL, 0);
		if (grayImageL.empty())
		{
			cout<< "no calib img give in " << "\n";
			return;
		}
		cv::cvtColor(grayImageL, rgbImageL, cv::COLOR_GRAY2BGR);
		
		bool isFindL;//, isFindR;

		isFindL = cv::findChessboardCorners(grayImageL, boardSize, cornerL, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
		if (isFindL == true /*&& isFindR == true*/)  //如果两幅图像都找到了所有的角点 则说明这两幅图像是可行的
		{
			cv::cornerSubPix(grayImageL, cornerL, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));		
			
			cv::drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
			
			//cv::imshow("test", imgvec[imgvec.size() - 1]);
			cv::imshow("chessboardL", rgbImageL);
			cv::imwrite(Limgpathout, rgbImageL);
			imagePointL.push_back(cornerL);
			goodFrameCount++;
			cv::waitKey(100);
		}
		else
			continue;
	}//i<ImagNum


	 /**********计算实际的校正点的三维坐标,根据实际标定格子的大小来设置****************/
	double intr1_arr[3][3] = { 0 }, intr2_arr[3][3] = { 0 }, dis1_arr[5] = { 0 }, dis2_arr[5] = { 0 };
	cv::Mat cameraMatrixL = cv::Mat(3, 3, CV_64FC1, intr1_arr); /* 摄像机内参数矩阵 */


	cv::Mat distCoeffsL = cv::Mat(1, 5, CV_64FC1, dis1_arr); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */

	std::vector<cv::Mat> tvecsMatL;//, tvecsMatR;  /* 每幅图像的旋转向量 */
	std::vector<cv::Mat> rvecsMatL;//, rvecsMatR; /* 每幅图像的平移向量 */

	std::vector<cv::Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardHeight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardWidth; colIndex++)
		{
			imgpoint.push_back(cv::Point3f(colIndex * squareSize, rowIndex * squareSize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < goodFrameCount; imgIndex++)
	{
		objRealPoint.push_back(imgpoint);
	}


	for (size_t k = 0; k < imagePointL[imagePointL.size() - 1].size(); k++)
	{

		cv::circle(rgbImageL, imagePointL[imagePointL.size() - 1][k], 5, cv::Scalar(0, 0, 255), 5);
		cout << objRealPoint[imagePointL.size() - 1][k] << endl;
		cv::imshow("chessboardL", rgbImageL);
		cv::waitKey(0);
	}

	//左右相机分别标定

	double rms = cv::calibrateCamera(objRealPoint, imagePointL, boardSize, cameraMatrixL, distCoeffsL, rvecsMatL, tvecsMatL, cv::CALIB_FIX_K3);

	cv::FileStorage intristor("calib.xml", cv::FileStorage::WRITE);
	intristor << "CameraMatrix_L" << cameraMatrixL;
	intristor << "DistCoeffs_L" << distCoeffsL;
	intristor << "rvecsMatL" << rvecsMatL;
	intristor << "tvecsMatL" << tvecsMatL;
	intristor << "RMS" << rms;
	intristor.release();

	return;
}


bool showthemakers()
{
	cv::Mat cameraMatrix, distCoeffs;
	cameraMatrix = cv::Mat::zeros(5, 1, CV_64FC1);
	float imgwidth = 1024.0;
	float imgheight = 768.0;
	double fovy = 24 * CV_PI / 180.0;
	double focalLength = (imgheight / 2.0) / (tan(fovy / 2.0));
	cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = focalLength;
	cameraMatrix.at<double>(0, 2) = imgwidth / 2.0;
	cameraMatrix.at<double>(1, 1) = focalLength;
	cameraMatrix.at<double>(1, 2) = imgheight / 2.0;
	distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);
	

	for (int i=0;i<3;i++)
	{
		cv::Mat src=cv::imread(cv::format("E:\\projects\\project_20200216trackcalibforpaper\\src\\imgs\\auto%d.jpg", i));
		if (src.empty())continue;
		//cv::threshold(src, src, 50, 255, cv::THRESH_BINARY);
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
		cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds);
		if (markerIds.size() == 0) { cout << "no maker" << endl; return false; }
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
		cv::aruco::drawDetectedMarkers(src, markerCorners, markerIds);

		vector<cv::Point3f>wdpts;
		vector<cv::Point2f>imgpts;
		//wdpts.emplace_back(0, 0, 0);
		//wdpts.emplace_back(1, 0, 0);
		//wdpts.emplace_back(0, 1, 0);
		//wdpts.emplace_back(0, 0, 1);

		cv::Mat rvec, tvec;
		for (int j = 0; j < markerIds.size(); j++)
		{
			cv::Point2f pp(0, 0);
			for (int p = 0; p < 4; ++p)
			{
				pp += markerCorners[j][p];
			}
			pp = pp / 4;

			if (markerIds[j] == 0)
			{	
				wdpts.emplace_back(0, 0, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j]==1)
			{
				wdpts.emplace_back(0.06, 0.06, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j]==2)
			{
				wdpts.emplace_back(0.06, 0.00, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 3)
			{
				wdpts.emplace_back(0.06, -0.06, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 4)
			{
				wdpts.emplace_back(0.00, 0.06, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 5)
			{
				wdpts.emplace_back(0.00, -0.06, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 6)
			{
				wdpts.emplace_back(-0.06, 0.06, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 7)
			{
				wdpts.emplace_back(-0.06, 0.00, 0);
				imgpts.emplace_back(pp);
			}
			else if (markerIds[j] == 8)
			{
				wdpts.emplace_back(-0.06, -0.06, 0);
				imgpts.emplace_back(pp);
			}
		}
		cv::solvePnP(wdpts, imgpts, cameraMatrix, distCoeffs, rvec, tvec);
		drawaxis(src, cameraMatrix, distCoeffs, rvec, tvec);
		cv::imshow("re", src);
		cv::waitKey(0);
		imwrite(cv::format("%d.jpg",i), src);
	}

}
