#pragma once
#include <opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <unsupported/Eigen/KroneckerProduct>
#include <opencv2/core/eigen.hpp>
#include <iostream>

void ChessSignalCalib();

using namespace std;


bool getRtformtargettoglobal(vector<Eigen::Matrix3f>&Rs,vector<Eigen::Vector3f>&ts);
bool getPiformfix2local( vector<Eigen::Vector3f>&Pi);
bool detecmakers(cv::Mat src, cv::Mat &dst, std::vector<int> &markerIds,std::vector<std::vector<cv::Point2f>> &markerCorners);
bool getRtandPi(vector<Eigen::Matrix3f>&Rs, vector<Eigen::Vector3f>&ts, vector<Eigen::Vector3f>&Pi);
bool calcalib(vector<Eigen::Matrix3f>Rs, vector<Eigen::Vector3f>ts, vector<Eigen::Vector3f>Pi, cv::Point3f &P0, cv::Mat &Rp, cv::Mat &tp);
bool getRTfromtxt(string rtfile, string Psifile, vector<Eigen::Matrix3f>&Rs, vector<Eigen::Vector3f>&ts, vector<Eigen::Vector3f>&Ps);
bool showthemakers();


class trackcalibfuns
{
public:
	trackcalibfuns();
	~trackcalibfuns();
};

