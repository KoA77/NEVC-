//AEB
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>
#include <stdio.h>
#include "SimOneEvaluationAPI.h"
#include <utilTargetObstacle.h>
#include <utilTargetLane.h>
#include <Windows.h>
#include <chrono>

//TraficLight
#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneId.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "assert.h"
#include<fstream>
#include<string>

using namespace std;

SimOne_Data_Gps Gps = SimOne_Data_Gps();
/*  
SimOne_Data_Gps
	float posX, posY, posZ （位置）米
	float oriX, oriY, oriZ （旋转）弧度
	float velX, velY, velZ （速度）米/秒
	float throttle(油门), brake(刹车), steering(轮胎转向角(度)), gear(齿轮位置)
	float accelX, accelY, accelZ （加速度）米/秒^2
	float angVelX, angVelY, angVelZ （角速度）弧度/秒
	float wheelSpeedFL, wheelSpeedFR, wheelSpeedRL, wheelSpeedRR (前后左右轮速度) 米/秒
	float odometer (里程表读数 (米))
	int extraStateSize (附加状态的数量)
	float extraStates (附加状态数据，索引由 MainVehicleExtraDataIndics 消息定义)
	bool isGPSLost (是否丢失 GPS 信号。GPS 信号丢失可能会影响 IMU 数据的有效性)
	cahr mainCarName (主车名称 (编码：utf-8))
	SimOne_Data_IMU imuData (IMU 数据)
*/
//to get caseinfo
SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();
/*  
SimOne_Data_CaseInfo
	char caseName [ 256 ] 案例名称，用于唯一标识特定案例
	char caseId [ 256 ] 案例ID，是特定案例的唯一标识符
	char taskId [ 256 ] 任务ID，用于标识与该案例相关联的任务
*/
SimOne_Data_WayPoints WayPoints;
/*
SimOne_Data_WayPoints
	char mainVehicleId [ 64 ] 主车辆的 ID
	int wayPointsSize 主车辆路点的数量
	SimOne_Data_WayPoints_Entry wayPoints [ 100 ] 主车辆的路点数组，最多 100 个
*/

#define p1 1
#define p2 0
#define p3 0
#define end_a -3
#define brake_a_max -7

// 场景id
#define CASE_1 ""
#define CASE_2 ""
#define CASE_3 ""

//to define taskid of case

int main()
{
	int VERSION = 1;
	//1. 数据存储初始化
	//判断路径是否存在，创建文件夹
	//获取系统时间转化成字符输出，用于生成唯一文件名
	SYSTEMTIME sys;
	GetLocalTime(&sys);

	std::stringstream ss;	//定义流
	std::string time_;		//定义字符串类型时间

	//将数字类型变量转化为流，依次表示年/月/日/小时/分钟/秒/毫秒/周几
	ss << sys.wYear << ":" << sys.wMonth
		<< ":" << sys.wDay << ":" << sys.wHour
		<< ":" << sys.wMinute << ":" << sys.wSecond
		<< ":" << sys.wMilliseconds << ":" << sys.wDayOfWeek;

	ss >> time_;		//将流转化为字符串类型变量，可改为 time_ = ss.str() 直接获取流内容
	std::fstream fs;
	std::string path = "E:\\Data\\" + std::to_string(VERSION) + "-" + time_ + ".txt";
	//存储源代码
	fs.open(path, std::ios::out);
	fs << "运行" << std::endl;

	int count = 0;
	//data monitor
	ofstream outputfile;
	outputfile.open("C:\\Users\\guotao\\OneDrive\\桌面\\比赛\\information.txt"); //桌面路径硬编码（C:\\Users\\guotao\\...）可能导致跨环境失败，建议改用相对路径或配置文件
	outputfile << "start" << endl;

	//2. AEB 状态与 SimOneAPI 初始化
	bool inAEBState = false;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);	//需确保与仿真平台连接成功，否则后续操作无效
	SimOneAPI::SetDriverName(MainVehicleId, "AUTO");				//车辆ID
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);	//驾驶模式
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);	//初始化评测服务
	//加载地图
	int timeout = 20;	//设置超时
	while (true) {
		//加载高精地图（HD Map），成功则记录日志并退出循环，失败则持续重试
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}
	SSD::SimPoint3D MainVehiclePos(Gps.posX, Gps.posY, Gps.posZ);	//主车的三维坐标（X, Y, Z）
	double ini_t = 0;				// 横向位置，初始化为 0
	double v0 = 0, v1 = 0, v2 = 0;	// 当前及前两帧速度（用于加速度计算或滤波）
	double e0 = 0, e1 = 0, e2 = 0;	// 控制误差（如PID中的历史误差）
	double a0 = 0;					// 当前加速度
	double the_throttle = 0, the_brake = 0;	 // 油门与刹车控制量（范围通常为0~1）
	SimOne_Data_Gps vGps;			// GPS数据容器（需通过API更新）
	SimOne_Data_Obstacle vObstacle;	// 障碍物数据容器
	/*
		int obstacleSize 障碍物的数量，即在 obstacle 数组中包含的障碍物的总数
		SimOne_Data_Obstacle_Entry obstacle [ 255 ] 包含所有障碍物详细信息的数组，数组大小限制为255个障碍物
	*/

	//3. 生成路径
	SSD::SimPoint3DVector inputPoints;	// 存储路径点的三维坐标容器
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();	//智能指针管理路径点对象
	//获取路径点
	if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
	{
		//遍历处理路径点
		for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) 
		{
			SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else
	{
		//错误处理
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
		return -1;
	}
	outputfile << "start1" << endl;		//日志输出
	SSD::SimPoint3DVector targetPath;	//存储生成的路由路径
	if (pWayPoints->wayPointsSize >= 2) 
	{
		//多路径点生成路径
		SSD::SimVector<int> indexOfValidPoints;	//有效点索引，可能用于过滤无效点
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) //根据输入的多个路径点生成全局路径
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
			return -1;
		}
	}
	else if (pWayPoints->wayPointsSize == 1) 
	{
		//单路径点获取车道中心线
		//1. 获取最近车道ID
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);	
		//2. 获取车道详细信息
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) 	
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
			return -1;
		}
		else 
		{
			targetPath = laneInfoInit.centerLine;	//赋值中心线
		}
	}
	else 
	{
		//无路径点错误处理
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
		return -1;
	}

	bool flag = SimOneAPI::GetCaseInfo(&pCaseInfoTest);	//获取案例信息
	//记录案例信息到文件
	string id = pCaseInfoTest.caseId;
	outputfile << "123:" << id << "name" << pCaseInfoTest.caseName << endl;
	outputfile << id << endl;
	outputfile << "??" << (id == CASE_41) << endl;


	if (flag)
	{
		//根据不同案例写算法
		if (id == CASE_1 || id == CASE_2 || id == CASE_3)//case1
		{
			outputfile << "1.2.3" << endl;
			while (true) {
				int frame = SimOneAPI::Wait();
				//获取当前案例运行情况：停止
				if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
					SimOneAPI::SaveEvaluationRecord(); //保存评价
					break;
				}
				//获取主车地图 GPS 信息
				std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
				if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
				}
				//获取障碍物信息
				std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
				if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
				}
				//获取当前案例运行情况：运行
				if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
					//simone 案例初始化
					if (!isSimOneInitialized) {
						SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
						isSimOneInitialized = true;
					}
					//主车空间位置信息
					SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
					double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ); //主车的速度（根据三维矢量求出速度的标量值）
					//维护最近三次的速度值
					v2 = v1;
					v1 = v0;
					v0 = mainVehicleSpeed;
					//最小距离初始化为 double 类型所能表示的最大数值，便于在遍历障碍物时能够找到最小距离值，不引发逻辑错误
					double minDistance = std::numeric_limits<double>::max(); 
					//std::vector<utilTargetObstacle::ObstacleStruct> allObstacles = utilTargetObstacle::GetObstacleList();
					int potentialObstacleIndex = pObstacle->obstacleSize; 						//获取潜在障碍物的位置索引
					SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos); 	//获取距离主车最近的道路ID
					SSD::SimString potentialObstacleLaneId = ""; 								//声明距离潜在障碍物最近的道路ID
					//根据障碍物大小扫描遍历，得到距离车辆最近的像素点信息和主车与障碍物的距离
					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) 
					{
						SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ); //获取障碍物空间位置信息
						SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos); //获取障碍物点所在道路ID
						if (mainVehicleLaneId == obstacleLaneId) //如果所在道路相同
						{
							double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos); //提取主车和障碍物距离
							if (obstacleDistance < minDistance) //主车和障碍物距离小于最小限制时
							{
								minDistance = obstacleDistance; 			//更新主车和障碍物最小距离
								potentialObstacleIndex = (int)i; 			//更新潜在障碍物位置索引
								potentialObstacleLaneId = obstacleLaneId; 	//潜在障碍物道路ID更新（同主车）
							}
						}
					}

					char str[50] = "mindistance=";
					char str1[50];
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, str);

					auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
					//获取障碍物速度（标量）
					double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);
					//将潜在障碍物的空间位置坐标 封装到 SimPoint3D
					SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
					//障碍物相对于道路参考线的 ST 坐标（沿道路参考线的纵向、横向坐标）
					double sObstacle = 0;
					double tObstacle = 0;
					//主车相对于道路参考线的 ST 坐标
					double sMainVehicle = 0;
					double tMainVehicle = 0;


					char str2[50] = "当前道路是否存在障碍物：";
					char str3[50] = "存在障碍物";
					char str4[50] = "不存在障碍物";
					bool isObstacleBehind = false;
					double start_distance = (0 - pow(v0, 2)) / 2.0 / end_a + 7; //GEN 20 NO 7（？？？）
					if (!potentialObstacleLaneId.Empty()) {
						//获取障碍物相对于道路参考线的 ST 坐标
						SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
						if (ini_t == 0)
						{
							ini_t = tObstacle; // 记录初始横向位置
						}
						//获取主车相对于道路参考线的 ST 坐标
						SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);
						//判断障碍物位置
						isObstacleBehind = (!(sMainVehicle >= sObstalce)) && (abs(tMainVehicle - tObstacle) < 3.5) && (minDistance <= start_distance);
					}
					else
					{

					}
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, str2);

					/* 根据障碍物位置动态调整车速和安全制动 */
					std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>(); // 创建车辆控制对象，准备设置油门、刹车等参数。

					// Control mainVehicle with SimOneDriver
					SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());	// Control mainVehicle with SimOneDriver 获取当前车辆控制权

					/* Control mainVehicle without SimOneDriver
					pControl->throttle = 0.5;
					pControl->brake = 0;
					pControl->steering = 0;
					pControl->handbrake = 0;
					pControl->isManualGear = 0;
					pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
					*/

					//默认目标速度
					double end_speed = 5.0;
					if (isObstacleBehind)
					{
						end_speed = obstacleSpeed; 
					}
					//安全距离计算（这里公式好像有问题）
					double safe_distance = (0 - pow(v0, 2)) / 2 / (brake_a_max)+7; 
					if (v0 <= end_speed)
					{	
						//加速控制
						double temp = pow(end_speed, 2) - 2 * (-end_a) *(minDistance - safe_distance); //期望速度平方
						if (temp >= pow(v0, 2))
						{
							double get_v = sqrt(temp);
							e2 = e1;
							e1 = e0;
							e0 = get_v - v0;
							double change_temp = 0.5 * (e0 - e1) + 0.3 * e0 + p3 * (e0 - 2 * e1 + e2);
							if (change_temp > 0)
							{
								the_throttle = change_temp;
								the_brake = 0;
							}
							else
							{
								the_throttle = 0;
								the_brake = -change_temp;
							}
						}
						else if (minDistance - safe_distance > 0)
						{
							the_throttle += 0.005;
							the_brake = 0;
						}
					}
					else
					{
						//减速控制
						double temp = pow(end_speed, 2) - 2 * end_a *(minDistance - safe_distance); //期望速度平方
						double get_v;
						if (temp <= pow(v0, 2))
						{
							get_v = sqrt(temp);
						}
						else
						{
							get_v = end_speed;
						}
						e2 = e1;
						e1 = e0;
						e0 = get_v - v0;
						double change_temp = p1 * (e0 - e1) + p2 * e0 + p3 * (e0 - 2 * e1 + e2);
						if (change_temp > 0)
						{
							the_throttle = change_temp;
							the_brake = 0;
						}
						else
						{
							the_throttle = 0;
							the_brake = -change_temp;
						}
					}
					
					pControl->throttle = the_throttle;
					pControl->brake = the_brake;
					SimOneAPI::SetDrive(MainVehicleId, pControl.get()); //设置主车的状态
				}
				else
				{
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
				}

				SimOneAPI::NextFrame(frame);
			}
		}
		else if (id == CASE_11)
		{
			while (true) {
				int frame = SimOneAPI::Wait();
				//获取当前案例运行情况：停止
				if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
					SimOneAPI::SaveEvaluationRecord(); //保存评价
					break;
				}
				//获取主车地图 GPS 信息
				std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
				if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
				}
				//获取障碍物信息
				std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
				if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
				}
				//获取当前案例运行情况：运行
				if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
					//simone 案例初始化
					if (!isSimOneInitialized) {
						SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
						isSimOneInitialized = true;
					}
					//主车空间位置信息
					SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
					double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

					double minDistance = std::numeric_limits<double>::max();
					int potentialObstacleIndex = pObstacle->obstacleSize;					
					SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
					SSD::SimString potentialObstacleLaneId = "";								
					//根据障碍物大小扫描遍历，得到距离车辆最近的像素点信息和主车与障碍物的距离
					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
						//获取障碍物空间位置信息
						SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
						//获取障碍物点所在道路ID
						SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
						if (mainVehicleLaneId == obstacleLaneId) 	//如果所在道路相同
						{
							//提取主车和障碍物距离
							double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);	
							if (obstacleDistance < minDistance) //主车和障碍物距离小于最小限制时
							{
								minDistance = obstacleDistance;			
								
								potentialObstacleIndex = (int)i;			
								potentialObstacleLaneId = obstacleLaneId;	
							}
						}
					}
		
					auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
					//获取障碍物速度（标量）
					double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);
		
					//将潜在障碍物的空间位置坐标 封装到 SimPoint3D
					SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
					//障碍物相对于道路参考线的 ST 坐标（沿道路参考线的纵向、横向坐标）
					double sObstalce = 0;
					double tObstacle = 0;
					//主车相对于道路参考线的 ST 坐标
					double sMainVehicle = 0;
					double tMainVehicle = 0;
		
					bool isObstalceBehind = false;
					if (!potentialObstacleLaneId.Empty()) 
					{
						//获取障碍物相对于道路参考线的 ST 坐标
						SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
						//获取主车相对于道路参考线的 ST 坐标
						SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);
						//判断障碍物位置
						isObstalceBehind = !(sMainVehicle >= sObstalce);
					}
		
					std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
		
					// Control mainVehicle with SimOneDriver
					SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());
		
					// Control mainVehicle without SimOneDriver
					/*pControl->throttle = 0.5;
					pControl->brake = 0;
					pControl->steering = 0;
					pControl->handbrake = 0;
					pControl->isManualGear = 0;
					pControl->gear = static_cast<ESimOne_Gear_Mode>(1);*/
		
					if (isObstalceBehind) //障碍物在后方时
					{
						double defaultDistance = 10.f;
						double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
						double defautlTimeToCollision = 3.4f;
						if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0) {
							inAEBState = true;
							pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
						}
		
						if (inAEBState) {
							pControl->throttle = 0.;
						}
					}	
					SimOneAPI::SetDrive(MainVehicleId, pControl.get());
				}
				else {
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
				}
		
				SimOneAPI::NextFrame(frame);
			}
		}
		else	//其余情况
		{

		}
	}
	
	return 0;
}