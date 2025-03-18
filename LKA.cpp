#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "SampleGetNearMostLane.h"
#include "SampleGetLaneST.h"
#include "SimOneEvaluationAPI.h"
#include <iostream>
#include <memory>
#include <cstring>

//Main function
//
#define ON (1)
#define OFF (0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))                   
#define MAX(a, b) (((a) > (b)) ? (a) : (b))                   
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

uint8_t left_lane = OFF;
uint8_t right_lane = OFF;
uint8_t change_road = OFF;
uint8_t use_fixd_steering = OFF;
uint16_t dis_CR = 5; // defalut distance when change road;
float defalut_brake = 0.0f;
float defalut_throttle = 0.12f;
SSD::SimString mainVehicleLaneId;
SSD::SimString mainVehicleLaneId_last;
SSD::SimPoint3DVector inputPoints;
SSD::SimPoint3DVector targetPath;
SSD::SimPoint3D targetPointPos;
void lane_reset(void);
void change_road_function(SSD::SimPoint3DVector * inps, SSD::SimPoint3D &start_p, SSD::SimPoint3D &end_p);
bool IsCarInLeftLane(SSD::SimPoint3D MainCarPos, SSD::SimString MainCarLandId, SimOne_Data_Obstacle *pObstacle, HDMapStandalone::MLaneLink laneLink, double &LefrCarSpeed);

int main()
{
    uint64_t fps_count = 0;
    bool inAEBState = false;
    int timeout = 20;
    bool isSimOneInitialized = false;
    const char* MainVehicleId = "0";
    bool isJoinTimeLoop = true;
    SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
    SimOneAPI::SetDriverName(MainVehicleId, "RIO");
    SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
    while (true) {
        if (SimOneAPI::LoadHDMap(timeout)) {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
            break;
        }
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
    }


    std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
    if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
    {
        for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
            SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
            inputPoints.push_back(inputWayPoints);
            //std::cout << "i: "<<i << "x: "<<pWayPoints->wayPoints[i].posX<<"\ty: "<< pWayPoints->wayPoints[i].posY << std::endl;
        }
        targetPointPos = inputPoints.back();
    }
    else {
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
        return -1;
    }

    //std::cout << "way_p size: " << pWayPoints->wayPointsSize << std::endl;

    if (pWayPoints->wayPointsSize >= 2) {
        SSD::SimVector<int> indexOfValidPoints;
        if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
            return -1;
        }
    }
    else if (pWayPoints->wayPointsSize == 1) {
        SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
        HDMapStandalone::MLaneInfo laneInfoInit;
        if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
            return -1;
        }
        else {
            targetPath = laneInfoInit.centerLine;
        }
    }
    else {
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
        return -1;
    }
    /*for (int i = 0; i < targetPath.size(); i++)
    {
        std::cout <<"i: "<<i << "\tx: " << targetPath[i].x << "\t y: " << targetPath[i].y << std::endl;
    }*/
    while (true) {
        /* reset for sth */
        fps_count++;
        lane_reset();
        int frame = SimOneAPI::Wait();

        if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
            SimOneAPI::SaveEvaluationRecord();
            break;
        }

        std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
        if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
        }

        std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
        if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
        }

        if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
            if (!isSimOneInitialized) {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
                isSimOneInitialized = true;
            }

            SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
            double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

            double minDistance = std::numeric_limits<double>::max();
            int potentialObstacleIndex = pObstacle->obstacleSize;
            mainVehicleLaneId_last = mainVehicleLaneId;
            mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
            if (fps_count == 1)
            {
                mainVehicleLaneId_last = mainVehicleLaneId;
            }
            SSD::SimString potentialObstacleLaneId = "";
            for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
                SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
                SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
                if (mainVehicleLaneId == obstacleLaneId) {
                    double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

                    if (obstacleDistance < minDistance) {
                        minDistance = obstacleDistance;
                        potentialObstacleIndex = (int)i;
                        potentialObstacleLaneId = obstacleLaneId;
                    }
                }
            }

            auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
            double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);
            SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
            double sObstalce = 0.;
            double tObstacle = 0.;

            double sMainVehicle = 0.;
            double tMainVehicle = 0.;

            bool isObstalceBehind = false;
            if (!potentialObstacleLaneId.Empty()) {

                SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
                SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                isObstalceBehind = !(sMainVehicle >= sObstalce);
            }

            std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
            std::unique_ptr<SimOne_Data_Signal_Lights> pLight = std::make_unique<SimOne_Data_Signal_Lights>();


            // Control mainVehicle with SimOneDriver
            //SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

            // Control mainVehicle without SimOneDriver
            pControl->throttle = 0.12f;
            pControl->brake = defalut_brake;
            pControl->steering = 0.f;
            pControl->handbrake = false;
            pControl->isManualGear = false;
            pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
            pLight->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_None;

            //std::cout << pObstacle->obstacleSize <<std:: endl;
            //std::cout << "target_path size: " << targetPath.size() << std::endl;
            //std::cout << "land id: "<< mainVehicleLaneId.GetString() <<std:: endl;

            HDMapStandalone::MLaneLink laneLink;
            SimOneAPI::GetLaneLink(mainVehicleLaneId, laneLink);
            if (strlen(laneLink.leftNeighborLaneName.GetString()) != 0)
            {
                left_lane = ON;
                //std::cout << "left: " << laneLink.leftNeighborLaneName.GetString() << std::endl;
            }
            if (strlen(laneLink.rightNeighborLaneName.GetString()) != 0)
            {
                right_lane = ON;
                //std::cout << "right: " << laneLink.rightNeighborLaneName.GetString() << std::endl;
            }
            //std::cout << "obs speed: " << obstacleSpeed<<std::endl;

            //std::cout << "now lane id: " << mainVehicleLaneId.GetString() << std::endl;
            //std::cout << "last lane id: " << mainVehicleLaneId_last.GetString() << std::endl;

            if (isObstalceBehind  && change_road == OFF)
            {


                if ((left_lane == ON || right_lane == ON) && minDistance <= 40 && std::abs(obstacleSpeed) <= 2 && fps_count >= 5)
                {
                    if (minDistance <= 20 && fps_count <= 400)
                    {
                        if (fps_count <= 150)
                        {
                            pControl->gear = static_cast<ESimOne_Gear_Mode>(2);
                            pControl->brake = 1.0f;
                            SimOneAPI::SetDrive(MainVehicleId, pControl.get());
                            std::cout << "waiting\n" << std::endl;
                        }
                        else
                        {
                            pControl->gear = static_cast<ESimOne_Gear_Mode>(2);
                            SimOneAPI::SetDrive(MainVehicleId, pControl.get());
                            std::cout << "back back\n" << std::endl;
                        }
                        goto END;
                    }
                    if (minDistance <= 22)
                    {
                        change_road = ON;
                        use_fixd_steering = ON;
                    }
                    else
                    {
                        if ((int(sqrtf(pow(pGps->velX, 2) + pow(pGps->velY, 2))) * 3.6f) > 10) {
                            pControl->throttle = 0.0f;
                            pControl->brake = 0.3f;
                        }
                        else {
                            pControl->throttle = 0.2f;
                            pControl->brake = 0.0f;
                        }
                    }
                }
                else
                {
                    if (pObstacle->obstacleSize == 3)
                    {
                        if (minDistance <= 11.5)
                        {
                            pControl->throttle = 0.f;
                            pControl->brake = 1.0f;
                        }
                    }
                    else
                    {
                        if (minDistance > 11.5 && minDistance <= 40)
                        {
                            double defaultDistance = 10.;
                            double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
                            double defautlTimeToCollision = 3.4;
                            if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0) {
                                inAEBState = true;
                                pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
                            }

                            if (inAEBState) {
                                pControl->throttle = 0.05f;
                            }
                            if (minDistance <= 25 && std::abs(obstacleSpeed)<=1.0 && potentialObstacle.type== ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
                            {
                                pControl->throttle = 0.f;
                                pControl->brake = 1.0f;
                            }
                        }
                        else if (minDistance>40 && potentialObstacle.type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
                        {
                            pControl->throttle = 0.2f;
                        }
                    }
                }

            }
            else if (change_road == ON)
            {
                pControl->throttle = 0.0f;
                if (left_lane == ON)
                {
                    pControl->steering = MINMAX(-dis_CR / (float)minDistance, -0.3f, 0.3f);
                    pLight->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_LeftBlinker;
                }
                else if (right_lane == ON)
                {
                    pControl->steering = MINMAX(dis_CR / (float)minDistance, -0.3f, 0.3f);
                    pLight->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_RightBlinker;
                }

                if (strcmp(mainVehicleLaneId.GetString(), mainVehicleLaneId_last.GetString()) != 0)
                {
                    change_road = OFF;
                    use_fixd_steering = OFF;
                    targetPath.clear();
                    change_road_function(&inputPoints, mainVehiclePos, targetPointPos);
                    SSD::SimVector<int> tempindexOfValidPoints;
                    if (!SimOneAPI::GenerateRoute(inputPoints, tempindexOfValidPoints, targetPath)) {
                        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed when finish change road");
                        return -1;
                    }
                }
            }

            double LefrCarSpeed = 0.0;
            if (IsCarInLeftLane(mainVehiclePos, mainVehicleLaneId, pObstacle.get(), laneLink, LefrCarSpeed))
            {
                if ((mainVehicleSpeed) > LefrCarSpeed / 2.0) {
                    pControl->throttle = 0.0f;
                    pControl->brake = 0.3f;
                }
                else {
                    pControl->throttle = 0.2f;
                    pControl->brake = 0.0f;
                }
            }
                        
            if (use_fixd_steering == OFF)
            {
                double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
                pControl->steering = (float)steering *1.5f;
                //std::cout << "sterring: "<<pControl->steering << std::endl;
            }
            else
            {
                //std::cout << "fix steering: "<<pControl->steering<<std::endl;
            }


            SimOneAPI::SetSignalLights(MainVehicleId, pLight.get());
            SimOneAPI::SetDrive(MainVehicleId, pControl.get());
        }
        else {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
        }
    END:
        SimOneAPI::NextFrame(frame);
    }
    return 0;
}


void lane_reset(void)
{
    left_lane = OFF;
    right_lane = OFF;
}


void change_road_function(SSD::SimPoint3DVector * inps, SSD::SimPoint3D &start_p, SSD::SimPoint3D &end_p)
{
    inps->clear();
    inps->push_back(start_p);
    inps->push_back(end_p);
}


bool IsCarInLeftLane(SSD::SimPoint3D MainCarPos, SSD::SimString MainCarLandId, SimOne_Data_Obstacle *pObstacle, HDMapStandalone::MLaneLink laneLink, double &LefrCarSpeed)
{
    bool res = false;
    SSD::SimPoint3D CarPos;
    double MinDistance = std::numeric_limits<double>::max();
    SSD::SimString CarLaneId = "";
    double s, t, s_toCenterLine, t_toCenterLine;
    for (size_t i = 0; i < pObstacle->obstacleSize; ++i)
    {
        if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
        {

            CarPos = { pObstacle->obstacle[i].posX,pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ };
            SimOneAPI::GetNearMostLane(CarPos, CarLaneId, s, t, s_toCenterLine, t_toCenterLine);
            double Distance = UtilMath::planarDistance(MainCarPos, CarPos);
            //std::cout << "dis : "<<Distance<<std::endl;

            if (strcmp(CarLaneId.GetString(), laneLink.leftNeighborLaneName.GetString()) == 0 && Distance <= 35.0)
            {
                std::cout << "car in left\n";
                if (Distance < MinDistance)
                {
                    MinDistance = Distance;
                    LefrCarSpeed = UtilMath::calculateSpeed(pObstacle->obstacle[i].velX, pObstacle->obstacle[i].velY, pObstacle->obstacle[i].velZ);
                    res = true;
                }
            }
        }
    }
    return res;
}