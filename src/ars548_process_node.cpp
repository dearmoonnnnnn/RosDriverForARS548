#include <ros/ros.h>
#include <ars548_process/objects.h>
#include <ars548_process/ObjectList.h>
#include <ars548_process/detections.h>
#include <ars548_process/DetectionList.h>
#include <ars548_process/RadarBasicStatus.h>
#include <ars548_process/AccelerationLateralCog.h>
#include <ars548_process/AccelerationLongitudinalCog.h>
#include <ars548_process/CharacteristicSpeed.h>
#include <ars548_process/DrivingDirection.h>
#include <ars548_process/SteeringAngleFrontAxle.h>
#include <ars548_process/VelocityVehicle.h>
#include <ars548_process/YawRate.h>

#include <thread>

#include "data_struct.h"
#include "data_process.h"
#include "converttype.h"

#include <fstream>
#include <nlohmann/json.hpp>


DataProcess data_process;
ConvertType cvt;

char rec_data[40000];
char send_data[1024];
int rec_length;

// const std::string  json_file_path =  "/home/dearmoon/datasets/NWU/日晴半室内低速2/4DRadar/RiQingBanShiNeiDiSu3.json";
// const std::string  json_file_path =  "/home/dearmoon/datasets/NWU/json/xr_firstPacket.json";
const std::string  json_file_path =  "/home/dearmoon/datasets/NWU/日晴不颠簸低速3/4DRadar/日晴不颠簸低速3.json";

RadarObjectList object_list;
RadarDetectionList detection_list;
RadarBasicStatus basic_status;


ros::Publisher objects_pub;
ros::Publisher detections_pub;
ros::Publisher status_pub;

void publishObjectList(RadarObjectList list, const std::string& timestampStr)
{
    ars548_process::objects obj;
    ars548_process::ObjectList obj_list;
    int i;

    double timestamp = std::stod(timestampStr);

    // obj_list.header.stamp = timestamp.toSec();  obj_list没有header成员
    obj_list.object_array.clear();

    obj_list.serviceID = list.serviceID;
    obj_list.MethodID = list.MethodID;
    obj_list.data_length = list.data_length;
    obj_list.clientID = list.clientID;
    obj_list.sessionID = list.sessionID;
    obj_list.protocol_version = list.protocol_version;
    obj_list.interface_version = list.interface_version;
    obj_list.message_type = list.message_type;
    obj_list.return_code = list.return_code;
    obj_list.CRC = list.CRC;
    obj_list.Length = list.Length;
    obj_list.SQC = list.SQC;
    obj_list.DataID = list.DataID;
    obj_list.Timestamp_Nanoseconds = list.Timestamp_Nanoseconds;
    obj_list.Timestamp_Seconds = list.Timestamp_Seconds;
    obj_list.Timestamp_SyncStatus = list.Timestamp_SyncStatus;
    obj_list.EventDataQualifier = list.EventDataQualifier;
    obj_list.ExtendedQualifier = list.ExtendedQualifier;
    obj_list.ObjectList_NumOfObjects = list.ObjectList_NumOfObjects;

    for (i=0; i < list.ObjectList_NumOfObjects; i++)
    {
        obj.header.frame_id = "world";
        obj.header.stamp = ros::Time(timestamp);

        obj.u_StatusSensor = list.object_array[i].u_StatusSensor;
        obj.u_ID = list.object_array[i].u_ID;
        obj.u_Age = list.object_array[i].u_Age;
        obj.u_StatusMeasurement = list.object_array[i].u_StatusMeasurement;
        obj.u_StatusMovement = list.object_array[i].u_StatusMovement;
        obj.u_Position_InvalidFlags = list.object_array[i].u_Position_InvalidFlags;
        obj.u_Position_Reference = list.object_array[i].u_Position_Reference;
        obj.u_Position_X = list.object_array[i].u_Position_X;
        obj.u_Position_X_STD = list.object_array[i].u_Position_X_STD;
        obj.u_Position_Y = list.object_array[i].u_Position_Y;
        obj.u_Position_Y_STD = list.object_array[i].u_Position_Y_STD;
        obj.u_Position_Z = list.object_array[i].u_Position_Z;
        obj.u_Position_Z_STD = list.object_array[i].u_Position_Z_STD;
        obj.u_Position_CovarianceXY = list.object_array[i].u_Position_CovarianceXY;
        obj.u_Position_Orientation = list.object_array[i].u_Position_Orientation;
        obj.u_Position_Orientation_STD = list.object_array[i].u_Position_Orientation_STD;
        obj.u_Existence_InvalidFlags = list.object_array[i].u_Existence_InvalidFlags;
        obj.u_Existence_Probability = list.object_array[i].u_Existence_Probability;
        obj.u_Existence_PPV = list.object_array[i].u_Existence_PPV;
        obj.u_Classification_Car = list.object_array[i].u_Classification_Car;
        obj.u_Classification_Truck = list.object_array[i].u_Classification_Truck;
        obj.u_Classification_Motorcycle = list.object_array[i].u_Classification_Motorcycle;
        obj.u_Classification_Bicycle = list.object_array[i].u_Classification_Bicycle;
        obj.u_Classification_Pedestrian = list.object_array[i].u_Classification_Pedestrian;
        obj.u_Classification_Animal = list.object_array[i].u_Classification_Animal;
        obj.u_Classification_Hazard = list.object_array[i].u_Classification_Hazard;
        obj.u_Classification_Unknown = list.object_array[i].u_Classification_Unknown;
        obj.u_Classification_Overdrivable = list.object_array[i].u_Classification_Overdrivable;
        obj.u_Classification_Underdrivable = list.object_array[i].u_Classification_Underdrivable;
        obj.u_Dynamics_AbsVel_InvalidFlags = list.object_array[i].u_Dynamics_AbsVel_InvalidFlags;
        obj.f_Dynamics_AbsVel_X = list.object_array[i].f_Dynamics_AbsVel_X;
        obj.f_Dynamics_AbsVel_X_STD = list.object_array[i].f_Dynamics_AbsVel_X_STD;
        obj.f_Dynamics_AbsVel_Y = list.object_array[i].f_Dynamics_AbsVel_Y;
        obj.f_Dynamics_AbsVel_Y_STD = list.object_array[i].f_Dynamics_AbsVel_Y_STD;
        obj.f_Dynamics_AbsVel_CovarianceXY = list.object_array[i].f_Dynamics_AbsVel_CovarianceXY;
        obj.u_Dynamics_RelVel_InvalidFlags = list.object_array[i].u_Dynamics_RelVel_InvalidFlags;
        obj.f_Dynamics_RelVel_X = list.object_array[i].f_Dynamics_RelVel_X;
        obj.f_Dynamics_RelVel_X_STD = list.object_array[i].f_Dynamics_RelVel_X_STD;
        obj.f_Dynamics_RelVel_Y = list.object_array[i].f_Dynamics_RelVel_Y;
        obj.f_Dynamics_RelVel_Y_STD = list.object_array[i].f_Dynamics_RelVel_Y_STD;
        obj.f_Dynamics_RelVel_CovarianceXY = list.object_array[i].f_Dynamics_RelVel_CovarianceXY;
        obj.u_Dynamics_AbsAccel_InvalidFlags = list.object_array[i].u_Dynamics_AbsAccel_InvalidFlags;
        obj.f_Dynamics_AbsAccel_X = list.object_array[i].f_Dynamics_AbsAccel_X;
        obj.f_Dynamics_AbsAccel_X_STD = list.object_array[i].f_Dynamics_AbsAccel_X_STD;
        obj.f_Dynamics_AbsAccel_Y = list.object_array[i].f_Dynamics_AbsAccel_Y;
        obj.f_Dynamics_AbsAccel_Y_STD = list.object_array[i].f_Dynamics_AbsAccel_Y_STD;
        obj.f_Dynamics_AbsAccel_CovarianceXY = list.object_array[i].f_Dynamics_AbsAccel_CovarianceXY;
        obj.u_Dynamics_RelAccel_InvalidFlags = list.object_array[i].u_Dynamics_RelAccel_InvalidFlags;
        obj.f_Dynamics_RelAccel_X = list.object_array[i].f_Dynamics_RelAccel_X;
        obj.f_Dynamics_RelAccel_X_STD = list.object_array[i].f_Dynamics_RelAccel_X_STD;
        obj.f_Dynamics_RelAccel_Y = list.object_array[i].f_Dynamics_RelAccel_Y;
        obj.f_Dynamics_RelAccel_Y_STD = list.object_array[i].f_Dynamics_RelAccel_Y_STD;
        obj.f_Dynamics_RelAccel_CovarianceXY = list.object_array[i].f_Dynamics_RelAccel_CovarianceXY;
        obj.u_Dynamics_Orientation_InvalidFlags = list.object_array[i].u_Dynamics_Orientation_InvalidFlags;
        obj.u_Dynamics_Orientation_Rate_Mean = list.object_array[i].u_Dynamics_Orientation_Rate_Mean;
        obj.u_Dynamics_Orientation_Rate_STD = list.object_array[i].u_Dynamics_Orientation_Rate_STD;
        obj.u_Shape_Length_Status = list.object_array[i].u_Shape_Length_Status;
        obj.u_Shape_Length_Edge_InvalidFlags = list.object_array[i].u_Shape_Length_Edge_InvalidFlags;
        obj.u_Shape_Length_Edge_Mean = list.object_array[i].u_Shape_Length_Edge_Mean;
        obj.u_Shape_Length_Edge_STD = list.object_array[i].u_Shape_Length_Edge_STD;
        obj.u_Shape_Width_Status = list.object_array[i].u_Shape_Width_Status;
        obj.u_Shape_Width_Edge_InvalidFlags = list.object_array[i].u_Shape_Width_Edge_InvalidFlags;
        obj.u_Shape_Width_Edge_Mean = list.object_array[i].u_Shape_Width_Edge_Mean;
        obj.u_Shape_Width_Edge_STD = list.object_array[i].u_Shape_Width_Edge_STD;
       
        obj_list.object_array.push_back(obj);
    }

    objects_pub.publish(obj_list);

}

void publishDetectionList(RadarDetectionList list, const std::string& timestampStr)
{
    
    ars548_process::detections det;
    ars548_process::DetectionList det_list;
    int i;

    // Split the string into seconds and nanoseconds
    size_t dotPos = timestampStr.find('.');
    
    // Extract the integer and decimal parts
    std::string secondsStr = timestampStr.substr(0, dotPos);
    std::string nanosecondsStr = timestampStr.substr(dotPos + 1);

    int32_t seconds = std::stoi(secondsStr);
    int32_t nanoseconds = std::stoi(nanosecondsStr);


    //std::cout << "seconds" << seconds<< std::endl;
    //std::cout << "nonoseconds" << nanoseconds << std::endl;
    //det_list.header.stamp = timestamp;
    det_list.detection_array.clear();

    det_list.serviceID = list.serviceID;
    det_list.MethodID = list.MethodID;
    det_list.data_length = list.data_length;
    det_list.clientID = list.clientID;
    det_list.sessionID = list.sessionID;
    det_list.protocol_version = list.protocol_version;
    det_list.interface_version = list.interface_version;
    det_list.message_type = list.message_type;
    det_list.return_code = list.return_code;
    det_list.CRC = list.CRC;
    det_list.Length = list.Length;
    det_list.SQC = list.SQC;
    det_list.DataID = list.DataID;
    det_list.Timestamp_Nanoseconds = list.Timestamp_Nanoseconds;
    det_list.Timestamp_Seconds = list.Timestamp_Seconds;
    det_list.Timestamp_SyncStatus = list.Timestamp_SyncStatus;
    det_list.EventDataQualifier = list.EventDataQualifier;
    det_list.ExtendedQualifier = list.ExtendedQualifier;
    det_list.Origin_InvalidFlags = list.Origin_InvalidFlags;
    det_list.Origin_Xpos = list.Origin_Xpos;
    det_list.Origin_Xstd = list.Origin_Xstd;
    det_list.Origin_Ypos = list.Origin_Ypos;
    det_list.Origin_Ystd = list.Origin_Ystd;
    det_list.Origin_Zpos = list.Origin_Zpos;
    det_list.Origin_Zstd = list.Origin_Zstd;
    det_list.Origin_Roll = list.Origin_Roll;
    det_list.Origin_Rollstd = list.Origin_Rollstd;
    det_list.Origin_Pitch = list.Origin_Pitch;
    det_list.Origin_Pitchstd = list.Origin_Pitchstd;
    det_list.Origin_Yaw = list.Origin_Yaw;
    det_list.Origin_Yawstd = list.Origin_Yawstd;
    det_list.List_InvalidFlags = list.List_InvalidFlags;
    det_list.List_RadVelDomain_Min = list.List_RadVelDomain_Min;
    det_list.List_RadVelDomain_Max = list.List_RadVelDomain_Max;
    det_list.List_NumOfDetections = list.List_NumOfDetections;
    det_list.Aln_AzimuthCorrection = list.Aln_AzimuthCorrection;
    det_list.Aln_ElevationCorrection = list.Aln_ElevationCorrection;
    det_list.Aln_Status = list.Aln_Status;

    for(i = 0; i < list.List_NumOfDetections; i++)
    {
        det.header.frame_id = "world";
        det.header.stamp = ros::Time(seconds, nanoseconds);
        // std::cout << "det.header.stamp:" << det.header.stamp << std::endl;

        det.f_x = list.detection_array[i].f_x;
        det.f_y = list.detection_array[i].f_y;
        det.f_z = list.detection_array[i].f_z;
        det.u_InvalidFlags = list.detection_array[i].u_InvalidFlags;
        det.f_RangeRate = list.detection_array[i].f_RangeRate;
        det.f_RangeRateSTD = list.detection_array[i].f_RangeRateSTD;
        det.s_RCS = list.detection_array[i].s_RCS;
        if(1)
        {
            std::cout << "/****** publishDetectionList " << std::endl;
            std::cout << "/------ 右值 s_RCS : " << list.detection_array[i].s_RCS << std::endl;
            std::cout << "/------ 左值 s_RCS : " << det.s_RCS << std::endl;
        }
        det.u_MeasurementID = list.detection_array[i].u_MeasurementID;
        det.u_PositivePredictiveValue = list.detection_array[i].u_PositivePredictiveValue;
        det.u_Classification = list.detection_array[i].u_Classification;
        det.u_MultiTargetProbability = list.detection_array[i].u_MultiTargetProbability;
        det.u_ObjectID = list.detection_array[i].u_ObjectID;
        det.u_AmbiguityFlag = list.detection_array[i].u_AmbiguityFlag;
        det.u_SortIndex = list.detection_array[i].u_SortIndex;

        det_list.detection_array.push_back(det);
    }

    detections_pub.publish(det_list);    
}

void publishBasicStatus(RadarBasicStatus status)
{
    ars548_process::RadarBasicStatus sts;

    sts.header.frame_id = "world";
    sts.header.stamp = ros::Time::now();

    sts.Longitudinal = status.Longitudinal;
    sts.Lateral = status.Lateral;
    sts.Vertical = status.Vertical;
    sts.Yaw = status.Yaw;
    sts.Pitch = status.Pitch;
    sts.PlugOrientation = status.PlugOrientation;
    sts.Length = status.Length;
    sts.Width = status.Width;
    sts.Height = status.Height;
    sts.Wheelbase = status.Wheelbase;
    sts.MaximumDistance = status.MaximumDistance;
    sts.FrequencySlot = status.FrequencySlot;
    sts.CycleTime = status.CycleTime;
    sts.TimeSlot = status.TimeSlot;
    sts.HCC = status.HCC;
    sts.Powersave_Standstill = status.Powersave_Standstill;
    sts.SensorIPAddress_0 = status.SensorIPAddress_0;
    sts.SensorIPAddress_1 = status.SensorIPAddress_1;
    sts.Configuration_counter = status.Configuration_counter;
    sts.Status_LongitudinalVelocity = status.Status_LongitudinalVelocity;
    sts.Status_LongitudinalAcceleration = status.Status_LongitudinalAcceleration;
    sts.Status_LateralAcceleration = status.Status_LateralAcceleration;
    sts.Status_YawRate = status.Status_YawRate;
    sts.Status_SteeringAngle = status.Status_SteeringAngle;
    sts.Status_DrivingDirection = status.Status_DrivingDirection;
    sts.Status_CharacteristicSpeed = status.Status_CharacteristicSpeed;
    sts.Status_Radar = status.Status_Radar;

    status_pub.publish(sts);   
}

void ProcessRadarData(const nlohmann::json& packet, int len)
{
    // 由于objecetlist中有重复名称的键:ars548.object，不能直接从键获取ars548.object的值，因此采用从字符流转换的方法
    // detectoinlist同理

    switch(len)
    {
        case 9401:
            // 如果数据长度是9401，从ip.fragments字段获取字符流，处理雷达数据
            try{
                std::string data_string =  packet["_source"]["layers"]["ip"]["ip.fragments"].get<std::string>();

                // std::cout << "data_string: " << data_string << std::endl;
                char *data_original =  cvt.hexStrToCharArray( data_string); 
                char *data_offset = data_original + 8 ;        // 跳过UDP头部


                // 获得时间戳 
                // frame.time_epoch为准确的时间戳
                // ARS548.Payload中的时间戳 Timestamp Nanoseconds和 Timestamp Seconds 是相对时间戳，需要加上frame.time_epoch
                std::string timestampStr = packet["_source"]["layers"]["frame"]["frame.time_epoch"];
                
                if(data_process.processObjectListMessage(data_offset, &object_list))
                {
                    if(object_list.ObjectList_NumOfObjects>0){
                        publishObjectList(object_list, timestampStr);
                        // std::cout << "***************publishObjectList*************" << std::endl; 
                    }
                            
                }  
            
                delete[] data_original;
             
            } catch(const  std::exception& e){
                std::cerr <<"Error occurred while getting the data of ObjecetList"  << e.what() << std::endl;
            }      
            
        break;

        case 35336:
            // 如果数据长度是35336，从ip.fragments字段获取字符流，处理雷达数据
            try{
                std::string data_string =  packet["_source"]["layers"]["ip"]["ip.fragments"].get<std::string>();

                //std::cout << "data_string: " << data_string << std::endl;
                char *data_original =  cvt.hexStrToCharArray( data_string); 
                char *data_offset = data_original + 8 ;        // 跳过UDP头部

                // 获得时间戳 
                // frame.time_epoch为准确的时间戳
                // ARS548.Payload中的时间戳 Timestamp Nanoseconds和 Timestamp Seconds 是相对时间戳，需要加上frame.time_epoch
                std::string timestampStr = packet["_source"]["layers"]["frame"]["frame.time_epoch"];

                if(data_process.processDetectionListMessage(data_offset, &detection_list))
                {
                    if(detection_list.List_NumOfDetections>0){
                        publishDetectionList(detection_list, timestampStr);
                        // std::cout << "***************publishDetectionList*************" << std::endl;
                    }
                }  
            
                delete[] data_original;
             
            } catch(const  std::exception& e){
                std::cerr << "Error occurred while getting the data of DetectionList" << e.what() << std::endl;
            }      
            
        break;

        // case 84:
        // json文件中没有字节流，也不存在相同名称的键，直接根据键值对获取对应的值
        //     try{
                
        //         if(data_process.processBasicStatusMessage(packet, &basic_status))
        //         {
        //             std::cout <<  "BasicStatus" << std::endl ;
        //         }    
                
        //     } catch(const  std::exception& e){ 
        //         std::cerr << "Error occurred while getting ARS data" << e.what() << std::endl;
        //     }           
            
        // break;

        default:
        break;
    }
}


void getDataFromJson(const std::string& filename) {

    /*********************处理json文件********************/
    // 打开并读取JSON文件
    std::ifstream jsonFile(filename);
    if (!jsonFile.is_open()) {
        std::cerr << "Failed to open JSON file: " << filename << std::endl;
    }

    // 将文件内容解析到nlohmann::json对象中
    nlohmann::json packets;
    try {
        jsonFile >> packets;
    } catch (const std::exception& e) {
        std::cerr << "Error occurred while parsing or accessing JSON data: " << e.what() << std::endl;
    }

     /*****遍历每一个数据包*****/
    for( int i = 0; i < packets.size(); i++){
        
        std::cout << "count: " << i+1 << std::endl;
        //判断是否存在udp_length字段，
        if(packets[i].contains("_source") && packets[i]["_source"].contains("layers") && packets[i]["_source"]["layers"].contains("udp")){

            int data_length = std::stoi( packets[i]["_source"]["layers"]["udp"]["udp.length"].get<std::string>() ) - 8 ;        //数据长度 = UDP长度 - UDP头部长度
            // std::cout << "data_length: " << data_length << std::endl;

            ProcessRadarData(packets[i], data_length);
        }

        // ARS数据包一定是UDP协议中的数据，若不存在udp_length 字段，跳过当前包
        else
            continue;

    }

}

//thread for receive radar data
void receiveThread()
{
    while(ros::ok())
    {
        getDataFromJson(json_file_path);
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ars548_process_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    objects_pub = nh.advertise<ars548_process::ObjectList>("/ars548_process/object_list", 10);
    detections_pub = nh.advertise<ars548_process::DetectionList>("/ars548_process/detection_list", 10);
    status_pub = nh.advertise<ars548_process::RadarBasicStatus>("/ars548_process/radar_status", 10);

    std::thread rec_thread = std::thread(receiveThread);  

    ros::spin();


    return 0;
}


