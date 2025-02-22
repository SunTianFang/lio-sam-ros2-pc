//
//   The interface of class "cPfLDSLaserScanner".
//
//   Author: yuminghao
//   Date:   2024. 01. 24
//

#include"Tools.h"
#include "BlueSeaLaser50c.h"


//#include"Project.h"


vector<float> data50c;
vector<float> intensity50c;
std::mutex mtx50c;



// 传入回调指针的方式打印
void CallBackMsg50c(int msgtype, void *param,int length)
{
    std::lock_guard<std::mutex> lock(mtx50c);
    switch (msgtype)
    {
    // 实时雷达点云数据
    case 1:
    {
        UserData *pointdata = (UserData *)param;

        data50c.clear();
        intensity50c.clear();
        if (pointdata->type == FRAMEDATA)
        {
//            if(last_indx == pointdata->idx)
//            {
//                return;
//            }
//            last_indx = pointdata->idx;
//            printf("frame idx:%d  %s\t%d \t num:%d timestamp:%d.%d\n", pointdata->idx, pointdata->connectArg1, pointdata->connectArg2, pointdata->data.framedata.N, pointdata->data.framedata.ts[0], pointdata->data.framedata.ts[1]);
//                std::cout << "%%%%%%%%%%%%%%&&&&&&&&&&&size: " << pointdata->framedata.data.size() << std::endl;
                for (int i = 0; i <pointdata->framedata.data.size(); i++)
                 {
    //                printf("%s\t%d \t%.5f\t%.3f\t%d\n", pointdata->connectArg1, pointdata->connectArg2,  pointdata->data.framedata.data[i].angle, pointdata->data.framedata.data[i].distance, pointdata->data.framedata.data[i].confidence);
                    data50c.emplace_back(pointdata->framedata.data[i].distance * 1000);



                 /*   if(pointdata->framedata.data[i].distance<0.5&& pointdata->framedata.data[i].distance>0.1)
                    {
                       printf("i=%d, indentsiy= %d    dis= %f\n", i, pointdata->framedata.data[i].confidence,pointdata->framedata.data[i].distance);
                    }
                */

                    if(pointdata->framedata.data[i].confidence > 254)//强度归一化
                        pointdata->framedata.data[i].confidence = 254;
                    intensity50c.emplace_back(pointdata->framedata.data[i].confidence);
                }
                //打印每两次之间的时间间隔
            static unsigned long long last_time = 0; // Declare the variable "last_time" before using it
            unsigned long long current_time = GetTickCount();
           
            if(last_time != 0)
            {
                #ifdef USE_BLACK_BOX
                    FILE_BlackBox(LaserBox, "^^^^^^^laser time interval=: ", current_time - last_time , 
                    ", current system time: ", current_time,", data50c.size: ", data50c.size());
                #endif
                cout << "laser time interval: "<< current_time - last_time  <<" time: " << current_time <<std::endl;
                if(current_time - last_time > 100)
                cout << "**************out********************"<< std::endl;
            }
            last_time = current_time;
        }
        else
        {
             printf("frame idx:%d  %s\t%d \t num:%d timestamp:%d.%d\n", pointdata->idx, pointdata->connectArg1, pointdata->connectArg2, pointdata->spandata.data.N, pointdata->spandata.data.ts[0], pointdata->spandata.data.ts[1]);
             for (int i = 0; i <pointdata->spandata.data.N; i++)
             {
//                printf("%s\t%d \t%.5f\t%.3f\t%d\n", pointdata->connectArg1, pointdata->connectArg2,  pointdata->data.spandata.data.points[i].angle, pointdata->data.spandata.data.points[i].distance, pointdata->data.spandata.data.points[i].confidence);
             }
        }
        break;
    }
    // 实时报警数据
    case 2:
    {
        LidarMsgHdr *zone = (LidarMsgHdr *)param;
        uint32_t event = zone->events;
        std::string text;
        if (zone->flags % 2 == 1)
        {
            // 硬件报警信息
            if (getbit(event, 0) == 1)
                text += "供电不足";
            if (getbit(event, 1) == 1)
                text += "电机堵转足";
            if (getbit(event, 2) == 1)
                text += "测距模块温度过高";
            if (getbit(event, 3) == 1)
                text += "网络错误";
            if (getbit(event, 4) == 1)
                text += "测距模块无输出";
            //printf("alarm MSG:%s\n", text.c_str());
        }
        if (zone->flags >= 0x100)
        {
            // 防区报警信息
            if (getbit(event, 12) == 1)
                text += "观察！！！";
            if (getbit(event, 13) == 1)
                text += "警戒！！！";
            if (getbit(event, 14) == 1)
                text += "报警！！！";
            if (getbit(event, 15) == 1)
                text += "遮挡！";
            if (getbit(event, 16) == 1)
                text += "无数据";
            if (getbit(event, 17) == 1)
                text += "无防区设置";
            if (getbit(event, 18) == 1)
                text += "系统内部错误";
            if (getbit(event, 19) == 1)
                text += "系统运行异常";
            if (getbit(event, 20) == 1)
                // 和上面的第四项重复，这里屏蔽
                // text+='网络错误\n'
                if (getbit(event, 21) == 1)
                    text += "设备更新中";
            if (getbit(event, 22) == 1)
                text += "零位输出";
            //printf("Active zone:%d\tMSG:%s\n", zone->zone_actived, text.c_str());
        }
        break;
    }
    // 获取网络款雷达的全局参数
    case 3:
    {
        EEpromV101 *eepromv101 = (EEpromV101 *)param;
        // 类型，编号，序列号
        printf("dev info: 设备编号:%d\t 序列号:%s\t 类型:%s\n", eepromv101->dev_id, eepromv101->dev_sn, eepromv101->dev_type);
        // ip地址 子网掩码 网关地址 默认目标IP  默认目标udp端口号  默认UDP对外服务端口号
        char tmp_IPv4[16] = {0};
        char tmp_mask[16] = {0};
        char tmp_gateway[16] = {0};
        char tmp_srv_ip[16] = {0};

        sprintf(tmp_IPv4, "%d.%d.%d.%d", eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2], eepromv101->IPv4[3]);
        sprintf(tmp_mask, "%d.%d.%d.%d", eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2], eepromv101->mask[3]);
        sprintf(tmp_gateway, "%d.%d.%d.%d", eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2], eepromv101->gateway[3]);
        sprintf(tmp_srv_ip, "%d.%d.%d.%d", eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2], eepromv101->srv_ip[3]);

        printf("dev info: ip地址:%s 子网掩码:%s 网关地址:%s 默认目标IP:%s  默认目标udp端口号:%d   默认UDP对外服务端口号:%d\n",
               tmp_IPv4, tmp_mask, tmp_gateway, tmp_srv_ip, eepromv101->srv_port, eepromv101->local_port);

        /*char tmp_ranger_bias[8] = {0};
        memcpy(tmp_ranger_bias, eepromv101->ranger_bias, sizeof(eepromv101->ranger_bias) - 1);*/
        // 转速 ,电机启动参数,FIR滤波阶数，圈数，分辨率，开机自动上传，固定上传，数据点平滑，去拖点，记录校正系数，网络心跳，记录IO口极性
        printf("dev info: 转速:%d 电机启动参数:%d FIR滤波阶数:%d 圈数:%d  分辨率:%d   开机自动上传:%d 固定上传:%d  数据点平滑:%d 去拖点:%d   网络心跳:%d  记录IO口极性:%d\n",
               eepromv101->RPM, eepromv101->RPM_pulse, eepromv101->fir_filter, eepromv101->cir, eepromv101->with_resample, eepromv101->auto_start,
               eepromv101->target_fixed, eepromv101->with_smooth, eepromv101->with_filter, eepromv101->net_watchdog, eepromv101->pnp_flags);

        printf("dev info:平滑系数：%d  激活防区：%d  上传数据类型：%d\n", eepromv101->deshadow, eepromv101->zone_acted, eepromv101->should_post);
        break;
    }

    // 获取雷达时间戳打印信息(网络款为雷达返回时间戳，串口款为本机接收到的时间戳)
    case 4:
    {
        DevTimestamp *devtimestamp = (DevTimestamp *)param;
        //printf("timestamp:lidar_ip:%s lidar_port:%d time:%d delay:%d\n", devtimestamp->ip, devtimestamp->port, devtimestamp->timestamp, devtimestamp->delay);
        break;
    }
    // 打印信息(也可以作为日志写入)
    case 8:
    {
        char result[512];
        memcpy(result, param, length);
        printf("info: %s\n", result);
        break;
    }
    case 9:
    {
        char result[512];
        memcpy(result, param, length);
        printf("error: %s\n", result);
        break;
    }
    }
    fflush(stdout);

}

void* LDS_50ScanProc(LPVOID pParam)
{
     cLDS_50LaserScanner* oLaserScanner = reinterpret_cast<cLDS_50LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}


cLDS_50LaserScanner::cLDS_50LaserScanner(int fAngRes, float fStartAng, float fEndAng,CLaserScannerParam *cParam):
    CRangeScanner(fAngRes, fStartAng, fEndAng, LDS_50)
{
    m_iNetType = -1;
    //m_LDSScanThread = 0;
    m_LaserScannerIp = "";

    m_iLaserPort = 6543;

    m_iScanFrequency = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
    m_nDiffValue = GetRealTimeTickCount() - GetTickCount();



}

cLDS_50LaserScanner::~cLDS_50LaserScanner()
{
    Stop();
}

bool cLDS_50LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseLDSLaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
    PthreadJoin(m_LDSScanThread);
    if (m_hKillThread != NULL)
    {
        CloseHandle(m_hKillThread);
        m_hKillThread = NULL;
    }

    if (m_hThreadDead != NULL)
    {
        CloseHandle(m_hThreadDead);
        m_hThreadDead = NULL;
    }

    m_bStarted = false;
    return true;
}




BOOL cLDS_50LaserScanner::ConnectToLDSLaser(const char*device_name,const int iPort)
{
    BlueSeaLidarSDK *lidarSDK =  BlueSeaLidarSDK::getInstance();
    int lidarID = lidarSDK->addUDPLidar(device_name,iPort);
//    const char *cfg_file_name = "/userdata/CarryBoy/NAV/blueseaConfig/bluesea_web.txt";
//        std::cout<<"cfg_file_name = "<<cfg_file_name<<std::endl;
    //根据配置文件路径添加相关的雷达
//    int lidarID = lidarSDK->addLidarByPath(cfg_file_name);
    if (!lidarID)
    {
        std::cout << "lidarID dont exist !!!" << std::endl;
        return false;
    }

    //传入数据信息的回调函数
    lidarSDK->setCallBackPtr(lidarID, CallBackMsg50c);

    //连接指定雷达，以及相关的线程池
    if (!lidarSDK->openDev(lidarID))
    {
        std::cout << "Open Lidar Failed !!!" << std::endl;
        return false;
    }


    //读取雷达的全局参数
    EEpromV101 eepromv101;
    if(lidarSDK->GetDevInfo(lidarID,&eepromv101))
        CallBackMsg50c(3,&eepromv101,sizeof(EEpromV101));


    Sleep(1000);
    return TRUE;
}

BOOL cLDS_50LaserScanner::CloseLDSLaserConnection()
{
    std::cout << "Trying to stop capture" << std::endl;

    return TRUE;
}

BOOL cLDS_50LaserScanner::Start(const char*device_name,
                                 const char*host_name,
                                 const int laserid,
                                 const int iPort,
                                 const int iNetType,
                                 const int iScanFrequency,
                                 const int iSamplesPerScan)
{
    if (m_bStarted){
        return true;
    }

    m_LaserScannerIp = device_name;

    m_iNetType = iNetType;
    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = iScanFrequency;
    m_nConnectTime = GetTickCount();
//    std::cout<<"iPort = "<<iPort<<std::endl;

    if(!ConnectToLDSLaser(device_name,m_iLaserPort))
    {
        std::cout<<"ConnectToLDSLaser Failed: "<<device_name<<std::endl;
        return FALSE;
    }
    else
    {
      std::cout<<"ConnectToLD50cLaserK"<<std::endl;
    }

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return FALSE;

    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);

    if(pthread_create(&m_LDSScanThread,&attr,LDS_50ScanProc,reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat LDSScanSupport Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat LDSScanSupport Pthread OK"<<std::endl;

    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}

//
//	获取Urg Scanner数据
//
void cLDS_50LaserScanner::SupportMeasure()
{
    std::lock_guard<std::mutex> lock(mtx50c);

    unsigned long long  raw_time = GetTickCount();
    if(data50c.size() <= 0 || intensity50c.size() <= 0)
        return ;

   // std::cout<<"By yu : Adding PointCloud !! data.size() = "<<data.size()<<std::endl;
   /* for(int i = 0; i< data.size(); i++)
    {
        if(i % 100 == 0)
         std::cout<<"  data["<<i<<"] = "<<data[i];
    }*/

    AddRawPointCloud(data50c, intensity50c, raw_time, raw_time);
    data50c.clear();
}
