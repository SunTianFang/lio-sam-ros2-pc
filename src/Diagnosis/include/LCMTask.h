#pragma once

#include "Tools.h"
#include "lcm/lcm.h"
#include "ThreadHelper.h"
#include "RawMap.h"
//for test
#include "LinuxSetting.h"
#include "Scan.h"

// for lcm to pad

#include "../../NavBase/LaserSensor/include/laser_t.h"
#define LCM_DEFAULT_URL "udpm://239.255.76.67:7667?ttl=2"



bool DeleteArea ( double topLeft_x, double topLeft_y, double downRight_x, double downRight_y );
bool DeleteMultipleAreas (int8_t *iparams, double *dparams);
bool DeleteFasthMapArea(double topLeft_x, double topLeft_y, double downRight_x, double downRight_y);

class LCMTask : public CThreadHelper
{
private:

    unsigned long long featureLastTime_;
    unsigned long long cloudLastTime_;
public:
    ~LCMTask(){}
    LCMTask(const LCMTask&)=delete;
    LCMTask& operator=(const LCMTask&)=delete;
    static LCMTask& GetLcmInstance(){
        static LCMTask instance;
        return instance;
    }
    int LCMInit();
    int LCMRecInit();
    int LCMUnInit( void );

    int LCMInitSubscrible(void *user);

    void LCMSendLocalizationMsg();
    bool GetLocalizationMsg(sensor::CRawScan& pScan);
    bool GetLocalizationMsg();
    lcm_t* GetLcmHandle(){
        return lcm;
    }

    bool SendCalibrationResult(int res,double *lasertf);
    bool WriteLaserParam(void);
    void SendWriteLaserParamResut( bool res );

    CScan * GetCopyScan();

    // lcm 向 pad 发送新的关键帧
    void SendScan ( int id, float* mapinfo, int ptsize, float *ptinfo );

    // lcm 向 pad 发送应答
    void SendNaviCommand ( int commandId );
    // lcm 向 pad 发送应答 扩展版
    void SendNaviCommandEx ( int commandId, int parm );
    void SendVersion(int commandId);
    // 导航给 pad 发送心跳, 1s发送一次
    void SendHeart();
protected:
    virtual void SupportRoutineProxy();

public:
    LCMTask(){
        lcm = lcm_create("udpm://224.0.0.1:7667?ttl=1");
        printf("create ----lcm addr : %p-----\n",lcm);
        nState_LocalMsg = 0;
        LocalMsgTick = 0;
        bLocalMsgReady = false;
        Scan_ = NULL;
        iLocType = -1;
        featureLastTime_ = GetTickCount();
        cloudLastTime_ = GetTickCount();
    }

    int iCalibRes;
    double dLaserPos[12];
    bool  m_bPadReceived;
    lcm_t *lcm;
    int nState_LocalMsg;
    unsigned long long LocalMsgTick;
    bool bLocalMsgReady;
    SIASUN_PTHREAD_T     m_LcmThread;
    SIASUN_PTHREAD_T     m_HeartThread;
   
    CScan *Scan_;
    std::mutex Scan_mutex;
    int iLocType;
    vector<int> vecIntensity;
    int16_t* dis;
    int16_t* lcmintensities;

    static bool         m_bNaviToPad;
};

