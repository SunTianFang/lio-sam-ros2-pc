#include "ZTypes.h"
#include "LCMTask.h"

#include "json/json.h"
#include "AffinePosture.h"

#include "robot_control_t.h"
#include "robot_control_t_new.h"


#include "RoboLocProto.h"   // add for lcm
#include "Project.h"
//include "calibration.h"



#include "blackboxhelper.hpp"

#include "yaml.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif
bool LCMTask::m_bNaviToPad = false;  // static 变量需要类外定义, 分配内存


SIASUN_PTHREAD_PROC_DECL LCMRecvTaskMinitor( LPVOID arg )
{
    LCMTask& mlcm = LCMTask::GetLcmInstance();
    while( 1 )
    {
        lcm_handle( mlcm.GetLcmHandle() );
        Sleep(50);
    }
    mlcm.LCMUnInit();
    return NULL;
}

// 导航给 pad 发送心跳线程, 1s发送一次 add 2022.06.06
SIASUN_PTHREAD_PROC_DECL NaviSendHeartToPad( LPVOID arg )
{
    while( 1 )
    {
        LCMTask::GetLcmInstance().SendHeart ();
        Sleep(1000);
    }

    return NULL;
}

SIASUN_PTHREAD_PROC_DECL LCMSendTask( LPVOID arg );



void PadMsgHandle ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t *PadControlData, void *user );

void PadMsgHandle_new ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t_new *PadControlData, void *user );

int LCMTask::LCMInit( void )
{
   // printf("LCMTask::LCMInit\n");
    m_bPadReceived = false;
    nState_LocalMsg = 2;
    LocalMsgTick = GetTickCount();
    CreateThread(50);
    return 0;
}

int LCMTask::LCMRecInit()
{
    Scan_= NULL;
 
    SiaSun_AfxBeginThread(LCMRecvTaskMinitor,(LPVOID)this, &m_LcmThread);

}
int LCMTask::LCMInitSubscrible(void *user)
{
    //robot_control_t_subscribe ( lcm, "NAVI_SERVICE_COMMAND", &PadMsgHandle, user );
    robot_control_t_new_subscribe ( lcm, "NAVI_SERVICE_COMMAND", &PadMsgHandle_new, user );
    SiaSun_AfxBeginThread(LCMRecvTaskMinitor,(LPVOID)this, &m_LcmThread);

    // 导航给 pad 发送心跳、状态
    SiaSun_AfxBeginThread ( NaviSendHeartToPad, (LPVOID)this, &m_HeartThread );
}

//  by cgd   delete grid in map
bool DeleteArea ( double topLeft_x, double topLeft_y, double downRight_x, double downRight_y )
{
    const char* fileStr = "ProbMap.txt";
    // 打开地图
    ifstream infile;
    infile.open ( WORK_PATH"ProbMap.txt", ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }

    ofstream offile_p;
    ofstream offile_y;
    string str_p = SHAREFILES_PATH"PM.pgm";
    string str_y = SHAREFILES_PATH"PM.yaml";
    vector<int> temp;

    offile_p.open(str_p,ios::out);
    offile_y.open(str_y,ios::out);
    if(!offile_p ||!offile_y)
        return false;

    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }

    int *mapData = new int [ nheight * nwidth ];    // 地图数据
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标

    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();


    // 删除区域
    int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
    int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
    int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
    int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
    printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

    col_min = max(0,col_min);
    col_max = min(col_max,nwidth);
    row_min = max(0,row_min);
    row_max = min(row_max,nheight);

   // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
  //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


    for ( int y = row_min; y < row_max; y++ )
        for ( int x = col_min; x < col_max; x++ )
            mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色

    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    offile_p<<"P2\n";
    offile_p<<nwidth<<" "<<nheight<<std::endl;
    offile_p<<"2\n";

    int j=0;
    for(int h = 0;h<nheight;h++)
    {
        for (int w = 0;w<nwidth; w++)
        {
            int index = nheight*nwidth-(h*nwidth+nwidth-1-w);
            int data = 0;
            if(mapData[index]>127)
              data = 2;
			else if (mapData[index] == 127)
			  data = 1;
            else
              data = 0;

              j++;
              offile_p<<data<<" ";
              if(j%nheight ==0)
                  offile_p<<std::endl;
        }
    }

    offile_y<<"image: PM.pgm\n";
    offile_y<<"resolution: "<<"0.05"<<std::endl;
    offile_y<<"origin: ["<<origin_position_x<<", "<<origin_position_y<<", 0.0]\n";
    offile_y<<"occupied_thresh: 0.5\n";
    offile_y<<"free_thresh: 0.5\n";
    offile_y<<"negate: 0\n";
    std::cout<<"YAML FILE SAVED, PLEASE EDIT PATH TO MAP!"<<std::endl;


    offile_y.close();
    offile_p.close();

    outFile.close();

    delete [] mapData;

    return true;
}


//dq 同时删除多个区域

bool DeleteProbMapMultipleAreas(std::string filename, int8_t *iparams, double *dparams)
{


    // 打开地图
    ifstream infile;
    infile.open ( filename, ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }

    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;
    double topLeft_x = 0;
    double downRight_x = 0;
    double downRight_y = 0;
    double topLeft_y = 0;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }
    std::cout<<"nheight: "<<nheight<<", nwidth: "<<nwidth<<std::endl;
    //int *mapData = new int [ nheight * nwidth ];    // 地图数据
    vector<int> mapData;
    mapData.resize(nheight * nwidth+100);
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标
    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();
    for(int i = 0; i < iparams[0]; i++)
    {
        topLeft_x = dparams[4*i];
        topLeft_y = dparams[4*i+1];
        downRight_x = dparams[4*i+2];
        downRight_y = dparams[4*i+3];
        int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
        int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
        int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
        int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
        printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

        col_min = max(0,col_min);
        col_max = min(col_max,nwidth);
        row_min = max(0,row_min);
        row_max = min(row_max,nheight);

        // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
        //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


        for ( int y = row_min; y < row_max; y++ )
            for ( int x = col_min; x < col_max; x++ )
            {
                int clear_index = y * nwidth + x;
                if(clear_index > 0 && clear_index < nheight * nwidth)
                mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色
            }
    }

    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        mapData.clear();
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    outFile.close();
    mapData.clear();



}


bool DeleteMultipleAreas_new ( int8_t *iparams, double *dparams )
{

#define PGM_VALUE_WHITE 2
#define PGM_VALUE_BLACK 0
#define PGM_VALUE_UNKOWN 1

    const char* fileStr = "ProbMap.txt";
    // 打开地图
    ifstream infile;
    infile.open ( WORK_PATH"ProbMap.txt", ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }



    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;
    double topLeft_x = 0;
    double downRight_x = 0;
    double downRight_y = 0;
    double topLeft_y = 0;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }
    std::cout<<"nheight: "<<nheight<<", nwidth: "<<nwidth<<std::endl;
    //int *mapData = new int [ nheight * nwidth ];    // 地图数据
    vector<int> mapData;
    mapData.resize(nheight * nwidth+100);
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标
    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();






    string CostNameTemp = SHAREFILES_PATH"PM.yaml";

    if (access(CostNameTemp.c_str(), F_OK) == 0)
    {
//        printf("1.txt exists.\n");
    }
    else
    {
        printf("PM.yaml no exists.\n");
        std::cout<<CostNameTemp<<" no exists"<<std::endl;
        return false;
    }

    YAML::Node Node = YAML::LoadFile(CostNameTemp);
    std::string PgmName = Node["image"].as<string>();
    std::vector<double> points =  Node["origin"].as<std::vector<double>>();
    float resoution = Node["resolution"].as<float>();
    float occupied_thresh = Node["occupied_thresh"].as<float>();
    float free_thresh = Node["free_thresh"].as<float>();
    int   negate = Node["negate"].as<float>();

    // image 文件读入数据
    std::ifstream infilePgm;


    string PgmNameTemp =SHAREFILES_PATH"PM.pgm";
    std::string strTail = PgmName.substr(PgmName.find('.')+1,PgmName.size());

     int  num_of_rows = 0, num_of_cols = 0;
     double x0 = 0;
     double y0 = 0;

     if(strTail != "pgm")
         return false;



          infilePgm.open(PgmNameTemp,ios::in);
          //判断是否打开
          if (!infilePgm.is_open())
          {
            std::cout << "open file error" << std::endl;
            return false;
          }
          std::string inputLine;
          std::getline(infilePgm, inputLine);
          if(inputLine != "P2")
          {
            cout << "pgm Version not p2, is : " << inputLine << endl;
            return false;
          }
          else
          {
            //cout << "Version : " << inputLine << endl;
          }

          stringstream ss;

          ss << infilePgm.rdbuf();   //read the third line : width and height
          ss >> num_of_cols >> num_of_rows;
          //cout << num_of_cols << " columns and " << num_of_rows << " rows" << endl;
          int iGridValue = -1;//灰度的可能最大值 maximum intensity value
          ss >> iGridValue;


          x0 = points.at(0);
          y0 = points.at(1);

          std::vector<int>  onerow;
          std::vector<std::vector<int>>  allrow;
          onerow.clear();
          allrow.clear();
          for (int row = 0; row <  num_of_rows; row++)
          {
             onerow.clear();
             for(int col = 0; col < num_of_cols ; col++)
             {
                 int iValue;
                 ss >> iValue;
                 onerow.push_back(iValue);

             }
             allrow.push_back(onerow);
          }
          infilePgm.close();



    // 删除区域
    std::cout<<"<<<<<<<<<<<<<<<<删除区域>>>>>>>>>>>>>>>>"<<std::endl;
    for(int i = 0; i < iparams[0]; i++)
    {
        topLeft_x = dparams[4*i];
        topLeft_y = dparams[4*i+1];
        downRight_x = dparams[4*i+2];
        downRight_y = dparams[4*i+3];
        int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
        int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
        int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
        int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
        printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

        col_min = max(0,col_min);
        col_max = min(col_max,nwidth);
        row_min = max(0,row_min);
        row_max = min(row_max,nheight);

        // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
        //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


        for ( int y = row_min; y < row_max; y++ )
            for ( int x = col_min; x < col_max; x++ )
            {
                int clear_index = y * nwidth + x;
                if(clear_index > 0 && clear_index < nheight * nwidth)
                mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色
            }


        col_min = (topLeft_x   - x0) / resolution;   // 左上角 x 下标
        col_max = (downRight_x - x0) / resolution;   // 右下角 x 下标
        row_min = (downRight_y - y0) / resolution;   // 右下角 y 下标
        row_max = (topLeft_y   - y0) / resolution;   // 左上角 y 下标
        printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

        col_min = max(0,col_min);
        col_max = min(col_max,num_of_cols);
        row_min = max(0,row_min);
        row_max = min(row_max,num_of_rows);


            for (int row = num_of_rows - row_max ; row <  num_of_rows - row_min; row++)
            {
                for(int col = col_min; col < col_max; col++)
                {
                    if(allrow.at(row).at(col) == PGM_VALUE_BLACK ||
                            allrow.at(row).at(col) == PGM_VALUE_UNKOWN)
                        allrow.at(row).at(col) = PGM_VALUE_WHITE;
                }
            }



    }
    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        mapData.clear();
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    outFile.close();
    mapData.clear();


    //string strBottomfile = WORK_PATH"bottomProbMap.txt";
    //DeleteProbMapMultipleAreas(strBottomfile, iparams, dparams);

    ofstream offile_p;
    string str_p = SHAREFILES_PATH"PM.pgm";

    offile_p.open(str_p,ios::out);

    if(!offile_p )
    {
        return false;
    }
    else
    {
        offile_p<<"P2\n";
        offile_p<<num_of_cols<<" "<<num_of_rows<<std::endl;
        offile_p<<"2\n";


        for (int row = 0; row <  num_of_rows; row++)
        {

           for(int col = 0; col < num_of_cols ; col++)
           {
               offile_p<<(int)allrow.at(row).at(col)<<" ";

               //offile_p<<PGM_VALUE_BLACK<<" ";


           }
           offile_p<<std::endl;
        }

     }
     allrow.clear();

     offile_p.close();




    return true;
}




//dq 同时删除多个区域
bool DeleteMultipleAreas ( int8_t *iparams, double *dparams )
{
    const char* fileStr = "ProbMap.txt";
    // 打开地图
    ifstream infile;
    infile.open ( WORK_PATH"ProbMap.txt", ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }
    ofstream offile_p;
    ofstream offile_y;
    string str_p = SHAREFILES_PATH"PM.pgm";
    string str_y = SHAREFILES_PATH"PM.yaml";
    vector<int> temp;

    offile_p.open(str_p,ios::out);
    offile_y.open(str_y,ios::out);
    if(!offile_p ||!offile_y)
        return false;

    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;
    double topLeft_x = 0;
    double downRight_x = 0;
    double downRight_y = 0;
    double topLeft_y = 0;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }
    std::cout<<"nheight: "<<nheight<<", nwidth: "<<nwidth<<std::endl;
    //int *mapData = new int [ nheight * nwidth ];    // 地图数据
    vector<int> mapData;
    mapData.resize(nheight * nwidth+100);
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标
    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();


    // 删除区域
    std::cout<<"<<<<<<<<<<<<<<<<删除区域>>>>>>>>>>>>>>>>"<<std::endl;
    for(int i = 0; i < iparams[0]; i++)
    {
        topLeft_x = dparams[4*i];
        topLeft_y = dparams[4*i+1];
        downRight_x = dparams[4*i+2];
        downRight_y = dparams[4*i+3];
        int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
        int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
        int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
        int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
        printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

        col_min = max(0,col_min);
        col_max = min(col_max,nwidth);
        row_min = max(0,row_min);
        row_max = min(row_max,nheight);

        // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
        //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


        for ( int y = row_min; y < row_max; y++ )
            for ( int x = col_min; x < col_max; x++ )
            {
                int clear_index = y * nwidth + x;
                if(clear_index > 0 && clear_index < nheight * nwidth)
                mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色
            }
    }
    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        mapData.clear();
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    offile_p<<"P2\n";
    offile_p<<nwidth<<" "<<nheight<<std::endl;
    offile_p<<"2\n";

    int j=0;
    for(int h = 0;h<nheight;h++)
    {
        for (int w = 0;w<nwidth; w++)
        {
            int index = nheight*nwidth-(h*nwidth+nwidth-1-w);
            int data = 0;
            if(mapData[index]>127)
              data = 2;
			else if (mapData[index] == 127)
			  data = 1;
            else
              data = 0;


              j++;
              offile_p<<data<<" ";
              if(j%nheight ==0)
                  offile_p<<std::endl;
        }
    }

    offile_y<<"image: PM.pgm\n";
    offile_y<<"resolution: "<<"0.05"<<std::endl;
    offile_y<<"origin: ["<<origin_position_x<<", "<<origin_position_y<<", 0.0]\n";
    offile_y<<"occupied_thresh: 0.5\n";
    offile_y<<"free_thresh: 0.5\n";
    offile_y<<"negate: 0\n";
    std::cout<<"YAML FILE SAVED, PLEASE EDIT PATH TO MAP!"<<std::endl;


    offile_y.close();
    offile_p.close();


    outFile.close();

    //delete [] mapData;
    mapData.clear();

    return true;
}

bool ReserveArea(int index, int wide_limits_num_x_cells, int topLeft_x, int topLeft_y, int downRight_x, int downRight_y)
{

    int y = index%wide_limits_num_x_cells;
    int x = index/wide_limits_num_x_cells;


    int s1 = (x-topLeft_x)*(x-downRight_x);
    int s2 = (y-topLeft_y)*(y-downRight_y);

    if(s1<=0 && s2<=0)
    {
        return false;

    }
    return true;
}


   
        

// by DQ PadMsgHandle_new
void PadMsgHandle_new ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t_new *PadControlData, void *user )
{

    switch ( PadControlData->commandid )
    {

        case 5: // pad开始建图
        {
	
        

        }
            break;
        case 6: // pad结束建图

     
            break;
        case 9: // pad删除区域
           
            break;
        case 10: // pad删除多区域
           
            break;

        case 20:   // pad 向 导航请求心跳, pad 请求一次, 导航应答一次
            LCMTask::m_bNaviToPad    = true;
            break;
        case 21:   // 录制dx

         

            break;


        case 22:    // 导航结束建图状态中, 结束优化后, pad 界面是否保存地图
        
              
            break;
        case 23:    // pad 请求 导航将 Probmap.txt 地图恢复
            break;

    /*
        case 25:    //by DQ 从pad获取区域
          
            break;

        case 26:    //by DQ 向pad发送模板
            
            break;
        case 27:    //by DQ 向pad发送区域
          
            break;
        case 28:    //by DQ 向pad发送地图、模板、区域
           
             
            break;
            */
        case 28:
            LCMTask::GetLcmInstance().SendVersion(60);
            break;

        case 29:
        {
            LCMTask::GetLcmInstance().m_bPadReceived = true;
            std::cout<<"receive m_bPadReceived\n ";
            break;
        }

        case 30:
            {
                std::cout<<"$$$$$$$$$$$$$$$ WriteLaserParam $$$$$$$$$$$$$$$$\n";
                bool res = LCMTask::GetLcmInstance().WriteLaserParam();
                LCMTask::GetLcmInstance().SendWriteLaserParamResut(res);
            }

            break;
        case 31:
            {
               std::cout<<"Mode_ExpandMap"<<std::endl;
            
            }
          break;

         case 32:   //expandmap
            {
               
                  std::cout<<"expandmap"<<std::endl;
             
            }
            break;
            case 33:   //send submap
               {
           
               }
            break;
            case 34:
            {
                            } 
            break;

            case 35:   // camera
            {
                           }
            break;

            //pad发送地图中定位区域和参数信息给nav 
            case 36:
            {
               

            }
            break;

            //case37 pad请求获取地图中定位区域框和参数数据
            case 37:
            {
               
            }
            break;
    }
}



int LCMTask::LCMUnInit( void )
{
    printf( "[%s]lcm: %p\n", __func__, lcm );

    lcm_destroy(lcm);

    return 0;
}



/*
 * lcm 向 pad 发送新的关键帧
 * param[in] id         帧ID
 * param[in] mapinfo    地图信息
 * param[in] ptsize     激光点数
 * param[in] ptinfo     激光数据
 */
void LCMTask::SendScan ( int id, float* mapinfo, int ptsize, float *ptinfo )
{
    laser_t cmd;
    long systemtime     = (long)time( (time_t *)0 );

    cmd.utime           = systemtime;
    cmd.rad0            = id;
    cmd.nranges         = ptsize;
    cmd.ranges          = ptinfo;
    cmd.nintensities    = 6;
    cmd.intensities     = mapinfo;

    std::cout << "SendScan "<<ptinfo[0]<<" "<<ptinfo[1]<<" "<<ptinfo[2]<<std::endl;


   // printf ( "UPDATE_SCAN\n" );
    laser_t_publish ( lcm, "UPDATE_POS", &cmd );
}

/*
 * lcm 向 pad 发送应答
 * param[in] commandId     应答ID
 */
void LCMTask::SendNaviCommand ( int commandId )
{
    //dq 11.15
    robot_control_t_new     NaviCommand;    // 导航向pad发送的应答

    NaviCommand.commandid = commandId;
    NaviCommand.ndparams = 0;
    NaviCommand.niparams = 0;
    NaviCommand.nsparams = 0;
    NaviCommand.nbparams = 0;

    //robot_control_t_publish( lcm, "NAVI_UI_COMMAND", &NaviCommand);
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &NaviCommand);
    //printf ( "Navi to Pad Command = %d\n", commandId );
}

/*
 * lcm 向 pad 发送应答 扩展版
 * param[in] commandId     应答ID
 * param[in] commandId     应答参数
 */
void LCMTask::SendNaviCommandEx ( int commandId, int parm )
{
    //dq 11.15
    robot_control_t_new     NaviCommandEx;    // 导航向pad发送的应答
    int8_t*             i_params = (int8_t*)calloc ( 1, sizeof (int8_t) );

    NaviCommandEx.commandid = commandId;
    NaviCommandEx.ndparams = 0;
    NaviCommandEx.niparams = 1;
    NaviCommandEx.nsparams = 0;
    NaviCommandEx.nbparams = 0;

    i_params[0] = parm;
    NaviCommandEx.iparams = i_params;

    //robot_control_t_publish( lcm, "NAVI_UI_COMMAND", &NaviCommandEx);
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &NaviCommandEx);
    //printf ( "Navi to Pad Command = %d\n", commandId );

    free (i_params);
}
// by dq send version
void LCMTask::SendVersion( int commandId )
{
    robot_control_t_new  Version;

    string version = VER_IN_BLACKBOX;

    Version.commandid = commandId;
    Version.ndparams = 0;
    Version.niparams = 0;
    Version.nsparams = 0;
    Version.nbparams = version.size();
    Version.bparams = (int8_t *)version.c_str();
    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &Version);
}





/*
 * by cgd 导航给 pad 发送心跳, 1s发送一次
 */
void LCMTask::SendHeart ()
{

#ifdef NAV_APP
    if ( LCMTask::m_bNaviToPad == true ) {
        // 导航向pad发送心跳, pad 请求一次, 导航应答一次
        LCMTask::m_bNaviToPad = false;
    }
    else {
        return;
    }

    //robot_control_t     HeartCommand;    // 导航向pad发送的心跳
    robot_control_t_new     HeartCommand;    // 导航向pad发送的心跳
    int8_t*             i_params = (int8_t*)calloc ( 10, sizeof (int8_t) );

    HeartCommand.commandid = 51;
    HeartCommand.ndparams = 0;
    HeartCommand.niparams = 10;
    HeartCommand.nsparams = 0;
    HeartCommand.nbparams = 0;

    // int workMode = RoboManagerSingleton::GetInstance()->GetCurWorkMode();
    // switch ( workMode )
    // {
    //     case RLP_MODE_LOCALIZATION:     // RLP_MODE_LOCALIZATION = 3
    //     {
    //         i_params[0] = 1;
    //         i_params[1] = 3;

    //         // auto pLocalize = LocalizeFactorySingleton::GetInstance();
    //         if(pLocalize)
    //         {
    //             if(pLocalize->IsRecordingImage())
    //                 i_params[1] = 10;
    //         }
    //      }

    //         break;


    //     case RLP_MODE_AUTOMAPPING:      // RLP_MODE_AUTOMAPPING = 5
    //      {

    //          i_params[0] = 2;
    //          i_params[1] = 3;  //zheng

    //          auto pMapping = LaserAutoMappingSingleton::GetInstance();

    //          if(pMapping->GetMappingMode()==mapping::Mode_BuildMap)
    //              i_params[2] = 0;

    //          if(pMapping->GetMappingMode()==mapping::Mode_ExpandMap)
    //              i_params[2] = 1;

    //      }

    //         break;

    //     case RLP_MODE_STOPMAPPING:      // RLP_MODE_STOPMAPPING = 6
    //         {
    //            i_params[0] = 3;
    //            int status = RoboManagerSingleton::GetInstance()->GetAutoMappingState();
    //            //结束建图状态, 1：结束建图, 正在后端优化中; 2：结束建图, 优化完成, 询问pad是否保存地图;  3 saveing  默认 / 复位

    //            i_params[1] = status-1;

    //         }

    //            break;

    //     case RLP_MODE_MAPPING:          // RLP_MODE_MAPPING = 2
    //         i_params[0] = 4;
    //         i_params[1] = 3;
    //         break;

    //     case RLP_MODE_CALIBRATION:
    //         i_params[0] = 4;
    //         i_params[1] = 3;
    //         break;

    // }


    HeartCommand.iparams = i_params;

    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &HeartCommand );
    printf ( "Navi to Pad Command = 51, iparam[0] =  %d, iparam[1]  = %d\n", i_params[0], i_params[1] );

    free (i_params);
#endif
}

bool LCMTask::WriteLaserParam(void)
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

   /* if(iCalibRes== CLBT_SUCCESS_ONE_LASER || iCalibRes==CLBT_SUCCESS_TWO_LASER )
    {
    }
    else
        return false;*/

    if(!FileLaserParm )
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        int SensorCount = 0;
        if (!LaserParmRoot["laser"].isNull())
        {
            SensorCount = LaserParmRoot["laser"].size();
        }
        else
        {
             FileLaserParm.close();
             return false;
        }

        for(int i = 0; i < SensorCount; i++)
        {

           /* if (!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["StartAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["StartAngle"] = str;

            }

            if (!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["EndAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["EndAngle"] = str;
            }*/

            // if(i==0 ||(iCalibRes == CLBT_SUCCESS_TWO_LASER&&i==1))
            // {
            //     int kk = 0;
            //     if(i==0)
            //         kk = 0;
            //     if(i==1)
            //         kk = 6;

            //     if (!LaserParmRoot["laser"][i]["x"].isNull()) {
            //         LaserParmRoot["laser"][i]["x"] = dLaserPos[kk+0];
            //     }


            //     if (!LaserParmRoot["laser"][i]["y"].isNull()) {
            //         LaserParmRoot["laser"][i]["y"] = dLaserPos[kk+1];
            //     }

            //     if (!LaserParmRoot["laser"][i]["thita"].isNull()) {
            //         LaserParmRoot["laser"][i]["thita"] = dLaserPos[kk+2];
            //     }

            // }

           /* if (!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MaxRange"] = str;
            }

            if (!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MinRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MinRange"] = str;
            }

            int nRangeCount = 0;
            if (!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                nRangeCount = LaserParmRoot["laser"][i]["VisualRange"].size();
            }

            for(int j = 0; j < nRangeCount; j++)
            {
                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"] = str;
                }

                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"] = str;
                }
            }*/
        }
    }

    FileLaserParm.close();


   // if(!bRet)
    {
        std::ofstream wFileLaserParm(WORK_PATH"LaserParm.json",std::ios::out );
        Json::StyledWriter WJ;
        if(!wFileLaserParm.is_open())
        {
             return false;
        }
        else
        {
             wFileLaserParm<<WJ.write(LaserParmRoot);
        }
        wFileLaserParm.close();
    }


    return bRet;
}

void LCMTask::SendWriteLaserParamResut( bool res )
{

    std::cout<<"SendWriteLaserParamResut  "<<res<<std::endl;
    robot_control_t_new  cmd;


    int8_t iparams[1];
    if(res)
        iparams[0] = 0;
    else
        iparams[0] = 1;

    cmd.commandid = 62;
    cmd.ndparams = 0;
    cmd.niparams = 1;
    cmd.iparams = iparams;
    cmd.nsparams = 0;
    cmd.nbparams = 0;

    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);
}




//
// by lishen
//
bool LCMTask::SendCalibrationResult(int res,double *lasertf)
{
/*
#ifdef NAV_APP

    if(res == CLBT_SUCCESS_ONE_LASER || res == CLBT_SUCCESS_TWO_LASER )
    {
         std::cout << '\n' << "------Calibration Results-------" << '\n' << "LiDAR-odom x: " << lasertf[0] << '\n'
                    << "LiDAR-odom y: " << lasertf[1] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[2]
                    << std::endl;
         std::cout << '\n' << "-------init laser pos-------" << '\n' << "LiDAR-odom x: " << lasertf[3] << '\n'
                    << "LiDAR-odom y: " << lasertf[4] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[5]
                    << std::endl;

         if(res == CLBT_SUCCESS_TWO_LASER)
         {
            std::cout << '\n' << "-------Calibration Results-------" << '\n' << "LiDAR-odom x: " << lasertf[6] << '\n'
                    << "LiDAR-odom y: " << lasertf[7] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[8]
                    << std::endl;
            std::cout << '\n' << "-------init laser pos-------" << '\n' << "LiDAR-odom x: " << lasertf[9] << '\n'
                    << "LiDAR-odom y: " << lasertf[10] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[11]
                    << std::endl;

         }

    }

    iCalibRes = res;
    for(int i=0;i<12;i++)
         dLaserPos[i] = lasertf[i];

    m_bPadReceived = false;
    robot_control_t_new  cmd;
    int8_t iparams[1];
    iparams[0] = res;

    cmd.commandid = 61;
    cmd.ndparams = 12;
    cmd.dparams = dLaserPos;
    cmd.niparams = 1;
    cmd.iparams = iparams;
    cmd.nsparams = 0;
    cmd.nbparams = 0;

    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);

    usleep(200000);

    std::cout<<"CalibRes " <<iCalibRes<< std::endl;

    int count = 0;

    while((count<5) && (!m_bPadReceived))
    {
        robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);
        usleep(200000);
        count++;
    }


#endif
*/
}




#pragma GCC pop_options
