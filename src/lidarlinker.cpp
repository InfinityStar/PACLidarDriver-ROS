#include "lidarlinker.h"

using namespace std;
using namespace PacLidar;

LidarLinker::LidarLinker(string ip, uint16_t port, string name):
    isCap(false),
    dataReceiver(0)
    ,_dtPropr(1)
{
    lidarName = name + "(" + ip + ":" + to_string(port) + ")";
    lidarSockAddr = new sockaddr_in();
    if ((lidarSockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        DBG_INFO << T_RED << "[" << lidarName << "]Error : Fata:Create socket failed:" << strerror(errno) << T_RESET << endl;
        exit(-1);
    }

    rcvtimeout.tv_sec = 3;
    rcvtimeout.tv_usec = 0;
    setsockopt(lidarSockFD, SOL_SOCKET, SO_RCVTIMEO, &rcvtimeout, sizeof(rcvtimeout));

    int quickAck = 1;
    setsockopt(lidarSockFD, IPPROTO_TCP, TCP_QUICKACK, &quickAck, sizeof(quickAck));

    bzero(lidarSockAddr, sizeof(struct sockaddr_in));
    lidarSockAddr->sin_family = AF_INET;
    lidarSockAddr->sin_addr.s_addr = inet_addr(ip.c_str());
    lidarSockAddr->sin_port = htons(port);

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_ERRORCHECK);
    pthread_mutex_init(&mutex, &mutexattr);
    pthread_cond_init(&cond_CopyPkg, NULL);

    onConnectionStateChanged = nullptr;
}

LidarLinker::~LidarLinker()
{
    DBG_INFO << T_BLUE << "[" << lidarName << "]Disconnecting from lidar..." << T_RESET << endl;
    if (disconnectFromLidar() == 0)
        ;
    DBG_INFO << T_GREEN << "[" << lidarName << "]Disconnected from lidar." << T_RESET << endl;

    if (lidarSockAddr != nullptr)
        delete lidarSockAddr;

    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond_CopyPkg);
}

int LidarLinker::connect2Lidar(uint32_t timeout_sec)
{
    DBG_INFO << T_BLUE << "[" << lidarName << "]Connecting to lidar..." << T_RESET << endl;
    if (onConnectionStateChanged != nullptr)
        onConnectionStateChanged(Connecting);
    bool foreverLoop = !timeout_sec;
    if (connect(lidarSockFD, (sockaddr *)lidarSockAddr, sizeof(struct sockaddr)) == -1)
    {
        DBG_INFO << T_RED << "[" << lidarName << "]Error : Failed to connect to lidar : " << strerror(errno) << T_RESET << endl
                 << T_BLUE << "Retrying..." << T_RESET << endl;
        sleep(1);
        int flag = fcntl(lidarSockFD, F_GETFL);
        fcntl(lidarSockFD, F_SETFL, flag | O_NONBLOCK);
        errno = 0;

        while (lidarSockFD > 0 && connect(lidarSockFD, (sockaddr *)lidarSockAddr, sizeof(struct sockaddr)) == -1)
        {
            if (errno != EAGAIN && errno != EINPROGRESS && errno != EALREADY)
                DBG_INFO << T_RED << "[" << lidarName << "]Error : Failed to connect to lidar : " << strerror(errno) << T_RESET << endl
                         << T_BLUE << "Retrying..." << T_RESET << endl;
            sleep(1);
            if (!foreverLoop)
            {
                if (!(--timeout_sec))
                {
                    errno = ETIMEDOUT;
                    if (lidarSockFD > 0)
                        fcntl(lidarSockFD, F_SETFL, flag);
                    DBG_INFO << T_RED << "[" << lidarName << "]Error : Connect to lidar timeout : " << strerror(errno) << T_RESET << endl;
                    return -1;
                }
                else
                    continue;
            }
        }
        if (lidarSockFD > 0)
            fcntl(lidarSockFD, F_SETFL, flag);
    }
    DBG_INFO << T_GREEN << "[" << lidarName << "]The connection has been successful." << T_RESET << endl;
    if (onConnectionStateChanged != nullptr)
        onConnectionStateChanged(Connected);
    return 0;
}

int LidarLinker::reconnect2Lidar()
{
    close(lidarSockFD);
    if ((lidarSockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        DBG_INFO << T_RED << "[" << lidarName << "]Fata : Failed to reset socket fd : " << strerror(errno) << T_RESET << endl;
        exit(-1);
    }
    setsockopt(lidarSockFD, SOL_SOCKET, SO_RCVTIMEO, &rcvtimeout, sizeof(rcvtimeout));
    return connect2Lidar();
}

int LidarLinker::disconnectFromLidar()
{
    if (pthread_kill(dataReceiver, 0) == 0)
        stopLidar();
    if (lidarSockFD > 0)
    {
        close(lidarSockFD);
        if (onConnectionStateChanged != nullptr)
            onConnectionStateChanged(Disconnected);
        return 0;
    }
    else
        return 0;
}

int LidarLinker::startupLidar()
{
    if (send_cmd_to_lidar(PacLidar::CTRL_START) < 0)
        return -1;

    if (pthread_self() != dataReceiver)
    {
        isCap = true;
        pthread_create(&dataReceiver, NULL, dataRecvFunc, (void *)this);
        while (pthread_kill(dataReceiver, 0) != 0)
        {
            DBG_INFO << T_BLUE << "[" << lidarName << "]Wait for receiver thread starting." << T_RESET << endl;
            msleep(1);
        }
    }
    return 0;
}

int LidarLinker::stopLidar()
{
    if (pthread_kill(dataReceiver, 0) == 0)
    {
        isCap = false;
        if (pthread_self() != dataReceiver)
            pthread_join(dataReceiver, NULL);
    }
    if (send_cmd_to_lidar(PacLidar::CTRL_STOP) < 0)
        return -1;
    else
        return 0;
}

int LidarLinker::setupLidar(LidarProp prop,int val)
{
    PacLidar::lidarCMD _cmd;
    if(prop == SCAN_RATE){
        switch (val)
        {
        case 10:
            _cmd = PacLidar::SET_SPEED_HZ_10;
            break;
        case 15:
            _cmd = PacLidar::SET_SPEED_HZ_15;
            break;
        case 20:
            _cmd = PacLidar::SET_SPEED_HZ_20;
            break;
        case 25:
            _cmd = PacLidar::SET_SPEED_HZ_25;
            break;
        case 30:
            _cmd = PacLidar::SET_SPEED_HZ_30;
            break;
        default:
            return -1;
            break;
        }

        if (send_cmd_to_lidar(_cmd) == -1)
        {
            DBG_INFO << T_RED << "[" << lidarName << "]Error : Setup lidar failed : " << strerror(errno) << T_RESET << endl;
            return -1;
        }
    }
    else if(prop == FILTER_LEVEL){
        switch (val)
        {
        case 0:
            _cmd = PacLidar::NO_WAVE_FILTERING;
            break;
        case 1:
            _cmd = PacLidar::WAVE_FILTERING_1;
            break;
        case 2:
            _cmd = PacLidar::WAVE_FILTERING_2;
            break;
        case 3:
            _cmd = PacLidar::WAVE_FILTERING_3;
            break;
        case 4:
            _cmd = PacLidar::WAVE_FILTERING_4;
            break;
        case 5:
            _cmd = PacLidar::WAVE_FILTERING_5;
            break;
        case 6:
            _cmd = PacLidar::WAVE_FILTERING_6;
            break;
        default:
            return -1;
            break;
        }
        if (send_cmd_to_lidar(_cmd) == -1)
        {
            DBG_INFO << T_RED << "[" << lidarName << "]Error : Setup lidar failed : " << strerror(errno) << T_RESET << endl;
            return -1;
        }
    }
    else if(prop == DATA_PROPORTION){
        if(val!=1 && val!=2 && val!=4) return -1;
        _dtPropr = val;
    }

    return 0;
}

int LidarLinker::getLidarScanData(std::vector<float>& ranges,std::vector<float>& intensities,double& startTime,double& scanTime,double& mesureTime)
{
    ranges.clear();
    intensities.clear();
    if (pthread_mutex_lock(&mutex) == 0)
    {
        for(int i = PAC_START_BEAM;i<PAC_MAX_BEAMS;++i)
        {
            auto range = oneCircleData[i].part1/1000.0;
            if(i%_dtPropr) continue;
            if(range)
            {
                ranges.push_back(range);
                intensities.push_back(oneCircleData[i].part3);
            }
            else
            {
                ranges.push_back(INFINITY);
                intensities.push_back(0);
            }
        }
        for(int i = 0;i<PAC_START_BEAM;++i)
        {
            auto range = oneCircleData[i].part1/1000.0;
            if(i%_dtPropr) continue;
            if(range)
            {
                ranges.push_back(range);
                intensities.push_back(oneCircleData[i].part3);
            }
            else
            {
                ranges.push_back(INFINITY);
                intensities.push_back(0);
            }
        }
        pthread_cond_signal(&cond_CopyPkg);
        pthread_mutex_unlock(&mutex);
        startTime = _pkgFirstRayTime;
        scanTime = _pkgScanTime;
        mesureTime = _complementMesureTime * _dtPropr;
        return ranges.size();
    }
    else
        return -1;
}

int LidarLinker::getLidarState(PacLidar::lidarState_t &state)
{
    if (lidarStatus.id != 0)
    {
        state = lidarStatus;
        return 0;
    }
    else
        return -1;
}

int LidarLinker::send_cmd_to_lidar(uint16_t cmd)
{
    if (lidarSockFD < 0)
        return -1;
    uint16_t netCMD = htons(cmd);
    int rtn = send(lidarSockFD, &netCMD, sizeof(netCMD), 0);
    if (rtn < 0 || rtn != sizeof(netCMD))
    {
        DBG_INFO << T_RED << "[" << lidarName << "]Error : Send cmd failed with errno " << errno << " : " << strerror(errno) << T_RESET << endl;
        if (errno == ECONNRESET)
        { //reset by peer
            DBG_INFO << T_BLUE << "[" << lidarName << "]Connection reset by peer,trying to reconnect it..." << T_RESET << endl;
            reconnect2Lidar();
            return send_cmd_to_lidar(cmd);
        }
    }
    else if (rtn == sizeof(netCMD))
        DBG_INFO << T_GREEN << "[" << lidarName << "]Send cmd : 0x" << std::hex << ntohs(netCMD) << " succed" << T_RESET << endl;
    msleep(50);
    return rtn;
}

void *LidarLinker::dataRecvFunc(void *recver)
{
    LidarLinker *rcvr = (LidarLinker *)recver;
    rcvr->capLidarData();
}

void LidarLinker::capLidarData()
{
    if (lidarSockFD < 0)
        return;
    
    PacLidar::LidarData_t sockDataPkg[PAC_NUM_OF_ONE_PKG];
    PacLidar::LidarData_t scanDataPkg[PAC_NUM_OF_ONE_SCAN];

    ssize_t recvedNum = 0;
    uint32_t buf_start_idx = 0;
    pthread_mutex_trylock(&mutex);
    PacLidar::LidarData_t dataRemains[PAC_NUM_OF_ONE_SCAN];
    bzero(dataRemains, sizeof(dataRemains));
    size_t remainPointsCnt = 0;
    double pkgEndStamp = 0;
    int pkgPointsCnt = 0;
    while (isCap)
    {
        size_t leftNum = sizeof(sockDataPkg);
        auto index = (uint8_t *)(&sockDataPkg);
        bool pkgHd = false, pkgTl = false;
        while (leftNum > 0)
        {
            bzero(index, leftNum);

            int quickAck = 1;
            setsockopt(lidarSockFD, IPPROTO_TCP, TCP_QUICKACK, &quickAck, sizeof(quickAck));

            recvedNum = recv(lidarSockFD, index, leftNum, 0);
            pkgEndStamp = ros::Time::now().toSec();
            if (recvedNum > 0)
            {
                /************检查包头***********/
                if (!pkgHd && sockDataPkg[0].part1 == PacLidar::LIDAR_DATA_HEADER_T.part1)
                {
                    if (sockDataPkg[0].part2 == PacLidar::LIDAR_DATA_HEADER_T.part2 &&
                        sockDataPkg[0].part3 == PacLidar::LIDAR_DATA_HEADER_T.part3)
                    {
                        // DBG_INFO << "Got pkg header!" << T_RESET << endl;
                        pkgHd = true;
                    }
                }
                /*******************************/
                if (pkgHd)
                {
                    leftNum -= recvedNum;
                    index += recvedNum;
                }
            }
            else if (recvedNum == 0)
            {
                DBG_INFO << T_BLUE << "[" << lidarName << "]Connection reset by peer,Trying to reconnect it..." << T_RESET << endl;
                if (onConnectionStateChanged)
                    onConnectionStateChanged(Disconnected);
                reconnect2Lidar();
                startupLidar();
            }
            else
            {
                DBG_INFO << T_RED << "[" << lidarName << "]Error : Receive msg from lidar failed with errno:" << errno << ": " << strerror(errno) << T_RESET << endl;
                if (errno == EAGAIN)
                {
                    DBG_INFO << T_RED << "[" << lidarName << "]Error : Server may be down,trying to reconnect it..." << T_RESET << endl;
                    if (onConnectionStateChanged)
                        onConnectionStateChanged(Disconnected);
                    reconnect2Lidar();
                    startupLidar();
                }
            }
        }

        /**************检查包尾***************/
        if (sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part1 == PacLidar::LIDAR_DATA_TAIL_T.part1)
        {
            if (sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part2 == PacLidar::LIDAR_DATA_TAIL_T.part2 &&
                sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part3 == PacLidar::LIDAR_DATA_TAIL_T.part3)
                pkgTl = true;
            pkgPointsCnt += 1000;
        }
        /***********************************/

        /*************提取数据-开始******************/
        if (pkgHd && pkgTl)
        {
            /*************提取雷达状态-开始**********************/
            lidarStatus.temprature = (sockDataPkg[PAC_IDX_OF_TMP].part1 << 16) + sockDataPkg[PAC_IDX_OF_TMP].part2;
            lidarStatus.id = (sockDataPkg[PAC_IDX_OF_ID].part1 << 16) + sockDataPkg[PAC_IDX_OF_ID].part2;
            lidarStatus.version = (sockDataPkg[PAC_IDX_OF_VER].part1 << 16) + sockDataPkg[PAC_IDX_OF_VER].part2;
            lidarStatus.speed = sockDataPkg[PAC_IDX_OF_SPD].part2;
            /*************提取雷达状态-结束**********************/

            /*************提取扫描数据-开始**********************/
            //拷贝至scan数组
            memcpy(scanDataPkg, sockDataPkg + 1, sizeof(scanDataPkg));

            //初始化起始角度和结束角度寻找的标志位
            static bool dataAvalible = false;

            //提取上次结余的数据
            if (remainPointsCnt>0 && remainPointsCnt <= PAC_NUM_OF_ONE_SCAN)
            {
                for (int i = 0; i < remainPointsCnt; i++)
                {
                    PacLidar::LidarData_t datai = dataRemains[i];
                    if (datai.part2 < PAC_MAX_BEAMS)
                        oneCircleData[datai.part2] = datai;
                }
                pkgPointsCnt += remainPointsCnt;
                bzero(dataRemains, sizeof(dataRemains));
                remainPointsCnt = 0;
            }

            //提取本次收到的数据
            bool isCoverEmptyBlock = false;
            for (size_t i = 0; i < PAC_NUM_OF_ONE_SCAN; i++)
            {
                PacLidar::LidarData_t datai = scanDataPkg[i];
                //如果角度与上个点重复，则舍弃此点
                if (i){
                    auto lastData = scanDataPkg[i - 1];
                    if(datai.part2 == lastData.part2)
                        continue;
                    
                    if((!isCoverEmptyBlock) && (datai.part2 - lastData.part2 > 1360)) //85*(1/PAC_ANGLE_RESOLUTION) = 1360
                        isCoverEmptyBlock = true;
                }

                if (datai.part2 < PAC_MAX_BEAMS)
                {
                    //若当前角度大于314且已保存的点数大于736（5760-5024）个，视为获取到了新的一帧
                    if (oneCircleData[datai.part2].part2 > 5024 && (pkgPointsCnt-PAC_NUM_OF_ONE_SCAN+i)>736)
                    {
                        dataAvalible = true;

                        remainPointsCnt = PAC_NUM_OF_ONE_SCAN - i;
                        memcpy(dataRemains, &scanDataPkg[i], remainPointsCnt * sizeof(PacLidar::LidarData_t));

                        pkgPointsCnt -= remainPointsCnt;

                        break;
                    }
                    oneCircleData[datai.part2] = datai;
                }
                else if (datai.part2 == PAC_MAX_BEAMS)
                    oneCircleData[0] = datai;
            }
            
            /*******************计算时间**********************/
            if(!isCoverEmptyBlock){
                //Mesure time
                int complementPointsCnt = 0;
                if(scanDataPkg[0].part2>(PAC_START_BEAM-int(1/PAC_ANGLE_RESOLUTION))){
                    complementPointsCnt += abs(scanDataPkg[0].part2-PAC_MAX_BEAMS);
                    complementPointsCnt += scanDataPkg[PAC_NUM_OF_ONE_SCAN-1].part2;
                } else {
                    complementPointsCnt += scanDataPkg[PAC_NUM_OF_ONE_SCAN-1].part2 - scanDataPkg[0].part2;
                }
                _complementMesureTime = (PAC_NUM_OF_ONE_SCAN*PAC_MESURE_TIME) / complementPointsCnt;

                //Scan Time
                _pkgScanTime = _complementMesureTime * PAC_MAX_BEAMS;
            }
            /**************************************************/

            //提取一周数据完毕，解锁
            if (dataAvalible)
            {
                //计算第一束时间戳
                _pkgFirstRayTime = pkgEndStamp-(remainPointsCnt*PAC_MESURE_TIME)-_pkgScanTime;

                // DBG_INFO << "Got one full pkg." << T_RESET << endl;
                struct timespec ts;
                timespec_get(&ts, TIME_UTC);
                ts.tv_nsec += 10000000; //10ms
                if (ts.tv_nsec > 1000000000)
                {
                    ts.tv_sec += 1;
                    ts.tv_nsec -= 1000000000;
                }
                pthread_cond_timedwait(&cond_CopyPkg, &mutex, &ts);
                pthread_mutex_unlock(&mutex);

                pthread_mutex_lock(&mutex);

                //清空数组 与 点计数器
                bzero(oneCircleData, sizeof(oneCircleData));
                pkgPointsCnt = 0;
                
                //重置标志
                dataAvalible = false;
            }
            /*************提取扫描数据-结束******************/
        }
        /*************提取数据-结束******************/
    }
    pthread_mutex_unlock(&mutex);
}

void LidarLinker::msleep(uint32_t msec)
{
    while (msec > 0)
    {
        usleep(1000);
        msec--;
    }
}

void LidarLinker::registerConnectionStateChangedCallback(void (*callback)(int))
{
    onConnectionStateChanged = callback;
}