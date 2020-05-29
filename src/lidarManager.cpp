#include "lidarManager.h"

using namespace std;

lidarManager::lidarManager(string ip, uint16_t port)
{

    lidarParam.speed = PacLidar::SET_SPEED_HZ_10;
    lidarParam.dataType = PacLidar::SET_DATA_CHECKED;

    lidarSockAddr = new sockaddr_in();
    if ((lidarSockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        cerr << "Fata:Create socket failed:" << strerror(errno) << std::endl;
    bzero(lidarSockAddr, sizeof(struct sockaddr_in));
    lidarSockAddr->sin_family = AF_INET;
    lidarSockAddr->sin_addr.s_addr = inet_addr(ip.c_str());
    lidarSockAddr->sin_port = htons(port);

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_FAST_NP);
    pthread_mutex_init(&mutex, &mutexattr);
}

lidarManager::~lidarManager()
{
    pthread_mutex_destroy(&mutex);

    if (dataReceiver)
    {
        isCap = false;
        pthread_join(dataReceiver, NULL);
    }

    if (lidarSockFD >= 0)
    {
        cout << "Close lidar" << endl;
        stopLidar();
        close(lidarSockFD);
    }

    if (lidarSockAddr != nullptr)
        delete lidarSockAddr;
}

int lidarManager::connectLidar()
{
    // int flag = fcntl(lidarSockFD, F_GETFL);
    // fcntl(lidarSockFD, F_SETFL, flag | O_NONBLOCK);
    cout << "Connecting..." << endl;
    int rtn = connect(lidarSockFD, (sockaddr *)lidarSockAddr, sizeof(struct sockaddr));
    // fcntl(lidarSockFD, F_SETFL, flag);
    if (rtn < 0)
    {
        cerr << "Error:connect to lidar failed:" << strerror(errno) << std::endl;
        return -1;
    }
    cout << "Conect to lidar succed." << endl;
    return rtn;
}

int lidarManager::reconnectLidar()
{
    close(lidarSockFD);
    if ((lidarSockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        cerr << "Fata:Reset socket failed:" << strerror(errno) << std::endl;
    while(connectLidar()!=0) sleep(1);
    return 0;
}

int lidarManager::startupLidar()
{
    return startupLidar(lidarParam.speed,lidarParam.dataType);
}

int lidarManager::startupLidar(PacLidar::lidarCMD speed, PacLidar::lidarCMD data_type)
{
    send_cmd_to_lidar(data_type);
    send_cmd_to_lidar(speed);
    if (send_cmd_to_lidar(PacLidar::CTRL_START) < 0)
        return -1;

    isCap = true;

    pthread_create(&dataReceiver, NULL, dataRecvFunc, (void *)this);

    return 0;
}

int lidarManager::stopLidar()
{
    if (send_cmd_to_lidar(PacLidar::CTRL_STOP) < 0)
        return -1;
    else
        return 0;
}

int lidarManager::setupLidar(PacLidar::lidarCMD speed,PacLidar::lidarCMD data_type)
{
    lidarParam.speed = speed;
    lidarParam.dataType = data_type;
    send_cmd_to_lidar(speed);
    send_cmd_to_lidar(data_type);
}

int lidarManager::getLidarScanByBeam(float *ranges, float *intensities, unsigned start_beam, unsigned stop_beam)
{
    assert(start_beam >= 0);
    assert(stop_beam <= PAC_MAX_BEAMS - 1);
    assert(stop_beam > start_beam);

    memset(ranges,      0, (stop_beam + 1 - start_beam) * sizeof(float));
    memset(intensities, 0, (stop_beam + 1 - start_beam) * sizeof(float));

    pthread_mutex_lock(&mutex);

    float range = 0;
    for (int i = start_beam; i <= stop_beam; ++i)
    {
        range = oneCircleData[i].part1 / 1000.0;
        if (range)
        {
            ranges[i] = range;
            intensities[i] = oneCircleData[i].part3;
        }
        else
        {
            ranges[i] = INFINITY;
            intensities[i] = 0;
        }
    }

    pthread_mutex_unlock(&mutex);
}


int lidarManager::getLidarScanByAngle(float *ranges, float *intensities, float start_angle, float stop_angle)
{
    unsigned start_beam = start_angle/PAC_ANGLE_RESOLUTION;
    unsigned stop_beam = stop_angle/PAC_ANGLE_RESOLUTION;
    stop_beam-=1;
    getLidarScanByBeam(ranges,intensities,start_beam,stop_beam);
}


int lidarManager::send_cmd_to_lidar(uint16_t cmd)
{
    if (lidarSockFD < 0)
        return -1;
    cmd = htons(cmd);
    int rtn = send(lidarSockFD, &cmd, sizeof(cmd), 0);
    if (rtn < 0 || rtn != sizeof(cmd))
        cout << "Send cmd failed:" << strerror(errno) << endl;
    else if (rtn == sizeof(cmd))
        cout << "Send cmd : 0x" << std::hex << ntohs(cmd) << " succed" << endl;
    msleep(50);
    return rtn;
}

void *lidarManager::dataRecvFunc(void *recver)
{
    lidarManager *rcvr = (lidarManager *)recver;
    rcvr->capLidarData();
}

void lidarManager::capLidarData()
{
    if (lidarSockFD < 0)
        return;
    uint64_t dtCnt = 0;
    ssize_t recvedNum = 0;
    uint32_t buf_start_idx = 0;
    while (isCap)
    {
        size_t leftNum = sizeof(sockDataPkg);
        auto index = (uint8_t *)(&sockDataPkg);
        bool pkgHd = false, pkgTl = false;
        while (leftNum > 0)
        {
            memset(index, 0, leftNum);

            recvedNum = recv(lidarSockFD, index, leftNum, 0);
            if (recvedNum > 0)
            {
                /************检查包头***********/
                if (!pkgHd && sockDataPkg[0].part1 == PacLidar::LIDAR_DATA_HEADER_T.part1)
                {
                    if (sockDataPkg[0].part2 == PacLidar::LIDAR_DATA_HEADER_T.part2 &&
                        sockDataPkg[0].part3 == PacLidar::LIDAR_DATA_HEADER_T.part3)
                    {
                        // cout << "Got pkg header!" << endl;
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
                cout << "Connection reset by peer,Ready to reconnecting." << endl;
                reconnectLidar();
                startupLidar();
            }
            else
            {
                cerr << "Error:Receive msg from lidar failed:" << strerror(errno) << std::endl;
            }
        }
        
        /**************检查包尾***************/
        if (sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part1 == PacLidar::LIDAR_DATA_TAIL_T.part1)
        {
            if (sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part2 == PacLidar::LIDAR_DATA_TAIL_T.part2 &&
                sockDataPkg[PAC_NUM_OF_ONE_PKG - 1].part3 == PacLidar::LIDAR_DATA_TAIL_T.part3)
                pkgTl = true;
        }
        /***********************************/

        /*************提取数据开始******************/
        if (pkgHd && pkgTl)
        {
            dtCnt++;
            /*************提取雷达状态-开始**********************/
            lidarStatus.temprature = (sockDataPkg[PAC_IDX_OF_TMP].part1 << 8) + sockDataPkg[PAC_IDX_OF_TMP].part2;
            lidarStatus.id = (sockDataPkg[PAC_IDX_OF_ID].part1 << 8) + sockDataPkg[PAC_IDX_OF_ID].part2;
            lidarStatus.version = (sockDataPkg[PAC_IDX_OF_VER].part1 << 8) + sockDataPkg[PAC_IDX_OF_VER].part2;
            lidarStatus.speed = sockDataPkg[PAC_IDX_OF_SPD].part2;
            /*************提取雷达状态-结束**********************/

            /*************提取扫描数据-开始**********************/
            memcpy(scanDataPkg, sockDataPkg + 1, sizeof(scanDataPkg));
            
            for (auto &i : scanDataPkg)
            {
                if (i.part2 < PAC_MAX_BEAMS)
                {
                    oneCircleData[i.part2] = i;
                }
            }

            if (dtCnt % 6 == 0)
            {
                pthread_mutex_unlock(&mutex);
                // cout << "Got one full pkg." << endl;
                dtCnt = 0;
                sleep(1);
                pthread_mutex_lock(&mutex);
                memset(oneCircleData, 0, sizeof(oneCircleData));
            }
            /*************提取扫描数据-结束******************/
        }
    }
}

void lidarManager::msleep(uint32_t msec)
{
    while (msec > 0)
    {
        usleep(1000);
        msec--;
    }
}

int32_t lidarManager::select_start_degree(PacLidar::LidarData_t *arr, size_t size)
{
    size = size / sizeof(PacLidar::LidarData_t);
    assert(size <= PAC_MAX_BEAMS);
    unsigned start = 0;
    unsigned end = size - 1;
    if (arr[0].part2 == 0)
        return 0;
    else
    {
        //判断是否有第起始角度的数据
        int32_t difference = arr[end].part2 - arr[start].part2;
        if (difference < 0)
        {
            //找到起始角度的数据
            unsigned idx = PAC_MAX_BEAMS - arr[start].part2;
            if (idx < size)
            {
                if (arr[idx].part2 == 0)
                    return idx;
                else
                {
                    for (int i = idx - arr[idx].part2; i < idx; i++)
                        if (arr[i].part2 == 0)
                            return i;
                }
            }
            else
                return -1;
        }
        //数组中不存在0度的数据
        else
            return -1;
    }
}