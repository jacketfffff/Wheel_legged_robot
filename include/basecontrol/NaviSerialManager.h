#ifndef NAVISERIALMANAGER_H
#define NAVISERIALMANAGER_H

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "basecontrol/SerialManager.h"

#define COMMAND_SIZE 8
#define ENCODER_SIZE 12
#define LOCK_SIZE 10
#define RESULT_SIZE COMMAND_SIZE*20

#define COMMAND_HEAD 0x35
#define ENCODER_HEAD 0X92
#define FORCESENSOR_HEAD 0X49

#define CONTROL_SIZE 44

class NaviSerialManager : public SerialManager{
public:
    struct ReadResult{
        char read_result[RESULT_SIZE];
        int read_bytes;
    };

private:
    std::queue<ReadResult> read_result_queue{};
    ReadResult read_results_;
    int read_used_bytes{};
    int command_size{};
    int  COMMAND_SIZE_{};
    mutable boost::shared_mutex queue_mutex_{};
    bool isAutoThreadRegistered_{};
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    void readWorker(int rate);
    int getCommandBeginIndex(int check_begin_index=0);
public:
    NaviSerialManager(std::string serial_addr, unsigned int baudrate, int command_size);
    NaviSerialManager(const SerialManager & serialManager);
    ~NaviSerialManager();
    void registerAutoReadThread(int rate);
    void receive();
    ReadResult & getReadResult()
    {
        boost::unique_lock<boost::shared_mutex> writeLock(queue_mutex_);
        if(!read_result_queue.empty())
        {
            read_results_ = read_result_queue.front();
            read_result_queue.pop();
        }
        else
            memset(&read_results_,0, sizeof(ReadResult));
        return read_results_;
    };
};


#endif //NAVISERIALMANAGER_H