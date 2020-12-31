#include "basecontrol/NaviSerialManager.h"
#include "ros/ros.h"
NaviSerialManager::NaviSerialManager(std::string serial_addr, unsigned int baudrate,int command_size = 8):SerialManager(serial_addr,baudrate)
{

}
NaviSerialManager::NaviSerialManager(const SerialManager & serialManager):SerialManager(serialManager)
{

}
NaviSerialManager::~NaviSerialManager()
{
    if(isAutoThreadRegistered_)
    {
        thread_ptr_->interrupt();
        thread_ptr_->join();
    }
}
void NaviSerialManager::registerAutoReadThread(int rate)
{   
    thread_ptr_.reset(new boost::thread(boost::bind(&NaviSerialManager::readWorker, this, rate)));
    isAutoThreadRegistered_=true;
}
void NaviSerialManager::readWorker(int rate)
{
    static ros::Rate loop_rate(rate);
    try
    {
        boost::this_thread::interruption_enabled();//检测线程是否允许中断
        while(true)
        {
            //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::universal_time();
            boost::this_thread::interruption_point();
            this->receive();
            //boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::universal_time();
            //boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate - (endTime - startTime).total_microseconds()));
            loop_rate.sleep();
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"now quit the read thread"<<std::endl;
    }
}
int NaviSerialManager::getCommandBeginIndex(int check_begin_index)
{
    for(int i=0;i<read_used_bytes-check_begin_index;++i)
    {
        //if(read_buffer[i+check_begin_index]==COMMAND_HEAD || read_buffer[i+check_begin_index]==KNEE_HEAD || read_buffer[i+check_begin_index]==ANKLE_HEAD)
        if(read_buffer[i+check_begin_index] == COMMAND_HEAD)
        {
            if(read_buffer[i+check_begin_index+8] == COMMAND_TAIL)
            {
                command_size = 8;
                return  i+check_begin_index;
            }
                //这里是要对每包数据进行二次检测
        }
        else if (read_buffer[i+check_begin_index] == FORCESENSOR_HEAD)
        {
            if(read_buffer[i+check_begin_index+12] == FORCESENSOR_TAIL)
            { 
                command_size = 12;
                return  i+check_begin_index;
            }        
        }
        else if (read_buffer[i+check_begin_index] == ENCODER_HEAD)
        {
            if (read_buffer[i+check_begin_index+12] == ENCODER_TAIL)
            {
                command_size = 12; 
                return i+check_begin_index;
            }
        }
        //return  i+check_begin_index;
    }
    return read_used_bytes;
}
void NaviSerialManager::receive()
{
    //serial_mutex_.lock();
    int receiveNumbers=read(m_dFd,&read_buffer[read_used_bytes],BUFFER_SIZE);
    //ROS_INFO_STREAM("the receive number is "<<receiveNumbers);
    //serial_mutex_.unlock();
    if(receiveNumbers>0)
    {
        serial_alive_ =true;
        read_used_bytes +=receiveNumbers;
        if(read_used_bytes>=command_size)
        {
            int commandBeginIndex{0};
            ReadResult temp{};
            for(int i=0;i<read_used_bytes;i+=command_size)
            {
                commandBeginIndex=getCommandBeginIndex(commandBeginIndex+i);
                if(commandBeginIndex<=read_used_bytes-command_size)
                {
                    memcpy(&temp.read_result[i], &read_buffer[commandBeginIndex], command_size);
                    temp.read_bytes += command_size;
                }
                else
                   break;
            }
            read_result_queue.push(temp);
            if(commandBeginIndex<read_used_bytes&&commandBeginIndex>read_used_bytes-command_size)
            {
                char transfer_buffer[read_used_bytes-commandBeginIndex]{};
                memcpy(&transfer_buffer[0],&read_buffer[commandBeginIndex],read_used_bytes-commandBeginIndex);
                memset(read_buffer,0,BUFFER_SIZE);
                memcpy(&read_buffer[0],&transfer_buffer[0],read_used_bytes-commandBeginIndex);
                read_used_bytes=read_used_bytes-commandBeginIndex;
            }
            else
            {
                memset(read_buffer,0,BUFFER_SIZE);
                read_used_bytes=0;
                司法的发送
            }
        }
        //Warning, ignore some data when the STM32 send wrong many times
        if(read_used_bytes>=BUFFER_SIZE-command_size)
        {
            read_used_bytes=0;
            memset(read_buffer,0,BUFFER_SIZE);
        }
    }
    else if(receiveNumbers<0)
    {
        serial_alive_ =false;
        read_used_bytes= 0;
        memset(read_buffer,0,BUFFER_SIZE);
    }

}