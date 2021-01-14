#ifndef NURSINGROBOT_LOGGER_HPP
#define NURSINGROBOT_LOGGER_HPP

#include <glog/logging.h>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>


/**
 * \class wrapper for glog
 *
 * */
class logger {
private:
    const std::string LOG_DIR_;
public:
    explicit logger(char *argv0,std::string log_dir = std::string())
            : LOG_DIR_(std::move(log_dir))
    {
        google::InitGoogleLogging(argv0);
        setParameter();
    }

    ~logger()
    {
        google::ShutdownGoogleLogging();
    }

    static void setFlagLogToStd(bool flag)
    {
        FLAGS_logtostderr = flag;
    }
    void setParameter()
    {
        google::SetStderrLogging(google::GLOG_ERROR);

        FLAGS_logtostderr = false;

        FLAGS_alsologtostderr = false;

        FLAGS_log_prefix = true;

        FLAGS_colorlogtostderr = true;

        google::InstallFailureSignalHandler();

        FLAGS_logbufsecs =0;

        FLAGS_max_log_size =100;

        FLAGS_stop_logging_if_full_disk = true;

        if (!LOG_DIR_.empty()&& access(LOG_DIR_.data(), 0) == -1){
            mkdir(LOG_DIR_.data(),0777);
        }
        else if(LOG_DIR_.empty()){
            FLAGS_logtostderr = true;
        }
        FLAGS_log_dir = LOG_DIR_;
    }
};


#endif //NURSINGROBOT_LOGGER_HPP
