#ifndef NURSINGROBOT_LOGGER_HPP
#define NURSINGROBOT_LOGGER_HPP

#include <glog/logging.h>

/**
 * \class wrapper for glog
 *
 * */
class logger {
private:
    const std::string LOG_DIR_;
public:
    logger(std::string log_dir, char *argv0)
            : LOG_DIR_(std::move(log_dir))
    {
        google::SetStderrLogging(google::GLOG_INFO);

        google::InitGoogleLogging(argv0);

        FLAGS_logtostderr = true;

        FLAGS_alsologtostderr = false;

        FLAGS_log_prefix = true;

        FLAGS_log_dir = LOG_DIR_;

        FLAGS_colorlogtostderr = true;

        google::InstallFailureSignalHandler();

        FLAGS_logbufsecs =0;

        FLAGS_max_log_size =100;
    }

    ~logger()
    {
        google::ShutdownGoogleLogging();
    }

    static void setFlagLogToStd(bool flag)
    {
        FLAGS_logtostderr = flag;
    }
};


#endif //NURSINGROBOT_LOGGER_HPP
