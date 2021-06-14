#ifndef __PARDON_ACTION_SERVER__UTIL_H__
#define __PARDON_ACTION_SERVER__UTIL_H__

#include<ros/ros.h>
#include<exception>

class InvalidParamException : public std::exception {
    const std::string paramName_;
  public:
    InvalidParamException(const std::string paramName);
    const char* what() const throw();
};

template<typename T>
T getParamValue(const std::string name, bool global = false){
    T value;
    if(global){
        if(!ros::param::get("/" + name, value))
            throw InvalidParamException(name);
    } else {
    if(!ros::param::get(name, value))
        throw InvalidParamException(name);
    }
    return value;
}

#endif