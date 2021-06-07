#include<pardon_action_server/util.h>

InvalidParamException::InvalidParamException(const std::string paramName) : paramName_(paramName) {}

const char* InvalidParamException::what() const throw() {
    return paramName_.c_str();
}
