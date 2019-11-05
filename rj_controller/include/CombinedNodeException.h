#ifndef COMBINEDNODEEXCEPTION_H
#define COMBINEDNODEEXCEPTION_H

#include <exception>
#include <string>

class CombinedNodeException : public std::exception {
    std::string error_message;
public:
    explicit CombinedNodeException(const std::string& message) : error_message(message) {}
    virtual ~CombinedNodeException() throw() {}
    virtual const char* what() const throw() {
        return error_message.c_str();
    }
};

#endif

