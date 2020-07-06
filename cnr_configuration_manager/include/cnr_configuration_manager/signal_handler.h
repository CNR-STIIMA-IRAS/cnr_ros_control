#ifndef __SIGNALHANDLER_H__
#define __SIGNALHANDLER_H_

#include <stdexcept>

class SignalException : public std::runtime_error
{
public:
   SignalException(const std::string& _message)
      : std::runtime_error(_message)
   {}
};

class SignalHandler
{
protected:
    static bool mbGotExitSignal;

public:
    SignalHandler();
    ~SignalHandler();

    static bool gotExitSignal();
    static void setExitSignal(bool _bExitSignal);

    void        setupSignalHandlers();
    static void exitSignalHandler(int _ignored);

};
#endif
