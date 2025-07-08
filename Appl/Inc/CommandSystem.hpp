#ifndef COMMANDSYSTEM_H
#define COMMANDSYSTEM_H

#include "CommandHandler.hpp"



#if CONF_USE_TIME == 1
class Com_gettime : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"gettime"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};


class Com_settime : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"settime"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};
#endif

class Com_meminfo : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"meminfo"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class Com_memdetinfo : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"memdetinfo"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class Com_gettasks : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"gettasks"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class Com_sysinfo : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"sysinfo"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class Com_siginfo : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"siginfo"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class CommandSystem_c :public CommandGroup_c
{
  #if CONF_USE_TIME == 1
  Com_gettime gettime;
  Com_settime settime;
  #endif
  Com_meminfo meminfo;
  Com_memdetinfo memdetinfo;
  Com_gettasks gettasks;
  Com_sysinfo sysinfo;
  Com_siginfo siginfo;

  public:

  CommandSystem_c(void);


};
#endif