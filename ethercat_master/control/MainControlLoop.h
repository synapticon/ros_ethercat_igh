/*
* Add Comment here
*/

#ifndef MAINCONTROLLOOP_H
#define MAINCONTROLLOOP_H

#include <mcx/mcx_core.h>

class MainControlLoop : public mcx::container::Module {
public:

    MainControlLoop() = default;

    ~MainControlLoop() override = default;

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;
};

#endif /* MAINCONTROLLOOP_H */