/*
* Add Comment here
*/

#ifndef DIGITALIO_H
#define DIGITALIO_H

#include <mcx/mcx_core.h>
#include "motorcortex_msgs/DigitalInputs.h"
#include "motorcortex_msgs/DigitalOutputs.h"

class DigitalIO : public mcx::container::Module {
public:

    DigitalIO() = default;

    ~DigitalIO() override = default;

    const motorcortex_msgs::DigitalInputs &getDIOFeedback() const {
        return digitalInputs_;
    }

    void setDigitalOutputs(const motorcortex_msgs::DigitalOutputs& digitalOutputsCommand) {
        digitalOutputs_ = digitalOutputsCommand;
    }

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    motorcortex_msgs::DigitalInputs digitalInputs_;
    motorcortex_msgs::DigitalOutputs digitalOutputs_;
};

#endif /* DIGITALIO_H */
