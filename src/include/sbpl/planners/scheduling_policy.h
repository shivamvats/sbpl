#ifndef sbpl_PLANNERS_SCHEDULING_POLICY_h
#define sbpl_PLANNERS_SCHEDULING_POLICY_h

#include <vector>

class SchedulingPolicy {
    public:
    SchedulingPolicy(int _num_queues) : m_num_queues{_num_queues} {};
    //virtual int getNextQueue(const std::vector<double>& state) = 0;
    virtual double getActionSpaceProb(int state_id, int hidx) = 0;
    int numQueues() const {return m_num_queues;}

    private:
    int m_num_queues;
};

#endif
