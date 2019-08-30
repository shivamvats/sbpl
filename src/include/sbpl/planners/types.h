#ifndef sbpl_PLANNERS_COMMON_h
#define sbpl_PLANNERS_COMMON_h

class SchedulingPolicy {
    public:
    SchedulingPolicy(int _num_queues) : m_num_queues{_num_queues} {};
    virtual int getNextQueue() = 0;
    int numQueues() const {return m_num_queues;}

    private:
    int m_num_queues;
};

#endif
