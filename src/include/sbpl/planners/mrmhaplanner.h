#ifndef sbpl_MRMHAPlanner_h
#define sbpl_MRMHAPlanner_h

#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/mhaplanner.h>
#include <sbpl/utils/heap.h>
#include "types.h"

class MRMHAPlanner : public MHAPlanner
{
public:

    MRMHAPlanner(
            DiscreteSpaceInformation* environment,
            Heuristic* hanchor,
            Heuristic** heurs,
            int hcount,
            SchedulingPolicy* policy=nullptr);

    virtual ~MRMHAPlanner();

    /// \sa SBPLPlanner::replan(std::vector<int>*, ReplanParams, int*)
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V) override;
    virtual int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V,
            int* solcost) override;
    virtual int replan(
            std::vector<int>* solution_stateIDs_V,
            ReplanParams params,
            int* solcost) override;
    void expand(MHASearchState* state, int hidx);

private:
    SchedulingPolicy* m_schedule_policy;
};

#endif
