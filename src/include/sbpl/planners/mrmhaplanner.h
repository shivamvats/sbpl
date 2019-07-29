/*
 * Copyright (c) 2015, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef sbpl_MRMHAPlanner_h
#define sbpl_MRMHAPlanner_h

#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/mhaplanner.h>
#include <sbpl/utils/heap.h>

class MRMHAPlanner : public MHAPlanner
{
public:

    MRMHAPlanner(
            DiscreteSpaceInformation* environment,
            Heuristic* hanchor,
            Heuristic** heurs,
            int hcount);

    virtual ~MRMHAPlanner();

    /// \sa SBPLPlanner::replan(std::vector<int>*, ReplanParams, int*)
    int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V) override;
    int replan(
            double allocated_time_sec,
            std::vector<int>* solution_stateIDs_V,
            int* solcost) override;
    virtual int replan(
            std::vector<int>* solution_stateIDs_V,
            ReplanParams params,
            int* solcost) override;
    void expand(MHASearchState* state, int hidx);

private:
    /*
    // Related objects
    Heuristic* m_hanchor;
    Heuristic** m_heurs;
    int m_hcount;           ///< number of additional heuristics used

    ReplanParams m_params;
    double m_initial_eps_mha;
    int m_max_expansions;

    double m_eps;           ///< current w_1
    double m_eps_mha;       ///< current w_2

    /// suboptimality bound satisfied by the last search
    double m_eps_satisfied; 

    int m_num_expansions;   ///< current number of expansion
    double m_elapsed;       ///< current amount of seconds

    int m_call_number;

    MHASearchState* m_start_state;
    MHASearchState* m_goal_state;

    std::vector<MHASearchState*> m_search_states;

    CHeap* m_open; ///< sequence of (m_hcount + 1) open lists

    bool check_params(const ReplanParams& params);

    bool time_limit_reached() const;

    int num_heuristics() const { return m_hcount + 1; }
    MHASearchState* get_state(int state_id);
    void init_state(MHASearchState* state, size_t mha_state_idx, int state_id);
    void reinit_state(MHASearchState* state);
    void reinit_search();
    void clear_open_lists();
    void clear();
    int compute_key(MHASearchState* state, int hidx);
    void expand(MHASearchState* state, int hidx);
    MHASearchState* state_from_open_state(AbstractSearchState* open_state);
    int compute_heuristic(int state_id, int hidx);
    int get_minf(CHeap& pq) const;
    void insert_or_update(MHASearchState* state, int hidx, int f);

    void extract_path(std::vector<int>* solution_path, int* solcost);

    bool closed_in_anc_search(MHASearchState* state) const;
    bool closed_in_add_search(MHASearchState* state) const;
    bool closed_in_any_search(MHASearchState* state) const;
    */
};

#endif
