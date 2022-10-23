/***************************** SCHEDULER.H ***********************************
- SAFARI GROUP

This file contains the different scheduling policies and row policies that the 
memory controller can use to schedule requests.

Current Memory Scheduling Policies:

1) FCFS - First Come First Serve
        This scheduling policy schedules memory requests chronologically

2) FCFSBank - Bank-Aware First Come First Serve
        This scheduling policy first checks if a request can be issued to
        a bank that is ready (i.e., it is not in the process of servicing
        a request), and prioritizes those requests. If multiple requests
        are to ready banks, then they are scheduled chronologically.
        Otherwise, it behaves the same way as FCFS.

3) FRFCFS - Frist Ready First Come First Serve
        This scheduling policy first checks if a request is READY (if requests
        are to an open row), if yes then it is prioritized. If multiple requests
        are ready, they they are scheduled chronologically. Otherwise, it 
        behaves the same way as FCFSBank. 

                _______________________________________

Current Row Policies:

1) Closed   - Precharges a row as soon as there are no pending references to 
              the active row.
2) ClosedAP - Closed Auto Precharge
3) Opened   - Precharges a row only if there are pending references to 
              other rows.
4) Timeout  - Precharges a row after X time if there are no pending references.
              'X' time can be changed by changing the variable timeout 
              on line number 267

*****************************************************************************/

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "DRAM.h"
#include "Request.h"
#include "Controller.h"
#include "Config.h" // Saugata
#include <vector>
#include <map>
#include <list>
#include <functional>
#include <cassert>

using namespace std;

namespace ramulator
{

template <typename T>
class Controller;

template <typename T>
class Scheduler
{
public:
    Controller<T>* ctrl;

    // 18-740
    enum class Type {
        FCFS, FCFSBank, FRFCFS, BLISS, Custom, MAX
    } type; 

    std::map<string, Type> name_to_scheduler = {
      {"FCFS", Type::FCFS},
      {"FCFSBank", Type::FCFSBank},
      {"FRFCFS", Type::FRFCFS},
      {"BLISS", Type::BLISS},
      {"Custom", Type::Custom},
    };

    // Saugata
    Scheduler(const Config& configs, Controller<T>* ctrl) : ctrl(ctrl) {
      // Initiating scheduler
      if (configs.contains("scheduler")) {
        type = name_to_scheduler[configs["scheduler"]];
      }
      else {
        type = Type::FRFCFS;
      }
    }

    list<Request>::iterator get_head(list<Request>& q)
    {
        // TODO make the decision at compile time
        if (type == Type::FCFS || type == Type::FCFSBank) { // 18-740
            //If queue is empty, return end of queue
            if (!q.size())
                return q.end();

            //Else return based on the policy
            auto head = q.begin();
            for (auto itr = next(q.begin(), 1); itr != q.end(); itr++)
                head = compare[int(type)](head, itr);

            return head;
        } 
        else { //Code to get around edge cases for FRFCFS
            
       //If queue is empty, return end of queue
            if (!q.size())
                return q.end();

       //Else return based on FRFCFS Scheduling Policy
            auto head = q.begin();
            for (auto itr = next(q.begin(), 1); itr != q.end(); itr++) {
                head = compare[int(type)](head, itr);
            }

            if (this->ctrl->is_ready(head) && this->ctrl->is_row_hit(head)) {
                return head;
            }

            // prepare a list of hit request
            vector<vector<int>> hit_reqs;
            for (auto itr = q.begin() ; itr != q.end() ; ++itr) {
                if (this->ctrl->is_row_hit(itr)) {
                    auto begin = itr->addr_vec.begin();
                    // TODO Here it assumes all DRAM standards use PRE to close a row
                    // It's better to make it more general.
                    auto end = begin + int(ctrl->channel->spec->scope[int(T::Command::PRE)]) + 1;
                    vector<int> rowgroup(begin, end); // bank or subarray
                    hit_reqs.push_back(rowgroup);
                }
            }
            // if we can't find proper request, we need to return q.end(),
            // so that no command will be scheduled
            head = q.end();
            for (auto itr = q.begin(); itr != q.end(); itr++) {
                bool violate_hit = false;
                if ((!this->ctrl->is_row_hit(itr)) && this->ctrl->is_row_open(itr)) {
                    // so the next instruction to be scheduled is PRE, might violate hit
                    auto begin = itr->addr_vec.begin();
                    // TODO Here it assumes all DRAM standards use PRE to close a row
                    // It's better to make it more general.
                    auto end = begin + int(ctrl->channel->spec->scope[int(T::Command::PRE)]) + 1;
                    vector<int> rowgroup(begin, end); // bank or subarray
                    for (const auto& hit_req_rowgroup : hit_reqs) {
                        if (rowgroup == hit_req_rowgroup) {
                            violate_hit = true;
                            break;
                        }  
                    }
                }
                if (violate_hit) {
                    continue;
                }
                // If it comes here, that means it won't violate any hit request
                if (head == q.end()) {
                    head = itr;
                } else {
                    head = compare[int(Type::FCFSBank)](head, itr);
                }
            }

            return head;
        }
    }

//Compare functions for each memory schedulers
private:
    typedef list<Request>::iterator ReqIter;
    function<ReqIter(ReqIter, ReqIter)> compare[int(Type::MAX)] = {
        // FCFS
        [this] (ReqIter req1, ReqIter req2) {
            // return the request with the oldest (i.e., smallest) arrival time
            if (req1->arrive <= req2->arrive) return req1;
            return req2;},

        // FCFSBank
        [this] (ReqIter req1, ReqIter req2) {
            bool ready1 = this->ctrl->is_ready(req1);
            bool ready2 = this->ctrl->is_ready(req2);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;
        },

        // FRFCFS
        [this] (ReqIter req1, ReqIter req2) {
            // for each request, check if:
            // - the bank is idle (is_ready()), and 
            // - if the requeest is a row hit (is_row_hit())
            bool ready1 = this->ctrl->is_ready(req1) && this->ctrl->is_row_hit(req1);
            bool ready2 = this->ctrl->is_ready(req2) && this->ctrl->is_row_hit(req2);

            // check if one is true and one is false
            if (ready1 ^ ready2) {
                // true is higher priority than false, so return the request that was true
                if (ready1) return req1;
                return req2;
            }

            // if both are true or both are false, break ties by arrival time (smaller = older)
            if (req1->arrive <= req2->arrive) return req1;
            return req2;
        },

        // 18-740: Add your BLISS scheduler comparison here
        // BLISS
        [this] (ReqIter req1, ReqIter req2) {
            // 18-740: ADD CODE BELOW THIS LINE
            //
            // SOME TIPS
            // - Treat the iterator object (ReqIter) like a pointer
            // - To determine which core generated a request, use: req1->coreid
            // - To access a variable inside the controller object, access it
            //     through the this->ctrl pointer (e.g., to access the row_hits
            //     stat in the controller, use: this->ctrl->row_hits)

            bool ready1 = this->ctrl->is_ready(req1) && this->ctrl->is_row_hit(req1);
            bool ready2 = this->ctrl->is_ready(req2) && this->ctrl->is_row_hit(req2);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;
            // 18-740: ADD CODE ABOVE THIS LINE
        },

        // 18-740: Add your Custom scheduler comparison here
        // Custom
        [this] (ReqIter req1, ReqIter req2) {
            // 18-740: ADD CODE BELOW THIS LINE
            bool ready1 = this->ctrl->is_ready(req1) && this->ctrl->is_row_hit(req1);
            bool ready2 = this->ctrl->is_ready(req2) && this->ctrl->is_row_hit(req2);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;
            // 18-740: ADD CODE ABOVE THIS LINE
        }
    };
};


// Row Precharge Policy
template <typename T>
class RowPolicy
{
public:
    Controller<T>* ctrl;

    enum class Type {
        Closed, ClosedAP, Opened, Timeout, MAX
    } type = Type::Timeout;

    int timeout = 50;

    RowPolicy(Controller<T>* ctrl) : ctrl(ctrl) {}

    vector<int> get_victim(typename T::Command cmd)
    {
        return policy[int(type)](cmd);
    }

private:
    function<vector<int>(typename T::Command)> policy[int(Type::MAX)] = {
        // Closed
        [this] (typename T::Command cmd) -> vector<int> {
            for (auto& kv : this->ctrl->rowtable->table) {
                if (!this->ctrl->is_ready(cmd, kv.first))
                    continue;
                return kv.first;
            }
            return vector<int>();},

        // ClosedAP
        [this] (typename T::Command cmd) -> vector<int> {
            for (auto& kv : this->ctrl->rowtable->table) {
                if (!this->ctrl->is_ready(cmd, kv.first))
                    continue;
                return kv.first;
            }
            return vector<int>();},

        // Opened
        [this] (typename T::Command cmd) {
            return vector<int>();},

        // Timeout
        [this] (typename T::Command cmd) -> vector<int> {
            for (auto& kv : this->ctrl->rowtable->table) {
                auto& entry = kv.second;
                if (this->ctrl->clk - entry.timestamp < timeout)
                    continue;
                if (!this->ctrl->is_ready(cmd, kv.first))
                    continue;
                return kv.first;
            }
            return vector<int>();}
    };

};


template <typename T>
class RowTable
{
public:
    Controller<T>* ctrl;

    struct Entry {
        int row;
        int hits;
        long timestamp;
    };

    map<vector<int>, Entry> table;

    RowTable(Controller<T>* ctrl) : ctrl(ctrl) {}

    void update(typename T::Command cmd, const vector<int>& addr_vec, long clk)
    {
        auto begin = addr_vec.begin();
        auto end = begin + int(T::Level::Row);
        vector<int> rowgroup(begin, end); // bank or subarray
        int row = *end;

        T* spec = ctrl->channel->spec;

        if (spec->is_opening(cmd))
            table.insert({rowgroup, {row, 0, clk}});

        if (spec->is_accessing(cmd)) {
            // we are accessing a row -- update its entry
            auto match = table.find(rowgroup);
            assert(match != table.end());
            assert(match->second.row == row);
            match->second.hits++;
            match->second.timestamp = clk;
        } /* accessing */

        if (spec->is_closing(cmd)) {
          // we are closing one or more rows -- remove their entries
          int n_rm = 0;
          int scope;
          if (spec->is_accessing(cmd))
            scope = int(T::Level::Row) - 1; //special condition for RDA and WRA
          else
            scope = int(spec->scope[int(cmd)]);

          for (auto it = table.begin(); it != table.end();) {
            if (equal(begin, begin + scope + 1, it->first.begin())) {
              n_rm++;
              it = table.erase(it);
            }
            else
              it++;
          }

          assert(n_rm > 0);
        } /* closing */
    }

    int get_hits(const vector<int>& addr_vec, const bool to_opened_row = false)
    {
        auto begin = addr_vec.begin();
        auto end = begin + int(T::Level::Row);

        vector<int> rowgroup(begin, end);
        int row = *end;

        auto itr = table.find(rowgroup);
        if (itr == table.end())
            return 0;

        if(!to_opened_row && (itr->second.row != row))
            return 0;

        return itr->second.hits;
    }

    int get_open_row(const vector<int>& addr_vec) {
        auto begin = addr_vec.begin();
        auto end = begin + int(T::Level::Row);

        vector<int> rowgroup(begin, end);

        auto itr = table.find(rowgroup);
        if(itr == table.end())
            return -1;

        return itr->second.row;
    }
};

} /*namespace ramulator*/

#endif /*__SCHEDULER_H*/
