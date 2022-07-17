/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<ebs_astar_planner/ebastar.h>
#include<costmap_2d/cost_values.h>

namespace ebs_astar_planner {

AStarExpansion::AStarExpansion(global_planner::PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    queue2_.clear();

    //get index of start point in the 1-d array
    int start_i = toIndex(start_x, start_y);
    //add start point to back of queue 1
    queue_.push_back(Index(start_i, 0));
    //get index of end point in the 1-d array
    int end_i = toIndex(end_x, end_y);
    //add end point to back of queue 2
    queue2_.push_back(Index(end_i, 1e10));

    //initialise entire map with potential of 1.0e10
    std::fill(potential, potential + ns_, POT_HIGH);
    //set potential of start point to be 0
    potential[start_i] = 0;
    //set potential of end point to be 1e10 (big?, for convenience)
    potential[end_i] = 1e10;

    int goal1_i = toIndex(end_x, end_y);
    int goal2_i = toIndex(start_x, start_y);
    //int goal2_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top1 = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1()); //priority queue
        queue_.pop_back();
        Index top2 = queue2_[0];
        std::pop_heap(queue2_.begin(), queue2_.end(), less1()); //priority queue
        queue2_.pop_back();

        //Begin search from start to end point
        int i = top1.i;
        if (i == goal1_i)
            return true;
        
        //Begin search from end to start point
        int j = top2.i;
        if (j == goal2_i)
        {
            //clear the queue
            while(!queue_.empty()){
                queue_.pop_back();
            }
            while(!queue2_.empty()){
                //get first element in heap
                Index out = queue2_[0];
                std::pop_heap(queue2_.begin(), queue2_.end(), greater1());
                queue2_.pop_back();
                //reverse order
                queue_.push_back(out);
                std::push_heap(queue_.begin(), queue_.end(), less1());
            }
            return true;
        }

        //Bi-directional search (meets at next_i)
        float before_cost = top1.cost; //the original cost of start point
        int i_next[4]={i+1,i-1,i+nx_,i-nx_};//points around i
        int j_next[4]={j+1,j-1,j+nx_,j-nx_};//points around j

        for(int i_dir=0;i_dir<4;i_dir++){
            for(int j_dir=0;j_dir<4;j_dir++){
                if(i_next[i_dir] == j_next[j_dir]){
                    keep_queue_(before_cost,i_next[i_dir]);
                    return true;
                }
            }
        }

        //forward search
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);
        //backward search
        add1(costs, potential, potential[j], j + 1, start_x, start_y);
        add1(costs, potential, potential[j], j - 1, start_x, start_y);
        add1(costs, potential, potential[j], j + nx_, start_x, start_y);
        add1(costs, potential, potential[j], j - nx_, start_x, start_y);

        cycle++;
    }

    return false;
}

//add points from start point
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    //don't select boundary points
    if (next_i < 0 || next_i >= ns_)
        return;

    //don't select obstacle points
    if (potential[next_i] < POT_HIGH)
        return;

    //don't select point if cost is greater then LETHAL or cost is unknown on costmap
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION)) 
        return;

    //If the above is not the case, then the cost of the node with the smallest estimated cost among the adjacent nodes of the next node plus the current cost is taken as the cost of the next point
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    /*potential[next_i] = p_calc_->calculatePotential1(potential, - costs[next_i] - neutral_cost_, next_i, prev_potential);*/
    int x = next_i % nx_, y = next_i / nx_; //x and y coordinates
    float distance = abs(end_x - x) + abs(end_y - y); //Manhattan distance

    //Push the next waypoint and its estimated total cost (traveled cost and estimated cost)
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    //The priority queue after the push is sorted from large to small, that is, the current forward path
    std::push_heap(queue_.begin(), queue_.end(), greater1()); 
}

//add points from end points
void AStarExpansion::add1(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    //don't select boundary points
    if (next_i < 0 || next_i >= ns_)
        return;

    //don't select obstacle points
    if (potential[next_i] < POT_HIGH)
        return;

    //don't select point if cost is greater then LETHAL or cost is unknown on costmap
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION)) 
        return;

    //If the above is not the case, then the cost of the node with the smallest estimated cost among the adjacent nodes of the next node plus the current cost is taken as the cost of the next point
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    /*potential[next_i] = p_calc_->calculatePotential1(potential, - costs[next_i] - neutral_cost_, next_i, prev_potential);*/
    int x = next_i % nx_, y = next_i / nx_; //x and y coordinates
    float distance = abs(end_x - x) + abs(end_y - y); //Manhattan distance

    //Push the next waypoint and its estimated total cost (traveled cost and estimated cost)
    queue2_.push_back(Index(next_i, potential[next_i] - distance * neutral_cost_));
    //The priority queue after the push is sorted from large to small, that is, the current forward path
    std::push_heap(queue2_.begin(), queue2_.end(), less1());
}

//maintains integrity of the queue
void AStarExpansion::keep_queue_(float before_cost,int i){
            int cost_add = 1;
            //adds one to cost
            queue_.push_back(Index(i,before_cost + 1));
            
            while(!queue2_.empty()){
                cost_add++;
                //get first element in the heap
                Index out = queue2_[0];
                std::pop_heap(queue2_.begin(), queue2_.end(), greater1());
                queue2_.pop_back();
                //put into heap
                queue_.push_back(Index(out.i ,before_cost + cost_add));
                std::push_heap(queue_.begin(), queue_.end(), greater1());
            }
}

} //end namespace ebs_astar_planner
