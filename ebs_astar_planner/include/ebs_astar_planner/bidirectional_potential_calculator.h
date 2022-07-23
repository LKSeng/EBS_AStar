#ifndef _EBASTAR_BIDIRECTIONAL_POTENTIAL_CALCULATOR_H
#define _EBASTAR_BIDIRECTIONAL_POTENTIAL_CALCULATOR_H

#include <global_planner/potential_calculator.h>
#include <algorithm>

namespace ebs_astar_planner {

class BidirectionalPotentialCalculator : public global_planner::PotentialCalculator {
    public:
        BidirectionalPotentialCalculator(int nx, int ny): PotentialCalculator(nx, ny) {}
        virtual ~BidirectionalPotentialCalculator() {}

        virtual float calculatePotential1(float* potential, unsigned char cost, int n, float prev_potential=-1){
            //the reference code calls such a function... but what does it do?
            //until then, this will just be an inefficient way to all calculatePotential
            return calculatePotential(potential, cost, n, prev_potential);
        }
};

} //end namespace ebs_astar_planner
#endif
