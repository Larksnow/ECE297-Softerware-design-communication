/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/class.cc to edit this template
 */

/* 
 * File:   m3.cpp
 * Author: guanyux1
 * 
 * Created on March 29, 2022, 12:30 PM
 */

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1header.h"
#include "globalStruct.h"
#include <sstream>
#include <math.h>
#include <queue>
#include <deque>
#include <vector>
//
//
//// Returns the time required to travel along the path specified, in seconds.
//// The path is given as a vector of street segment ids, and this function can
//// assume the vector either forms a legal path or has size == 0.  The travel
//// time is the sum of the length/speed-limit of each street segment, plus the
//// given turn_penalty (in seconds) per turn implied by the path.  If there is
//// no turn, then there is no penalty. Note that whenever the street id changes
//// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double computePathTravelTime(const double turn_penalty,
                                const std::vector<StreetSegmentIdx>& path){
    double pathTime = 0;
    for (int i=0; i<path.size(); i++){
        pathTime += time_of_streetseg[path[i]];
        if (i<path.size()-1){
            if (street_segments_info[path[i]].streetID!=street_segments_info[path[i+1]].streetID){
                pathTime +=turn_penalty;
            }
        }
    }
    return pathTime;
}


// Returns a path (route) between the start intersection (1st object in intersect_ids pair)
// and the destination intersection(2nd object in intersect_ids pair),
// if one exists. This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds).  If no path exists, this routine
// returns an empty (size == 0) vector.  If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the destination intersection.


void relax(double turn_penalty, intersect_path* target, intersect_path* from, StreetSegmentIdx path);

std::vector<StreetSegmentIdx> findPathBetweenIntersections(
                  const double turn_penalty,
                  const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids){
   
    std::priority_queue<intersect_path> path;
    intersect_path current = all_intersect_path[intersect_ids.first];
    std::vector<bool> checked_intersection_copy = checked_intersection;
    current.time = 0;
    std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>> neighbour;
    path.push(current);
    IntersectionIdx destination = intersect_ids.second;
    LatLon final_position = all_intersect_path[intersect_ids.second].position;
    IntersectionIdx current_id;
    while(!path.empty()){
        current = path.top();
        current_id = current.id;
        
        if(current_id == destination){
            return current.passed;
        }
        path.pop();
        if(checked_intersection_copy[current_id]){
            continue;
        }
        checked_intersection_copy[current_id] = true;
        
        neighbour = neighbour_intersects[current_id];
        for(int i = 0; i < neighbour.size(); ++i){
            intersect_path next = all_intersect_path[neighbour[i].first];
            
            if(!checked_intersection_copy[next.id]){
                std::pair<LatLon, LatLon> points = std::make_pair(next.position,final_position);
                next.under_estimate = (findDistanceBetweenTwoPoints(points)/highest_speedlimit);
                if(next.under_estimate > turn_penalty){
                    next.under_estimate -= turn_penalty;
                }else{
                    next.under_estimate = 0.0;
                }
                relax(turn_penalty, &next, &current, neighbour[i].second);
                path.push(next);
            }
        }        
    }
    std::vector<StreetSegmentIdx> didnotReach;
    return didnotReach;
}
//take two intersections, one as target and one as from. If the time using for travel to target from from is less than target's original time, over write it and overwrite the path, then input the in-between streetseg into the path
void relax(double turn_penalty, intersect_path* target, intersect_path* from, StreetSegmentIdx path){
    double time_diff = time_of_streetseg[path];
    double from_time = from->time;
    if(!from->passed.empty() && street_segments_info[from->passed.back()].streetID != street_segments_info[path].streetID){
        time_diff += turn_penalty;
    }
    from_time += time_diff;
    if(target->time >(from_time)){ 
        
        target->time = from_time;
        target->passed = from->passed;
        target->passed.push_back(path);
    }
}

