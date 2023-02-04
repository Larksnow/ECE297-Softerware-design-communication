/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/class.cc to edit this template
 */

/* 
 * File:   m4.cpp
 * Author: guanyux1
 * 
 * Created on April 11, 2022, 9:15 p.m.
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
#include "m4.h"
#include <cstdio>
#include <ctime>
#include <thread>
double runTime;
struct Information{
    std::pair<IntersectionIdx, IntersectionIdx> startEnd;
    std::vector<StreetSegmentIdx> Inforpath;
};
bool checkAvailabe(
            std::vector<CourierSubPath>& finalPath);
double minTime;


struct judge{
    IntersectionIdx Inter;
    bool isPickup;
    bool canVisit;
    bool visited;
};

bool legalityTest(std::vector<DeliveryInf> deliveries,
                  std::vector<CourierSubPath> path);
std::vector <CourierSubPath> procedureThreeOptSwap(std::vector<CourierSubPath>& Path, int i, int j, int k);


std::vector<CourierSubPath> threeOpt(
                  
                  std::vector<CourierSubPath>& finalPath);

std::vector <CourierSubPath> procedureTwoOptSwap(std::vector<CourierSubPath>& Path, int i, int k) ;



std::vector<CourierSubPath> twoOpt(
                
                  std::vector<CourierSubPath>& finalPath, std::clock_t start
       );

std::vector<CourierSubPath> opt2(std::vector<CourierSubPath> finalPath, double time);

std::pair<double, std::vector<CourierSubPath>> swapopt2(std::vector<CourierSubPath> finalPath, int i, int j);

double calculateTime(std::vector<CourierSubPath>& finalPath);

std::vector<CourierSubPath> nodeShifting(std::vector<CourierSubPath>& finalPath, double time);

std::pair<double, std::vector<CourierSubPath>> swapnodeShifting(std::vector<CourierSubPath>& finalPath, int i, int j,int time);

IntersectionIdx closestIntersect(LatLon location,const std::vector<IntersectionIdx> others){
    double closestDistance = 999999999.0;
    IntersectionIdx output = -1;
    for(IntersectionIdx i : others){
        double distance = findDistanceBetweenTwoPoints(std::make_pair(location, getIntersectionPosition(i)));
        if(closestDistance > distance){
            closestDistance = distance;
            output = i;
        }
    }
    return output;
}
LatLon midPoints(std::vector<IntersectionIdx> input){
    double lat = 0;
    double lon = 0;
    int count = 0;
    for(int i = 0; i < input.size();i++){
        lat += getIntersectionPosition(input[i]).latitude();
        lon += getIntersectionPosition(input[i]).longitude();
        count += 1;
    }
    if(count != 0){
        lat /= count;
        lon /= count;
        LatLon output = LatLon(lat,lon);
        return output;
    }else{
        LatLon output = LatLon(0,0);
        return output;
    }
}
float global_turn_penalty;

bool relax1(intersect_path* target, intersect_path* from, StreetSegmentIdx path);
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::pair<double, std::vector<StreetSegmentIdx>>>> findDistanceBetweenIntersections(
                  const double turn_penalty,
                  const std::vector<IntersectionIdx>& intersect_ids,
                  const std::vector<IntersectionIdx>& depots);

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::pair<double, std::vector<StreetSegmentIdx>>>> findDistanceBetweenIntersections(                 
                  const double turn_penalty,
                  const std::vector<IntersectionIdx>& intersect_ids,
                  const std::vector<IntersectionIdx>& depots){
    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::pair<double, std::vector<StreetSegmentIdx>>>> output;
    int intersect_ids_size = intersect_ids.size();
    int depots_size = depots.size();
    #pragma omp parallel for
    for(int j = 0; j < intersect_ids_size; j++){
        std::priority_queue<intersect_path> path;
        intersect_path current = all_intersect_path[intersect_ids[j]];
        std::vector<bool> checked_intersection_copy = checked_intersection;
        current.time = 0;
        std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>> neighbour;
        path.push(current);
        IntersectionIdx current_id;
        while(!path.empty()){
            current = path.top();
            current_id = current.id;
            double current_time = current.time;
            std::vector<StreetSegmentIdx> current_passed = current.passed;
            path.pop();
            if(checked_intersection_copy[current_id]){
                continue;
            }
            checked_intersection_copy[current_id] = true;
            bool end = true;
            
            for(int i = 0; i < intersect_ids_size; i++){
                if(current_id == intersect_ids[i]){
                    #pragma omp critical
                    output[intersect_ids[j]][current_id] = std::make_pair(current_time, current_passed);
                }
                if(!checked_intersection_copy[intersect_ids[i]]){
                    end = false;
                }
            }
            
            for(int i = 0; i < depots_size; i++){
                if(current_id == depots[i]){
                    #pragma omp critical
                    output[intersect_ids[j]][current_id] = std::make_pair(current_time, current_passed);
                }
                if(!checked_intersection_copy[depots[i]]){
                    end = false;
                }
//                if(i != j && output[intersect_ids[j]][depots[i]].first == 0){
//                    output[intersect_ids[j]][depots[i]] = std::make_pair(std::numeric_limits<double>::infinity(),empty);
//                }
            }
            if(end){
                break;
            }
            neighbour = neighbour_intersects[current_id];
            
            for(int i  = 0, endl = neighbour.size(); i != endl; i++){
                if(!checked_intersection_copy[neighbour[i].first]){
                    intersect_path next = all_intersect_path[neighbour[i].first];
                    if(relax1(&next, &current, neighbour[i].second)){
                        path.push(next);
                    }
                }
            }      
        }
    }
    #pragma omp parallel for
    for(int j = 0; j < depots_size; j++){
        std::priority_queue<intersect_path> path;
        intersect_path current = all_intersect_path[depots[j]];
        std::vector<bool> checked_intersection_copy = checked_intersection;
        current.time = 0;
        std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>> neighbour;
        path.push(current);
        IntersectionIdx current_id;
        while(!path.empty()){
            current = path.top();
            current_id = current.id;
            double current_time = current.time;
            std::vector<StreetSegmentIdx> current_passed = current.passed;
            path.pop();
            if(checked_intersection_copy[current_id]){
                continue;
            }
            checked_intersection_copy[current_id] = true;
            bool end = true;
           
            for(int i = 0; i < intersect_ids_size; i++){
                if(current_id == intersect_ids[i]){
                    #pragma omp critical
                    output[depots[j]][current_id] = std::make_pair(current_time, current_passed);
                }
                if(!checked_intersection_copy[intersect_ids[i]]){
                    end = false;
                }
//                if(i != j && output[depots[j]][intersect_ids[i]].first == 0){
//                    output[depots[j]][intersect_ids[i]] = std::make_pair(std::numeric_limits<double>::infinity(),empty);
//                }
            }
            if(end){
                break;
            }
            neighbour = neighbour_intersects[current_id];
            
            for(int i  = 0, endl = neighbour.size(); i != endl; i++){
                if(!checked_intersection_copy[neighbour[i].first]){
                    intersect_path next = all_intersect_path[neighbour[i].first];
                    if(relax1(&next, &current, neighbour[i].second)){
                        path.push(next);
                    }
                }
            }       
        }
    }
    return output;
}

//take two intersections, one as target and one as from. If the time using for travel to target from from is less than target's original time, over write it and overwrite the path, then input the in-between streetseg into the path
bool relax1(intersect_path* target, intersect_path* from, StreetSegmentIdx path){
    double time_diff = time_of_streetseg[path];
    double from_time = from->time;
    if(!from->passed.empty() && street_segments_info[from->passed.back()].streetID != street_segments_info[path].streetID){
        time_diff += global_turn_penalty;
    }
    from_time += time_diff;
    if(target->time >(from_time)){ 
        
        target->time = from_time;
        target->passed = from->passed;
        target->passed.push_back(path);
        return true;
    }
    return false;
}

std::vector<DeliveryInf> deliveries1;
std::vector<IntersectionIdx> depots1;
std::vector<IntersectionIdx> all_intersections_1;
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::pair<double, std::vector<StreetSegmentIdx>>>> map;
std::multimap<IntersectionIdx, IntersectionIdx> pickupDropOff_1;
std::vector<CourierSubPath> travelingCourier(
            const float turn_penalty,
            const std::vector<DeliveryInf>& deliveries,
            const std::vector<IntersectionIdx>& depots){

//    std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, std::pair<double, std::vector<StreetSegmentIdx>>>> map;
    //std::vector<std::vector<std::pair<double, std::vector<StreetSegmentIdx>>>> map;
     std::clock_t start;
    start = std::clock();
    IntersectionIdx closestPickUpInter;
    deliveries1 = deliveries;
    depots1 = depots;
    global_turn_penalty = turn_penalty;
    std::vector<StreetSegmentIdx> findPath;
    std::multimap<IntersectionIdx, IntersectionIdx> pickupDropOff;
    std::vector<IntersectionIdx> all_intersections_input;
    std::vector<IntersectionIdx> all_intersections;
    std::vector<CourierSubPath> finalPath;
    deliveries1 = deliveries;
    CourierSubPath path;
    for(int i = 0; i< deliveries.size(); i++){
        pickupDropOff.emplace(deliveries[i].pickUp, deliveries[i].dropOff);
        all_intersections_input.push_back(deliveries[i].pickUp);
        all_intersections_input.push_back(deliveries[i].dropOff);
        all_intersections.push_back(deliveries[i].pickUp);
    }
    all_intersections_1 = all_intersections_input;
    std::multimap<double, Information> closestPickUp;
    pickupDropOff_1 = pickupDropOff;
    map = findDistanceBetweenIntersections(turn_penalty, all_intersections_input, depots);
    std::clock_t mapClock;
    mapClock = std::clock();
    double mapClockTime = (mapClock-start)/(double)CLOCKS_PER_SEC;
 
//    IntersectionIdx  = closestIntersect(midPoints(all_intersections_input),depots);

//    path.start_intersection = start_int;
//    IntersectionIdx next_int = closestIntersect(getIntersectionPosition(start_int), all_intersections);
//    path.end_intersection = next_int;
//    //path.subpath = map[start_int][next_int].second;
//    path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//    if(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//        std::vector<IntersectionIdx> all_intersections_copy = all_intersections;
//        std::vector<IntersectionIdx> depots_copy = depots;
//        while(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//            if(!all_intersections_copy.empty()){
//                all_intersections_copy.erase(std::remove(all_intersections_copy.begin(), all_intersections_copy.end(), path.end_intersection), all_intersections_copy.end());
//                next_int = closestIntersect(getIntersectionPosition(start_int), all_intersections_copy);
//                path.end_intersection = next_int;
//                path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//            }else if(!depots_copy.empty()){
//                std::vector<IntersectionIdx> all_intersections_copy = all_intersections;
//                depots_copy.erase(std::remove(depots_copy.begin(), depots_copy.end(), path.start_intersection), depots_copy.end());
//                start_int = closestIntersect(midPoints(all_intersections_input),depots_copy);
//                path.start_intersection = start_int;
//                next_int = closestIntersect(getIntersectionPosition(start_int), all_intersections);
//                path.end_intersection = next_int;
//                path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//            }else{
//                break;
//            }
//        }
//    }
//    finalPath.push_back(path);
//    IntersectionIdx dropOffLoc;
//    while((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
//        dropOffLoc = (*(pickupDropOff.find(path.end_intersection))).second;
//
//        pickupDropOff.erase (pickupDropOff.find(path.end_intersection));
//        all_intersections.push_back(dropOffLoc);
//    }
//    all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());        
//    while(!all_intersections.empty()){
//        start_int = next_int;
//        next_int = closestIntersect(getIntersectionPosition(start_int), all_intersections);
//        path.start_intersection = start_int;
//        path.end_intersection = next_int;
//        path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//        
//        if(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//            std::vector<IntersectionIdx> all_intersections_copy = all_intersections;
//            while(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//                if(!all_intersections_copy.empty()){
//                    all_intersections_copy.erase(std::remove(all_intersections_copy.begin(), all_intersections_copy.end(), path.end_intersection), all_intersections_copy.end());
//                    next_int = closestIntersect(getIntersectionPosition(start_int), all_intersections_copy);
//                    path.end_intersection = next_int;
//                    path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//                }else{
//                    break;
//                }
//            }
//        }
//           
//        
//        finalPath.push_back(path);
//        if((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
//            while((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
//                dropOffLoc = (*(pickupDropOff.find(path.end_intersection))).second;
//                pickupDropOff.erase (pickupDropOff.find(path.end_intersection));
//                all_intersections.push_back(dropOffLoc);
//            }
//                all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());   
//        }else{ //dropoff
//                all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());        
//        }
//    }
//    start_int = next_int;
//    next_int = closestIntersect(getIntersectionPosition(start_int), depots);
//    path.start_intersection = start_int;
//    path.end_intersection = next_int;
//    path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//    
//    if(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//        std::vector<IntersectionIdx> depots_copy = depots;
//        while(path.subpath.empty() && path.start_intersection!=path.end_intersection){
//            if(!depots_copy.empty()){
//                depots_copy.erase(std::remove(depots_copy.begin(), depots_copy.end(), path.end_intersection), depots_copy.end());
//                next_int = closestIntersect(getIntersectionPosition(start_int), depots_copy);
//                path.end_intersection = next_int;
//                path.subpath = findPathBetweenIntersections(turn_penalty, std::make_pair(start_int,next_int));
//            }else{
//                break;
//            }
//        }
//    }
//    
//    finalPath.push_back(path);
    
    
       
        
//        finalPath = nodeShifting(finalPath,current_time);
//        duration = (std::clock()-start)/(double)CLOCKS_PER_SEC;
//        std::cout<<duration<<std::endl;
//        if(duration>=40.0){
//            break;
////        }
//        if(calculateTime(finalPath) == compare){
//            break;
//        }
        
    
    //std::cout<<duration<<std::endl;
    
    
    Information info;
        //#pragma omp parallel for
    for(int i = 0; i < depots.size(); i++){
        double shorterst_time = std::numeric_limits<double>::infinity();
        for(auto j : pickupDropOff){
            if(map[depots[i]][j.first].first < shorterst_time && map[depots[i]][j.first].second.size()!=0){
                shorterst_time = map[depots[i]][j.first].first;
                closestPickUpInter = j.first;
                findPath = map[depots[i]][j.first].second;
//               
            }
        }
         int x = 0;
        for(x = 0; x<findPath.size(); x++){
        }
//         
        info.Inforpath = findPath;
        info.startEnd.first = depots[i];
        info.startEnd.second = closestPickUpInter;
        closestPickUp.emplace(shorterst_time, info);
    }
    
    info = (*(closestPickUp.begin())).second;
    path.end_intersection = info.startEnd.second;
    path.start_intersection = info.startEnd.first;
    path.subpath = info.Inforpath;

    finalPath.push_back(path);
    IntersectionIdx dropOffLoc;
    while((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
        dropOffLoc = (*(pickupDropOff.find(path.end_intersection))).second;

        pickupDropOff.erase (pickupDropOff.find(path.end_intersection));
        all_intersections.push_back(dropOffLoc);
    }
    all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());        
    
    closestPickUp.clear();
    
    
    while(!all_intersections.empty()){
        double shorterst_time = std::numeric_limits<double>::infinity();
        for(int i = 0; i<all_intersections.size(); i++){
               if(map[path.end_intersection][all_intersections[i]].first < shorterst_time && map[path.end_intersection][all_intersections[i]].second.size()!=0){
                shorterst_time = map[path.end_intersection][all_intersections[i]].first;
                closestPickUpInter = all_intersections[i];

                findPath = map[path.end_intersection][all_intersections[i]].second;
               
               
            }

        }
       
       
        path.start_intersection = path.end_intersection;
        path.end_intersection = closestPickUpInter;
        path.subpath = findPath;
        finalPath.push_back(path);

        if((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
             while((*(pickupDropOff.find(path.end_intersection))).first==path.end_intersection){
                 dropOffLoc = (*(pickupDropOff.find(path.end_intersection))).second;
                 pickupDropOff.erase (pickupDropOff.find(path.end_intersection));
                 all_intersections.push_back(dropOffLoc);
            }
                 all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());   
        }else{ //dropoff
                 all_intersections.erase(std::remove(all_intersections.begin(), all_intersections.end(), path.end_intersection), all_intersections.end());        

        }
        
    }
    
    for(int i = 0; i<depots.size(); i++){
                double shorterst_time = std::numeric_limits<double>::infinity();
        if(map[path.end_intersection][depots[i]].first < shorterst_time && map[path.end_intersection][depots[i]].second.size()!=0){
                shorterst_time = map[path.end_intersection][depots[i]].first;
                closestPickUpInter = depots[i];
                findPath = map[path.end_intersection][depots[i]].second;
        }
    }
       path.start_intersection = path.end_intersection;
       path.end_intersection = closestPickUpInter;
       path.subpath = findPath;
       finalPath.push_back(path);
       
       
        double current_time = calculateTime(finalPath);
         minTime = current_time;
         srand (time(NULL));
         if(finalPath.size()>30){
            while(((runTime-mapClockTime)+(mapClockTime/4))<47.5){
               
            current_time = calculateTime(finalPath);
            finalPath = nodeShifting(finalPath,current_time);
            finalPath = threeOpt(finalPath);
             finalPath = twoOpt(
                   finalPath, start);
            }
            
        }
        
        return finalPath;
}





std::pair<double, std::vector<CourierSubPath>> swapopt2(std::vector<CourierSubPath> finalPath, int i, int j){
    std::vector<CourierSubPath> output = finalPath;
    
    for(int a =0; a < i; a++){
        output.push_back(finalPath[a]);
    }
    for(int a = j-1; a >= i; a--){
        output.push_back(finalPath[a]);
    }
    for(int a = j; a < finalPath.size(); a++){
        output.push_back(finalPath[a]);
    }
    if(checkAvailabe(output)){
        return std::make_pair(calculateTime(output),output);
    }else{
        return std::make_pair(-1.0,output);
    }
}
std::pair<double, std::vector<CourierSubPath>> changeing(std::vector<IntersectionIdx>& ints);
double calculateTime(std::vector<CourierSubPath>& finalPath){
    double output = 0;
    for(CourierSubPath i : finalPath){
        output += map[i.start_intersection][i.end_intersection].first;
    }
    return output;
}
std::vector<CourierSubPath> nodeShifting(std::vector<CourierSubPath>& finalPath, double time){
    int i = std::rand() % (finalPath.size()-2)+1;
    int j =std::rand() % (finalPath.size()-i-1)+i + 1;
    
//    for(int i = 1; i < (finalPath.size())-2; i++){
  //      for(int j = i+1; j < (finalPath.size())-1; j++){
            std::pair<double, std::vector<CourierSubPath>> current = swapnodeShifting(finalPath,i,j,time);
            if(current.first != -1.0 && current.first<time){
                return current.second;
            }
    //    }
    //}
    return finalPath;
}
std::pair<double, std::vector<CourierSubPath>> swapnodeShifting(std::vector<CourierSubPath>& finalPath, int i, int j,int time){
    std::pair<double, std::vector<CourierSubPath>> output;
    CourierSubPath path;
    IntersectionIdx temp;
    std::vector<IntersectionIdx> ints;
    ints.push_back(finalPath[0].start_intersection);
    for(CourierSubPath a : finalPath){
        ints.push_back(a.end_intersection);
    }
    temp = ints[i];
    ints[i] = ints[j];
    ints[j] = temp;
    output = changeing(ints);
    if(output.second.empty()){
        return std::make_pair(time,finalPath);
    }
        

    if(checkAvailabe(output.second)){
        return output;
        
    }else{
        return std::make_pair(-1.0,output.second);
    }
}

std::pair<double, std::vector<CourierSubPath>> changeing(std::vector<IntersectionIdx>& ints){
    double time = 0;
    std::vector<CourierSubPath> output;
    std::vector<CourierSubPath> empty;
    CourierSubPath path;
    for(int i = 0; i < ints.size()-1; i++ ){
        path.start_intersection = ints[i];
        path.end_intersection = ints[i+1];
        path.subpath = map[ints[i]][ints[i+1]].second;
        time += map[ints[i]][ints[i+1]].first;
        if(path.subpath.empty() && path.start_intersection!= path.end_intersection){
            return std::make_pair(0,empty);
        }
        output.push_back(path);
    }
    return std::make_pair(time,output);
}

std::vector<CourierSubPath> twoOpt(
                  std::vector<CourierSubPath>& finalPath, std::clock_t start
){
    std::vector<CourierSubPath> existing_route = finalPath;
    std::vector<CourierSubPath> new_route;
  
    double newtime;
    int a = rand()%32;
    int i = rand() % (finalPath.size()-2-a)+1;
    int k = i+a+1;
            new_route = procedureTwoOptSwap(finalPath, i, k);
              runTime = (std::clock()-start)/(double)CLOCKS_PER_SEC;
              
                //std::cout<<duration<<std::endl;
//                 if(duration>=45){
//                        return existing_route;
//                    }
              bool test = checkAvailabe(
            new_route);
           
            if(test) {
                newtime = calculateTime(new_route);
                if(newtime < minTime){
                existing_route = new_route;
                minTime = newtime;
            }
            }
        
    
    return existing_route;
}

std::vector<CourierSubPath> threeOpt(
                   std::vector<CourierSubPath>& finalPath
){
    std::vector<CourierSubPath> existing_route = finalPath;
    std::vector<CourierSubPath> new_route;
  
    double newtime;
    int a = rand()%10;
    int b = rand()%10;
    int i = rand() % (finalPath.size()-2-a-b)+1;
    int j = i+a+1;
    int k = i+a+b;
            new_route = procedureThreeOptSwap(finalPath, i, j,k);
             
              
                //std::cout<<duration<<std::endl;
//                 if(duration>=45){
//                        return existing_route;
//                    }
              bool test = checkAvailabe(
            new_route);
           
            if(test) {
                newtime = calculateTime(new_route);
                if(newtime < minTime){
                existing_route = new_route;
                minTime = newtime;
            }
            }
        
    
    return existing_route;
}


//

std::vector <CourierSubPath> procedureTwoOptSwap(std::vector<CourierSubPath>& Path, int i, int k) {
    std::vector <IntersectionIdx> swap;
    std::vector <IntersectionIdx> allInter;
    CourierSubPath c;
    std::vector<CourierSubPath> finalPath;
    allInter.push_back(Path[0].start_intersection);
    for(int x = 0; x< Path.size(); x++){
        allInter.push_back(Path[x].end_intersection);
    }
//    for(int x = 0; x<i; x++){
//        swap.push_back(allInter[x]);
//    }
//    for(int x = k-1; x>=i; x--){
//        swap.push_back(allInter[x]);
//    }
//    for(int x = k; x<allInter.size(); x++){
//        swap.push_back(allInter[x]);
//    }
    std::reverse(allInter.begin()+i, allInter.begin()+k);
    for(int x = 0; x<allInter.size()-1; x++){
        c.start_intersection = allInter[x];
        c.end_intersection = allInter[x+1];
        c.subpath = map[allInter[x]][allInter[x+1]].second;
        finalPath.push_back(c);
    }
    return finalPath;
}


std::vector <CourierSubPath> procedureThreeOptSwap(std::vector<CourierSubPath>& Path, int i, int j, int k) {
    std::vector <IntersectionIdx> swap;
    std::vector <IntersectionIdx> allInter;
    CourierSubPath c;
    std::vector<CourierSubPath> finalPath;
    allInter.push_back(Path[0].start_intersection);
    for(int x = 0; x< Path.size(); x++){
        allInter.push_back(Path[x].end_intersection);
    }
    int choose = rand()%7+1;
    if(choose==1){
         std::reverse(allInter.begin()+i, allInter.begin()+j);
    }if(choose==2){
         std::reverse(allInter.begin()+j, allInter.begin()+k);
    }if(choose==3){
         std::reverse(allInter.begin()+i, allInter.begin()+k);
    }if(choose==4){
         std::reverse(allInter.begin()+i, allInter.begin()+k);
         std::reverse(allInter.begin()+i, allInter.begin()+(i+k-j));
         std::reverse(allInter.begin()+(i+k-j), allInter.begin()+k);
    }if(choose==5){
         std::reverse(allInter.begin()+i, allInter.begin()+j);
         std::reverse(allInter.begin()+j, allInter.begin()+k);
    }if(choose==6){
        std::reverse(allInter.begin()+i, allInter.begin()+k);
         std::reverse(allInter.begin()+i, allInter.begin()+(i+k-j));
    }if(choose==7){
          std::reverse(allInter.begin()+i, allInter.begin()+k);
          std::reverse(allInter.begin()+(i+k-j), allInter.begin()+k);
    }

    for(int x = 0; x<allInter.size()-1; x++){
        c.start_intersection = allInter[x];
        c.end_intersection = allInter[x+1];
        c.subpath = map[allInter[x]][allInter[x+1]].second;
        finalPath.push_back(c);
    }
    return finalPath;
}


bool legalityTest(std::vector<DeliveryInf> deliveries,
                  std::vector<CourierSubPath> path){
    std::vector<judge> allInter;
    judge Deter;
  
     for(int i = 0; i<path.size(); i++){
        if(path[i].subpath.size()==0){
            return false;
        }
    }
    path.erase(path.begin());
    path.erase(path.end());
    for(int i = 0; i<deliveries.size(); i++){
        Deter.Inter = deliveries[i].pickUp;
        Deter.visited = false;
        Deter.canVisit = true;
        Deter.isPickup = true;
        allInter.push_back(Deter);
        Deter.Inter = deliveries[i].dropOff;
        Deter.visited = false;
        Deter.canVisit = false;
        Deter.isPickup = false;
        allInter.push_back(Deter);
    }
    bool test = false;
    for (int i = 0; i<allInter.size(); i++){
        if(allInter[i].Inter== path[0].start_intersection && allInter[i].canVisit==true && allInter[i].isPickup==true){
            test=true;
            allInter[i].canVisit=false;
            allInter[i].visited=true;
            allInter[i+1].canVisit = true;
        }
    }
    if(!test){
        return false;
    }
   
    
    for (int j = 0; j<path.size(); j++){
        test = false;
    for (int i = 0; i<allInter.size(); i++){
        if(allInter[i].Inter== path[j].end_intersection && allInter[i].canVisit==true && allInter[i].visited==false){
        test=true;
        if(allInter[i].isPickup){
            allInter[i].canVisit=false;
            allInter[i].visited=true;
            allInter[i+1].canVisit = true;
        }else{
            allInter[i].canVisit=false;
            allInter[i].visited=true;
        }
    }
}
    if(!test){
        return false;
    }
    }
    
    
    return true;
}



bool checkAvailabe(
            std::vector<CourierSubPath>& finalPath){
    std::vector<bool> deliveries_picked_up(deliveries1.size(), false);
    std::vector<bool> deliveries_dropped_off(deliveries1.size(), false);

    std::multimap<IntersectionIdx,int> intersections_to_pick_up;
    std::multimap<IntersectionIdx,int> intersections_to_drop_off;

    for(int i = 0; i < deliveries1.size(); ++i) {
        IntersectionIdx pickup_int = deliveries1[i].pickUp;
        IntersectionIdx dropoff_int = deliveries1[i].dropOff;

        intersections_to_pick_up.insert(std::make_pair(pickup_int, i));
        intersections_to_drop_off.insert(std::make_pair(dropoff_int, i));
    }
    for (int i = 0; i < finalPath.size(); i++) {
        CourierSubPath subpath = finalPath[i];
        IntersectionIdx start_intersection = subpath.start_intersection;
        IntersectionIdx end_intersection = subpath.end_intersection;
        if (subpath.subpath.empty() && subpath.start_intersection != subpath.end_intersection) {
            return false;
        }
        if (i > 0){
            if (start_intersection != finalPath[i - 1].end_intersection){
                return false;
            }
            if (intersections_to_pick_up.find(start_intersection) != intersections_to_pick_up.end()){
                auto range = intersections_to_pick_up.equal_range(start_intersection);
                for(auto j = range.first; j != range.second; ++j) {
                    int delivery_idx = j->second;
                    deliveries_picked_up[delivery_idx] = true;
                }
            }
            if(intersections_to_drop_off.find(start_intersection) != intersections_to_drop_off.end()) {
                auto range = intersections_to_drop_off.equal_range(start_intersection);
                for(auto j = range.first; j != range.second; ++j) {
                    int delivery_idx = j->second;
                    if(deliveries_picked_up[delivery_idx]) {
                        deliveries_dropped_off[delivery_idx] = true;
                    }
                }
            }
        }
    }
    for(int i = 0; i < deliveries1.size(); i++){
        if(!deliveries_picked_up[i]){
            return false;
        }
        if(!deliveries_dropped_off[i]){
            return false;
        }
    }
    
    
//    std::vector<bool> checked_intersection_copy = checked_intersection;
//    for(auto i : pickupDropOff){
//        checked_intersection_copy[i.first]=true;
//    }
//    for(int i = 1; i < finalPath.size(); i++){
//        if(checked_intersection_copy[finalPath[i].start_intersection]){
//            while((*(pickupDropOff.find(finalPath[i].start_intersection))).first==finalPath[i].start_intersection){
//                checked_intersection_copy[(*(pickupDropOff.find(finalPath[i].start_intersection))).second] = true;
//                pickupDropOff.erase (pickupDropOff.find(finalPath[i].start_intersection));
//            }
//        }else{
//            return false;
//        }
//    }
//    for(IntersectionIdx i : all_intersections_1){
//        if(!checked_intersection_copy[i]){
//            return false;
//        }
//    }
    return true;
}
