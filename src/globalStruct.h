/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/class.h to edit this template
 */

/* 
 * File:   gloabalStruct.h
 * Author: guanyux1
 *
 * Created on March 8, 2022, 7:35 PM
 */

#ifndef GLOBALSTRUCT_H
#define GLOBALSTRUCT_H
#include <unordered_map>
#include <map>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m1.h"
struct feature_graphic{
    short r;
    short g;
    short b;
    short transparency;
    std::vector<ezgl::point2d> points;
    double area;
};
struct intersect_data{
    std::string name;
    bool highlight;
    bool select_start;
    bool select_end;
    double x;
    double y;
};
struct streetseg_data{
    std::string name;
    std::vector<double> points_x;
    std::vector<double> points_y;
    std::string type;
    bool oneway;
    float speadLimit;
    bool display;
};

struct path_seg{
    StreetSegmentIdx id;
    std::string instruction;
    std::string direction;
    double speedlimit;
    double length;
    std::string streetname;
    std::string next_streetname;  
};


struct intersect_path{
    double time;
    double under_estimate;
    LatLon position;
    std::vector<StreetSegmentIdx> passed;
    IntersectionIdx id;
    bool operator<(const intersect_path& rhs) const
    {

        return time + under_estimate > rhs.time + rhs.under_estimate;
    }
    intersect_path(IntersectionIdx idx){
        id = idx;
        time = 999999999.0;
        under_estimate = 0;
        position = getIntersectionPosition(idx);
    }
};

#endif /* GLOBALSTRUCT_H */

