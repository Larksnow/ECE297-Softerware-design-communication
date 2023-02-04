/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cFiles/file.h to edit this template
 */

/* 
 * File:   m1header.h
 * Author: wangh420
 *
 * Created on March 7, 2022, 8:32 PM
 */

#ifndef M1HEADER_H
#define M1HEADER_H

#endif /* M1HEADER_H */
#include <unordered_map>
#include <map>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "globalStruct.h"

extern std::vector<std::vector<StreetSegmentIdx> > intersection_street_segments; //this 2D vector copy intersections and its relevant segments from API
extern std::vector<std::vector<IntersectionIdx>> street_Intersections; //this 2D vector store intersections belongs to a street for all streets 
extern std::vector<StreetSegmentInfo> street_segments_info; //this vector copy all segments' structs from  API
extern std::vector<double> street_segments_length; //this vector store length for all segments
extern std::unordered_map<OSMID, LatLon> latlonFromOSMID; //In this map, given a OSMID(Key) return its LatLon
extern std::unordered_map<StreetIdx,std::vector<IntersectionIdx>> findIntersection; //In this map, given a StreetId(Key) return all intersections(a vector) on that street
extern std::unordered_map<StreetIdx,double> street_length; //In this map, given a StreetId(Key) return its length;
extern std::multimap<std::string, StreetIdx> prefix_of_name; //In this map, given a StreetName(Key) return its StreetId;
extern std::unordered_map<std::string, std::vector<StreetIdx>> street_names;//In this map, given a street name as a key, it will return the streetIdx that street with the name
extern std::vector<double> time_of_streetseg;//this vector take streetsegmentidx as input and output the time require to pass that streetseg
extern std::unordered_map<IntersectionIdx, std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>>> neighbour_intersects;//this unordered map take intersectionidx a input and output all intersection that's near the given idx with a pair of their in-between streetseg
extern std::vector<intersect_path> all_intersect_path;//this vector stores all intersect_path in the order of intersectionidx

extern std::multimap<std::string, IntersectionIdx> prefix_of_inter_name;//In this map, given a Intersection name(Key) return its StreetId;
extern std::vector<IntersectionIdx> findInterIdsFromPartialInterName(std::string inter_prefix);
extern std::vector<intersect_data> intersections_data;///this vector stores all intersect_data in the order of intersectionId
extern std::vector<bool> checked_intersection;//contains the boolean for each intestsection checked information when finding shortest path, to see which intersect is already checked
extern double highest_speedlimit;//contains the value for the highest speed limit in the city
extern int select_count;
extern double latLontoX(LatLon point, double avgLati); 
extern double latLontoY(LatLon point);  
extern double avglatLontoX(LatLon point);
extern double xtoLon(double x);
extern double ytoLat(double y);
extern float max_Lati, min_Lati,avg_Lati;//Holds the value for max/min latitude/longitude and average latitude of the loaded map
extern float max_Lon, min_Lon;
extern std::vector<feature_graphic> feature_graph;//this vector stores all feature_graphic for features with closed area
extern std::vector<intersect_data> intersections_data;//this vector stores all intersect_data in the order of intersectionId
extern std::vector<streetseg_data> streetsegments_data;//this vector stores all streetseg_data in the order of streetsegId
extern std::unordered_map<OSMID, OSMWay*> find_osmbyId;//In this map, given a OSMID as a key return the reference to the OSMWay that holds the id
extern std::multimap<double, int> feature_rank_by_area;//In this map, given a feature area as a key, it will return the index of the feature with the given area in feature_graph
