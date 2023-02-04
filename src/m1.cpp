/*
 * Copyright 2022 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <math.h>
#include <string.h>
#include <cmath>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include<cstdlib>
#include<vector>
#include <unordered_map>
#include <set>
#include <iterator>
#include <map>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "globalStruct.h"
#include "m3.h"
#include "m4.h"
// Speed Requirement:
//
// High: this function must be very fast, and is likely to require creation of
// data structures that allow you to rapidly return the right data rather than
// always going to the StreetsDatabaseAPI or OSMDatabaseAPI
//
// Moderate: this function is speed tested, but a more straightforward
// implementation (e.g. just calling the proper streetsDatabaseAPI or
// OSMDatabaseAPI functions) should pass the speed tests.
//
// None: this function is not speed tested.

//global variables here

double highest_speedlimit = 0;//contains the value for the highest speed limit in the city
std::vector<bool> checked_intersection;//contains the boolean for each intestsection checked information when finding shortest path, to see which intersect is already checked
std::vector<intersect_path> all_intersect_path;//this vector stores all intersect_path in the order of intersectionidx
std::vector<double> time_of_streetseg;//this vector take streetsegmentidx as input and output the time require to pass that streetseg
std::unordered_map<IntersectionIdx, std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>>> neighbour_intersects;//this unordered map take intersectionidx a input and output all intersection that's near the given idx with a pair of their in-between streetseg
std::unordered_map<OSMID,const OSMWay*> find_osmbyId;//In this map, given a OSMID as a key return the reference to the OSMWay that holds the id
std::vector<streetseg_data> streetsegments_data;//this vector stores all streetseg_data in the order of streetsegId
std::vector<intersect_data> intersections_data;//this vector stores all intersect_data in the order of intersectionId
std::vector<feature_graphic> feature_graph;//this vector stores all feature_graphic for features with closed area
std::vector<std::vector<StreetSegmentIdx> > intersection_street_segments; //this 2D vector copy intersections and its relevant segments from API
std::vector<std::vector<IntersectionIdx>> street_Intersections; //this 2D vector store intersections belongs to a street for all streets 
std::vector<StreetSegmentInfo> street_segments_info; //this vector copy all segments' structs from  API
std::vector<double> street_segments_length; //this vector store length for all segments
std::unordered_map<OSMID, LatLon> latlonFromOSMID; //In this map, given a OSMID(Key) return its LatLon
std::unordered_map<StreetIdx,std::vector<IntersectionIdx>> findIntersection; //In this map, given a StreetId(Key) return all intersections(a vector) on that street
std::unordered_map<StreetIdx,double> street_length; //In this map, given a StreetId(Key) return its length;
std::multimap<std::string, StreetIdx> prefix_of_name; //In this map, given a StreetName(Key) return its StreetId;
std::multimap<std::string, IntersectionIdx> prefix_of_inter_name; //In this map, given a Intersection name(Key) return its StreetId;
std::unordered_map<std::string, std::vector<StreetIdx>> street_names;//In this map, given a street name as a key, it will return the streetIdx that street with the name
std::multimap<double, int> feature_rank_by_area;//In this map, given a feature area as a key, it will return the index of the feature with the given area in feature_graph
float max_Lati, min_Lati,avg_Lati;//Holds the value for max/min latitude/longitude and average latitude of the loaded map
float max_Lon, min_Lon;
int select_count = 0;//count clicked intersections as start/end points
//helper funcions here
std::string streetOSM(std::string fileName);// Transfer StreetDatabase file path into OSMDatabase file path to load OMSDatabase
std::string streetOSM(std::string fileName) {
    int place = fileName.find(".");
    std::string result = fileName.substr(0,place);
    result += ".osm.bin";
    return result;
}

bool Equal(std::string inputOne, std::string inputTwo);                         //use for compare two inputs are equal prefix without space/capital letter
bool Equal(std::string inputOne, std::string inputTwo) { 
    std::string temp = inputOne.substr(0,inputTwo.length());                    //make inputone the same length as inputtwo, therefore they can compare directly
    if(temp == inputTwo){
        return true;
    }
    return false;
}
double latLontoX(LatLon point, double avgLati);                                 //get the x position for a given latlon point and its average latitude value
double latLontoX(LatLon point, double avgLati) {
    double Latavg=kDegreeToRadian*(avgLati);
    double result = kEarthRadiusInMeters*point.longitude()*kDegreeToRadian*cos(Latavg);
    return result;

}
double latLontoY(LatLon point);                                                 //get the y position for a given latlon point
double latLontoY(LatLon point) {
    double result = kEarthRadiusInMeters*point.latitude()*kDegreeToRadian;
    return result;
}
/* get the x position for a given latlon point without require of average latitude, the map's average latitude will be used*/
double avglatLontoX(LatLon point);
double avglatLontoX(LatLon point){
    return latLontoX(point, avg_Lati);
}
/*get the longitude value for the input x value*/
double xtoLon(double x);
double xtoLon(double x){
    double Latavg=kDegreeToRadian*(avg_Lati);
    double lon = x / (kEarthRadiusInMeters*kDegreeToRadian*cos(Latavg));
    return lon;
}
/*get the latitude value for the input y value*/
double ytoLat(double y);
double ytoLat(double y){
    double lat = y / (kEarthRadiusInMeters*kDegreeToRadian);
    return lat;
}
std::vector<IntersectionIdx> findInterIdsFromPartialInterName(std::string inter_prefix);


//higher_level functions here
/*Define all global variable*/
bool loadMap(std::string map_streets_database_filename) {
    std::string osmInput = streetOSM(map_streets_database_filename);
    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename)&&loadOSMDatabaseBIN(osmInput); //Indicates whether the map has loaded successfully
    std::cout << "loadMap: " << map_streets_database_filename << std::endl;
	max_Lati = -90;
   	min_Lati = 90;
        max_Lon = -180;
   	min_Lon = 180;
    if(load_successful) {
        intersection_street_segments.resize(getNumIntersections()); 
        for(int intersection=0; intersection<getNumIntersections(); ++intersection) { //This loop initialize all containers about intersections 
            double intersection_lat = getIntersectionPosition(intersection).latitude();
            double intersection_lon = getIntersectionPosition(intersection).longitude();
            if(intersection_lat > max_Lati){
                max_Lati = intersection_lat;
            }
            if(intersection_lat < min_Lati){
                min_Lati = intersection_lat;
            }
            if(intersection_lon > max_Lon){
                max_Lon = intersection_lon;
            }
            if(intersection_lon < min_Lon){
                min_Lon = intersection_lon;
            }
			for(int i=0; i<getNumIntersectionStreetSegment(intersection); ++i) {       
				int ss_id=getIntersectionStreetSegment(i,intersection);                         
				intersection_street_segments[intersection].push_back(ss_id);             
            }
	}
	avg_Lati = (min_Lati + max_Lati)/2;
        for(int intersection=0; intersection<getNumIntersections(); ++intersection) { //This loop initialize all containers about intersections 
            intersect_path each_intersect = intersect_path(intersection);

            std::string name = getIntersectionName(intersection);
            all_intersect_path.push_back(each_intersect);
            checked_intersection.push_back(false);
            intersect_data current;
            select_count = 0;
            current.select_end = false;
            current.select_start = false;
            current.highlight = false;
            current.name = name;
            current.x = avglatLontoX(getIntersectionPosition(intersection));
            current.y = latLontoY(getIntersectionPosition(intersection));
            intersections_data.push_back(current);
            int number_seg_of_intersection = getNumIntersectionStreetSegment(intersection);
            for(int i = 0; i < number_seg_of_intersection; ++i){
                StreetSegmentIdx inbetween_streetseg = getIntersectionStreetSegment(i,intersection);
                StreetSegmentInfo inbetween_streetseginfo = getStreetSegmentInfo(inbetween_streetseg);
                IntersectionIdx intersect_1 = inbetween_streetseginfo.from;
                IntersectionIdx intersect_2 = inbetween_streetseginfo.to;
                std::pair<IntersectionIdx, StreetSegmentIdx> streetseg_Between;
                if(intersect_1 != intersection && !inbetween_streetseginfo.oneWay){
                    streetseg_Between = std::make_pair(intersect_1,inbetween_streetseg);
                    neighbour_intersects[intersection].push_back(streetseg_Between);
                }else if(intersect_2 != intersection) {
                    streetseg_Between = std::make_pair(intersect_2,inbetween_streetseg);
                    neighbour_intersects[intersection].push_back(streetseg_Between);
                }
            }
            name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
            std::transform(name.begin(),name.end(),name.begin(),::tolower);
            if(intersection_street_segments[intersection].size() > 2){
                prefix_of_inter_name.emplace(name,intersection);
            } 
        }
        for(int idx = 0; idx < getNumberOfWays(); ++idx){
            find_osmbyId[getWayByIndex(idx)->id()] = getWayByIndex(idx);
        }
         
	for(int i=0; i<getNumStreetSegments(); ++i) { //This loop initialize street segment infor container      
            street_segments_info.push_back(getStreetSegmentInfo(i));
           
        }
	for(int j=0; j<street_segments_info.size(); ++j) { //This loop initialize the rest containers about segments
            if(street_segments_info[j].speedLimit > highest_speedlimit){
                highest_speedlimit = street_segments_info[j].speedLimit;
            }
            streetseg_data current;
            current.name = getStreetName(street_segments_info[j].streetID);
            current.oneway = street_segments_info[j].oneWay;
            current.points_x.push_back(avglatLontoX(getIntersectionPosition(street_segments_info[j].from)));
            current.points_y.push_back(latLontoY(getIntersectionPosition(street_segments_info[j].from)));
            for(int k = 0; k < street_segments_info[j].numCurvePoints; ++k){
                current.points_x.push_back(avglatLontoX(getStreetSegmentCurvePoint(k,j)));
                current.points_y.push_back(latLontoY(getStreetSegmentCurvePoint(k,j)));
            }
            current.points_x.push_back(avglatLontoX(getIntersectionPosition(street_segments_info[j].to)));
            current.points_y.push_back(latLontoY(getIntersectionPosition(street_segments_info[j].to)));
            current.speadLimit = street_segments_info[j].speedLimit;
            const OSMWay* ways = find_osmbyId[street_segments_info[j].wayOSMID];
            std::string key,value;
            if(getTagCount(ways) >= 2){
                std::tie(key,value) = getTagPair(ways,1);
            }else{
                std::tie(key,value) = getTagPair(ways,0);
            }
            current.type = value;
            streetsegments_data.push_back(current);
            street_segments_length.push_back(findStreetSegmentLength(j));
            
            StreetIdx streetid = street_segments_info[j].streetID;
            findIntersection[streetid].push_back(street_segments_info[j].from);
            findIntersection[streetid].push_back(street_segments_info[j].to);    
            
            street_length[streetid] += street_segments_length.at(j);    
            time_of_streetseg.push_back(findStreetSegmentTravelTime(j));
            
	}
        for(int k = 0; k < getNumStreets(); k++) { //This loop initialize all containers about streets
            std::string name = getStreetName(k);                          
            street_names[name].push_back(k);
            name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
            std::transform(name.begin(),name.end(),name.begin(),::tolower);
            prefix_of_name.emplace(name,k);
            
            street_Intersections.push_back(findIntersectionsOfStreet(k));
        }
        for(int l = 0; l < getNumberOfNodes(); l++){ //This loop initialize all containers about nodes 
            latlonFromOSMID[getNodeByIndex(l)->id()] = getNodeCoords(getNodeByIndex(l));
        }
        int count = 0;
        for(int feature_id = 0; feature_id < getNumFeatures(); feature_id++){
            int numOfPoints = getNumFeaturePoints(feature_id);
            feature_graphic current_feature;
            
            if (!(numOfPoints < 3 || getFeaturePoint(0, feature_id).longitude() != getFeaturePoint(numOfPoints - 1, feature_id).longitude() ||
                    getFeaturePoint(0, feature_id).latitude() != getFeaturePoint(numOfPoints - 1, feature_id).latitude())) { //check if the feature is a closed area, if not, return 0
                double feature_x, feature_y;
                for (int i = 0; i < getNumFeaturePoints(feature_id); i++) {
                    feature_x = avglatLontoX(getFeaturePoint(i, feature_id));
                    feature_y = latLontoY(getFeaturePoint(i, feature_id));
                    ezgl::point2d pair_points = ezgl::point2d(feature_x, feature_y);
                    current_feature.points.push_back(pair_points);
                }
                if (getFeatureType(feature_id) == PARK) {
                    current_feature.r = 172;
                    current_feature.g = 219;
                    current_feature.b = 184;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == BEACH) {
                    current_feature.r = 254;
                    current_feature.g = 239;
                    current_feature.b = 195;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == LAKE) {
                    current_feature.r = 156;
                    current_feature.g = 192;
                    current_feature.b = 249;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == RIVER) {
                    current_feature.r = 156;
                    current_feature.g = 192;
                    current_feature.b = 249;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == ISLAND) {
                    current_feature.r = 158;
                    current_feature.g = 214;
                    current_feature.b = 173;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == BUILDING) {
                    current_feature.r = 150;
                    current_feature.g = 150;
                    current_feature.b = 150;
                    current_feature.transparency = 255/2;
                } else if (getFeatureType(feature_id) == GREENSPACE) {
                    current_feature.r = 172;
                    current_feature.g = 219;
                    current_feature.b = 184;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == GOLFCOURSE) {
                    current_feature.r = 168;
                    current_feature.g = 218;
                    current_feature.b = 181;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == STREAM) {
                    current_feature.r = 175;
                    current_feature.g = 238;
                    current_feature.b = 238;
                    current_feature.transparency = 255;
                } else if (getFeatureType(feature_id) == GLACIER) {
                    current_feature.r = 240;
                    current_feature.g = 255;
                    current_feature.b = 255;
                    current_feature.transparency = 255;
                }
                current_feature.area = findFeatureArea(feature_id);
                feature_rank_by_area.emplace(current_feature.area,count);
                feature_graph.push_back(current_feature);
                count += 1;
            }
        }
    }
//        std::string test=getIntersectionName(1123);
//        std::cout<<test<<std::endl;
//    std::pair<IntersectionIdx, IntersectionIdx> test_two_point = std::make_pair(26612, 28232);
//    std::vector<StreetSegmentIdx> test_1 = findPathBetweenIntersections(3.0,test_two_point);
//    for(int i : test_1){
//        std::cout << i << std::endl;
//    }  
//      std::vector<IntersectionIdx> test2;
//      test2 = findInterIdsFromPartialInterName("young");
//      for (int i = 0; i<test2.size(); i++){
//          std::cout<<test2[i]<<std::endl;
//      }
//    std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(30.00000000000000000, std::make_pair(26612, 40220));
//    for(auto i : path){
//        std::cout << i << std::endl;
//    }
//    double used_time = computePathTravelTime(15.00000000000000000,path);
//    std::cout << "output time" << used_time << std::endl;
//    std::cout << "length" << path.size() << std::endl;
////    std::cout << "speed" << highest_speedlimit << std::endl;
//        std::vector<DeliveryInf> deliveries;
//        std::vector<IntersectionIdx> depots;
//        std::vector<CourierSubPath> result_path;
//        float turn_penalty;
//         deliveries = {DeliveryInf(40220, 37954), DeliveryInf(90705, 82446), DeliveryInf(29107, 44932), DeliveryInf(28232, 60062), DeliveryInf(28232, 82070), DeliveryInf(45899, 26959), DeliveryInf(28232, 120267), DeliveryInf(90705, 120353)};
//        depots = {26612, 45787, 77377};
//        turn_penalty = 30.000000000;
//        std::cout<<"1"<<std::endl;
//        result_path = travelingCourier(turn_penalty, deliveries, depots);
//        std::cout<<"1"<<std::endl;
//        for (int i = 0; i< result_path.size(); i++){
//            if(result_path[i].subpath.size() == 0){
//                std::cout<<result_path[i].start_intersection<<std::endl;
//                std::cout<<result_path[i].end_intersection<<std::endl;
//            }
//
//        }
//    
    return load_successful;
}


// Close the map (if loaded)
// Speed Requirement --> moderate
void closeMap() {
    std::cout << "closed map" << std::endl;
    intersection_street_segments.clear(); //clear all containers for next load
    street_segments_info.clear();
    street_segments_length.clear();
    street_length.clear();
    prefix_of_name.clear();
    prefix_of_inter_name.clear();
    latlonFromOSMID.clear();
    findIntersection.clear();
    streetsegments_data.clear();
    intersections_data.clear();
    feature_graph.clear();
    find_osmbyId.clear();
    street_names.clear();
    feature_rank_by_area.clear();
    time_of_streetseg.clear();
    all_intersect_path.clear();
    neighbour_intersects.clear();
    checked_intersection.clear();
    closeStreetDatabase(); //close all database for next load
    closeOSMDatabase();
}


// Returns the distance between two (lattitude,longitude) coordinates in meters
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> points) {
    LatLon firstpoint=points.first;  //extract two LatLons form pairs
    LatLon secondpoint=points.second;
    double Latavg=(0.5*kDegreeToRadian*(firstpoint.latitude()+secondpoint.latitude()));    //compute averageLatitude for simpler calculation
    double firstx=kEarthRadiusInMeters*firstpoint.longitude()*kDegreeToRadian*cos(Latavg); //compute x,y coordinates for each point for simpler calculation
    double firsty=kEarthRadiusInMeters*firstpoint.latitude()*kDegreeToRadian;
    double secondx=kEarthRadiusInMeters*secondpoint.longitude()*kDegreeToRadian*cos(Latavg);
    double secondy=kEarthRadiusInMeters*secondpoint.latitude()*kDegreeToRadian;
    return sqrt((secondy-firsty)*(secondy-firsty)+(secondx-firstx)*(secondx-firstx)); //calculate distance based on algorism
}

double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {
    auto segmentinfo=street_segments_info[street_segment_id]; //create a struct to load information inside the function for calculation
    double result=0;
    std::pair<LatLon, LatLon> points; // this pair holds two points' LatLon whose distance is calculating currently
    if(segmentinfo.numCurvePoints==0) {
	points.first=getIntersectionPosition(segmentinfo.from); //if there are no curves, segment distance will be distance between two intersections
	points.second=getIntersectionPosition(segmentinfo.to);
	result+=findDistanceBetweenTwoPoints(points);
    } 
    else {
	points.first=getIntersectionPosition(segmentinfo.from); //calculate distance from head intersection to first curve point
	points.second=getStreetSegmentCurvePoint(0, street_segment_id);
	result+=findDistanceBetweenTwoPoints(points);
	for(int pointNum=0; pointNum<segmentinfo.numCurvePoints-1; ++pointNum) {
            points.first=getStreetSegmentCurvePoint(pointNum,street_segment_id); // calculate distance between two curve points
            points.second=getStreetSegmentCurvePoint(pointNum+1, street_segment_id); // repeat untill the last curve point
            result+=findDistanceBetweenTwoPoints(points);
	}
	points.first=getStreetSegmentCurvePoint(segmentinfo.numCurvePoints-1,street_segment_id); //calculate distance from last curve point to tail intersection
	points.second=getIntersectionPosition(segmentinfo.to);
	result+=findDistanceBetweenTwoPoints(points);
	}
    return result;
}

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    double result;
    double speed_limit=street_segments_info[street_segment_id].speedLimit; //read speed limit data from segment information vector
    double distance=street_segments_length[street_segment_id]; //read segment's length from segment length vector
    if (speed_limit!=0){
        result=distance/speed_limit; //calculate based on algorism
    }else{
        result = 99999999.0;
    }
    return result;
}

bool intersectionsAreDirectlyConnected(std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
    IntersectionIdx firstpoint = intersection_ids.first; //break out the pair to get two intersections' id for next checking
    IntersectionIdx secondpoint = intersection_ids.second;
    bool connect=false; //return false if no street segment matches
	for(int i=0; i<intersection_street_segments[firstpoint].size(); ++i) { //try to match among  all street segments these two intersections have
		for(int j=0; j<intersection_street_segments[secondpoint].size(); ++j) { //by checking if they share one street segment id
			if(intersection_street_segments[secondpoint][j]==intersection_street_segments[firstpoint][i]) {//find any shared street segment id first
                            if(street_segments_info[intersection_street_segments[firstpoint][i]].oneWay){ //make sure this segment is not oneway
				if(firstpoint==street_segments_info[intersection_street_segments[firstpoint][i]].from){
                                    connect=true;
                                    break;
                                }
                            }
                            else{connect=true;}
			}
		}
	}
    return connect;
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    std::vector<StreetIdx> result;
    std::string givenName = street_prefix;
    givenName.erase(std::remove(givenName.begin(), givenName.end(), ' '), givenName.end());     //change the string name, which have the content of street_prefix, into all lower cases and erase all spaces
    std::transform(givenName.begin(),givenName.end(),givenName.begin(),::tolower);
    std::string limit;                                                      //assign limit which will be the next letter of the initial of the street_prefix
    limit += (givenName[0] + 1);
    for(auto tempStreet = prefix_of_name.lower_bound(givenName);tempStreet != prefix_of_name.lower_bound(limit);++tempStreet) {
	if(Equal(tempStreet->first,givenName)) {                      //check every street that start with the string name to next prefix of name to check if is equal prefix as provided using equal function
            result.push_back(tempStreet->second);
	}else{
            return result;                                  //until the first time a street is not equal to name, it will end the loop and return the result
        }
    }    
    return result;
}
//take a string as input, output a vecter of intersectionidx with name begin with the given string
std::vector<IntersectionIdx> findInterIdsFromPartialInterName(std::string inter_prefix) {
    std::vector<IntersectionIdx> result;
    std::string givenName = inter_prefix;
    givenName.erase(std::remove(givenName.begin(), givenName.end(), ' '), givenName.end());     //change the string name, which have the content of street_prefix, into all lower cases and erase all spaces
    std::transform(givenName.begin(),givenName.end(),givenName.begin(),::tolower);
    std::string limit;                                                      //assign limit which will be the next letter of the initial of the street_prefix
    limit += (givenName[0] + 1);
    for(auto tempStreet = prefix_of_inter_name.lower_bound(givenName);tempStreet != prefix_of_inter_name.lower_bound(limit);++tempStreet) {
	if(Equal(tempStreet->first,givenName)) {                      //check every street that start with the string name to next prefix of name to check if is equal prefix as provided using equal function
            result.push_back(tempStreet->second);
	}else{
            return result;                                  //until the first time a street is not equal to name, it will end the loop and return the result
        }
    }    
    return result;
}


double findStreetLength(StreetIdx street_id) {
    double result=street_length[street_id];                  //find the streetlength inside the vector street_length
    return result;
}

POIIdx findClosestPOI(LatLon my_position, std::string POIname) {                //returns -1 if no POIname is found
    POIIdx result = -1;
    double distance = -1.0;
    std::pair <LatLon,LatLon> locations;
    for(POIIdx i = 0; i < getNumPointsOfInterest(); ++i) {
	if(getPOIType(i) == POIname) {                                          //check every POI that have POIName as provided
            locations = std::make_pair(my_position,getPOIPosition(i));
            double tempDistance = findDistanceBetweenTwoPoints(locations);      //calculate the current POI distance to current position
            if(tempDistance < distance || distance < 0 ) {                      //check if in initial condition and find POI with the lowest distance
		result = i;
		distance = tempDistance;
            }
	}
    }
    return result;
}

double findFeatureArea(FeatureIdx feature_id) {
    double result = 0.0;
    int numOfPoints = getNumFeaturePoints(feature_id);
    if(numOfPoints < 3 || getFeaturePoint(0,feature_id).longitude() != getFeaturePoint(numOfPoints - 1,feature_id).longitude() ||
	getFeaturePoint(0,feature_id).latitude() != getFeaturePoint(numOfPoints - 1,feature_id).latitude()) {    //check if the feature is a closed area, if not, return 0
        return 0.0;
    }
    double avgLati = 0.0;
    for(int i = 0; i < numOfPoints - 1; i++) {                                  //find the average value for latitude which will be used to find the position of x
        avgLati += getFeaturePoint(i,feature_id).latitude();
    }
    avgLati /= (numOfPoints - 1);                                               
    int j = numOfPoints - 2;                                                    //j initially assigned to last term
    for(int i = 0; i < numOfPoints-1; i++) {
        result += latLontoX(getFeaturePoint(i,feature_id),avgLati) * latLontoY(getFeaturePoint(j,feature_id))
            - latLontoX(getFeaturePoint(j,feature_id),avgLati) * latLontoY(getFeaturePoint(i,feature_id));      //using shoelace formula to calculate the area
        j = i;                                                                  //j assign to previous vertex to i
    }
    return abs(result/2);                                                          //shoelace formula have a 1/2 in the front, so final result should divide by 2
}

LatLon findLatLonOfOSMNode (OSMID OSMid) {
    LatLon result = latlonFromOSMID[OSMid];                                 //find the latlon inside the vector latlonFromOSMID
    return result;
}

IntersectionIdx findClosestIntersection(LatLon my_position) {
    IntersectionIdx closest = -1;
    double minDistance = -1;
    for (IntersectionIdx i  = 0; i<getNumIntersections(); i++) {            //traverse through all intersections
        std::pair<LatLon, LatLon> distance;
        distance = std::make_pair(my_position,getIntersectionPosition(i));  //calculate the distance from a intersection to my_position
        double curDistance = findDistanceBetweenTwoPoints(distance);
        if(curDistance<minDistance || minDistance<0) {                      //compare and save the closest distance
            closest = i;
            minDistance = curDistance;
        }
    }
    return closest;                                                        
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {
    return intersection_street_segments[intersection_id];               //call global vector intersection_street_segments;
}

std::vector<std::string> findStreetNamesOfIntersection(IntersectionIdx intersection_id) {
    std::vector<std::string> result;
    for(int i = 0; i < intersection_street_segments[intersection_id].size(); ++i){         //traverse all street segment connecting to intersection_id
        StreetSegmentIdx SegmentId = intersection_street_segments[intersection_id][i];       
        result.push_back(getStreetName(street_segments_info[SegmentId].streetID));      //street name from each streetSegment Idx using struct street segment info
    }
    return result;
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    std::vector<IntersectionIdx> result;
    result = findIntersection[street_id];                               //use unordered map to find intersectionIdx
    std::sort(result.begin(),result.end());                             //check and erase duplicate in result vector
    result.erase(std::unique(result.begin(),result.end()),result.end());
    return result;
}

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids) {
    std::vector<IntersectionIdx> result;
    std::vector<IntersectionIdx> firstStreetInter = street_Intersections[street_ids.first];            //intersections of street id 1 using global vector street_Intersections
    std::vector<IntersectionIdx> secondStreetInter  = street_Intersections[street_ids.second];         //intersections of street id 2 using global vector street_Intersections
    
    std::vector<IntersectionIdx> IntertwoStreet(firstStreetInter.size() + secondStreetInter.size());   //find the common intersectionidx from both streets
    std::vector<int>::iterator it, end;
    end = std::set_intersection(
        firstStreetInter.begin(), firstStreetInter.end(),
        secondStreetInter.begin(), secondStreetInter.end(),
        IntertwoStreet.begin());
    
    for (it = IntertwoStreet.begin(); it != end; it++){
        result.push_back(*it);
    }
    return result;
}

