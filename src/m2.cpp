/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/class.cc to edit this template
 */

/* 
 * File:   m2.cpp
 * Author: guanyux1
 * 
 * Created on March 5, 2022, 10:03 p.m.
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
#include "OSMDatabaseAPI.h"
#include <iomanip>
//gobal variables are here//

//helper functions are here//
//getMaxposition(std::vector<LatLon>);
//main functions are here//
double min_x, min_y, max_x, max_y;
#define zoom_level_one 3000;
#define zoom_level_two 10000;
#define zoom_level_three 25000;
int feature_size_one = 20000;
int feature_size_two = 10000;
int feature_size_three = 3000;
void draw_main_canvas(ezgl::renderer*g);
void find_button(GtkWidget* /*widget*/, ezgl::application* application);//call back function of find interesections
void speed_button(GtkWidget* /*widget*/, ezgl::application* application);//call back function of speed limit 
void path_button(GtkWidget* /*widget*/, ezgl::application* application);//call back function of finding path
void reset_button(GtkWidget* /*widget*/, ezgl::application* application);//call back function of clear path and start/end points
void clear_button(GtkWidget* /*widget*/, ezgl::application* application);//call back function of clearing all highlights

void find_Bar(GtkWidget *widget, ezgl::application *application);
void initial_setup(ezgl::application *application, bool a);
ezgl::point2d centerStreet(StreetSegmentIdx SegmentID);
double boundX(StreetSegmentIdx SegmentID);//set bounds for street names to prevent 
double degree(StreetSegmentIdx SegmentID);
double arrow_degree(StreetSegmentIdx SegmentID);
void clicked_intersect(IntersectionIdx intersection_id, ezgl::application *application);
//void right_clicked_intersect(IntersectionIdx intersection_id, ezgl::application *application);
void on_dialog_response(GtkDialog *dialog, gint response_id, gpointer user_data);
//void on_dialog_response_right(GtkDialog *dialog, gint response_id, gpointer user_data);
double absolute(double x);
int get_zoom_level(ezgl::renderer*g);
void search1(GtkWidget* /*widget*/, gpointer data);
void search2(GtkWidget* /*widget*/, gpointer data);
void POIL(GtkWidget* /*widget*/,gpointer data, ezgl::renderer*g);
void CityL(GtkWidget* /*widget*/,gpointer data);
std::string name_direction(StreetSegmentIdx SegmentID);
void manual_button(GtkWidget *widget, ezgl::application *application);
void sideBarShow(GtkWidget */*widget*/, ezgl::application *application);
void sideBarHide(GtkWidget */*widget*/, ezgl::application *application);
void hideSearchBar(GtkWidget* /*widget*/, gpointer data);
double angle(IntersectionIdx start, IntersectionIdx end);
void Instructions_Button(GtkWidget */*widget*/, ezgl::application *application);

std::vector<std::pair<double,double>> CurvePoints(StreetSegmentIdx SegmentID);
ezgl::point2d centercurve(StreetSegmentIdx SegmentID);
double curveBoundX(StreetSegmentIdx SegmentID);
std::string curve_name_direction(StreetSegmentIdx SegmentID);
double curveDegree(StreetSegmentIdx SegmentID);
double arrow_curveDegree(StreetSegmentIdx SegmentID);
bool colored = false;
#define PI 3.14159265
void draw_main_canvas_feature(ezgl::renderer*g, int zoom_level);
void draw_main_canvas_POI(ezgl::renderer*g, int zoom_level);
void draw_main_canvas_Streetseg(ezgl::renderer*g, int zoom_level);
void draw_main_canvas_Intersection(ezgl::renderer*g,int zoom_level);
void draw_main_canvas_StreetName(ezgl::renderer*g,int zoom_level);
void draw_main_canvas_Highlight(ezgl::renderer*g, int zoom_level);

std::string compass(IntersectionIdx start, IntersectionIdx middle, IntersectionIdx end);


void hideSearchBar1(GtkWidget* /*widget*/, gpointer data);

void draw_main_canvas_Pathfounded(ezgl::renderer*g);



void move_screen(ezgl::renderer*g, IntersectionIdx id);// move the screen to one intersection as centre
std::vector<StreetSegmentIdx> path_founded;// a container to store the A* result
std::vector<path_seg> path_instruction;// a container used to display the path
std::vector<IntersectionIdx> findIntersectionOnDirection(std::vector<StreetSegmentIdx> path,//store all intersections appears in A* result 
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids);
std::string direction(IntersectionIdx start, IntersectionIdx end);
int iii=0;
bool bar = false;
bool cafe = false;
bool restaurant = false;
bool fastfood = false;
bool school = false;
bool library = false;
bool parking = false;
bool bus = false;
bool fuel = false;
bool finance = false;
bool hospital = false;
bool pharmacy = false;
bool entertainment = false;
bool marketplace = false;
bool publicservice = false;
bool waste = false;
bool sex = false;
bool art = false;
bool repair = false;
bool other = false;
bool showPOI = false;
bool move = false;
bool display_path;
IntersectionIdx center_point = 0;
IntersectionIdx clicked_inter = 0;
std::pair<IntersectionIdx, IntersectionIdx> start_end; 
bool click_mode = 0;

void act_on_mouse_press(ezgl::application *application, GdkEventButton *event, double x, double y);
/*Enable traving mouse pressing, when user press mouse with control pressed will highlight the nearest intersect and show a dialog*/
void act_on_mouse_press(ezgl::application *application, GdkEventButton *event, double x, double y) {
    application->update_message("Mouse Clicked");
    double lon = xtoLon(x);
    double lat = ytoLat(y);
    LatLon current = LatLon(lat, lon);
    std::stringstream ss;
    if (event->button == 1 && event->state & GDK_CONTROL_MASK) {//left click reaction(intersection information)
        clicked_inter = findClosestIntersection(current);
        intersections_data[clicked_inter].highlight = true;
        ss << "Intersection Selected: " << intersections_data[clicked_inter].name;
        if(intersections_data[clicked_inter].highlight == true){
            clicked_intersect(clicked_inter,application);
        } 
    }
    if(event->button == 3) {//right click reaction(choose start/end point)
        if(select_count == 2){
            ss << "You have already choose 2 intersections!";
            application->update_message(ss.str()); 
            return;
        }
        clicked_inter = findClosestIntersection(current);
        select_count += 1;
        if(select_count == 1){
            intersections_data[clicked_inter].select_start = true;
            start_end.first=clicked_inter;
            std::cout<<"start is : "<<start_end.first<<std::endl;
        }
        else if(select_count == 2){
            intersections_data[clicked_inter].select_end = true;
            start_end.second=clicked_inter;
            std::cout<<"end is : "<<start_end.second<<std::endl;
        }   
        ss << "Intersection Selected: " << intersections_data[clicked_inter].name; 
    }
    std::cout<<"count is "<<select_count<<std::endl;
    application->update_message(ss.str()); // Message at screen bottom
    application->refresh_drawing();
}

//function to initialize main window to draw map
void drawMap() {
    
    ezgl::application::settings settings;
    settings.main_ui_resource =
            "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    ezgl::application application(settings);
    //above is setting up//   
    LatLon min_boundary = LatLon(min_Lati, min_Lon);
    LatLon max_boundary = LatLon(max_Lati, max_Lon);
    min_x = avglatLontoX(min_boundary);
    max_x = avglatLontoX(max_boundary);
    min_y = latLontoY(min_boundary);
    max_y = latLontoY(max_boundary);
    //GTK features
    ezgl::rectangle initial_world({min_x, min_y},{max_x, max_y});
    ezgl::color background(241,243,244,255);
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world,background);
    application.run(initial_setup, act_on_mouse_press, nullptr, nullptr);
               
}
/*get the zoom level for the current visable area of the map*/
int get_zoom_level(ezgl::renderer*g) {
    //split in to 4 zoom level to draw different feature
    int levelOne = zoom_level_one;
    int levelTwo = zoom_level_two;
    int levelThree = zoom_level_three;
    if((g->get_visible_world().right() - g->get_visible_world().left())<levelOne){
        return 1;
    }else if((g->get_visible_world().right() - g->get_visible_world().left())<levelTwo){
        return 2;
    }else if((g->get_visible_world().right() - g->get_visible_world().left())<levelThree){
        return 3;
    }else {
        return 4;
    }
}
//draw all components on the map including feature, 
//street, intersection, POI, name and highlights
void draw_main_canvas(ezgl::renderer*g) {
    std::cout<<"Drawmap"<<std::endl;
    int zoom_level = get_zoom_level(g);  
    draw_main_canvas_feature(g,zoom_level);
    draw_main_canvas_Streetseg(g, zoom_level);
    draw_main_canvas_Intersection(g, zoom_level);
    if(showPOI == true){
        draw_main_canvas_POI(g,zoom_level);
    }
    draw_main_canvas_Highlight(g,zoom_level);
    draw_main_canvas_Pathfounded(g);
    draw_main_canvas_StreetName(g, zoom_level);
    if(move){
        move_screen(g,center_point);
        move=false;
    }  
             
}
/*Draws all the feature depending on their feature area and zoom level
 only largest feature will draw in zoom level 4, while all the feature will draw in zoom level 1, etc*/
void draw_main_canvas_feature(ezgl::renderer*g, int zoom_level){
    for (auto i = feature_rank_by_area.rbegin(); i != feature_rank_by_area.rend(); ++i) {
        //check for zoom level and feature area type to avoid feature misoverlapping
        if(i->first > feature_size_one){
            if(zoom_level <= 4){
                g->set_color(feature_graph[i->second].r, feature_graph[i->second].g, feature_graph[i->second].b, feature_graph[i->second].transparency);
                g->fill_poly(feature_graph[i->second].points);
            }
        }else if(i->first > feature_size_two){
            if(zoom_level <= 3){
                g->set_color(feature_graph[i->second].r, feature_graph[i->second].g, feature_graph[i->second].b, feature_graph[i->second].transparency);
                g->fill_poly(feature_graph[i->second].points);
            }
        }else if(i->first > feature_size_three){
            if(zoom_level <= 2){
                g->set_color(feature_graph[i->second].r, feature_graph[i->second].g, feature_graph[i->second].b, feature_graph[i->second].transparency);
                g->fill_poly(feature_graph[i->second].points);
            }
        }else{
            if(zoom_level <= 1){
                g->set_color(feature_graph[i->second].r, feature_graph[i->second].g, feature_graph[i->second].b, feature_graph[i->second].transparency);
                g->fill_poly(feature_graph[i->second].points);
            }
        }
    }
}
/*Draw all the street segments depending on their type and zoom level
 only most important street will draw in zoom level 4, while all the street will draw in zoom level 1, etc*/
void draw_main_canvas_Streetseg(ezgl::renderer*g, int zoom_level){
    for(int i = 0; i < streetsegments_data.size(); ++i){
        for(int j = 1; j < streetsegments_data[i].points_x.size(); j++){
            //display street according to the type of street and zoom level (different color and width)
            if(streetsegments_data[i].type == "motorway" || streetsegments_data[i].type == "trunk"){
                if(zoom_level <= 3){    
                    g->set_color(253,226,147,255);
                    g->set_line_width(8);
                    g->draw_line({streetsegments_data[i].points_x[j-1], streetsegments_data[i].points_y[j-1]},
                    {
                        streetsegments_data[i].points_x[j], streetsegments_data[i].points_y[j]
                    });
                }
            }else if(streetsegments_data[i].type == "primary" || streetsegments_data[i].type == "secondary" ){
                if(zoom_level <= 3){
                    g->set_color(253,226,147,255);
                    g->set_line_width(8);
                    g->draw_line({streetsegments_data[i].points_x[j-1], streetsegments_data[i].points_y[j-1]},
                    {
                        streetsegments_data[i].points_x[j], streetsegments_data[i].points_y[j]
                    });
                }
            }else if(streetsegments_data[i].type == "tertiary" || streetsegments_data[i].type == "unclassified"){
                if(zoom_level <= 2){
                
                    g->set_color(128,128,128,255/2);
                    g->set_line_width(6);
                    g->draw_line({streetsegments_data[i].points_x[j-1], streetsegments_data[i].points_y[j-1]},
                    {
                        streetsegments_data[i].points_x[j], streetsegments_data[i].points_y[j]
                    });
                }
            }else{
                if(zoom_level <= 1){
                    g->set_color(128,128,128, 255/2);
                    g->set_line_width(6);
                    g->draw_line({streetsegments_data[i].points_x[j-1], streetsegments_data[i].points_y[j-1]},
                    {
                        streetsegments_data[i].points_x[j], streetsegments_data[i].points_y[j]
                    });
                }
            }
        }
    }
}

//draw all POIs according to POI categorie
void draw_main_canvas_POI(ezgl::renderer*g, int zoom_level){
    if(zoom_level <= 1){         
        for(int poi_id = 0; poi_id < getNumPointsOfInterest(); poi_id++){
            g->set_font_size(10);
            g->set_color(ezgl::BLUE);
            double poi_x = avglatLontoX(getPOIPosition(poi_id));
            double poi_y = latLontoY(getPOIPosition(poi_id));
            ezgl::point2d center = ezgl::point2d(poi_x, poi_y-60);
            std::string type = getPOIType(poi_id);
            std::string name = getPOIName(poi_id);
            double scale=1;
            //check for type, display POI name, POI picture accordingl
            //food and drink
            if((type=="bar"||type=="pub"||type=="biergarten")&&(bar)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bar&pub.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="cafe")&&(cafe)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/cafe.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="restaurant"||type=="food_court")&&(restaurant)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/restaurant.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="fast_food"||type=="ice_cream")&&(fastfood)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/fastfood.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //education
            else if((type=="college"||type=="driving_school"||type=="kindergarten"||type=="language_school"||type=="music_school"||type=="school"||type=="university")&&(school)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/school.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="library"||type=="toy_library")&&(library)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/library.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //transportation
            else if((type=="bicycle_parking"||type=="motorcycle_parking"||type=="parking"||type=="parking_entrance"||type=="parking_space")&&(parking)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/parking.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="bicycle_repair_station"||type=="bicycle_rental"||type=="boat_rental"||type=="boat_sharing"||type=="car_rental"||type=="car_sharing"||
                type=="car_wash"||type=="vehicle_inspection"||type=="charging_station"||type=="ferry_terminal"||type=="taxi")&&(repair)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/repair.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="bus_station")&&(bus)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/bus.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="fuel")&&(fuel)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/fuel.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //finance
            else if((type=="bank"||type=="atm"||type=="bureau_de_change")&&(finance)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/finance.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //healthcare
            else if((type=="clinic"||type=="dentist"||type=="doctors"||type=="hospital"||type=="baby_hatch"||type=="nursing_home"||type=="veterinary")&&(hospital)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/hospital.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="pharmacy")&&(pharmacy)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/pharmacy.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //entertainment
            else if((type=="casino"||type=="community_centre"||type=="gambling"||type=="nightclub"||
                type=="conference_centre"||type=="events_venue"||type=="social_centre"||type=="fountain")&&(entertainment)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/entertainment.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="arts_centre"||type=="cinema"||type=="theatre"||type=="studio"||type=="public_bookcase"||type=="planetarium")&&(art)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/art.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            else if((type=="brothel"||type=="love_hotel"||type=="stripclub"||type=="swingerclub")&&(sex)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/sex.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //public service
            else if((type=="courthouse"||type=="fire_station"||type=="police"||type=="post_office"||type=="prison"||type=="social_facility"||
                type=="post_box"||type=="post_depot"||type=="ranger_station"||type=="townhall")&&(publicservice)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/publicservice.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //shopping
            else if((type=="marketplace"||type=="shop")&&(marketplace)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/marketplace.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
            //waste management
            else if((type=="sanitary_dump_station"||type=="recycling"||type=="waste_basket"||type=="waste_disposal"||type=="waste_transfer_station")&&(waste)){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/waste.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);    
            }
            //other
            else if(other){
                g->draw_text(center,name);
                ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/other.png");
                g->draw_surface(png_surface, {poi_x, poi_y},scale);
                ezgl::renderer::free_surface(png_surface);
            }
        }
    }
}

//print all street names on the map
void draw_main_canvas_StreetName(ezgl::renderer*g, int zoom_level){
    if(zoom_level<=3){
        for (int i = 0; i < street_segments_info.size(); i++) {
            std::string streetName = streetsegments_data[i].name;
            g->set_color(ezgl::BLACK);
            if(streetName!="<unknown>"){
                if(street_segments_info[i].numCurvePoints == 0){
                    if(streetsegments_data[i].oneway){
                        streetName = name_direction(i);
                    }
                    g->set_text_rotation(degree(i));
                    g->draw_text(centerStreet(i), streetName, boundX(i), 30);
                }else{
                    if(streetsegments_data[i].oneway){
                        streetName = curve_name_direction(i);
                    }
                    g->set_text_rotation(curveDegree(i));
                    g->draw_text(centercurve(i),streetName,curveBoundX(i),30);
                }
            }
        }
    }
}
/*draw all the intersection on map depending on highlight and zoom level
 all highlighted intersection will draw at all zoom level while normal intersection draw when zoom level is 1
 highlighted intersection will be in red colour with larger size*/
void draw_main_canvas_Intersection(ezgl::renderer*g,int zoom_level){
    for (int i = 0; i < intersections_data.size(); ++i) {
        if (intersections_data[i].highlight) {
            g->set_color(255,0,0,255);
            double radius = 3*zoom_level*zoom_level;
            g->fill_arc({intersections_data[i].x, intersections_data[i].y},radius,0,360);
        } 
        else if(intersections_data[i].select_start) {//display start point of navigation
            ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/start.png");
            g->draw_surface(png_surface, {intersections_data[i].x, intersections_data[i].y},1);
            ezgl::renderer::free_surface(png_surface);
        }
        else if(intersections_data[i].select_end) {//display end point of navigation
            ezgl::surface *png_surface = ezgl::renderer::load_png("libstreetmap/resources/end.png");
            g->draw_surface(png_surface, {intersections_data[i].x, intersections_data[i].y},1);
            ezgl::renderer::free_surface(png_surface);
        }
        else {
            if(zoom_level <= 1){
                g->set_color(255,255,255,255);
                double radius = 0.001;
                g->fill_arc({intersections_data[i].x, intersections_data[i].y},radius,0,360);
            }
        }           
    }
}
//function for highlighting all streets with different speed level
void draw_main_canvas_Highlight(ezgl::renderer*g, int zoom_level){
    if(colored && zoom_level<=2){
        float highspeed=22.222;
        float midspeed=11.111;
        for(int i=0;i<street_segments_info.size();++i){
            float speedlimit=street_segments_info[i].speedLimit;
            if(speedlimit<=midspeed){
                g->set_color(255, 69, 0, 255);
                g->set_line_width(5);   
            }
            else if(speedlimit>midspeed && speedlimit<=highspeed){
                g->set_color(30,144,255, 255);
                g->set_line_width(5);
            }
            else if(speedlimit>highspeed){
                g->set_color(50, 205, 50, 255);
                g->set_line_width(5);        
            }
            double from_x = streetsegments_data[i].points_x[0];
            double from_y = streetsegments_data[i].points_y[0];
            double to_x, to_y;
            for(int j=1;j<streetsegments_data[i].points_x.size();++j){
                    to_x = streetsegments_data[i].points_x[j];
                    to_y = streetsegments_data[i].points_y[j];
                    g->draw_line({from_x,from_y},{to_x,to_y});
                    from_x = to_x;
                    from_y = to_y;
            }
        }
    }
}

void draw_main_canvas_Pathfounded(ezgl::renderer*g){
    if(display_path){
        g->set_color(255, 140, 0, 255);
        g->set_line_width(5);
        for(int i=0;i<path_founded.size();++i){
            double from_x = streetsegments_data[path_founded[i]].points_x[0];
            double from_y = streetsegments_data[path_founded[i]].points_y[0];
            double to_x, to_y;
            for(int j=1;j<streetsegments_data[path_founded[i]].points_x.size();++j){
                    to_x = streetsegments_data[path_founded[i]].points_x[j];
                    to_y = streetsegments_data[path_founded[i]].points_y[j];
                    g->draw_line({from_x,from_y},{to_x,to_y});
                    from_x = to_x;
                    from_y = to_y;
            }  
            if(street_segments_info[path_founded[i]].oneWay){   
                g->set_font_size(50);
                std::string arrow = ">";
                double arrowdegree;
                if(street_segments_info[i].numCurvePoints == 0){
                    arrowdegree = arrow_degree(path_founded[i]);
                    g->set_text_rotation(arrowdegree);
                    g->draw_text(centerStreet(path_founded[i]), arrow, boundX(path_founded[i]),200);
                }
                else{
                    arrowdegree = arrow_curveDegree(path_founded[i]);
                    g->set_text_rotation(arrowdegree);
                    g->draw_text(centercurve(path_founded[i]), arrow, curveBoundX(path_founded[i]),200);
                } 
            }
        }      
    }   
}
/*Initialize dialog response*/
void on_dialog_response(GtkDialog *dialog, gint response_id, gpointer /*user_data*/){
    // For demonstration purposes, this will show the int value of the response type
    std::cout << "response is ";
    switch(response_id) {
        case GTK_RESPONSE_ACCEPT:
            std::cout << "GTK_RESPONSE_ACCEPT ";
            break;
        case GTK_RESPONSE_DELETE_EVENT:
            std::cout << "GTK_RESPONSE_DELETE_EVENT (i.e. X button) ";
            break;
        case GTK_RESPONSE_REJECT:
            std::cout << "GTK_RESPONSE_REJECT ";
            break;
        default:
            std::cout << "UNKNOWN ";
            break;
    }
    std::cout << "(" << response_id << ")\n";
    //This will cause the dialog to be destroyed
    gtk_widget_destroy(GTK_WIDGET (dialog));
}

void clicked_intersect(IntersectionIdx intersection_id, ezgl::application *application){
    LatLon location = getIntersectionPosition(intersection_id);
    std::vector<std::string> street_names_involved;
    street_names_involved = findStreetNamesOfIntersection(intersection_id);
    GObject *window; // the parent window over which to add the dialog
    GtkWidget *content_area; // the content area of the dialog (i.e. where to put stuff in the dialog)
    GtkWidget *label; // the label we will create to display a message in the content area
    GtkWidget *dialog; // the dialog box we will create
    // get a pointer to the main application window
    window = application->get_object(application->get_main_window_id().c_str());
    // Create the dialog window. Modal windows prevent interaction with other windows in the same application
    dialog = gtk_dialog_new_with_buttons(
    "Intersection",
    (GtkWindow*) window,
    GTK_DIALOG_MODAL,
    NULL,
    GTK_RESPONSE_ACCEPT,
    NULL,
    GTK_RESPONSE_REJECT,
    NULL
    );
    // Create a label and attach it to the content area of the dialog
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    std::string label_content = "";
    label_content += "Latitude: ";
    label_content += std::to_string(location.latitude());
    label_content += "\nLongitude: ";
    label_content += std::to_string(location.longitude());
    label_content += "\nStreets:";
    for(int i=0; i < street_names_involved.size(); i++){
        if(street_names_involved[i] != "<unknown>"){
            label_content += "\n";
            label_content += street_names_involved[i];
        }
    }

    label_content += "\nID: ";
    label_content += std::to_string(intersection_id);
    
    label = gtk_label_new(label_content.c_str());
    gtk_container_add(GTK_CONTAINER(content_area), label);
    // The main purpose of this is to show dialogs child widget, label
    gtk_widget_show_all(dialog);
    // Connecting the "response" signal from the user to the associated callback function
    g_signal_connect(
        GTK_DIALOG(dialog),
        "response",
        G_CALLBACK(on_dialog_response),
        NULL
    );
    street_names_involved.clear();
    // END: CODE FOR SHOWING DIALOG
}

void initial_setup(ezgl::application *application, bool /*a*/) {
    application->create_button("Find", 3, find_button);
    application->create_button("Speed Limit", 9 , speed_button);
   // application->create_button("Find Path", 10 , path_button);
   // application->create_button("Reset", 11 , reset_button);
    application->create_button("Clear Highlight", 12 , clear_button);
    application->create_button("User Manual", 13, manual_button);
    GObject *s1 = application->get_object("SearchBar1");
    g_signal_connect(s1, "changed",G_CALLBACK(search1),application);
    GObject *s2 = application->get_object("SearchBar2");
    g_signal_connect(s2, "changed",G_CALLBACK(search2),application);
    GObject *s3 = application->get_object("POIList");
    g_signal_connect(s3, "changed", G_CALLBACK(POIL),application);
    GObject *s4 = application->get_object("CityList");
    g_signal_connect(s4, "changed", G_CALLBACK(CityL),application);
    GObject *s5 = application->get_object("sideBarButtonShow");
    g_signal_connect(s5, "clicked", G_CALLBACK(sideBarShow),application);
    GObject *s6 = application->get_object("sideBarButtonHide");
    g_signal_connect(s6, "clicked", G_CALLBACK(sideBarHide),application);
    gtk_widget_hide(GTK_WIDGET(s6));
    GtkGrid *sideBar = (GtkGrid*)application->get_object("sideBar");
    gtk_widget_hide(GTK_WIDGET(sideBar));
    GObject *s7 = application->get_object("hideSearchBar1");
    g_signal_connect(s7, "changed",G_CALLBACK(hideSearchBar),application);
    GObject *s8 = application->get_object("hideSearchBar2");
    g_signal_connect(s8, "changed",G_CALLBACK(hideSearchBar1),application);
    GObject *findPath = application->get_object("FindHidden");
    g_signal_connect(findPath, "clicked", G_CALLBACK(path_button),application);
    GObject *Reset = application->get_object("ResetHide");
    g_signal_connect(Reset, "clicked", G_CALLBACK(reset_button),application);
    GObject *Instruction = application->get_object("InstructionButton");
    g_signal_connect(Instruction, "clicked", G_CALLBACK(Instructions_Button),application);
}
void Instructions_Button(GtkWidget */*widget*/, ezgl::application *application){
    std::vector<IntersectionIdx> InterFounded = findIntersectionOnDirection(path_founded, 
    start_end);
    GtkImage *Image = (GtkImage*)application->get_object("Photo");
    GtkLabel *text = (GtkLabel*)application->get_object("Label");
    int i =  iii;
    if(i>=path_founded.size()-1){
        text = (GtkLabel*)application->get_object("Label");  
        Image = (GtkImage*)application->get_object("Photo");
        std::string directionss = "You have reached the destination!";
        std::string picture1;
        picture1 = "libstreetmap/resources/finish.png";
        gtk_label_set_text(text, directionss.c_str());
        gtk_image_set_from_file(Image, picture1.c_str());
        return;
    }       
        if((i+1)<path_founded.size()){
            if(street_segments_info[path_founded[i]].streetID==street_segments_info[path_founded[i+1]].streetID){
                 Image = (GtkImage*)application->get_object("Photo");
                 text = (GtkLabel*)application->get_object("Label");
                std::string picture1;
                picture1 = "libstreetmap/resources/forward.png";
                gtk_image_set_from_file(Image, picture1.c_str());
                double length = 0;
                while((street_segments_info[path_founded[i]].streetID==street_segments_info[path_founded[i+1]].streetID)){
                    length += street_segments_length[path_founded[i]];
                    if(i<path_founded.size()){
                        
                        ++i;
                        ++iii;
                    }else{
                        break;
                    }
                }
                length += street_segments_length[path_founded[i]];
                std::ostringstream streamDecimal;
                streamDecimal << std::fixed;
                streamDecimal << std::setprecision(2);
                streamDecimal << length;
                std::string streamDecimalString = streamDecimal.str();
                std::string directions = direction(InterFounded[i],InterFounded[i+1]);
                std::string directionss = "Head " + directions + " on "+ getStreetName(street_segments_info[path_founded[i]].streetID)+"\n" + " for " + streamDecimalString + " meters.";
                gtk_label_set_text(text, directionss.c_str());
//                do{
//                    if(i<path_founded.size()){
//                        ++i;
//                    }else{
//                        break;
//                    }
//                }while((street_segments_info[path_founded[i]].streetID==street_segments_info[path_founded[i+1]].streetID));
//                 iii=i;
                
                
            }
            else if(street_segments_info[path_founded[i]].streetID!=street_segments_info[path_founded[i+1]].streetID){
                 Image = (GtkImage*)application->get_object("Photo");
                 text = (GtkLabel*)application->get_object("Label");
                std::string direction = compass(InterFounded[i],InterFounded[i+1],InterFounded[i+2]);
                std::string picture;
                if(direction=="turn left"){
                    picture = "libstreetmap/resources/turn_left.png";
                }
                if(direction=="turn right"){
                    picture = "libstreetmap/resources/turn_right.png";
                }
                
                if (picture!=""){
                    gtk_image_set_from_file(Image, picture.c_str());
                }
                if((i+1)<path_founded.size()){
                std::string dir = direction +" from "+ getStreetName(street_segments_info[path_founded[i]].streetID)+ "\nto " + getStreetName(street_segments_info[path_founded[i+1]].streetID);
                gtk_label_set_text(text, dir.c_str());
                }
                
                 iii=iii+1;
            }
        }
}

void sideBarShow(GtkWidget */*widget*/, ezgl::application *application){
    GtkGrid *sideBar = (GtkGrid*)application->get_object("sideBar");
    gtk_widget_show(GTK_WIDGET(sideBar));
    GObject *show = application->get_object("sideBarButtonShow");
    gtk_widget_hide(GTK_WIDGET(show));
    GObject *hide = application->get_object("sideBarButtonHide");
    gtk_widget_show(GTK_WIDGET(hide));
}

void sideBarHide(GtkWidget */*widget*/, ezgl::application *application){
    GtkGrid *sideBar = (GtkGrid*)application->get_object("sideBar");
    gtk_widget_hide(GTK_WIDGET(sideBar));
    GObject *show = application->get_object("sideBarButtonShow");
    gtk_widget_show(GTK_WIDGET(show));
    GObject *hide = application->get_object("sideBarButtonHide");
    gtk_widget_hide(GTK_WIDGET(hide));
}
//call back function for user manual bar button
void manual_button(GtkWidget */*widget*/, ezgl::application *application){
    application->update_message("User Manual Displayed");
    application->refresh_drawing();
    
    GObject *window;
    GtkWidget *content_area;
    GtkWidget *label;
    GtkWidget *dialog;
    
    window = application->get_object(application->get_main_window_id().c_str());
    dialog = gtk_dialog_new_with_buttons(
            "USER MANUAL",
            (GtkWindow*) window,
            GTK_DIALOG_MODAL,
            ("OK"),
            GTK_RESPONSE_ACCEPT,
            NULL
            );
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    label = gtk_label_new(
            "1. Feel free to use the arrows to move around the map\n"
            "2. The zoom in and zoom out button enable you different views of the map\n"
            "3. The zoom fit button gives you holistic view of the map\n"
            "4. Press proceed to close the map\n"
            "5. Enter two street names in the search bar on the top left and click the find\n"
            "button to get the intersections between two streets highlighted!\n"
            "6. Click Speed Limit button to show speed for streets\n"
            "               Red - below 40km/h\n"
            "               Blue - below 80km/h\n"
            "               Green - above 80km/h\n"
            "7. Arrows in the street name represents direction for one ways\n"
            "8. Click Clear Highlight button to unselect all intersections\n"
            "9. You can use the first dropdown menu on the top right to switch between maps\n"
            "10. You can see and draw  all Point of Interests with the second dropdown menu\n"
            "11. Control and left click on the intersections to see information about it\n"
            "12. If you want to open navigation, click on the show button\n"
            "               you can either:\n"
            "               enter two intersections name \n"
            "               or \n"
            "               right click on two intersections\n"
            "13. Click Find Path button to see the path displayed\n"
            "14. Click Next button to see navigation information displayed\n\n"

            "Press OK to close the User Manual!\n");
    gtk_container_add(GTK_CONTAINER(content_area),label);
    gtk_widget_show_all(dialog);
    g_signal_connect(
        GTK_DIALOG(dialog),
        "response",
        G_CALLBACK(on_dialog_response),
        NULL
    );
}

//call back function for drop down menu to switch the cities
void CityL(GtkWidget* /*widget*/,gpointer data){
    auto application = static_cast<ezgl::application*>(data);
    GtkComboBox *CityList = (GtkComboBox*)application->get_object("CityList");
    gchar* Cityg = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(CityList));
    std::string City = Cityg;

    if (City == "Beijing, China"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/beijing_china.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Cairo, Egypt"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/cairo_egypt.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Cape Town, South Africa"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Golden Horseshoe, Canada"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Hamilton, Canada"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/hamilton_canada.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Hong Kong, China"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/hong-kong_china.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Iceland"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/iceland.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Interlaken, Switzerland"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/interlaken_switzerland.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "London, England"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/london_england.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Moscow, Russia"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/moscow_russia.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "New Delhi, India"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/new-delhi_india.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "New York, USA"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/new-york_usa.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Rio De Janeiro, Brazil"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Saint Helena"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/saint-helena.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Singapore"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/singapore.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Sydney, Australia"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/sydney_australia.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Tehran, Iran"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/tehran_iran.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Tokyo, Japan"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/tokyo_japan.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
    else if (City == "Toronto, Canada"){
        closeMap();
        loadMap("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
        LatLon min_boundary = LatLon(min_Lati, min_Lon);
        LatLon max_boundary = LatLon(max_Lati, max_Lon);
        min_x = avglatLontoX(min_boundary);
        max_x = avglatLontoX(max_boundary);
        min_y = latLontoY(min_boundary);
        max_y = latLontoY(max_boundary);
        ezgl::rectangle new_world= ezgl::rectangle({min_x,min_y},{max_x,max_y});
        application->change_canvas_world_coordinates ("MainCanvas", new_world);
        application->refresh_drawing();
    }
}

//call back function for POI dropdown menu
void POIL(GtkWidget* /*widget*/,gpointer data, ezgl::renderer*/*g*/){
    auto application = static_cast<ezgl::application*>(data);
    GtkComboBox *POIList = (GtkComboBox*)application->get_object("POIList");
    gchar* POI = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(POIList));
    std::string type = POI; 
    if (type == "None") {
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
        showPOI = false;
    }
    else if (type == "Bar & Pub") {
        bar=true;
        showPOI = true;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Cafe") {
        cafe=true;
        showPOI = true;
        bar=false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Restaurant") {
        restaurant=true;
        showPOI = true;
        bar=false;
        cafe = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Fast Food") {
        fastfood=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    //education
    else if (type == "School" ) {
        school=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Library" ) {
        library=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    //transportation
    else if (type == "Parking") {
        parking=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Bus Station") {
        bus=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Fuel") {
        fuel=true;bar=false;
        showPOI = true;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    //finance
    else if (type == "Finance") {
        finance=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    //healthcare
    else if (type == "Hospital") {
        hospital=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Pharmacy") {
        pharmacy=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        entertainment = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    //entertainment
    else if (type == "Entertainment") {
        entertainment=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        marketplace = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Market Place") {
        marketplace=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        publicservice = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }  
    //public service
    else if (type == "Public Service") {
        publicservice=true;
        showPOI = true;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        waste = false;
        sex = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "18+") {
        sex = true;
        showPOI = true;
        publicservice=false;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        waste = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Waste Management") {
        waste = true;
        showPOI = true;
        sex = false;
        publicservice=false;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        art = false;
        repair = false;
        other = false;
    }
    else if (type == "Transportation Repair & Rent") {
        repair = true;
        showPOI = true;
        waste = false;
        sex = false;
        publicservice=false;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        art = false;
        other = false;
    }
    else if (type == "Arts & Culture"){
        art = true;
        showPOI = true;
        repair = false;
        waste = false;
        sex = false;
        publicservice=false;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
        other = false;
    }
    else if (type == "Others") {
        other = true;
        showPOI = true;
        art = false;
        repair = false;
        waste = false;
        sex = false;
        publicservice=false;
        bar=false;
        cafe = false;
        restaurant = false;
        fastfood = false;
        school = false;
        library = false;
        parking = false;
        bus = false;
        fuel = false;
        finance = false;
        hospital = false;
        pharmacy = false;
        entertainment = false;
        marketplace = false;
    }
    application->refresh_drawing();
}

void hideSearchBar(GtkWidget* /*widget*/, gpointer data){
    auto application = static_cast<ezgl::application*>(data);
    GtkEntry *text_entry = (GtkEntry*) application->get_object("hideSearchBar1");
    GtkListStore *store  = (GtkListStore*)application->get_object("hideSearchBarL1");
    //GtkEntryCompletion *entry_completion = (GtkEntryCompletion*)application->get_object("entrycompletion1");
    
    const gchar* text = gtk_entry_get_text(text_entry);
    const std::string word(text);
    
    gtk_list_store_clear(store);
    GtkTreeIter iter;
    std::vector<IntersectionIdx> interIds = findInterIdsFromPartialInterName(word);
    for(IntersectionIdx interId = 0; interId<intersections_data.size();interId++){
        std::string id = getIntersectionName(interId);
        gtk_list_store_append(store,&iter);
        gtk_list_store_set(store,&iter,0,id.c_str(),-1);
    }
       
}

void hideSearchBar1(GtkWidget* /*widget*/, gpointer data){
    auto application = static_cast<ezgl::application*>(data);
    GtkEntry *text_entry = (GtkEntry*) application->get_object("hideSearchBar2");
    GtkListStore *store  = (GtkListStore*)application->get_object("hideSearchBarL2");
    //GtkEntryCompletion *entry_completion = (GtkEntryCompletion*)application->get_object("entrycompletion1");
    
    const gchar* text = gtk_entry_get_text(text_entry);
    const std::string word(text);
    
    gtk_list_store_clear(store);
    GtkTreeIter iter;
    std::vector<IntersectionIdx> interIds = findInterIdsFromPartialInterName(word);
    for(IntersectionIdx interId = 0; interId<intersections_data.size();interId++){
        std::string id = getIntersectionName(interId);
        gtk_list_store_append(store,&iter);
        gtk_list_store_set(store,&iter,0,id.c_str(),-1);
    }
       
}
//call back function for search bar 1
void search1(GtkWidget* /*widget*/, gpointer data){
    auto application = static_cast<ezgl::application*>(data);
    GtkEntry *text_entry = (GtkEntry*) application->get_object("SearchBar1");
    GtkListStore *store  = (GtkListStore*)application->get_object("ListStore1");
    //GtkEntryCompletion *entry_completion = (GtkEntryCompletion*)application->get_object("entrycompletion1");
    
    const gchar* text = gtk_entry_get_text(text_entry);
    const std::string word(text);
    
    gtk_list_store_clear(store);
    GtkTreeIter iter;
    std::vector<StreetIdx> streetIds = findStreetIdsFromPartialStreetName(word);
    for(StreetIdx streetId = 0; streetId<street_Intersections.size();streetId++){
        std::string id = getStreetName(streetId);
        gtk_list_store_append(store,&iter);
        gtk_list_store_set(store,&iter,0,id.c_str(),-1);
    }
       
}

//call back function for search bar 2
void search2(GtkWidget* /*widget*/, gpointer data){
    auto application = static_cast<ezgl::application*>(data);
    GtkEntry *text_entry = (GtkEntry*) application->get_object("SearchBar2");
    GtkListStore *store  = (GtkListStore*)application->get_object("ListStore2");
    //GtkEntryCompletion *entry_completion = (GtkEntryCompletion*)application->get_object("entrycompletion2");
    
    const gchar* text = gtk_entry_get_text(text_entry);
    const std::string word(text);
    
    gtk_list_store_clear(store);
    GtkTreeIter iter;
    std::vector<StreetIdx> streetIds = findStreetIdsFromPartialStreetName(word);
    for(StreetIdx streetId = 0; streetId<street_Intersections.size();streetId++){
        std::string id = getStreetName(streetId);
        gtk_list_store_append(store,&iter);
        gtk_list_store_set(store,&iter,0,id.c_str(),-1);                                                                                                                      
    }
      
}
bool find_button_pressed = false;

//call back function for find button
void find_button(GtkWidget* /*widget*/, ezgl::application* application) {
    application->update_message("Test Button Pressed");
    std::pair<StreetIdx, StreetIdx> streetids;
    std::vector<IntersectionIdx> findInter;
    find_button_pressed = !find_button_pressed;
    GtkEntry* text_entry1 = (GtkEntry *) application->get_object("SearchBar1");
    GtkEntry* text_entry2 = (GtkEntry *) application->get_object("SearchBar2");
    const char* street1 = gtk_entry_get_text(text_entry1);
    const char* street2 = gtk_entry_get_text(text_entry2);
    std::string streets1 = std::string(street1);
    std::string streets2 = std::string(street2);
    
    if(streets1 == "" && streets2 !=""){
        for(StreetIdx i : street_names[streets2]){
            std::vector<IntersectionIdx> a = findIntersectionsOfStreet(i);
            findInter.insert(findInter.end(), a.begin(), a.end());
            a.clear();
        }
        for (int i = 0; i < findInter.size(); i++) {
        int interid = findInter[i];
        intersections_data[interid].highlight=true;
    }
        application->refresh_drawing();
        return;
    }
    if(streets1!="" && streets2 == ""){
        for(StreetIdx i : street_names[streets1]){
            std::vector<IntersectionIdx> a = findIntersectionsOfStreet(i);
            findInter.insert(findInter.end(), a.begin(), a.end());
            a.clear();
        }
        for (int i = 0; i < findInter.size(); i++) {
        int interid = findInter[i];
        intersections_data[interid].highlight=true;
    }
        application->refresh_drawing();
        return;
    }
    if(streets1=="" && streets2 ==""){
        return;
    }
    for(StreetIdx i : street_names[streets1]){
        for(StreetIdx j : street_names[streets2]){
            streetids.first = i;
            streetids.second = j;
            std::vector<IntersectionIdx> a = findIntersectionsOfTwoStreets(streetids);
            findInter.insert(findInter.end(), a.begin(), a.end());
            a.clear();
        }
    }
    for (int i = 0; i < findInter.size(); i++) {
        int interid = findInter[i];
        intersections_data[interid].highlight=true;
    }
    if(findInter.size()!=0){
        move = true;
        center_point = findInter[0];
        application->refresh_drawing();
    }
    application->refresh_drawing();
}

//call back function for speed limit button, show all speed limit
void speed_button(GtkWidget* /*widget*/, ezgl::application* application){
    application->update_message("SpeedLimit displayed");
    colored =! colored;
    application->refresh_drawing();
}
void path_button(GtkWidget* /*widget*/, ezgl::application* application){
	if(select_count == 0){
    	GtkEntry *text_entry1 = (GtkEntry*) application->get_object("hideSearchBar1");
    	GtkEntry *text_entry2 = (GtkEntry*) application->get_object("hideSearchBar2");
    	const char* inter1 = gtk_entry_get_text(text_entry1);
    	const char* inter2 = gtk_entry_get_text(text_entry2);
    	std::string inters1 = std::string(inter1);
    	std::string inters2 = std::string(inter2);
    	if(inters1!=""){
        	select_count += 1;
        	inters1.erase(std::remove(inters1.begin(), inters1.end(), ' '), inters1.end());
        	std::transform(inters1.begin(),inters1.end(),inters1.begin(),::tolower);
        	start_end.first = prefix_of_inter_name.find(inters1)->second;
        	intersections_data[start_end.first].select_start = true;
    	}
    	else{
        	application->update_message("An invalid street, please retry");
        	return;
    	}
    	if(inters2!=""){
        	select_count += 1;
        	inters2.erase(std::remove(inters2.begin(), inters2.end(), ' '), inters2.end());
        	std::transform(inters2.begin(),inters2.end(),inters2.begin(),::tolower);  	 
        	start_end.second = prefix_of_inter_name.find(inters2)->second;   	 
        	intersections_data[start_end.second].select_end = true;
    	}
    	else{
        	application->update_message("An invalid street, please retry");
        	return;
    	}
	}

	if(select_count<2){
    	application->update_message("Too few arguments, please choose 2 intersections");
    	return;
	}
	else{
    	application->update_message("Path found");
    	path_founded=findPathBetweenIntersections(10,start_end);
    	display_path=true;
        
        
        GtkImage *Image = (GtkImage*)application->get_object("Photo");
        std::string picture1;
        picture1 = "libstreetmap/resources/navigation.png";
        gtk_image_set_from_file(Image, picture1.c_str());
                
                
        GtkLabel *text = (GtkLabel*) application->get_object("Label");
       
        double pathTime = computePathTravelTime(10,
                                path_founded)/60;
        std::ostringstream streamDecimal;
        streamDecimal << std::fixed;
        streamDecimal << std::setprecision(2);
        streamDecimal << pathTime;
        std::string stringTime = streamDecimal.str();
         std::string word = "Start Intersection: \n" + getIntersectionName(start_end.first) + "\n\n" + "End Intersection: \n" 
                + getIntersectionName(start_end.second) + "\n\n" 
                +"The total travel time is:\n" + stringTime+ " minutes";
        gtk_label_set_text(text, word.c_str()); 
        iii=0;
	}
  	application->refresh_drawing();
}

void reset_button(GtkWidget* /*widget*/, ezgl::application* application){
        application->update_message("Path finding reset");
        intersections_data[start_end.first].select_start=false;
        intersections_data[start_end.second].select_end=false;
        start_end.first=0;
        start_end.second=0;
        select_count = 0;
        display_path = false;
        application->refresh_drawing();
}

//call back function for clear button, clear all highlights in graph
void clear_button(GtkWidget* /*widget*/, ezgl::application* application){
    application->update_message("All highlights cleared");
    for(int i=0; i<intersections_data.size();++i){
            intersections_data[i].highlight=false;
    }
    application->refresh_drawing();
}

std::vector<IntersectionIdx> findIntersectionOnDirection(std::vector<StreetSegmentIdx> path, 
    const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids){
    std::vector<IntersectionIdx> result;
    IntersectionIdx first = intersect_ids.first;
    IntersectionIdx last = intersect_ids.second;
    result.push_back(first);
    for (int i = 0; i<path.size()-1; i++){
        if(first==street_segments_info[path[i]].from){
            first=street_segments_info[path[i]].to;
            result.push_back(first);
        }else if(first==street_segments_info[path[i]].to){
            first=street_segments_info[path[i]].from;
            result.push_back(first);
        }
    }
    result.push_back(last);
    return result;
}

std::string compass(IntersectionIdx start, IntersectionIdx middle, IntersectionIdx end){
    double x1 = intersections_data[start].x;
    double y1 = intersections_data[start].y;
    double x2 = intersections_data[middle].x;
    double y2 = intersections_data[middle].y;
    double x3 = intersections_data[end].x;
    double y3 = intersections_data[end].y;
    double xx1 = x1-x2;
    double yy1 = y1-y2;
    double xx2  = x3-x2;
    double yy2 = y3-y2;
    std::string turn;
    
    double dot = xx1*xx2+yy1*yy2;
    double det = xx1*yy2-yy1*xx2;
    double angle = atan2(det,dot);
    
    if(angle==0 || angle==180 || angle ==-180){
        turn = "";
    }
    else if (angle>0){
        turn="turn right";
    }
    else if (angle<0){
        turn = "turn left";
    }
    return turn;
}

std::string direction(IntersectionIdx start, IntersectionIdx end){
    double degree = angle(start,end);
    std::string direction= "";
    if(degree>=-22.5 && degree<22.5){
        direction = "East";
    }else if(degree>=22.5 && degree<67.5){
         direction = "North East";
    }else if(degree>=67.5 && degree<112.5){
         direction = "North";
    }else if(degree>=112.5 && degree<157.5){
         direction = "North West";
    }else if((degree>=157.5 && degree<180 )||( degree>=-180 && degree<-157.5)){
         direction = "West";
    }else if(degree>=-67.5&& degree<-22.5){
         direction = "South East";
    }else if(degree>=-112.5 && degree<-67.5){
         direction = "South";
    }else if(degree>=-157.5 && degree<-112.5){
         direction = "South West";
    }
    return direction;
}

double angle(IntersectionIdx start, IntersectionIdx end){
    double x1 = intersections_data[start].x;
    double y1 = intersections_data[start].y;
    double x2 = intersections_data[end].x;
    double y2 = intersections_data[end].y;
    double x = x2-x1;
    double y = y2-y1;
    double degrees = 0;
    if((x)!=0){
    degrees = atan((y)/(x))*180.0/PI;
    
    }else if (y>0){
        degrees = 90;
    }else if (y<0){
        degrees = -90;
    }
    if(y<0 && x<0){
        degrees = -(180-degrees);

    }else if(x<0 && y>0){
        degrees = 180+degrees;
    }
    
    return degrees;
    
    
}

//return the center of non curve street segment
ezgl::point2d centerStreet(StreetSegmentIdx SegmentID) {
    double x0 = streetsegments_data[SegmentID].points_x[0];
    double x1 = streetsegments_data[SegmentID].points_x[1];
    double y0 = streetsegments_data[SegmentID].points_y[0];
    double y1 = streetsegments_data[SegmentID].points_y[1];
    double centerX = (x0+x1)/ 2;
    double centerY = (y0+y1)/ 2;
    ezgl::point2d center = ezgl::point2d(centerX, centerY);
    return center;
}

//return the length of non-curve street segment as boundx
double boundX(StreetSegmentIdx SegmentID) {
    double x0 = streetsegments_data[SegmentID].points_x[0];
    double x1 = streetsegments_data[SegmentID].points_x[1];
    double y0 = streetsegments_data[SegmentID].points_y[0];
    double y1 = streetsegments_data[SegmentID].points_y[1];
    double x = (x1- x0)*(x1- x0);
    double y = (y1- y0)*(y1- y0);
    double boundv = sqrt(absolute(x + y));
    return boundv;
}

//return the rotation degree for non-curve segments
double degree(StreetSegmentIdx SegmentID){
    double to_x = streetsegments_data[SegmentID].points_x[1];
    double to_y = streetsegments_data[SegmentID].points_y[1];
    double from_x = streetsegments_data[SegmentID].points_x[0];
    double from_y = streetsegments_data[SegmentID].points_y[0];
    double x = absolute(to_x - from_x);
    double y = absolute(to_y - from_y);
    double degrees;
    if (x == 0){
        return 90;
    }else{
        degrees = absolute(atan(y/x)*180.0/PI);
    }
    if ((to_x - from_x)<=0 && (to_y - from_y)>0){
        degrees = -degrees;
    }
    if ((to_x - from_x)>=0 && (to_y - from_y)<0){
        degrees = -degrees;
    }
    
    return degrees;
}

double arrow_degree(StreetSegmentIdx SegmentID){
    double to_x = streetsegments_data[SegmentID].points_x[1];
    double to_y = streetsegments_data[SegmentID].points_y[1];
    double from_x = streetsegments_data[SegmentID].points_x[0];
    double from_y = streetsegments_data[SegmentID].points_y[0];
    double x = absolute(to_x - from_x);
    double y = absolute(to_y - from_y);
    double degrees;
    if (x == 0 && (to_y - from_y)>0){
        return 90;
    }
    else if(x == 0 && (to_y - from_y)<0){
        return -90;
    }
    else{
        degrees = absolute(atan(y/x)*180.0/PI);
    }
    if ((to_x - from_x)<0 && (to_y - from_y)>=0){
        degrees = 180.0 - degrees;
    }
    else if ((to_x - from_x)<0 && (to_y - from_y)<=0){
        degrees = 180.0 + degrees;
    }
    else if ((to_x - from_x)>0 && (to_y - from_y)<=0){
        degrees = -degrees;
    }
    
    return degrees;
}
//This function returns the updated name + direction for one way non-curve segments
std::string name_direction(StreetSegmentIdx SegmentID){
    double to_x = streetsegments_data[SegmentID].points_x[1];
    double to_y = streetsegments_data[SegmentID].points_y[1];
    double from_x = streetsegments_data[SegmentID].points_x[0];
    double from_y = streetsegments_data[SegmentID].points_y[0];
    std::string name_with_direction = streetsegments_data[SegmentID].name;
    if ((to_x - from_x)<=0 && (to_y - from_y)>0){
        name_with_direction += "<<<";
    }
    else if ((to_x - from_x)>=0 && (to_y - from_y)<0){
        name_with_direction += ">>>";
    }else if((to_x - from_x)>=0 && (to_y - from_y)>=0){
        name_with_direction += ">>>";
    }else if((to_x - from_x)<=0 && (to_y - from_y)<=0){
        name_with_direction += "<<<";
    }
    return name_with_direction;
}
//return the absolute value of a double x
double absolute(double x){
    if (x>=0){
        return x;
    }else{
        return -x;
    }
}

//return the vector storing curve point start end point and magnitude information
std::vector<std::pair<double,double>> CurvePoints(StreetSegmentIdx SegmentID){
    //get vector storing all the x and y points in a curve street segment
    std::vector<double> pointx = streetsegments_data[SegmentID].points_x;
    std::vector<double> pointy = streetsegments_data[SegmentID].points_y;
    double maxLength = 0;
    //return vector (information about longest curve segment)
    std::vector<std::pair<double,double>> lpoints;
    std::pair<double,double> from;
    std::pair<double,double> to;
    std::pair<double,double> magnitude;
    magnitude.second = 0;
    for (int i = 0; i<pointx.size()-1;i++){
        //calculate the change in y and x
        double deltax = pointx[i+1]-pointx[i];
        double deltay = pointy[i+1]-pointy[i];
        //calculate the length of the longest line in curve segment
        double length = sqrt(absolute(deltax*deltax+deltay*deltay));
        //if calculated length is longer, renew lpoints
        if (length>maxLength){
            from.first = pointx[i];
            from.second = pointy[i];
            to.first = pointx[i+1];
            to.second = pointy[i+1];
            maxLength = length;
            magnitude.first = maxLength;
        } 
    }
    
    lpoints.push_back(from);
    lpoints.push_back(to);
    lpoints.push_back(magnitude);
    return lpoints;
}

//return the center point of the longest segment in a curved street segment
ezgl::point2d centercurve(StreetSegmentIdx SegmentID) {
    //get the vector storing curve point start end point and magnitude information
    std::vector<std::pair<double,double>> curvepoints = CurvePoints(SegmentID);
    //calculate the center point
    double centerX = (curvepoints[1].first+curvepoints[0].first)/ 2;
    double centerY = (curvepoints[1].second+curvepoints[0].second) / 2;
    ezgl::point2d center = ezgl::point2d(centerX, centerY);
    return center;
}

//return the length of longest distance between curve points in a street segment
double curveBoundX(StreetSegmentIdx SegmentID){
    //get the vector storing curve point start end point and magnitude information
    std::vector<std::pair<double,double>> curvepoints = CurvePoints(SegmentID);
    return curvepoints[2].first;
}

//return the rotation degree of text for curve points
double curveDegree(StreetSegmentIdx SegmentID){
    //get the vector storing curve point start end point and magnitude information
    std::vector<std::pair<double,double>> curvepoints = CurvePoints(SegmentID);
    double to_x = curvepoints[1].first;
    double to_y = curvepoints[1].second;
    double from_x = curvepoints[0].first;
    double from_y = curvepoints[0].second;
    //change in x and change in y
    double deltax = absolute(to_x - from_x);
    double deltay = absolute(to_y - from_y);
    double degrees;
    //can not divide by 0 for arctan
    if (deltax == 0){
        return 90;
    }else{
        //find angle with arctan
        degrees = absolute(atan(deltay/deltax)*180.0/PI);
    }
    
    //degree = -degree for quadrant 2 and 4
    if ((to_x - from_x)<=0 && (to_y - from_y)>0){
        degrees = -degrees;
    }
    if ((to_x - from_x)>=0 && (to_y - from_y)<0){
        degrees = -degrees;
    }
    
    return degrees;
}

double arrow_curveDegree(StreetSegmentIdx SegmentID){
    //get the vector storing curve point start end point and magnitude information
    std::vector<std::pair<double,double>> curvepoints = CurvePoints(SegmentID);
    double to_x = curvepoints[1].first;
    double to_y = curvepoints[1].second;
    double from_x = curvepoints[0].first;
    double from_y = curvepoints[0].second;
    //change in x and change in y
    double deltax = absolute(to_x - from_x);
    double deltay = absolute(to_y - from_y);
    double degrees;
    //can not divide by 0 for arctan
    if (deltax == 0 && deltay>0){
        return 90;
    }
    else if(deltax == 0 && deltay<0){
        return -90;
    }
    else{
        //find angle with arctan
        degrees = absolute(atan(deltay/deltax)*180.0/PI);
    }
    
    //degree = -degree for quadrant 2 3 4
    if ((to_x - from_x)<0 && (to_y - from_y)>=0){
        degrees = 180.0 - degrees;
    }
    else if ((to_x - from_x)<0 && (to_y - from_y)<=0){
        degrees = 180.0 + degrees;
    }
    else if ((to_x - from_x)>0 && (to_y - from_y)<=0){
        degrees = -degrees;
    }
    
    return degrees;
}

//This function returns the updated name + direction for one way curve segments
std::string curve_name_direction(StreetSegmentIdx SegmentID){
    //get the vector storing curve point start end point and magnitude information
    std::vector<std::pair<double,double>> curvepoints = CurvePoints(SegmentID);
    double to_x = curvepoints[1].first;
    double to_y = curvepoints[1].second;
    double from_x = curvepoints[0].first;
    double from_y = curvepoints[0].second;
    //get name without direction
    std::string name_with_direction = streetsegments_data[SegmentID].name;
    //determine direction of one way and add it in name
    if ((to_x - from_x)<=0 && (to_y - from_y)>0){
        name_with_direction += "<<<";
    }
    else if ((to_x - from_x)>=0 && (to_y - from_y)<0){
        name_with_direction += ">>>";
    }
    else if((to_x - from_x)>=0 && (to_y - from_y)>=0){
        name_with_direction += ">>>";
    }
    else if((to_x - from_x)<=0 && (to_y - from_y)<=0){
        name_with_direction += "<<<";
    }
    return name_with_direction;
}

void move_screen(ezgl::renderer*g, IntersectionIdx id){
    LatLon id_lati=getIntersectionPosition(id);
    double id_x=avglatLontoX(id_lati);
    double id_y=latLontoY(id_lati);
    double length=300;
    double width=250;
    ezgl::point2d downleft(id_x-length/2,id_y-width/2);
    ezgl::point2d upright(id_x+length/2,id_y+width/2);
    ezgl::rectangle new_screen(downleft,upright); 
    g->set_visible_world(new_screen);
}