/*
 *  ROS GPSD Map Viewer using OpenStreetMap & OSMGpsMap
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GPSD_VIEWER_H
#define GPSD_VIEWER_H

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <pthread.h>
#include <glib.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <osmgpsmap/osm-gps-map.h>
#include <gpsd_viewer/gui/AppData.h>

void* startGUI(void*);
void* startROS(void*);
void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr&);

#endif
