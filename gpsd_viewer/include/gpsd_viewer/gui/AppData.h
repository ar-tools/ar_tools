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

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include <gtk/gtk.h>
#include <osmgpsmap/osm-gps-map.h>

class AppData
{
	public:

	    GtkWidget * window;
	    GtkWidget * map_box;
	    GtkWidget * map_container;
	    GtkWidget * statusbar;
	    GtkWidget * range_zoom;
		 GdkEventButton *event;  

	    OsmGpsMap * map;
	    OsmGpsMapSource_t map_provider;
	    bool draw_path;
	    int map_zoom_max;
	    int map_current_zoom;
	    const char * repo_uri;
	    const char * friendly_name;
    	 char * cachedir;	
};

#endif
