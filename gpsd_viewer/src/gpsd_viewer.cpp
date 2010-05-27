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

#include "gpsd_viewer/gpsd_viewer.h"
#include <ros/package.h>

AppData * data;

int
main( int argc, char **argv )
{
	pthread_t rosThread;
	pthread_t guiThread;
	
	pthread_create(&guiThread, NULL, startGUI, NULL);
	sleep(1);
	pthread_create(&rosThread, NULL, startROS, NULL);

	// wait guiThread to continu
	pthread_join(guiThread, NULL);
	ros::shutdown();

	return(0);
}

void
gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	ROS_INFO("Receiving gps data");
	
	//Center osmmap on gps data received
   osm_gps_map_set_center(data->map,msg->pose.pose.position.x,msg->pose.pose.position.y);

	//Draw point 
   osm_gps_map_draw_gps (data->map,msg->pose.pose.position.x,msg->pose.pose.position.y,10);
    
   gchar *msg_statusbar = g_strdup_printf("lat:%f, lon:%f",(float)msg->pose.pose.position.x,(float)msg->pose.pose.position.y);
   gtk_statusbar_push(GTK_STATUSBAR(data->statusbar),1,msg_statusbar);
    
	if(!data->draw_path)
		osm_gps_map_clear_gps(data->map);
}

void* 
startROS(void*)
{
	int argc = 0;
	char** argv;

	ros::init(argc, argv, "gpsd_viewer");

	ros::NodeHandle n;
	ros::Subscriber odo_sub;
	odo_sub=n.subscribe("/gps_odom", 1, &gpsOdomCallback);	

	ROS_INFO("Spinning");
	ros::spin();

	pthread_exit(NULL);
}

void* 
startGUI(void*)
{
	GtkBuilder * builder;
	GError * error = NULL;
	GtkObject * gtk_adj;
	char gui_filename[FILENAME_MAX];
	int start_zoom = 15;
   coord_t ccny_coord = {40.818551,-73.948674};
	int argc = 0;
	char** argv;
	
	std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
   sprintf(gui_filename,"%s/%s",package_path.c_str(),"gui.glade");


   gtk_init (&argc, &argv);

   // Allocate data structure
   data = g_slice_new(AppData);
    
   // Some initialisation
   gdk_window_set_debug_updates(false);
   data->draw_path=false;
   data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
   data->map_zoom_max = 18;
   data->map_current_zoom = start_zoom;
   data->repo_uri = osm_gps_map_source_get_repo_uri(data->map_provider);
   data->friendly_name = osm_gps_map_source_get_friendly_name(data->map_provider);

   char *mapcachedir;
   mapcachedir = osm_gps_map_get_default_cache_directory();
   data->cachedir = g_build_filename(mapcachedir,data->friendly_name,NULL);
   g_free(mapcachedir);

   // Create new GtkBuilder object
   builder = gtk_builder_new();
   // Load UI from file
   if( ! gtk_builder_add_from_file( builder, gui_filename, &error ) )
   {
       g_warning( "%s", error->message );
       g_free( error );
		 pthread_exit(NULL);
   }

   // Get main window pointer from UI
   data->window = GTK_WIDGET( gtk_builder_get_object( builder, "window" ) );	

	// Create the OsmGpsMap object
	data->map=(OsmGpsMap*)g_object_new (OSM_TYPE_GPS_MAP,"map-source",data->map_provider,
								  "tile-cache",data->cachedir,"proxy-uri",g_getenv("http_proxy"),NULL);

	//Set the starting coordinates and zoom level for the map
   osm_gps_map_set_zoom(data->map,start_zoom);
   osm_gps_map_set_center(data->map,ccny_coord.rlat,ccny_coord.rlon);

	// Add the map to the box
   data->map_box = GTK_WIDGET(gtk_builder_get_object(builder,"hbox3"));
   gtk_box_pack_start (GTK_BOX(data->map_box), GTK_WIDGET(data->map), TRUE, TRUE, 0);

   data->statusbar = GTK_WIDGET(gtk_builder_get_object(builder,"statusbar1"));
   data->map_container = GTK_WIDGET(gtk_builder_get_object(builder,"hbox2"));

	data->range_zoom = GTK_WIDGET(gtk_builder_get_object(builder,"vscale1"));
	gtk_adj=gtk_adjustment_new(start_zoom,1,data->map_zoom_max+1,1,1,1);
   gtk_range_set_adjustment(GTK_RANGE(data->range_zoom),GTK_ADJUSTMENT(gtk_adj));
   
   // Connect signals
   gtk_builder_connect_signals( builder, data );

   // Destroy builder, since we don't need it anymore
   g_object_unref( G_OBJECT( builder ) );

   // Show window. All other widgets are automatically shown by GtkBuilder
   gtk_widget_show_all( data->window );

   // Start main loop
   gtk_main();

   return( 0 );
}

