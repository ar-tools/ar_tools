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

#include "gpsd_viewer/gui/callbacks.h"

extern "C" G_MODULE_EXPORT 
void on_mainwindow_destroy (GtkObject *object, gpointer user_data)
{
	gtk_main_quit ();
}

extern "C" G_MODULE_EXPORT 
void on_checkbuttonDrawPath_toggled(GtkToggleButton * togglebutton, AppData * data)
{
	if(data->draw_path)
		data->draw_path=false;
	else
		data->draw_path=true;
}

extern "C" G_MODULE_EXPORT 
void on_menuitem_ClearPath_activate(GtkMenuItem * togglebutton, AppData * data)
{
	osm_gps_map_clear_gps(data->map);
}

extern "C" G_MODULE_EXPORT 
void on_menuitemMapProvider_group_changed(GtkRadioMenuItem* radiobutton, AppData* data)
{
	GSList *tmp_list;
	GtkObject *gtk_adj;
   GtkCheckMenuItem *check_menu_item;
   GtkCheckMenuItem *tmp_menu_item;	
   char *mapcachedir;
	coord_t current_coord;
	
   check_menu_item = GTK_CHECK_MENU_ITEM (radiobutton);
  
	if(!check_menu_item->active)
    {
		tmp_menu_item = NULL;
		tmp_list=radiobutton->group;
		int i=g_slist_length(tmp_list)+1;
		current_coord = osm_gps_map_get_co_ordinates(data->map,(data->map_container)->allocation.width/2,
																	(data->map_container)->allocation.height/2);
      
		while(tmp_list)
		{
			tmp_menu_item = GTK_CHECK_MENU_ITEM(tmp_list->data);
			tmp_list = tmp_list->next;
			i--;
			if (tmp_menu_item->active && (tmp_menu_item != check_menu_item))
				break;
			tmp_menu_item = NULL;
		}
		
		switch(i)
		{
			case 1:
				data->map_provider=OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
				data->map_zoom_max=18;
				break;
			case 2:
				data->map_provider=OSM_GPS_MAP_SOURCE_OPENSTREETMAP_RENDERER;
				data->map_zoom_max=18;
				break;
			case 3:
				data->map_provider=OSM_GPS_MAP_SOURCE_OSM_PUBLIC_TRANSPORT;
				data->map_zoom_max=18;
				break;
			case 4:
				data->map_provider=OSM_GPS_MAP_SOURCE_OPENCYCLEMAP;
				data->map_zoom_max=18;
				break;
			case 5:
				data->map_provider=OSM_GPS_MAP_SOURCE_MAPS_FOR_FREE;
				data->map_zoom_max=11;
				break;
			case 6:
				data->map_provider=OSM_GPS_MAP_SOURCE_GOOGLE_STREET;
				data->map_zoom_max=17;
				break;
			case 7:
				data->map_provider=OSM_GPS_MAP_SOURCE_GOOGLE_SATELLITE;
				data->map_zoom_max=18;
				break;
			case 8:
				data->map_provider=OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_STREET;
				data->map_zoom_max=17;
				break;
			case 9:
				data->map_provider=OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_SATELLITE;
				data->map_zoom_max=17;
				break;
			case 10:
				data->map_provider=OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_HYBRID;
				data->map_zoom_max=17;
				break;
			default:
				data->map_provider=OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
				data->map_zoom_max=17;
		}
		
		// Change name of cache directory 
		data->friendly_name = osm_gps_map_source_get_friendly_name(data->map_provider);
		mapcachedir = osm_gps_map_get_default_cache_directory();
		data->cachedir = g_build_filename(mapcachedir,data->friendly_name,NULL);
		
		// Change map source and update box & window 
		data->map=(OsmGpsMap*)g_object_new (OSM_TYPE_GPS_MAP,"map-source",data->map_provider,
				"tile-cache",data->cachedir,"proxy-uri",g_getenv("http_proxy"),NULL);
	   gtk_container_remove(GTK_CONTAINER(data->map_container),data->map_box);
	   data->map_box=gtk_hbox_new(TRUE,0);
	   gtk_box_pack_start(GTK_BOX(data->map_container), data->map_box, TRUE, TRUE, 0);
	   gtk_box_pack_start(GTK_BOX(data->map_box), GTK_WIDGET(data->map), TRUE, TRUE, 0);
	    
	   // Set map coordinates & zoom to the current ones
	   osm_gps_map_set_center(data->map, (current_coord.rlat/ M_PI * 180.0), (current_coord.rlon/ M_PI * 180.0));
	   gtk_adj=gtk_adjustment_new(data->map_current_zoom,1,data->map_zoom_max+1,1,1,1);
	   gtk_range_set_adjustment(GTK_RANGE(data->range_zoom),GTK_ADJUSTMENT(gtk_adj));
	   osm_gps_map_set_zoom(data->map,data->map_current_zoom);
	    
	   gtk_widget_show_all( data->window );
	} 
}

gboolean on_vscaleZoom_change_value(GtkRange *range, GtkScrollType scroll,
                                    gdouble value, AppData * data)
{
	data->map_current_zoom = (int)value;
	osm_gps_map_set_zoom(data->map,(int)value);
	return false;
}

/*
 * note: not yet implemented in gui
 * 
extern "C" G_MODULE_EXPORT 
void on_menuitem_Cache_toggled(GtkRadioMenuItem* radiobutton, AppData* data)
{
	// Note: in order to change cache setting, 
	// reloading the map_provider is needed 
	 GSList *tmp_list;
    GtkCheckMenuItem *check_menu_item;
    GtkCheckMenuItem *tmp_menu_item;
    char *mapcachedir;	
	    
	check_menu_item = GTK_CHECK_MENU_ITEM (radiobutton);
	
	if (!check_menu_item->active)
    {
		tmp_menu_item = NULL;
		tmp_list=radiobutton->group;
		int i=0;
      
		while (tmp_list)
		{
			tmp_menu_item = GTK_CHECK_MENU_ITEM(tmp_list->data);
			tmp_list = tmp_list->next;
			i++;
			if (tmp_menu_item->active && (tmp_menu_item != check_menu_item))
				break;
			tmp_menu_item = NULL;
		}
		
		switch(i)
		{
			case 1:
				mapcachedir = osm_gps_map_get_default_cache_directory();
				data->cachedir = g_build_filename(mapcachedir,data->friendly_name,NULL);
				break;
			case 2:
				data->cachedir = (char *)OSM_GPS_MAP_CACHE_DISABLED;
				break;	
			default:
				mapcachedir = osm_gps_map_get_default_cache_directory();
				data->cachedir = g_build_filename(mapcachedir,data->friendly_name,NULL);
		}
		
		gchar *msg_statusbar = g_strdup_printf("Note: In order to change cache setting, reloading the map_provider is needed");
		gtk_statusbar_push(GTK_STATUSBAR(data->statusbar),1,msg_statusbar);
	}
}*/
