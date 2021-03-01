""" 
    Jupyter .ipynb Example
        m = UbxMap(basemapName="Mapnik", animate = True, zoom=17, width="99%", height="720px")
        m.Load()
        m.DrawRoute()
        m.Display()
"""
from ubxparser import UbxParser
from ipyleaflet import Map, basemaps, basemap_to_tiles, Marker, Circle, Polyline, MeasureControl, \
                        AntPath,FullScreenControl,ScaleControl, DivIcon, Popup, AwesomeIcon, SearchControl
#import matplotlib.pyplot as plt
from pandas import DataFrame, Series
import ipywidgets as widgets
from ipywidgets import interact, interact_manual, Layout, HTML, FileUpload
from IPython.core.display import display
import os

# classes
class UbxMap():
    def __init__(self, basemapName = "Mapnik", animate=False, zoom = 16, width="100%", height="720px"):        
        """Parse the UBX File and put it into a DataFrame"""
        self.OnLocationChanged = None
        self.OnDataLoaded = None     
        self.OnDataFileChanged = None
        self._animate = animate

        # map
        self._basemaps = {
            "Mapnik" : basemaps.OpenStreetMap.Mapnik,
            "Satellite" : basemaps.Esri.WorldImagery,
            "WorldStreetMap" : basemaps.Esri.WorldStreetMap,
            "BlackAndWhite" : basemaps.OpenStreetMap.BlackAndWhite,
            "France" : basemaps.OpenStreetMap.France,
            "HOT" : basemaps.OpenStreetMap.HOT,
            "OpenTopoMap" : basemaps.OpenTopoMap,    
            "WorldTopoMap" : basemaps.Esri.WorldTopoMap,
            }
        self._basemapName = basemapName
        self._basemap = self._basemaps[self._basemapName]
        self._zoom = zoom
        self._startLocation = (49,8)
        self._m = Map(
            basemap=self._basemap,
            center=self._startLocation, 
            zoom=self._zoom,
            close_popup_on_click=False
            )
        self._m.layout.width = width
        self._m.layout.height = height

        self._addControls()
        # parser
        self._parser = UbxParser()
        
        # GUI elements
        self._fileSelector = widgets.Dropdown(value = self.UbxFiles[0], placeholder = "Log Files", options= self.UbxFiles, description="Log File", ensureoption = False, disabled=False)
        self._fileSelector.observe(self._on_filename_changed, names='value')

        self._waypointSlider = widgets.IntSlider(value = 0, description = "Waypoint", min= 0, max=0, layout=widgets.Layout(width='50%'))
        self._waypointSlider.observe(self._on_waypoint_changed, names='value')

        self._mapSelector = widgets.Dropdown(value=self._basemapName, options=list(self._basemaps.keys()), description="Map", ensureoption = True, disabled=False)
        self._mapSelector.observe(self._on_mapstyle_changed, names='value')

        self._upload = FileUpload(accept='.pubx', multiple=True)
        self._upload.observe(self._on_file_upload_changed, names="value")

        self._header = HTML("<b><font size='2' color='orange'>pixelmaps</font></b>")

        self._gui =  widgets.VBox([widgets.HBox([self._header, self._fileSelector, self._waypointSlider, self._mapSelector, self._upload]), self._m])
    
    @property
    def CurrentLocation(self):
        return self._waypointMarker.location

    @property
    def RouteLine(self):
        return self._routeLine

    @property
    def Map(self):
        return self._m

    @property
    def WaypointSliderWidget(self):
        return self._waypointSlider

    @property
    def FileSelectorWidget(self):
        return self._fileSelector

    @property
    def MaxNumberOfWaypoints(self):
        return len(self._df)-1

    @property
    def UbxFiles(self):
        ubxFiles = []
        dir = os.listdir()
        ubxFiles = []
        for file in dir:
            if file.endswith(".pubx"):
                ubxFiles.append(file)
        if len(ubxFiles) == 0:
            ubxFiles.append("<NoLogFiles>")
        return ubxFiles
       
    def setCurrentWaypoint(self, value):
        self._currentWaypoint = value
        row = self._df.iloc[self._currentWaypoint]
        speed = row["groundspeed"]
        gz = row["gravityz"]
        ml = row["miclevel"]
        lat = row["latitude"]
        lng = row["longitude"]
        head = row["headingofmotion"]    
        self._waypointMessage.value = f"Moving {self._parser.direction(head)} with {speed:.0f} Km/h"
        #self._waypointMarker.rotation_angle = head        
        self._waypointMarker.location = (lat,lng)        
        self._center()


    def Append(self, filename):
        self._filename = filename        
        self._df = self._df.append(DataFrame.from_dict(self._parser.read(self._filename)))
        self._updateData()
        
    def Load(self, filename = "<First>"):
        if filename == "<First>":
            self._filename = self.UbxFiles[0]
            if self._filename == "<NoLogFiles>":
                return
        else:
            self._filename = filename
        self._df = DataFrame.from_dict(self._parser.read(self._filename))
        self._updateData()
        self._clearLayers()
        
    def _updateData(self):
                # Get the start location
        self._startLocation = (self._df.at[0,'latitude'],self._df.at[0,'longitude'])        

        # Calculate the route average speed
        self._averageSpeed = self._df["groundspeed"].mean()
        self._minSpeed = self._df["groundspeed"].min()
        self._maxSpeed = self._df["groundspeed"].max()
        self._averageHOF = self._df["headingofmotion"].mean()
        # Get the route points to draw the route
        self._routePoints = self._df.loc[:, ["latitude", "longitude"]].values.tolist()

        self._waypointSlider.max = self.MaxNumberOfWaypoints

        if self.OnDataLoaded:
            self.OnDataLoaded(self)

    def _addControls(self):
        # Add Controls
        self._m.add_control(MeasureControl(position='bottomleft', active_color = 'orange', primary_length_unit = 'kilometers'))
        self._m.add_control(FullScreenControl())
        self._m.add_control(ScaleControl(position='bottomleft'))
        searchMarker = Marker(icon=AwesomeIcon(name="check", marker_color='green', icon_color='darkred'))
        self._m.add_control(SearchControl( position="topleft",url='https://nominatim.openstreetmap.org/search?format=json&q={s}',zoom=self._zoom,marker=searchMarker))

    def Display(self):
        self._fileSelector.options = self.UbxFiles
        self._waypointSlider.max = self.MaxNumberOfWaypoints
        return self._gui

    def move(self, location):
        self._waypointMarker.location = location

    def _on_filename_changed(self,change):
        newFile = change["new"]
        self.Load(newFile)
        self.DrawRoute()
        if self.OnDataFileChanged:
            self.OnDataFileChanged(self, change["new"])

    def _on_file_upload_changed(self, change):
        newFile = change["new"]        
        with open(newFile, 'wb') as output_file: 
            for uploaded_filename in self._upload.value:
                content = self._upload.value[uploaded_filename]['content']   
                output_file.write(content)        

    def _on_mapstyle_changed(self, change):
        self._basemapName = change["new"]
        self._m.layers = [basemap_to_tiles(self._basemaps[self._basemapName])]
        self.DrawRoute()

    def _on_waypoint_changed(self, change):
        self._currentWaypoint = int(change['new'])
        self.setCurrentWaypoint(self._currentWaypoint)


    def _on_location_changed(self, event):
        # Do some computation given the new marker location, accessible from `event['new']
        print(event["new"])

    def _addWaypoint(self):
        # Waypoint
        self._waypointMarker = Marker(location = self._startLocation, draggable = False)
        self._waypointMarker.observe(self._on_location_changed, 'location')        
        self._m.add_layer(self._waypointMarker)

        # Waypoint Popup Message
        self._waypointMessage = HTML()
        self._waypointMarker.popup = self._waypointMessage

    def DrawRoute(self):
        # Draw the route        
        #self._clearLayers()
        self._routeLine = AntPath( locations=self._routePoints,dash_array=[1, int(self._averageSpeed)], delay=int(self._averageSpeed*500),color ="red",pulse_color="blue")#color='#7590ba',pulse_color='#3f6fba')
        self._m.add_layer(self._routeLine)        
        self._addWaypoint()
        self._center()
        self._addStats()
        if self._animate:
            self._center()

    def _center(self):
        self._m.center = self._waypointMarker.location

    def _centerTo(self, location):
        self._m.center = location

    def _clearLayers(self):
        self._m.clear_layers()
        self._m.layers = [basemap_to_tiles(self._basemap)]

    def calcSpeedZGravityThreshold(self, speed, grvityz):
        threshold = 10.5        
        if speed<6: #walking
            threshold = 12.5
        elif speed>=6 and speed<10: # running
            threshold = 14.5
        elif speed>10 and speed<25: # bike
            threshold = 16.5
        return threshold

    def _addStats(self):
        oldloc = (0,0)
        # Iterate through the route points
        for index, row in self._df.iterrows():
            # Get various row info
            speed = row["groundspeed"]
            gz = row["gravityz"]
            ml = row["miclevel"]
            lat = row["latitude"]
            lng = row["longitude"]
            loc = (lat,lng)

            # Add speed markers
            if speed < 2 and self._parser.distance(loc, oldloc) > 0.01:
                #m.add_layer(Marker(location=loc, draggable=False, title="Stopped"))
                self._m.add_layer(Circle(location=loc, radius=5, color="green", fill_color="green"))
                oldloc = loc
                if self._animate:
                    self._centerTo(loc)

            # Add Z Gravity markers (bumps)
            if gz>self.calcSpeedZGravityThreshold(speed, gz):
                self._m.add_layer(Circle(location=loc, radius=int(gz/2), color="red", fill_color="red"))
                if self._animate:
                    self._centerTo(loc)

            # Add Sound Markers
            if ml>=1.5:
                self._m.add_layer(Circle(location=loc, radius=int(ml*5), color="orange", fill_color="orange"))
                if self._animate:
                    self._centerTo(loc)

if __name__ == "__main__":
    pass    