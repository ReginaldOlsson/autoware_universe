# autoware_parking_spot_rviz_plugin

RViz panel that lists named parking spaces from the Lanelet2 map and publishes a goal pose when one is selected.

Parking spaces are Lanelet2 linestrings with `type=parking_space` and a `name` (or `alias`) tag. `RouteHandler` exposes those names; this panel searches them.

## How to use

1. Launch planning simulator with a map that has named parking spaces, for example `sample-map-planning`.
2. In RViz, open **Panels > Add New Panel** and select `ParkingSpotPanel`.
3. Wait until `/map/vector_map` is received. The list shows names such as `office` and `home`.
4. Filter with the search box, select a spot, then press **Set Goal** (or double-click). The panel publishes `geometry_msgs/PoseStamped` to `/planning/mission_planning/goal`.

The goal topic can be changed in the panel.

## Example OSM tag

```xml
<tag k="type" v="parking_space" />
<tag k="width" v="3" />
<tag k="name" v="office" />
```
