## Common 
Contains custom ros types and some other utilities used throughout the project.

## File Structure

```
~/ros2_ws/common/
├── adaptive_navigation_interfaces/    # custom ros2 types for adaptive navigation
├── adaptive_navigation_utilities/     
├── pioneer_interfaces/                # custom ros2 types for pioneers and cluster
├── reference_srv/
│   └── gps_reference.py               # this ros node creates a GPS coordinate reference point for the pioneers. Set the coordinate in the file and run during tests.


```
