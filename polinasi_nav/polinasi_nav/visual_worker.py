"""Build and serialize visual-only messages in a process without a ROS node."""
import time
import numpy as np


def build_visual_snapshot(data):
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path
    from visualization_msgs.msg import Marker, MarkerArray
    from sensor_msgs_py import point_cloud2
    from rclpy.serialization import serialize_message
    started = time.perf_counter()
    stamp = Time(sec=data['stamp'][0], nanosec=data['stamp'][1])
    header = Header(frame_id='map', stamp=stamp)
    output = {}

    def marker(points, resolution, namespace, identifier, colour):
        m = Marker(header=header, ns=namespace, id=identifier,
                   type=Marker.CUBE_LIST, action=Marker.ADD)
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = float(resolution)
        m.color.r, m.color.g, m.color.b, m.color.a = colour
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        return m

    if 'grid' in data:
        grid = data['grid']
        markers = []
        for identifier, state, colour in [(0, 1, [1., .2, .1, .6]),
                                         (1, 0, [.1, .8, .2, .05]),
                                         (2, -1, [.5, .5, .5, .03])]:
            ids = np.argwhere(grid.state == state)
            ids = ids[::max(1, int(np.ceil(len(ids)/6000)))]
            markers.append(marker(grid.lo+(ids+.5)*grid.res, grid.res, 'voxels', identifier, colour))
        output['map_pub'] = serialize_message(MarkerArray(markers=markers))
        routes = dict(data['routes'])
        if data['trajectory'] is not None:
            tr = data['trajectory']
            routes['path_pub'] = [tr.sample(t)[0] for t in np.linspace(0., tr.duration, 150)]
        for publisher, points in routes.items():
            path = Path(header=header)
            for point in points:
                pose = PoseStamped(header=header)
                pose.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            output[publisher] = serialize_message(path)
        output['executed_pub'] = serialize_message(Path(
            header=Header(frame_id='odom', stamp=stamp), poses=data['executed']))
    if 'global_points' in data:
        output['global_map_pub'] = serialize_message(MarkerArray(markers=[marker(
            data['global_points'], data['resolution'], 'global_observed_occupied', 0, [1., .35, .05, .65])]))
        if data.get('cloud_points') is not None:
            output['global_cloud_pub'] = serialize_message(point_cloud2.create_cloud_xyz32(header, data['cloud_points']))
    return output, time.perf_counter()-started
