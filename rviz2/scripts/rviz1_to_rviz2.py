#!/usr/bin/env python3

import argparse
import copy
import sys
import warnings
import yaml


def del_maybe(dictionary, key):
    if key in dictionary:
        del dictionary[key]


def migrate_panels(panels):
    panels_rviz2 = []

    for panel_dict in panels:
        if 'Class' not in panel_dict:
            print('Unknown panel format, skipping:' + repr(panel_dict), file=sys.stderr)
            continue

        if 'rviz/Displays' == panel_dict['Class']:
            panels_rviz2.append(migrate_panel_displays(panel_dict))
        elif 'rviz/Selection' == panel_dict['Class']:
            panels_rviz2.append(migrate_panel_selection(panel_dict))
        elif 'rviz/Tool Properties' == panel_dict['Class']:
            panels_rviz2.append(migrate_panel_tool_properties(panel_dict))
        elif 'rviz/Views' == panel_dict['Class']:
            panels_rviz2.append(migrate_panel_views(panel_dict))
        elif 'rviz/Time' == panel_dict['Class']:
            panels_rviz2.append(migrate_panel_time(panel_dict))
        else:
            print('Unknown panel type, skipping:' + panel_dict['Class'], file=sys.stderr)


    return panels_rviz2


def migrate_panel_displays(panel_dict):
    panel_rviz2 = copy.deepcopy(panel_dict)
    panel_rviz2['Class'] = 'rviz_common/Displays'
    # Not sure all display plugins will be present, so clear 'expanded'
    panel_rviz2['Property Tree Widget']['Expanded'] = None
    return panel_rviz2


def migrate_panel_selection(panel_dict):
    panel_rviz2 = copy.deepcopy(panel_dict)
    panel_rviz2['Class'] = 'rviz_common/Selection'
    return panel_rviz2


def migrate_panel_tool_properties(panel_dict):
    panel_rviz2 = copy.deepcopy(panel_dict)
    panel_rviz2['Class'] = 'rviz_common/Tool Properties'
    # Not sure all tools will be present, so clear 'expanded'
    panel_rviz2['Expanded'] = None
    return panel_rviz2


def migrate_panel_views(panel_dict):
    panel_rviz2 = copy.deepcopy(panel_dict)
    panel_rviz2['Class'] = 'rviz_common/Views'
    return panel_rviz2


def migrate_panel_time(panel_dict):
    panel_rviz2 = copy.deepcopy(panel_dict)
    panel_rviz2['Class'] = 'rviz_common/Time'
    return panel_rviz2


def migrate_visualization_manager(vm_dict):
    vm_rviz2 = {
        'Class': str(vm_dict.get('Class', '')),
        'Displays': migrate_displays(vm_dict['Displays']),
        'Enabled': bool(vm_dict.get('Enabled', True)),
        'Name': str(vm_dict.get('Name', 'root')),
        'Tools': migrate_visualization_manager_tools(vm_dict['Tools']),
        'Value': bool(vm_dict.get('Value', True)),
        'Views': migrate_visualization_manager_views(vm_dict['Views']),
        'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
        'Global Options': {
            'Background Color': str(vm_dict.get('Global Options', {}).get('Background Color', '48; 48; 48')),
            'Fixed Frame': str(vm_dict.get('Global Options', {}).get('Fixed Frame', 'map')),
            'Frame Rate': int(vm_dict.get('Global Options', {}).get('Frame Rate', 30)),
        }
    }
    return vm_rviz2


def migrate_displays(displays_list):
    migration_functions = {
        'rviz/Grid': auto_migrate_display,
        'rviz/Axes': migrate_display_axes,
        'rviz/Camera': migrate_display_camera,
        'rviz/FluidPressure': auto_migrate_display,
        'rviz/GridCells': auto_migrate_display,
        'rviz/Group': migrate_display_group,
        'rviz/Illuminance': auto_migrate_display,
        'rviz/Image': migrate_display_image,
        'rviz/InteractiveMarkers': migrate_display_interactive_markers,
        'rviz/LaserScan': migrate_display_laser_scan,
        'rviz/Map': migrate_display_map,
        'rviz/Marker': migrate_display_marker,
        'rviz/MarkerArray': migrate_display_marker_array,
        'rviz/Odometry': auto_migrate_display,
        'rviz/Path': auto_migrate_display,
        'rviz/PointCloud': migrate_display_point_cloud,
        'rviz/PointCloud2': migrate_display_point_cloud2,
        'rviz/PointStamped': auto_migrate_display,
        'rviz/Polygon': auto_migrate_display,
        'rviz/Pose': auto_migrate_display,
        'rviz/PoseArray': auto_migrate_display,
        'rviz/PoseWithCovariance': auto_migrate_display,
        'rviz/Range': auto_migrate_display,
        'rviz/RelativeHumidity': auto_migrate_display,
        'rviz/RobotModel': migrate_display_robot_model,
        'rviz/TF': migrate_display_tf,
        'rviz/Temperature': auto_migrate_display,
        'rviz/WrenchStamped': migrate_display_wrench_stamped,
    }

    displays_rviz2 = []
    for display_dict in displays_list:
        if display_dict['Class'] in migration_functions:
            displays_rviz2.append(migration_functions[display_dict['Class']](display_dict))
        else:
            print(f"Cannot migrate display {display_dict['Class']} - skipping", file=sys.stderr)
    return displays_rviz2 if displays_rviz2 else None


def auto_migrate_display(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    assert display_dict['Class'].startswith('rviz/')
    rviz2['Class'] = 'rviz_default_plugins' + display_dict['Class'][4:]
    if 'Topic' in display_dict:
        kwargs = {'name': display_dict['Topic']}
        if 'Queue Size' in display_dict:
            kwargs['depth'] = display_dict['Queue Size']
            del rviz2['Queue Size']
        if 'Unreliable' in display_dict:
            kwargs['reliable'] = not display_dict['Unreliable']
            del rviz2['Unreliable']

        rviz2['Topic'] = migrate_topic(**kwargs)
    return rviz2


def migrate_display_axes(display_dict):
    rviz2 = auto_migrate_display(display_dict)
    del_maybe(rviz2, 'Alpha')
    return rviz2


def migrate_display_camera(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/Camera'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Image Topic'],
        depth=display_dict['Queue Size'],
        reliable=not display_dict['Unreliable'])
    del rviz2['Image Topic']
    del rviz2['Queue Size']
    del rviz2['Unreliable']
    # TODO(sloretz) what is the ROS 2 equivalent to Transport Hint?
    del_maybe(rviz2, 'Transport Hint')
    del rviz2['Topic']['Filter size']
    rviz2['Far Plane Distance'] = 100
    # Not sure all plugins that wil be migrated, so clear visibility
    rviz2['Visibility'] = {}
    return rviz2


def migrate_display_group(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_common/Group'
    rviz2['Displays'] = migrate_displays(display_dict['Displays'])
    return rviz2


def migrate_display_image(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/Image'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Image Topic'],
        depth=display_dict['Queue Size'],
        reliable=not display_dict['Unreliable'])
    del rviz2['Image Topic']
    del rviz2['Queue Size']
    del rviz2['Unreliable']
    # TODO(sloretz) what is the ROS 2 equivalent to Transport Hint?
    del_maybe(rviz2, 'Transport Hint')
    del rviz2['Topic']['Filter size']
    return rviz2


def migrate_display_interactive_markers(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/InteractiveMarkers'
    rviz2['Interactive Markers Namespace'] = display_dict['Update Topic']
    del rviz2['Update Topic']
    return rviz2


def migrate_display_laser_scan(display_dict):
    rviz2 = auto_migrate_display(display_dict)
    rviz2['Max Intensity'] = 4096
    rviz2['Min Intensity'] = 0
    return rviz2


def migrate_display_map(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/Map'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Topic'],
        reliable=not display_dict['Unreliable'])
    rviz2['Update Topic'] = migrate_topic(
        name=display_dict['Topic'] + '_updates',
        reliable=not display_dict['Unreliable'])
    del rviz2['Update Topic']['Filter size']
    del rviz2['Unreliable']
    return rviz2


def migrate_display_marker(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/Marker'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Marker Topic'],
        depth=display_dict['Queue Size'])
    del rviz2['Marker Topic']
    del rviz2['Queue Size']
    return rviz2


def migrate_display_marker_array(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/MarkerArray'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Marker Topic'],
        depth=display_dict['Queue Size'])
    del rviz2['Marker Topic']
    del rviz2['Queue Size']
    del rviz2['Topic']['Filter size']
    return rviz2


def migrate_display_point_cloud(display_dict):
    rviz2 = auto_migrate_display(display_dict)
    rviz2['Max Intensity'] = 4096
    rviz2['Min Intensity'] = 0
    return rviz2


def migrate_display_point_cloud2(display_dict):
    rviz2 = auto_migrate_display(display_dict)
    rviz2['Max Intensity'] = 4096
    rviz2['Min Intensity'] = 0
    return rviz2


def migrate_display_robot_model(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/RobotModel'
    del rviz2['Robot Description']
    rviz2['Description File'] = ""
    rviz2['Description Source'] = "Topic"
    rviz2['Description Topic'] = migrate_topic(name="robot_description")
    del rviz2['Description Topic']['Filter size']
    rviz2['Mass Properties'] = {'Inertia': False, 'Mass': False}
    return rviz2


def migrate_display_tf(display_dict):
    rviz2 = auto_migrate_display(display_dict)
    del_maybe(rviz2, 'Marker Alpha')
    return rviz2


def migrate_display_wrench_stamped(display_dict):
    rviz2 = copy.deepcopy(display_dict)
    rviz2['Class'] = 'rviz_default_plugins/Wrench'
    rviz2['Topic'] = migrate_topic(
        name=display_dict['Topic'],
        depth=display_dict['Queue Size'],
        reliable=not display_dict['Unreliable'])
    del rviz2['Queue Size']
    del rviz2['Unreliable']
    rviz2['Accept NaN Values'] = False
    del_maybe(rviz2, 'Hide Small Values')
    return rviz2


def migrate_visualization_manager_tools(tools_list):
    tools_rviz2 = []
    for tool_dict in tools_list:
        name = tool_dict['Class']
        if name in ('rviz/Interact', 'rviz/MoveCamera', 'rviz/Select', 'rviz/FocusCamera'):
           rviz2_dict = copy.deepcopy(tool_dict)
           rviz2_dict['Class'] = 'rviz_default_plugins' + name[4:]
           tools_rviz2.append(rviz2_dict)
        elif name == 'rviz/Measure':
            tools_rviz2.append({
                'Class': 'rviz_default_plugins/Measure',
                'Line color': '128; 128; 0'
                })
        elif name == 'rviz/SetInitialPose':
            rviz2 = {
                'Class': 'rviz_default_plugins/SetInitialPose',
                'Topic': migrate_topic(name = tool_dict.get('Topic', '/initialpose')),
                }
            if 'X std deviation' in tool_dict:
                rviz2['Covariance x'] = float(tool_dict['X std deviation'])**2
            if 'Y std deviation' in tool_dict:
                rviz2['Covariance y'] = float(tool_dict['Y std deviation'])**2
            if 'Theta std deviation' in tool_dict:
                rviz2['Covariance yaw'] = float(tool_dict['Theta std deviation'])**2
            del rviz2['Topic']['Filter size']
            tools_rviz2.append(rviz2)
        elif name == 'rviz/SetGoal':
            rviz2 = {
                'Class': 'rviz_default_plugins/SetGoal',
                'Topic': migrate_topic(name = tool_dict.get('Topic', '/goal_pose')),
                }
            del rviz2['Topic']['Filter size']
            tools_rviz2.append(rviz2)
        elif name == 'rviz/PublishPoint':
            rviz2 = {
                'Class': 'rviz_default_plugins/PublishPoint',
                'Single click': bool(tool_dict.get('Single click', True)),
                'Topic': migrate_topic(name = tool_dict.get('Topic', '/clicked_point')),
                }
            del rviz2['Topic']['Filter size']
            tools_rviz2.append(rviz2)
        else:
            print(f"Unknown tool {tool_dict['Class']}, skipping", file=sys.stderr)

    return tools_rviz2


def migrate_visualization_manager_views(view_dict):
    rviz2 = {
            'Current': migrate_view(view_dict['Current']),
            'Saved': [],
        }
    if view_dict['Saved']:
        for saved_view in view_dict['Saved']:
            rviz2_view = migrate_view(saved_view)
            if rviz2_view:
                rviz2['Saved'].append(rviz2_view)
    if not rviz2['Saved']:
        rviz2['Saved'] = None
    return rviz2


def migrate_view(view_dict):
    rviz2_view = {}
    if 'rviz/Orbit' == view_dict['Class']:
        return migrate_view_orbit(view_dict)
    elif 'rviz/FPS' == view_dict['Class']:
        return migrate_view_fps(view_dict)
    elif 'rviz/ThirdPersonFollower' == view_dict['Class']:
        return migrate_view_third_person_follower(view_dict)
    elif 'rviz/TopDownOrtho' == view_dict['Class']:
        return migrate_view_top_down_ortho(view_dict)
    elif 'rviz/XYOrbit' == view_dict['Class']:
        return migrate_view_xy_orbit(view_dict)
    print(f"Unable to migrate view type {view_dict['Class']} - skipping", file=sys.stderr)


def migrate_view_orbit(view_dict):
    rviz2_view = copy.deepcopy(view_dict)
    rviz2_view['Class'] = 'rviz_default_plugins/Orbit'
    rviz2_view['Value'] = 'Orbit (rviz)'
    del_maybe(rviz2_view, 'Field of View')
    return rviz2_view


def migrate_view_fps(view_dict):
    rviz2_view = copy.deepcopy(view_dict)
    rviz2_view['Class'] = 'rviz_default_plugins/FPS'
    rviz2_view['Value'] = 'FPS (rviz_default_plugins)'
    del_maybe(rviz2_view, 'Roll')
    return rviz2_view


def migrate_view_third_person_follower(view_dict):
    rviz2_view = copy.deepcopy(view_dict)
    rviz2_view['Class'] = 'rviz_default_plugins/ThirdPersonFollower'
    rviz2_view['Value'] = 'ThirdPersonFollower (rviz_default_plugins)'
    del_maybe(rviz2_view, 'Field of View')
    return rviz2_view


def migrate_view_top_down_ortho(view_dict):
    rviz2_view = copy.deepcopy(view_dict)
    rviz2_view['Class'] = 'rviz_default_plugins/TopDownOrtho'
    rviz2_view['Value'] = 'TopDownOrtho (rviz_default_plugins)'
    return rviz2_view


def migrate_view_xy_orbit(view_dict):
    rviz2_view = copy.deepcopy(view_dict)
    rviz2_view['Class'] = 'rviz_default_plugins/XYOrbit'
    rviz2_view['Value'] = 'XYOrbit (rviz_default_plugins)'
    del_maybe(rviz2_view, 'Field of View')
    return rviz2_view


def migrate_topic(name, depth=5, reliable=False):
    return {
        'Depth': depth,
        'Durability Policy': 'Volatile',
        'Filter size': 10,
        'History Policy': 'Keep Last',
        'Reliability Policy': 'Reliable' if reliable else 'Best Effort',
        'Value': str(name),
    }


def migrate_window_geometry(window_geometry):
    # Return unmodified
    return copy.deepcopy(window_geometry)


def migrate_to_rviz2(rviz1_config):
    """Given an RViz 1 config dictionary, return an RViz 2 config dictionary."""

    rviz1_sections = {
        'Panels',
        'Preferences',
        'Toolbars',
        'Visualization Manager',
        'Window Geometry'}

    unknown_top_level = set(rviz1_config.keys()).difference(rviz1_sections)
    if unknown_top_level:
        print(
            "I don't know how to convert these sections - skipping" + repr(
                unknown_top_level), file=sys.stderr)

    rviz2_config = {}

    p_key = 'Panels'
    assert p_key in rviz1_sections
    rviz2_config[p_key] = migrate_panels(rviz1_config[p_key])

    vm_key = 'Visualization Manager'
    assert vm_key in rviz1_sections
    rviz2_config[vm_key] = migrate_visualization_manager(rviz1_config[vm_key])

    wg_key = 'Window Geometry'
    assert wg_key in rviz1_sections
    rviz2_config[wg_key] = migrate_window_geometry(rviz1_config[wg_key])

    return rviz2_config


def parse_arguments():
    parser = argparse.ArgumentParser(description='Convert RViz 1 files to 2')
    parser.add_argument('input', metavar='INPUT', help='Path to RViz 1 config')
    parser.add_argument(
        'output', metavar='OUTPUT', help='Path to write RViz 2 config')

    return parser.parse_args()


def parse_yaml_file(path):
    with open(path, 'r') as fin:
        return yaml.safe_load(fin)


def write_yaml_file(output, yaml_dict):
    if '-' == output:
        yaml.dump(yaml_dict, stream=sys.stdout)
    else:
        with open(output, 'w') as fout:
            yaml.dump(yaml_dict, stream=fout)


def main():
    args = parse_arguments()
    rviz1_config = parse_yaml_file(args.input)
    rviz2_config = migrate_to_rviz2(rviz1_config)
    write_yaml_file(args.output, rviz2_config)


if __name__ == '__main__':
    main()
