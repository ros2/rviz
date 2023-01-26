#!/usr/bin/env python3

import os
import subprocess
import sys

import yaml


def script():
    return os.environ['_SCRIPT_PATH']


def config(name):
    return os.path.join(os.environ['_CONFIG_PATH'], name)


def test_convert_all_supported_configs():
    result = subprocess.run(
        [sys.executable, script(), config('all_supported_configs.rviz'), '-'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True)

    output = result.stdout.decode()
    assert len(output) > 0
    assert len(result.stderr) == 0

    yaml_output = yaml.safe_load(output)
    assert 'Panels' in yaml_output.keys()


def test_convert_fuse_simple_tutorial_config():
    result = subprocess.run(
        [sys.executable, script(), config('fuse_simple_tutorial.rviz'), '-'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True)

    output = result.stdout.decode()
    assert len(output) > 0
    assert len(result.stderr) == 0

    yaml_output = yaml.safe_load(output)
    assert 'Background Color' in yaml_output['Visualization Manager']['Global Options'].keys()


def test_convert_range_sensor_tutorial_config():
    result = subprocess.run(
        [sys.executable, script(), config('range_sensor_tutorial.rviz'), '-'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True)

    output = result.stdout.decode()
    assert len(output) > 0
    assert len(result.stderr) == 0

    yaml_output = yaml.safe_load(output)
    assert 'Panels' in yaml_output.keys()
    assert 'Background Color' in yaml_output['Visualization Manager']['Global Options'].keys()


def test_convert_all_ros1():
    result = subprocess.run(
        [sys.executable, script(), config('all_ros1.rviz'), '-'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True)

    output = result.stdout.decode()
    assert len(output) > 0
    assert len(result.stderr) != 0

    actual_stderr = result.stderr.decode()
    expected_stderr = (
        "Cannot migrate display rviz/AccelStamped - skipping",
        "Cannot migrate display rviz/DepthCloud - skipping",
        "Cannot migrate display rviz/Effort - skipping",
        "Cannot migrate display rviz/TwistStamped - skipping",
        "Cannot migrate display rviz_plugin_tutorials/Imu - skipping",
        "Unable to migrate view type rviz/FrameAligned - skipping")

    for expected_line, line in zip(expected_stderr, actual_stderr.split('\n')):
        # Strip line endings for Linux/Windows compatibility
        assert expected_line.strip() == line.strip()

    yaml_output = yaml.safe_load(output)
    assert 'Panels' in yaml_output.keys()
