import copy
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
from rover_bringup import profile
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def with_changes(**sections):
    p = copy.deepcopy(profile.DEFAULT)
    for section, values in sections.items():
        p[section].update(values)
    return p


@pytest.fixture
def rover_home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROVER_HOME', str(tmp_path))
    return tmp_path


# ---------------- validate ----------------

def test_default_is_valid():
    p, errors = profile.validate(profile.DEFAULT)
    assert errors == [] and p == profile.DEFAULT


def test_missing_values_fall_back_to_default():
    p, errors = profile.validate({'platform': {'length': 0.6}})
    assert errors == []
    assert p['platform']['length'] == 0.6
    assert p['platform']['width'] == profile.DEFAULT['platform']['width']
    assert profile.validate(None) == (profile.DEFAULT, [])


@pytest.mark.parametrize('value, expected', [
    (True, True), (False, False), ('false', False), ('False', False), ('no', False),
    ('true', True), ('on', True), (0, False), (1, True)])
def test_bools_are_parsed_not_cast(value, expected):
    p, errors = profile.validate({'lidar': {'mask': value}})
    assert errors == [] and p['lidar']['mask'] is expected


@pytest.mark.parametrize('value', ['maybe', 2, None, [1]])
def test_bad_bool_is_an_error(value):
    _, errors = profile.validate({'lidar': {'mask': value}})
    assert len(errors) == 1 and errors[0].startswith('lidar.mask')


@pytest.mark.parametrize('value', [0.0, 5.0, float('nan'), float('inf'), 'abc', None, True])
def test_bad_numbers_are_rejected(value):
    p, errors = profile.validate({'platform': {'length': value}})
    assert len(errors) == 1 and errors[0].startswith('platform.length')
    assert p['platform']['length'] == profile.DEFAULT['platform']['length']


def test_numeric_strings_and_integer_fields():
    p, errors = profile.validate({'platform': {'length': '0.5'},
                                  'drive': {'ticks_per_rev_left': 90.4}})
    assert errors == []
    assert p['platform']['length'] == 0.5
    assert p['drive']['ticks_per_rev_left'] == 90
    assert isinstance(p['drive']['ticks_per_rev_left'], int)


def test_unknown_drive_type_is_rejected():
    _, errors = profile.validate({'drive': {'type': 'mecanum'}})
    assert errors == ["drive.type: 'mecanum'"]


# ---------------- lidar mask ----------------

@pytest.mark.parametrize('mask_from, mask_to, yaw, native', [
    (165, 185, 0, [175.0, 195.0]),   # behind: what the robot used to have hard-coded
    (0, 20, 0, [340.0, 360.0]),      # touching 0 from the left
    (340, 360, 0, [0.0, 20.0]),      # touching 0 from the right
    (340, 0, 0, [0.0, 20.0]),        # same sector, 0 written as 0
    (170, 190, 90, [260.0, 280.0]),  # lidar turned 90 deg left: sector is at its 80..100
    (170, 190, 180, None),           # lidar turned around: sector crosses its own 0
    (350, 10, 0, None),              # crosses 0
    (30, 30, 0, None),               # empty
])
def test_native_mask(mask_from, mask_to, yaw, native):
    lidar = dict(profile.DEFAULT['lidar'], mask_from=mask_from, mask_to=mask_to, yaw=yaw)
    assert profile.native_mask(lidar) == native


def test_inexpressible_mask_is_a_validation_error_only_when_enabled():
    _, errors = profile.validate({'lidar': {'mask': True, 'mask_from': 350, 'mask_to': 10}})
    assert len(errors) == 1 and errors[0].startswith('lidar.mask')
    _, errors = profile.validate({'lidar': {'mask': False, 'mask_from': 350, 'mask_to': 10}})
    assert errors == []


# ---------------- derive ----------------

def test_footprint_includes_wheels_sticking_out():
    d = profile.derive(profile.DEFAULT)
    # Wheels: track 0.63 + width 0.08 -> 0.355 each side; + 0.02 padding.
    assert d['footprint'] == [[0.297, 0.375], [0.297, -0.375], [-0.297, -0.375], [-0.297, 0.375]]
    assert d['size'] == pytest.approx({'length': 0.5945, 'width': 0.75}, abs=1e-3)
    assert d['inscribed_radius'] == 0.297
    assert d['inflation_radius'] >= d['circumscribed_radius']


def test_footprint_follows_offset_rotation_centre():
    # Small wheels close together, so the platform alone sets the length.
    d = profile.derive(with_changes(drive={'center_x': 0.1, 'track': 0.3, 'wheel_width': 0.02,
                                           'wheel_diameter': 0.05, 'wheelbase': 0.1},
                                    safety={'padding': 0.0}))
    xs = [x for x, _ in d['footprint']]
    # Platform 0.45 long, its centre 0.1 m behind the rotation centre.
    assert max(xs) == pytest.approx(0.125) and min(xs) == pytest.approx(-0.325)
    assert d['platform_offset_x'] == -0.1


def test_diff2_has_one_axle():
    d = profile.derive(with_changes(drive={'type': 'diff2'}))
    assert len(d['wheels']) == 2 and all(w['x'] == 0.0 for w in d['wheels'])


def test_sensors_are_relative_to_base_link():
    d = profile.derive(with_changes(lidar={'x': 0.1, 'height': 0.3, 'yaw': 90.0},
                                    drive={'center_x': 0.05}))
    r = profile.DEFAULT['drive']['wheel_diameter'] / 2
    assert d['lidar']['x'] == pytest.approx(0.05)
    assert d['lidar']['z'] == pytest.approx(0.3 - r)
    assert d['lidar']['yaw'] == pytest.approx(1.5708, abs=1e-4)


def test_slip_factor_scales_the_commanded_separation():
    d = profile.derive(with_changes(drive={'turn_slip_factor': 1.5}))
    assert d['wheel_separation_cmd'] == pytest.approx(0.63 * 1.5)


# ---------------- files ----------------

def test_save_and_load_round_trip(rover_home):
    p = with_changes(platform={'length': 0.5})
    saved, errors = profile.save(p)
    assert errors == [] and saved == p
    assert profile.load() == p
    assert not list(rover_home.glob('.*.tmp'))


def test_save_rejects_invalid_and_keeps_the_old_file(rover_home):
    profile.save(profile.DEFAULT)
    saved, errors = profile.save({'platform': {'length': -1}})
    assert saved is None and errors
    assert profile.load() == profile.DEFAULT


def test_load_survives_a_broken_file(rover_home, capsys):
    (rover_home / 'robot.yaml').write_text('platform: {length: [unclosed')
    assert profile.load() == profile.DEFAULT
    (rover_home / 'robot.yaml').write_text('platform: {length: 99}\n')
    assert profile.load() == profile.DEFAULT
    assert 'platform.length' in capsys.readouterr().err


def generated(prof, tmp_path):
    target = tmp_path / 'gen' / 'nav2_params.yaml'
    out = profile.nav2_params(prof, PACKAGE / 'config' / 'nav2_params.yaml', target)
    assert out == str(target)
    return yaml.safe_load(target.read_text())


def test_nav2_params_get_the_robots_shape(tmp_path):
    params = generated(profile.DEFAULT, tmp_path)
    d = profile.derive(profile.DEFAULT)
    expected = '[' + ','.join(f'[{x},{y}]' for x, y in d['footprint']) + ']'
    for costmap in ('local_costmap', 'global_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        assert p['footprint'] == expected
        assert p['inflation_layer']['inflation_radius'] == d['inflation_radius']
    monitor = params['collision_monitor']['ros__parameters']
    assert monitor['people']['topic'] == '/perception/people_cloud'
    # Going round people: both costmaps carry them, with a clearing ring.
    for costmap in ('local_costmap', 'global_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        assert p['plugins'].index('people_layer') < p['plugins'].index('inflation_layer')
        layer = p['people_layer']
        assert layer['people']['topic'] == '/perception/people_obstacles'
        assert layer['people']['marking'] and not layer['people']['clearing']
        assert layer['people_clearing']['clearing'] and not layer['people_clearing']['marking']
    # Near people a speed cap; ahead of them a trajectory-aware brake, no fixed stop box.
    near, ahead = monitor['PeopleSlowdown'], monitor['PeopleApproach']
    assert near['action_type'] == 'limit' and near['linear_limit'] == profile.PEOPLE_NEAR_SPEED
    assert near['angular_limit'] == profile.MAX_TURN_RATE
    assert ahead['action_type'] == 'approach'
    assert ahead['time_before_collision'] == profile.PEOPLE_TIME_BEFORE_COLLISION
    assert ahead['footprint_topic'] == '/local_costmap/published_footprint'
    assert 'PeopleStop' not in monitor['polygons']


def test_the_shipped_params_file_is_the_default_profile():
    # The simulator uses config/nav2_params.yaml as it is: it must say what
    # the launch files would generate for the default robot.
    source = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
    assert profile.apply_profile(copy.deepcopy(source), profile.DEFAULT) == source


# ---------------- speed ----------------

def test_default_top_speed_is_5_kmh():
    assert profile.DEFAULT['drive']['max_speed'] * 3.6 == pytest.approx(5.0, abs=0.01)


@pytest.mark.parametrize('speed', [0.3, 0.8, 1.39, 2.0])
def test_people_brake_time_covers_reacting_and_braking(speed):
    # Up to the highest max_speed a profile may have (LIMITS: 2 m/s).
    assert profile.stopping_distance(speed) / speed <= profile.PEOPLE_TIME_BEFORE_COLLISION


@pytest.mark.parametrize('speed', [0.08, 0.2, 0.3, 0.5])
def test_low_speeds_keep_the_original_tuning(speed):
    assert profile.min_turn_radius(speed) == 0.5
    assert profile.approach_distance(speed) == 0.4


@pytest.mark.parametrize('speed', [0.6, 1.0, 1.39, 2.0])
def test_fast_tuning_follows_the_physics(speed):
    r = profile.min_turn_radius(speed)
    assert speed * speed / r <= profile.LATERAL_ACCEL + 0.01
    # The slow-down before a goal starts at least a braking distance out.
    assert profile.approach_distance(speed) >= speed * speed / (2 * profile.DECEL)


def test_speed_limits_follow_max_speed(tmp_path):
    prof = with_changes(drive={'max_speed': 1.0})
    params = generated(prof, tmp_path)
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [1.0, 0.0, profile.MAX_TURN_RATE]
    assert smoother['max_accel'][0] == profile.ACCEL
    assert smoother['max_decel'][0] == -profile.DECEL
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['regulated_linear_scaling_min_radius'] == profile.min_turn_radius(1.0)
    assert follow['approach_velocity_scaling_dist'] == profile.approach_distance(1.0)
    assert follow['max_lookahead_dist'] >= follow['min_lookahead_dist']
    # Every wheel can reach full speed while turning as fast as allowed.
    d = profile.derive(prof)
    assert d['wheel_speed_cap'] >= 1.0 + profile.MAX_TURN_RATE * d['wheel_separation_cmd'] / 2


@pytest.mark.parametrize('value', [0.0, 5.0])
def test_max_speed_has_limits(value):
    _, errors = profile.validate({'drive': {'max_speed': value}})
    assert errors and errors[0].startswith('drive.max_speed')


def test_xacro_args_are_plain_mappings():
    args = profile.xacro_args(profile.DEFAULT)
    assert 'drive_type:=skid4' in args
    assert all(':=' in a and ' ' not in a for a in args)


@pytest.mark.skipif(shutil.which('xacro') is None, reason='xacro not installed')
@pytest.mark.parametrize('drive_type, wheels', [('skid4', 4), ('diff2', 2)])
def test_urdf_builds_from_the_profile(drive_type, wheels):
    p = with_changes(drive={'type': drive_type}, lidar={'yaw': 90.0})
    urdf = subprocess.run(
        ['xacro', str(PACKAGE / 'urdf' / 'rover.urdf.xacro'), *profile.xacro_args(p)],
        check=True, capture_output=True, text=True, env=dict(os.environ)).stdout
    robot = ET.fromstring(urdf)
    joints = {j.get('name'): j for j in robot.iter('joint')}
    assert sum(1 for n in joints if n.endswith('_wheel_joint')) == wheels
    d = profile.derive(p)
    laser = joints['laser_joint'].find('origin')
    xyz = [float(v) for v in laser.get('xyz').split()]
    assert xyz == pytest.approx([d['lidar']['x'], d['lidar']['y'], d['lidar']['z']], abs=1e-4)
    assert float(laser.get('rpy').split()[2]) == pytest.approx(d['lidar']['yaw'], abs=1e-4)


def test_reversed_drive_is_a_plain_switch():
    assert profile.DEFAULT['drive']['reversed'] is False
    p, errors = profile.validate({'drive': {'reversed': 'true'}})
    assert errors == [] and p['drive']['reversed'] is True
