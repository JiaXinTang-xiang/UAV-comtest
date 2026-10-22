"""Geometry, coordinate and trajectory regressions for the competition entry."""
import math
from pathlib import Path

import pytest
import yaml

from uav_competition.frames import MapFrame
from uav_competition.path_planner import CoursePlanner
from uav_competition.trajectory import smooth_path, PathTracker, limit_velocity

PACKAGE = Path(__file__).parents[1]


def test_rotated_nested_include_and_arbitrary_names(tmp_path):
    model = tmp_path/'models/partition'
    model.mkdir(parents=True)
    (model/'model.sdf').write_text('''<sdf version="1.6"><model name="arbitrary"><static>true</static>
      <link name="l"><pose>1 0 0 0 0 0</pose><collision name="c"><pose>0 0 1.5 0 0 0</pose>
      <geometry><box><size>4 0.2 3</size></box></geometry></collision></link></model></sdf>''')
    world = tmp_path/'course.world'
    world.write_text('''<sdf version="1.6"><world name="test"><include><uri>model://partition</uri>
      <pose>15 15 0 0 0 1.5707963267948966</pose></include></world></sdf>''')
    planner=CoursePlanner(world,.25,bounds=(10,20,10,20),model_paths=[str(tmp_path/'models')])
    assert not planner.free((15,16))
    assert planner.free((16,16))
    assert planner.plan((13,16),(17,16))
    assert not planner.free((10.1,15))


def test_unsupported_mesh_and_missing_model_are_errors(tmp_path):
    world=tmp_path/'world.sdf'
    world.write_text('''<sdf><world name="w"><model name="official"><static>true</static><link name="l">
      <collision name="c"><geometry><mesh><uri>file://wall.stl</uri></mesh></geometry></collision>
      </link></model></world></sdf>''')
    with pytest.raises(ValueError,match='Unsupported static geometry'):
        CoursePlanner(world)
    world.write_text('<sdf><world name="w"><include><uri>model://does-not-exist</uri></include></world></sdf>')
    with pytest.raises(ValueError,match='Cannot resolve'):
        CoursePlanner(world)


def test_world_map_local_round_trip():
    frame=MapFrame({'map_origin_world':[12,-4,0], 'map_yaw_world':math.pi/2,
                    'px4_enu_yaw_world':0.2})
    assert frame.to_world(2,3) == pytest.approx((9,-2))
    assert frame.to_map(*frame.to_world(2,3)) == pytest.approx((2,3))
    assert frame.map_to_local(*frame.local_to_map(1.3,-2.1)) == pytest.approx((1.3,-2.1))


@pytest.mark.parametrize('route',['left','right'])
def test_both_routes_smooth_without_cutting_walls(route):
    cfg=yaml.safe_load((PACKAGE/'config/mission.yaml').read_text())
    profile=cfg['route_profiles'][route]
    planner=CoursePlanner(PACKAGE/'worlds/competition_course.world',profile['clearance'])
    start=cfg['mission']['takeoff']
    for goal in [*profile['targets'].values(),cfg['mission']['landing_target']]:
        raw=planner.plan(start,goal)
        path=smooth_path(raw,planner.visible)
        assert path[0] == tuple(start) and path[-1] == tuple(goal)
        assert all(planner.visible(a,b) for a,b in zip(path,path[1:]))
        start=goal


def test_dynamic_obstacle_replan_then_smooth():
    planner=CoursePlanner(PACKAGE/'worlds/competition_course.world',.7)
    planner.dynamic=[(3.7,y) for y in (4.8,4.9,5,5.1,5.2)]
    raw=planner.plan((5,5),(2,5))
    path=smooth_path(raw,planner.visible)
    assert len(path)>2
    assert all(planner.visible(a,b) for a,b in zip(path,path[1:]))


def test_rounding_and_tracker_do_not_cut_blocked_corner():
    path=[(0,0),(1,0),(1,1)]
    free=lambda a,b: all(x<=1.001 and y>=-.001 for x,y in [a,b])
    rounded=smooth_path(path,free)
    assert len(rounded)>3
    tracker=PathTracker(rounded)
    targets=[]
    for point in rounded:
        target,error=tracker.target(point,.15,free)
        assert error<.1
        assert free(point,target)
        targets.append(target)
    assert targets[-1] == pytest.approx((1,1))


def test_reference_velocity_changes_are_acceleration_limited():
    previous=[.4,0]
    for _ in range(40):
        velocity=limit_velocity(previous,[0,.4],.35,.05)
        assert math.dist(previous,velocity) <= .35*.05+1e-9
        previous=velocity
    assert velocity==pytest.approx([0,.4])


@pytest.mark.parametrize('route',['left','right'])
def test_acceleration_limited_reference_can_follow_complete_route(route):
    """Kinematic reference regression; this is not a PX4/physics flight test."""
    cfg=yaml.safe_load((PACKAGE/'config/mission.yaml').read_text())
    profile=cfg['route_profiles'][route]
    planner=CoursePlanner(PACKAGE/'worlds/competition_course.world',profile['clearance'])
    position=cfg['mission']['takeoff']
    elapsed=0
    for goal in [*profile['targets'].values(),cfg['mission']['landing_target']]:
        path=smooth_path(planner.plan(position,goal),planner.visible)
        tracker=PathTracker(path)
        velocity=[0,0]
        for _ in range(3000):
            target,error=tracker.target(position,.2,planner.visible)
            assert error < profile['corridor_limit']
            distance=math.dist(position,target)
            speed=min(tracker.speed(profile['max_xy_speed'],.15,.35),math.sqrt(2*.35*distance))
            desired=[(target[k]-position[k])*speed/distance if distance else 0 for k in range(2)]
            velocity=limit_velocity(velocity,desired,.35,.05)
            next_=[position[k]+velocity[k]*.05 for k in range(2)]
            assert planner.visible(position,next_)
            position=next_
            elapsed+=.05
            if math.dist(position,goal)<.10:
                break
        else:
            pytest.fail('Reference failed to reach the next region')
    assert elapsed < cfg['mission']['max_mission_seconds']
