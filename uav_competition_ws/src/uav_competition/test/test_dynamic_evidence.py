"""A completed flight alone must never be reported as a dynamic avoidance test."""
import copy
import importlib.util
from pathlib import Path

script = Path(__file__).resolve().parents[4]/'scripts/check_dynamic_avoidance.py'
spec = importlib.util.spec_from_file_location('check_dynamic_avoidance', script)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def records():
    mission = {'started_unix':100, 'finished_unix':200, 'state':'COMPLETE',
               'visited':[{'region':r} for r in ('AREA_2','AREA_3','AREA_4','AREA_5','LAND_APPROACH')],
               'replans':[{'unix_time':151, 'region':'AREA_4', 'reason':'lidar_path_blocked',
                           'obstacle_points':8, 'path':[[0,0],[1,1],[2,0]]}]}
    injection = {'unix_time':150, 'action':'add', 'success':True, 'mission':'AREA_4'}
    return mission, injection


def test_spawn_live_replan_completed_mission_evidence():
    mission, injection = records()
    assert checker.evaluate(mission,injection)['status'] == 'EVIDENCE_COMPLETE'


def test_stale_spawn_old_replan_or_region_transition_cannot_pass():
    original, injection = records()
    for patch in ({'unix_time':149}, {'reason':'region_transition'}, {'region':'AREA_3'}, {'path':[]}):
        mission = copy.deepcopy(original)
        mission['replans'][0].update(patch)
        assert checker.evaluate(mission,injection)['status'] == 'INCOMPLETE'
    injection['unix_time'] = 50
    assert checker.evaluate(original,injection)['status'] == 'INCOMPLETE'
