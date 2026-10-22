#!/usr/bin/env python3
"""Summarize real obstacle injection and lidar replanning evidence, never invent PASS."""
import argparse
import json
from pathlib import Path


def evaluate(mission, injection):
    associated = (mission.get('started_unix', float('inf')) <= injection.get('unix_time', 0)
                  <= mission.get('finished_unix', 0))
    after_spawn = [r for r in mission.get('replans', [])
                   if associated and r.get('unix_time', 0) >= injection['unix_time']
                   and r.get('region') == injection.get('mission')]
    detected = [r for r in after_spawn
                if r.get('reason') == 'lidar_path_blocked' and r.get('obstacle_points', 0) > 0]
    # Retries may be the successful plan after the first blocked attempt.
    changed = []
    pending = False
    for replan in after_spawn:
        if replan.get('reason') == 'region_transition':
            pending = False
        if replan.get('reason') == 'lidar_path_blocked' and replan.get('obstacle_points', 0) > 0:
            pending = True
        if pending and replan.get('path') and replan.get('obstacle_points', 0) > 0:
            changed.append(replan)
            pending = False
    checks = {
        'real_obstacle_spawn_record': associated and injection.get('action') == 'add' and injection.get('success') is True,
        'lidar_triggered_replanning': bool(detected),
        'replanning_produced_path': bool(changed),
        'mission_completed_afterwards': mission.get('state') == 'COMPLETE' and not mission.get('failure'),
        'all_regions_visited': all(name in [v['region'] for v in mission.get('visited', [])]
                                   for name in ['AREA_2','AREA_3','AREA_4','AREA_5','LAND_APPROACH']),
    }
    return {'status':'EVIDENCE_COMPLETE' if all(checks.values()) else 'INCOMPLETE',
            'checks':checks,'route':mission.get('route'),
            'planning_seconds':[r.get('planning_seconds') for r in changed],
            'scope':'Recorded spawn + live lidar replan + completed mission; not a physical clearance or optimality certificate'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission',type=Path)
    parser.add_argument('--injection',type=Path,default=Path(__file__).resolve().parents[1]/'logs/obstacle_injection.json')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    mission=json.loads(args.mission.read_text())
    injection=json.loads(args.injection.read_text())
    result=evaluate(mission,injection)
    text=json.dumps(result,ensure_ascii=False,indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)
    return 0 if result['status']=='EVIDENCE_COMPLETE' else 1


if __name__=='__main__':
    raise SystemExit(main())
