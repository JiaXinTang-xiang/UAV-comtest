"""2D clearance-aware A* and collision-checked line-of-sight simplification.

Obstacles come from the same world collision boxes as Gazebo. The vehicle
is represented by a disk; clearance includes its radius and tracking margin.
"""
import heapq
import math
import xml.etree.ElementTree as ET
import time


class CoursePlanner:
    def __init__(self, world, clearance=0.65, resolution=0.1, altitude=2.5):
        self.clearance, self.resolution = clearance, resolution
        self.boxes = []
        for model in ET.parse(world).getroot().find('world').findall('model'):
            if not model.get('name', '').startswith(('wall_', 'boundary_')):
                continue
            pose = [float(x) for x in model.findtext('pose', '0 0 0 0 0 0').split()]
            for collision in model.findall('link/collision'):
                size = collision.findtext('geometry/box/size')
                if not size:
                    continue
                sx, sy, sz = map(float, size.split())
                if abs(pose[5]) > 1e-6:
                    raise ValueError('Rotated wall requires polygon collision support')
                if pose[2] - sz / 2 <= altitude <= pose[2] + sz / 2:
                    self.boxes.append((pose[0]-sx/2, pose[0]+sx/2, pose[1]-sy/2, pose[1]+sy/2))
        self.dynamic = []

    def blocked_reason(self, p):
        """Distinguish physical walls from remembered lidar observations."""
        x, y = p
        if not (self.clearance <= x <= 10-self.clearance and self.clearance <= y <= 10-self.clearance):
            return 'outside planning bounds'
        for x0, x1, y0, y1 in self.boxes:
            if math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1)) <= self.clearance:
                return f'static wall box={(x0, x1, y0, y1)}'
        for q in self.dynamic:
            distance = math.dist(p, q)
            if distance <= self.clearance:
                return f'lidar observation={q}, distance={distance:.3f}m'
        return ''

    def free(self, p):
        return not self.blocked_reason(p)

    def visible(self, a, b):
        steps = max(1, math.ceil(math.dist(a, b)/(self.resolution/2)))
        return all(self.free((a[0]+(b[0]-a[0])*i/steps, a[1]+(b[1]-a[1])*i/steps)) for i in range(steps+1))

    def plan(self, start, goal, timeout=2.0):
        deadline = time.monotonic() + timeout
        start, goal = tuple(start), tuple(goal)
        for label, point_to_check in (('start', start), ('goal', goal)):
            reason = self.blocked_reason(point_to_check)
            if reason:
                raise ValueError(f'{label}={point_to_check} blocked: {reason}; clearance={self.clearance}m')
        if self.visible(start, goal):
            return [start, goal]
        r = self.resolution
        def point(k): return (k[0]*r, k[1]*r)
        def key(p): return (round(p[0]/r), round(p[1]/r))
        first = key(start)
        if not self.visible(start, point(first)):
            raise ValueError('Start cannot connect to planning grid')
        queue = [(math.dist(start, goal), first)]
        cost, parent = {first: 0.0}, {}
        visited = set()
        found = None
        moves = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]
        while queue:
            if time.monotonic() > deadline:
                raise ValueError('Planning time budget exceeded')
            _, current = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            p = point(current)
            if math.dist(p, goal) <= 2*r and self.visible(p, goal):
                found = current
                break
            for dx, dy in moves:
                nxt = current[0]+dx, current[1]+dy
                q = point(nxt)
                if not self.visible(p, q):
                    continue
                g = cost[current] + r*math.hypot(dx, dy)
                if g < cost.get(nxt, float('inf')):
                    cost[nxt], parent[nxt] = g, current
                    heapq.heappush(queue, (g+math.dist(q, goal), nxt))
        if found is None:
            raise ValueError(f'No collision-free path: {start} -> {goal}')
        path = [goal, point(found)]
        while found in parent:
            found = parent[found]
            path.append(point(found))
        path.append(start)
        path.reverse()
        simplified = [start]
        i = 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not self.visible(path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified
