"""Collision-checked rounded paths and bounded reference velocity changes."""
import math


def smooth_path(path, visible, radius=0.30, spacing=0.05):
    points = [tuple(p) for p in path]
    if len(points) < 3:
        return points
    result = [points[0]]
    for a, b, c in zip(points, points[1:], points[2:]):
        ab, bc = math.dist(a, b), math.dist(b, c)
        if min(ab, bc) < 1e-6:
            continue
        cut = min(radius, ab*0.3, bc*0.3)
        accepted = False
        # A rounded corner may cut into an inflated wall. Shrink the radius,
        # then retain the original corner if every tested radius is blocked.
        for _ in range(5):
            entry = tuple(b[i]+(a[i]-b[i])*cut/ab for i in range(2))
            exit_ = tuple(b[i]+(c[i]-b[i])*cut/bc for i in range(2))
            n = max(4, math.ceil(2*cut/spacing))
            curve = [tuple((1-t)**2*entry[i]+2*(1-t)*t*b[i]+t*t*exit_[i]
                           for i in range(2)) for t in (j/n for j in range(n+1))]
            candidate = [result[-1], *curve, c]
            if all(visible(p, q) for p, q in zip(candidate, candidate[1:])):
                result.extend(p for p in curve if math.dist(result[-1], p) > 1e-6)
                accepted = True
                break
            cut *= 0.5
        if not accepted:
            result.append(b)
    result.append(points[-1])
    if not all(visible(a, b) for a, b in zip(result, result[1:])):
        return points
    return result


def project(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    u = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.0
    return u, (a[0]+u*dx, a[1]+u*dy)


class PathTracker:
    """Progress along nearby segments, never jump to a distant crossing."""
    def __init__(self, path):
        self.path = path
        self.index = 0

    def target(self, xy, lookahead, visible):
        while self.index < len(self.path)-2:
            u, _ = project(xy, self.path[self.index], self.path[self.index+1])
            if u < 0.98 and math.dist(xy, self.path[self.index+1]) > 0.06:
                break
            self.index += 1
        _, nearest = project(xy, self.path[self.index], self.path[self.index+1])
        remaining, target = lookahead, nearest
        for next_ in self.path[self.index+1:]:
            length = math.dist(target, next_)
            if length >= remaining and length > 0:
                target = tuple(target[i]+(next_[i]-target[i])*remaining/length for i in range(2))
                break
            remaining -= length
            target = tuple(next_)
        # The straight tracking chord is checked too, not just the curve.
        if not visible(xy, target):
            target = tuple(self.path[self.index+1])
        return target, math.dist(xy, nearest)

    def speed(self, maximum, corner_speed, acceleration):
        # Slow down ahead of any remaining sharp corner; tight curves and the
        # final target also get a lower speed through the stopping envelope.
        speed = maximum
        distance = 0.0
        for j in range(self.index+1, len(self.path)-1):
            a, b, c = self.path[j-1:j+2]
            if j > self.index+1:
                distance += math.dist(a, b)
            ab, bc = math.dist(a, b), math.dist(b, c)
            if ab*bc <= 1e-9:
                continue
            angle = math.acos(max(-1, min(1, sum((b[k]-a[k])*(c[k]-b[k]) for k in range(2))/(ab*bc))))
            if angle > 0.08:
                curvature = angle/max(0.025, (ab+bc)/2)
                turn = min(maximum, max(corner_speed, math.sqrt(acceleration/max(curvature, 1e-6))))
                speed = min(speed, math.sqrt(turn*turn+2*acceleration*distance))
        return speed


def limit_velocity(previous, desired, acceleration, dt):
    delta = [desired[i]-previous[i] for i in range(2)]
    length = math.hypot(*delta)
    scale = min(1.0, acceleration*dt/length) if length else 1.0
    return [previous[i]+delta[i]*scale for i in range(2)]
