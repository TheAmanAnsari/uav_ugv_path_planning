import math
from config import o, d, height
import numpy as np

def eucl_dist(P1, P2, z=None):
    x1, y1 = P1
    x2, y2 = P2
    if z is None:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    else:
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + z**2)
    
def calc_eqOfLine_coeff(P1, P2):
    x1, y1 = P1
    x2, y2 = P2

    a = y2 - y1
    b = x1 - x2
    c = (x2*y1) - (x1*y2) 
    return a, b, c

def line_intersects_circle(coeff, center, radius):
    # This functions checks if a given line intersects with a given circle.
    a, b, c = coeff
    x, y = center

    distance = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))
    return radius > distance

def calculate_radius(l, h):                 # l = slant height (tether length) and h = height (altitude)
    return round(math.sqrt(l**2 - h**2), 3)

def generate_non_cardinal_edges(peripheral_nodes):
    green_edges = [
        (peripheral_nodes[i], peripheral_nodes[j])
        for i in range(len(peripheral_nodes))
        for j in range(i + 1, len(peripheral_nodes))
    ]

    green_edges_with_angles = []
    for edge in green_edges:
        x1, y1 = edge[0]
        x2, y2 = edge[1]
        angle = math.atan2((y2 - y1), (x2 - x1))
        green_edges_with_angles.append(edge + (angle,))

    non_cardinal_edges = [
        e for e in green_edges_with_angles
        if abs(e[2]) not in [0.0, math.pi / 2, math.pi]
    ]

    return non_cardinal_edges


def generate_lines(o, d):
    """Generate a fixed set of line segments used for UGV point generation."""
    return [[(o, d),       (d, (o + d) / 2)],
            [(o, d),       (d, o)],
            [(o, d),       ((o + d) / 2, o)],
            [(o, (o + d) / 2), ((o + d) / 2, d)],
            [(o, (o + d) / 2), (d, d)],
            [(o, (o + d) / 2), (d, o)],
            [(o, (o + d) / 2), ((o + d) / 2, o)],
            [(o, o),       ((o + d) / 2, d)],
            [(o, o),       (d, d)],
            [(o, o),       (d, (o + d) / 2)],
            [((o + d) / 2, d), (d, (o + d) / 2)],
            [((o + d) / 2, o), (d, (o + d) / 2)],
            [((o + d) / 2, o), (d, d)],
            [((o + d) / 2, d), (d, o)],
            [(o, (o + d) / 2), (d, (o + d) / 2)],
            [((o + d) / 2, d), ((o + d) / 2, o)]
            # [(o, d), (d, d)],
            # [(o, o), (d, o)],
            # [(o, d), (o, o)],
            # [(d, d), (d, o)],
        ]

def generate_peripheral_nodes(o, d):
    """  
        d = n-1, o =  0 for nodes to be on the grid 
        d = n,   o = -1 for nodes to be 1 unit away from the grid
        d = n+1, o = -2 for nodes to be 2 unit away from the grid
        d = n-2, o = +1 for nodes to be 1 unit inside the grid 
    """
    return [(o, d)      , ((o+d)/2, d),     (d, d), 
            (o, (o+d)/2),                   (d, (o+d)/2),
            (o, o)      , ((o+d)/2, o),     (d, o)]


# peripheral_nodes = generate_peripheral_nodes(o, d)
# Lines = generate_lines(o, d)

def compute_gv_points(Lines):
    gvPoints = []
    for line in Lines:
        (x1, y1), (x2, y2) = line
        Dist = eucl_dist((x1, y1), (x2, y2))

        if Dist < 2:
            continue   # skip short segments
        
        d = 1
        d_vector = ((x2-x1)/Dist, (y2-y1)/Dist)
        points = []
        # for n in range(1, round(Dist/d)):
        for n in range(1, int(Dist // d)):
            x_n = x1 + n * d * d_vector[0]
            y_n = y1 + n * d * d_vector[1]
            x_n, y_n = round(x_n, 3), round(y_n, 3)
            points.append((x_n, y_n))

        # gvPoints.append(((x1, y1), (x2, y2), points))
        gvPoints.append(points)
    return gvPoints

def compute_gv_points_v2(Lines):
    gvPoints = {}
    new_gvPoints = []

    for line in Lines:
        # ensure line is hashable
        line_key = (tuple(line[0]), tuple(line[1]))

        (x1, y1), (x2, y2) = line_key
        Dist = eucl_dist((x1, y1), (x2, y2))

        # need space for interior points
        if Dist < 2:
            d = 0.5
        else:
            d = 1
        
        
        d_vector = ((x2-x1)/Dist, (y2-y1)/Dist)

        points = []
        steps = int(Dist // d)

        for n in range(1, steps):
            x_n = round(x1 + n*d*d_vector[0], 3)
            y_n = round(y1 + n*d*d_vector[1], 3)
            points.append((x_n, y_n))

        if points:
            gvPoints[line_key] = points
            l = [line_key[0]] + points + [line_key[1]]
            new_gvPoints.append(l)

    return gvPoints, new_gvPoints

def line_intersects_obstacle(P1, P2, cuboid):
    x1, y1 = P1
    x2, y2 = P2

    z1, z2 = 0, height 
    
    # Direction vector
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    
    xs = [v[0] for v in cuboid]
    ys = [v[1] for v in cuboid]
    zs = [v[2] for v in cuboid]
    
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = 0, max(zs)
    
    def slab_intersection(p, d, min_b, max_b):
        if d == 0:
            if p < min_b or p > max_b:
                return float('inf'), float('-inf')
            return float('-inf'), float('inf')
        
        t1 = (min_b - p) / d
        t2 = (max_b - p) / d
        
        return min(t1, t2), max(t1, t2)
    
    tx1, tx2 = slab_intersection(x1, dx, xmin, xmax)
    ty1, ty2 = slab_intersection(y1, dy, ymin, ymax)
    tz1, tz2 = slab_intersection(z1, dz, zmin, zmax)
    
    t_min = max(tx1, ty1, tz1)
    t_max = min(tx2, ty2, tz2)
    
    return t_max >= t_min and t_max >= 0 and t_min <= 1

def line_intersects_any_obstacle(P1, P2, cuboids):
    for cuboid in cuboids:
        if line_intersects_obstacle(P1, P2, cuboid):
            return True
    return False


def generate_ugv_candidate_sets(waypoints, Lines, gvPoints, radius):
    candidate_sets = []
    for waypoint in waypoints:
        x, y = waypoint
        intersecting_lines = [
            line for line in Lines
            if line_intersects_circle(
                calc_eqOfLine_coeff(line[0], line[1]),
                (x, y), radius
            )
        ]
        s = []
        for line in intersecting_lines:
            idx = Lines.index(line)
            for point in gvPoints[idx]:
                if eucl_dist((x, y), point) < radius:
                    s.append(point)
        if s:
            candidate_sets.append(s)
    return candidate_sets

def generate_ugv_candidate_sets_v2(waypoints, gvPoints, radius, cuboids):
    candidate_sets = []
    candidate_dict = {}

    for waypoint in waypoints:
        x, y = waypoint
        s = []

        for line, points in gvPoints.items():

            # check if line intersects waypoint circle
            if line_intersects_circle(
                calc_eqOfLine_coeff(line[0], line[1]),
                (x, y), 
                radius
            ):
                # check points on that line
                points = [line[0]] + points + [line[1]]
                for point in points:
                    if eucl_dist((x, y), point) < radius:
                        if not line_intersects_any_obstacle((x, y), point, cuboids):
                            s.append(point)

        if s:
            candidate_sets.append(s)
            candidate_dict[(x, y)] = s

    return candidate_sets, candidate_dict

def segment_intersects_sphere(A, B, center, r):
    A = np.array([A[0], A[1], 0.0], dtype=float)
    B = np.array([B[0], B[1], 0.0], dtype=float)
    C = np.array(center, dtype=float)

    d = B - A

    # Projection parameter
    t = np.dot(C - A, d) / np.dot(d, d)

    # Restrict to the segment
    t = np.clip(t, 0.0, 1.0)

    # Closest point on the segment
    P = A + t * d

    # Distance from sphere center
    dist = np.linalg.norm(P - C)

    return dist <= r

def generate_ugv_candidate_sets_v3(waypoints, gvPoints, radius, cuboids):
    candidate_sets = []
    candidate_dict = {}

    for waypoint in waypoints:
        x_a, y_a = waypoint
        s = []

        for line, points in gvPoints.items():

            # check if line intersects waypoint circle
            if segment_intersects_sphere(A=line[0], B=line[1],
                center=(x_a, y_a, height), r=radius):
                # check points on that line
                points = [line[0]] + points + [line[1]]
                for point in points:
                    if eucl_dist((x_a, y_a), point, z=height) < radius:
                        if not line_intersects_any_obstacle((x_a, y_a), point, cuboids):
                            s.append(point)

        if s:
            candidate_sets.append(s)
            candidate_dict[(x_a, y_a)] = s
        else:
            candidate_sets.append([None])
            candidate_dict[(x_a, y_a)] = [None]

    return candidate_sets, candidate_dict


def generate_final_road_edges_and_points(circle_points, gvPoints, Lines):
    new_edges, gv_points_on_new_edges = [], []
    for cp in circle_points:
        for points in gvPoints:
            if cp in points:
                idx = gvPoints.index(points)
                new_edges.append(Lines[idx])
                gv_points_on_new_edges.append(points)

    return new_edges, gv_points_on_new_edges

def filter_gvpoints_by_cone_intersection(new_gvPoints, top_vertices, h, L):

    r = math.sqrt(L**2 - h**2)  # cone base radius
    
    filtered_points = []

    for line in new_gvPoints:
        new_line = []

        for (a, b) in line:
            intersects = False

            for obstacle in top_vertices:

                xs = [p[0] for p in obstacle]
                ys = [p[1] for p in obstacle]
                z_o = obstacle[0][2]

                xmin, xmax = min(xs), max(xs)
                ymin, ymax = min(ys), max(ys)

                # Determine cone slice radius
                if z_o >= h:
                    rho = r
                else:
                    rho = (r / h) * z_o

                # Closest point on rectangle
                x_closest = max(xmin, min(a, xmax))
                y_closest = max(ymin, min(b, ymax))

                dist_sq = (x_closest - a) ** 2 + (y_closest - b) ** 2

                if dist_sq <= rho ** 2:
                    intersects = True
                    break

            if not intersects:
                new_line.append((a, b))

        if new_line:
            filtered_points.append(new_line)

    return filtered_points
