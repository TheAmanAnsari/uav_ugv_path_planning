def segment_intersection(seg1, seg2):
    (x1, y1), (x2, y2) = seg1
    (x3, y3), (x4, y4) = seg2

    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)

    # Parallel or collinear
    if denom == 0:
        return None

    px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
    py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom

    intersection = (round(px, 3), round(py, 3))

    # Check the point lies on both segments
    def on_segment(p, q, r):
        return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and
                min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)

    if on_segment((x1,y1), intersection, (x2,y2)) and \
       on_segment((x3,y3), intersection, (x4,y4)):
        return intersection

    return None

def split_segment(segment, point):
    p1, p2 = segment
    return [(p1, point), (point, p2)]

def same_point(p, q, eps=1e-9):
    return abs(p[0]-q[0]) < eps and abs(p[1]-q[1]) < eps


def is_endpoint(segment, point):
    p1, p2 = segment
    return same_point(p1, point) or same_point(p2, point)

def split_all_intersections(lines):
    new_lines = lines[:]
    intersections = []
    i = 0

    while i < len(new_lines):
        line1 = new_lines[i]
        split_occurred = False

        for j in range(i+1, len(new_lines)):
            line2 = new_lines[j]

            pt = segment_intersection(line1, line2)

            # Skip if no intersection OR intersection is at endpoints
            if pt is None:
                continue
            if is_endpoint(line1, pt) or is_endpoint(line2, pt):
                continue
            
            intersections.append(pt)

            # Remove original lines
            new_lines.pop(j)
            new_lines.pop(i)

            # Add split segments
            new_lines.extend(split_segment(line1, pt))
            new_lines.extend(split_segment(line2, pt))

            split_occurred = True
            break

        if not split_occurred:
            i += 1
    
    intersections = list(set(intersections))

    return new_lines, intersections


if __name__ == "__main__":
    grid_size = 10
    o, d = -1, grid_size

    Lines = [
        [(o, d),           (d, (o + d) / 2)],
        [(o, d),           (d, o)],
        [(o, d),           ((o + d) / 2, o)],
        [(o, (o + d) / 2), ((o + d) / 2, d)],
        [(o, (o + d) / 2), (d, d)],
        [(o, (o + d) / 2), (d, o)],
        [(o, (o + d) / 2), ((o + d) / 2, o)],
        [(o, o),           ((o + d) / 2, d)],
        [(o, o),           (d, d)],
        [(o, o),           (d, (o + d) / 2)],
        [((o + d) / 2, d), (d, (o + d) / 2)],
        [((o + d) / 2, o), (d, (o + d) / 2)],
        [((o + d) / 2, o), (d, d)],
        [((o + d) / 2, d), (d, o)]
    ]

    result = split_all_intersections(Lines)
    for r in result:
        print(r)