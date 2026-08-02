
def cross_product(O, A, B):
    return (A[0] - O[0]) * (B[1] - O[1]) - (A[1] - O[1]) * (B[0] - O[0])


def on_segment(A, B, X):
    return (min(A[0], B[0]) <= X[0] <= max(A[0], B[0]) and
            min(A[1], B[1]) <= X[1] <= max(A[1], B[1]))


def find_intersection_point(A, B, P, Q):
    dx1 = B[0] - A[0]   # direction vector of AB (x component)
    dy1 = B[1] - A[1]   # direction vector of AB (y component)
    dx2 = Q[0] - P[0]   # direction vector of PQ (x component)
    dy2 = Q[1] - P[1]   # direction vector of PQ (y component)

    # Determinant of the coefficient matrix
    D = dx1 * (-dy2) - (-dx2) * dy1

    if D == 0:
        return None  # Lines are parallel or coincident

    # Solve for parameter t along line AB
    t = ((P[0] - A[0]) * (-dy2) - (P[1] - A[1]) * (-dx2)) / D

    # Intersection point
    x = A[0] + t * dx1
    y = A[1] + t * dy1

    return (x, y)


def segments_intersect(line, vline):
    A, B = line 
    P, Q = vline

    # d1: Which side of line AB is P on?
    d1 = cross_product(A, B, P)

    # d2: Which side of line AB is Q on?
    d2 = cross_product(A, B, Q)

    # d3: Which side of line PQ is A on?
    d3 = cross_product(P, Q, A)

    # d4: Which side of line PQ is B on?
    d4 = cross_product(P, Q, B)


    if (d1 * d2 < 0) and (d3 * d4 < 0):
        point = find_intersection_point(A, B, P, Q)
        return {
            'intersects': True,
            'type': 'proper',
            'intersection_point': point,
            'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
        }

    if d1 == 0 and on_segment(A, B, P):
        return {
            'intersects': True,
            'type': 'endpoint/collinear',
            'intersection_point': P,
            'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
        }

    if d2 == 0 and on_segment(A, B, Q):
        return {
            'intersects': True,
            'type': 'endpoint/collinear',
            'intersection_point': Q,
            'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
        }

    if d3 == 0 and on_segment(P, Q, A):
        return {
            'intersects': True,
            'type': 'endpoint/collinear',
            'intersection_point': A,
            'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
        }

    if d4 == 0 and on_segment(P, Q, B):
        return {
            'intersects': True,
            'type': 'endpoint/collinear',
            'intersection_point': B,
            'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
        }

    # ------------------------------------------------------------------ #
    #  Step 4: No intersection                                            #
    # ------------------------------------------------------------------ #

    return {
        'intersects': False,
        'type': 'none',
        'intersection_point': None,
        'd1': d1, 'd2': d2, 'd3': d3, 'd4': d4
    }


def print_report(A, B, P, Q, result):
    """
    Print a clean, readable report of the intersection test.
    """
    print("=" * 55)
    print("       SEGMENT INTERSECTION REPORT")
    print("=" * 55)
    print(f"  Segment 1 : A{A}  -->  B{B}")
    print(f"  Segment 2 : P{P}  -->  Q{Q}")
    print("-" * 55)
    print(f"  d1  [P vs line AB] : {result['d1']}")
    print(f"  d2  [Q vs line AB] : {result['d2']}")
    print(f"  d3  [A vs line PQ] : {result['d3']}")
    print(f"  d4  [B vs line PQ] : {result['d4']}")
    print("-" * 55)
    print(f"  Intersects?        : {'YES ✓' if result['intersects'] else 'NO ✗'}")
    print(f"  Intersection Type  : {result['type']}")
    if result['intersection_point']:
        px, py = result['intersection_point']
        print(f"  Intersection Point : ({px:.4f}, {py:.4f})")
    print("=" * 55)


# ------------------------------------------------------------------ #
#  Test Cases                                                         #
# ------------------------------------------------------------------ #

if __name__ == "__main__":

    # print("\n--- TEST CASE 1: Proper Intersection ---")
    # A, B = (0, 0), (4, 0)
    # P, Q = (2, -2), (2, 3)
    # result = segments_intersect(A, B, P, Q)
    # print_report(A, B, P, Q, result)

    # print("\n--- TEST CASE 2: No Intersection (outside segment) ---")
    # A, B = (0, 0), (2, 0)
    # P, Q = (3, -1), (3, 5)
    # result = segments_intersect(A, B, P, Q)
    # print_report(A, B, P, Q, result)

    # print("\n--- TEST CASE 3: Endpoint Touching ---")
    # A, B = (0, 0), (4, 4)
    # P, Q = (4, 4), (4, 8)
    # result = segments_intersect(A, B, P, Q)
    # print_report(A, B, P, Q, result)

    # print("\n--- TEST CASE 4: Diagonal Cross ---")
    # A, B = (0, 4), (4, 0)
    # P, Q = (0, 0), (4, 4)
    # result = segments_intersect(A, B, P, Q)
    # print_report(A, B, P, Q, result)

    # line = ((-1, 10), (2.667, 8.167))
    # vline = [(2.25, 2.25, 0.0), (2.25, 1.75, 0.0)]


    Lines = [((-1, 10), (2.667, 8.167)), ((2.667, 8.167), (4.5, 10)), ((-1, 4.5), (0.833, 2.667)), ((-1, -1), (0.833, 2.667)), ((7.25, 7.25), (10, 10)), ((8.167, 2.667), (10, 4.5)), ((8.167, 2.667), (10, -1)), ((2.667, 6.333), (4.501, 7.25)), ((1.75, 7.25), (2.667, 8.167)), ((-1, 10), (1.75, 7.25)), ((1.75, 7.25), (2.667, 6.333)), ((2.667, 6.333), (4.5, 4.5)), ((-1, 4.5), (1.2, 5.6)), ((2.667, 2.667), (3.4, 1.2)), ((3.4, 1.2), (4.5, -1)), ((-1, 4.5), (1.2, 3.4)), ((1.2, 3.4), (2.667, 2.667)), ((0.833, 2.667), (1.2, 3.4)), ((4.5, -1), (5.6, 1.2)), ((5.6, 1.2), (6.333, 2.667)), ((0.833, 2.667), (1.75, 1.75)), ((-1, -1), (1.75, 1.75)), ((1.75, 1.75), (4.5, 4.5)), ((4.5, 10), (6.334, 8.166)), ((6.334, 8.166), (7.25, 7.25)), ((6.334, 8.166), (10, 10)), ((7.25, 7.25), (8.166, 6.333)), ((8.166, 6.333), (10, 4.5)), ((8.166, 6.333), (10, 10)), ((6.333, 2.667), (7.8, 3.4)), ((7.8, 3.4), (10, 4.5)), ((7.8, 3.4), (8.167, 2.667)), ((7.25, 1.75), (8.167, 2.667)), ((4.5, 4.5), (7.25, 1.75)), ((7.25, 1.75), (10, -1)), ((2.667, 8.167), (3.4, 7.8)), ((3.4, 7.8), (4.501, 7.25)), ((3.4, 7.8), (4.5, 10)), ((6.334, 6.334), (7.25, 7.25)), ((-1, 4.5), (0.833, 6.333)), ((0.833, 6.333), (1.75, 7.25)), ((-1, 10), (0.833, 6.333)), ((0.833, 6.333), (1.2, 5.6)), ((1.2, 5.6), (2.666, 6.333)), ((2.666, 6.333), (2.667, 6.333)), ((2.666, 6.333), (3.4, 7.8)), ((1.2, 5.6), (1.75, 4.5)), ((1.75, 4.5), (2.667, 2.667)), ((1.2, 3.4), (1.75, 4.5)), ((1.75, 4.5), (2.666, 6.333)), ((-1, -1), (2.667, 0.833)), ((2.667, 0.833), (3.4, 1.2)), ((1.75, 1.75), (2.667, 0.833)), ((2.667, 0.833), (4.5, -1)), ((3.4, 1.2), (4.5, 1.75)), ((4.5, 1.75), (6.333, 2.667)), ((2.667, 2.667), (4.5, 1.75)), ((4.5, 1.75), (5.6, 1.2)), ((5.6, 1.2), (6.333, 0.833)), ((6.333, 0.833), (10, -1)), ((4.5, -1), (6.333, 0.833)), ((6.333, 0.833), (7.25, 1.75)), ((4.501, 7.25), (5.6, 7.799)), ((5.6, 7.799), (6.334, 8.166)), ((4.5, 10), (5.6, 7.799)), ((7.8, 5.601), (8.166, 6.333)), ((6.334, 6.334), (7.8, 5.601)), ((7.8, 5.601), (10, 4.5)), ((4.501, 7.25), (6.332, 6.335)), ((6.332, 6.335), (6.334, 6.334)), ((5.6, 7.799), (6.332, 6.335)), ((4.5, 4.5), (6.333, 6.333)), ((6.333, 6.333), (6.334, 6.334)), ((6.332, 6.335), (6.333, 6.333)), ((6.333, 2.667), (7.25, 4.5)), ((7.25, 4.5), (7.8, 5.601)), ((6.333, 6.333), (7.25, 4.5)), ((7.25, 4.5), (7.8, 3.4))]
    print(f"len(Lines): {len(Lines)}")
    base_vertices_line_segments = [[(2.25, 2.25, 0.0), (2.25, 1.75, 0.0)], 
                                   [(2.25, 1.75, 0.0), (1.75, 2.25, 0.0)], 
                                   [(1.75, 2.25, 0.0), (1.75, 1.75, 0.0)], 
                                   [(1.75, 1.75, 0.0), (2.25, 2.25, 0.0)], 
                                   [(3.25, 6.25, 0.0), (3.25, 4.75, 0.0)], 
                                   [(3.25, 4.75, 0.0), (2.75, 6.25, 0.0)], 
                                   [(2.75, 6.25, 0.0), (2.75, 4.75, 0.0)], 
                                   [(2.75, 4.75, 0.0), (3.25, 6.25, 0.0)], 
                                   [(6.375, 4.5, 0.0), (6.375, 1.5, 0.0)], 
                                   [(6.375, 1.5, 0.0), (5.625, 4.5, 0.0)], 
                                   [(5.625, 4.5, 0.0), (5.625, 1.5, 0.0)], 
                                   [(5.625, 1.5, 0.0), (6.375, 4.5, 0.0)], 
                                   [(7.125, 4.5, 0.0), (7.125, 3.5, 0.0)], 
                                   [(7.125, 3.5, 0.0), (6.375, 4.5, 0.0)], 
                                   [(6.375, 4.5, 0.0), (6.375, 3.5, 0.0)], 
                                   [(6.375, 3.5, 0.0), (7.125, 4.5, 0.0)]]

    filtered_lines = []

    for line in Lines:
        intersects_any = False

        for vline in base_vertices_line_segments:
            if segments_intersect(line, vline)['intersects']:
                intersects_any = True
                break

        if not intersects_any:
            filtered_lines.append(line)

    print(f"len(filtered_lines): {len(filtered_lines)}")

    # result = segments_intersect(line, vline)['intersects']
    # print(result['intersects'])
