from collections import defaultdict
import math

def voronoi(ugv_dict_inverted, height):
    """
    Assign each UAV cell to exactly one feasible UGV based on the
    minimum 3D Euclidean distance.

    Parameters
    ----------
    ugv_dict_inverted : dict
        {ugv_position: [uav_cells]}
    height : float
        UAV flight altitude.

    Returns
    -------
    dict
        {ugv_position: [uniquely assigned uav_cells]}
    """

    # Build cell -> feasible UGVs
    cell_to_ugvs = defaultdict(list)

    for ugv, cells in ugv_dict_inverted.items():
        for cell in cells:
            cell_to_ugvs[cell].append(ugv)

    # Initialize output
    ugv_unique_cells = {ugv: [] for ugv in ugv_dict_inverted}

    # Assign each cell to its nearest feasible UGV
    for cell, candidate_ugvs in cell_to_ugvs.items():

        cx, cy = cell
        cz = height

        nearest_ugv = min(
            candidate_ugvs,
            key=lambda ugv: math.sqrt(
                (cx - ugv[0])**2 +
                (cy - ugv[1])**2 +
                cz**2
            )
        )

        ugv_unique_cells[nearest_ugv].append(cell)

    return ugv_unique_cells