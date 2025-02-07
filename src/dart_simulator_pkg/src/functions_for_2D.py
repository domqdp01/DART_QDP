import numpy as np
from itertools import combinations
from scipy.optimize import linprog

def remove_redundant_constraints(Hp, hp):
    """Removes redundant constraints from the system Hp * x <= hp"""

    # Check if the system is infeasible before proceeding
    if Hp.size == 0 or hp.size == 0:
        # print("⚠️  The system is empty, returning empty arrays.")
        return np.empty((0, Hp.shape[1] if Hp.ndim == 2 else 0)), np.empty((0,))

    if not set_is_feasible(Hp, hp):
        # print("❌ Error: The set of given inequalities is infeasible.")
        return np.empty((0, Hp.shape[1])), np.empty((0,))

    num_constraints, dimension = Hp.shape
    epsilon = 1e-6
    non_redundant_Hp = []
    non_redundant_hp = []

    for i in range(num_constraints):
        Hp_active = Hp[i]
        hp_active = hp[i]

        # Remove the i-th constraint
        Hp_truncated = np.delete(Hp, i, axis=0)
        hp_truncated = np.delete(hp, i, axis=0)

        # Solve the LP problem
        res = linprog(-Hp_active, A_ub=Hp_truncated, b_ub=hp_truncated, method='highs')

        # Additional check to avoid errors
        if res.status == 2 or res.fun is None:
            # print(f"⚠️  Constraint {i} is invalid or LP infeasible, keeping it.")
            non_redundant_Hp.append(Hp_active)
            non_redundant_hp.append(hp_active)
        elif -res.fun > hp_active + epsilon:
            non_redundant_Hp.append(Hp_active)
            non_redundant_hp.append(hp_active)

    # Convert arrays to bidimensional format even if empty
    non_redundant_Hp = np.array(non_redundant_Hp).reshape(-1, dimension) if non_redundant_Hp else np.empty((0, dimension))
    non_redundant_hp = np.array(non_redundant_hp).reshape(-1) if non_redundant_hp else np.empty((0,))

    
    # non_redundant_Hp, non_redundant_hp = Hp, hp
    return non_redundant_Hp, non_redundant_hp

def set_is_feasible(Hp, hp):
    """Check if the polytope defined by Hp * x <= hp is feasible."""
    res = linprog(np.zeros(Hp.shape[1]), A_ub=Hp, b_ub=hp, method='highs')
    return res.success


def generate_combinations(p, n):
    """Generates all combinations of p hyperplanes from n"""
    return list(combinations(range(n), p))

def compute_vertices(HpConst, hpConst):
    """
    Finds the vertices of the polytope defined by H_p x <= h_p
    """
    # Remove redundant constraints
    Hp, hp = remove_redundant_constraints(HpConst, hpConst)
    # print(f"Hp.shape after removing redundant constraints: {Hp.shape}")
    
    vertices = []
    epsilon = 1e-6
    p = Hp.shape[1]  # Dimension of the space
    n = Hp.shape[0]  # Number of constraints
    
    # Generate all combinations of p hyperplanes
    combinations_list = generate_combinations(p, n)
    
    for indices in combinations_list:
        A = Hp[list(indices), :]
        b = hp[list(indices)]
        
        # Check if the system is well-conditioned
        if abs(np.linalg.det(A)) < epsilon:
            continue
        
        # Solve Ax = b
        x = np.linalg.solve(A, b)
        
        # Check if x satisfies all inequalities
        if np.all(Hp @ x - hp <= epsilon):
            vertices.append(x)
    
    return vertices

def underapproximate_convex_polytope(vertices, directions):
    if len(vertices) == 0:
        raise ValueError("'vertices' array is empty!")
    if len(directions) == 0:
        raise ValueError("'directions' array is empty!")
    
    dim = directions[0].shape[0]
    for vertex in vertices:
        if vertex.shape[0] != dim:
            raise ValueError("Vertex size does not match direction size")
    
    extremum_vertices = []
    extremum_vertex_direction_indices = []
    
    # Find extremum vertices for each direction
    for direction_index, direction in enumerate(directions):
        projections = np.dot(vertices, direction)
        max_index = np.argmax(projections)
        extremum_vertices.append(vertices[max_index])
        extremum_vertex_direction_indices.append(direction_index)
    
    # Create hyperplanes
    Hp = np.array([directions[i] for i in extremum_vertex_direction_indices])
    hp = np.array([np.dot(directions[i], extremum_vertices[j])
                    for j, i in enumerate(extremum_vertex_direction_indices)])
    
    # Remove redundant constraints
    Hp, hp = remove_redundant_constraints(Hp, hp)
    
    return Hp, hp