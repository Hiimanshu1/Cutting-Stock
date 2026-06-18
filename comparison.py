"""
Standalone 2D Cutting Stock Subproblem Solvers
================================================
INPUT  : dual variables (objective coefficients) from master LP
OUTPUT : new column (pattern) to add to master

Two implementations:
  1. solve_subproblem_gurobi  -- Big-M MIP via Gurobi
  2. solve_subproblem_cpsat   -- NoOverlap2D via OR-Tools CP-SAT

Both return the same structure:
  {
    "rc"       : float,          # reduced cost (< 0 = improving column)
    "pattern"  : dict,           # {item: count} e.g. {'A':3, 'B':2}
    "layout"   : list of dicts,  # [{"item","x","y","w","h"}, ...]
    "status"   : str,            # "OPTIMAL" / "FEASIBLE" / "INFEASIBLE"
    "solve_time": float          # seconds
  }
  Returns None if no feasible solution found.
"""

import time
import gurobipy as gp
from gurobipy import GRB
from ortools.sat.python import cp_model


# ============================================================
# PROBLEM DATA  (edit this section)
# ============================================================

items_data = {
    'A': {'demand': 10, 'w': 3, 'h': 2},
    'B': {'demand': 8,  'w': 4, 'h': 3},
    
}

W, H = 15, 15   # sheet dimensions

TIME_LIMIT = 60  # seconds per solver

# Example dual variables from master LP
# (replace with actual duals from your master solve)
duals = {
    'A': 1,
    'B': 1,
    'C': 1,
    'D': 1,
}


# ============================================================
# SHARED SETUP — expanded items
# ============================================================

def build_expanded_items(items_data, W, H):
    """
    Build list of item copies using copy-reduction strategy:
      max_copies = min(demand, floor(W*H / (w*h)))
    Returns:
      expanded_items : list of int indices
      copy_to_item   : dict {index: item_type}
    """
    expanded_items = []
    copy_to_item   = {}
    counter        = 1

    for item, v in items_data.items():
        max_copies = min(v['demand'], (W * H) // (v['w'] * v['h']))
        for _ in range(max_copies):
            expanded_items.append(counter)
            copy_to_item[counter] = item
            counter += 1

    return expanded_items, copy_to_item


# ============================================================
# SOLVER 1 — GUROBI MIP (Big-M disjunctive formulation)
# ============================================================

def solve_subproblem_gurobi(duals, items_data, W, H,
                             time_limit=TIME_LIMIT,
                             mip_gap=0.0,
                             verbose=False):
    """
    Solve pricing subproblem with Gurobi MIP.

    Parameters
    ----------
    duals      : dict {item: dual_value}  -- objective coefficients
    items_data : dict with 'w','h','demand' per item
    W, H       : sheet dimensions
    time_limit : solver time limit in seconds
    mip_gap    : MIP optimality gap (0.0 = proven optimal)
    verbose    : show Gurobi log

    Returns
    -------
    dict with keys: rc, pattern, layout, status, solve_time
    """

    items = list(items_data.keys())
    widths  = {k: v['w'] for k, v in items_data.items()}
    heights = {k: v['h'] for k, v in items_data.items()}

    expanded_items, copy_to_item = build_expanded_items(items_data, W, H)

    sub = gp.Model("Pricing_Gurobi")
    sub.Params.OutputFlag  = 1 if verbose else 0
    sub.Params.TimeLimit   = time_limit
    sub.Params.MIPGap      = mip_gap
    sub.Params.Threads     = 4

    # --------------------------------------------------------
    # Variables
    # --------------------------------------------------------

    z    = {}   # binary: item copy placed
    xpos = {}   # x coordinate of lower-left corner
    ypos = {}   # y coordinate of lower-left corner
    L    = {}   # i is left  of j
    R    = {}   # i is right of j
    B    = {}   # i is below j
    T    = {}   # i is above j

    for n in expanded_items:
        z[n]    = sub.addVar(vtype=GRB.BINARY,     name=f"z_{n}")
        xpos[n] = sub.addVar(lb=0,                 name=f"x_{n}")
        ypos[n] = sub.addVar(lb=0,                 name=f"y_{n}")

    pairs = []
    for ii in range(len(expanded_items)):
        for jj in range(ii + 1, len(expanded_items)):
            i, j = expanded_items[ii], expanded_items[jj]
            pairs.append((i, j))
            L[(i,j)] = sub.addVar(vtype=GRB.BINARY)
            R[(i,j)] = sub.addVar(vtype=GRB.BINARY)
            B[(i,j)] = sub.addVar(vtype=GRB.BINARY)
            T[(i,j)] = sub.addVar(vtype=GRB.BINARY)

    sub.update()

    # --------------------------------------------------------
    # Tight big-M values
    # --------------------------------------------------------

    # --------------------------------------------------------
    # Constraints
    # --------------------------------------------------------

    # Boundary — item stays inside sheet
    for n in expanded_items:
        item = copy_to_item[n]
        w, h = widths[item], heights[item]
        Mx_i = W - w
        My_i = H - h
        sub.addConstr(xpos[n] + w <= W + Mx_i * (1 - z[n]))
        sub.addConstr(ypos[n] + h <= H + My_i * (1 - z[n]))

    # Non-overlap — disjunctive constraints
    for (i, j) in pairs:
        wi = widths[copy_to_item[i]];  hi = heights[copy_to_item[i]]
        wj = widths[copy_to_item[j]];  hj = heights[copy_to_item[j]]
        Mx_ij = W - min(wi, wj)
        My_ij = H - min(hi, hj)

        sub.addConstr(xpos[i] + wi <= xpos[j] + Mx_ij * (1 - L[(i,j)]))
        sub.addConstr(xpos[j] + wj <= xpos[i] + Mx_ij * (1 - R[(i,j)]))
        sub.addConstr(ypos[i] + hi <= ypos[j] + My_ij * (1 - B[(i,j)]))
        sub.addConstr(ypos[j] + hj <= ypos[i] + My_ij * (1 - T[(i,j)]))

        # at least one direction active when both placed
        sub.addConstr(L[(i,j)] + R[(i,j)] + B[(i,j)] + T[(i,j)]
                      >= z[i] + z[j] - 1)

        # directional exclusivity
        sub.addConstr(L[(i,j)] + R[(i,j)] <= 1)
        sub.addConstr(B[(i,j)] + T[(i,j)] <= 1)

    # Symmetry breaking — same type copies ordered
    for item in items:
        same = [n for n in expanded_items if copy_to_item[n] == item]
        for k in range(len(same) - 1):
            sub.addConstr(z[same[k]] >= z[same[k+1]])
            # lexicographic x ordering
            sub.addConstr(xpos[same[k]] <=
                          xpos[same[k+1]] + W * (1 - z[same[k+1]]))

    # Pairwise infeasibility
    for (i, j) in pairs:
        wi = widths[copy_to_item[i]];  hi = heights[copy_to_item[i]]
        wj = widths[copy_to_item[j]];  hj = heights[copy_to_item[j]]
        if wi + wj > W and hi + hj > H:
            sub.addConstr(z[i] + z[j] <= 1)

    # --------------------------------------------------------
    # Objective : minimize reduced cost = 1 - sum(dual * z)
    # --------------------------------------------------------

    sub.setObjective(
        1 - gp.quicksum(duals[copy_to_item[n]] * z[n]
                        for n in expanded_items),
        GRB.MINIMIZE
    )

    # --------------------------------------------------------
    # Solve
    # --------------------------------------------------------

    t0 = time.time()
    sub.optimize()
    solve_time = time.time() - t0

    if sub.SolCount == 0:
        return None

    rc = sub.ObjVal

    # --------------------------------------------------------
    # Extract pattern and layout
    # --------------------------------------------------------

    pattern = {item: 0 for item in items}
    layout  = []

    for n in expanded_items:
        if z[n].X > 0.5:
            item = copy_to_item[n]
            pattern[item] += 1
            layout.append({
                "item": item,
                "x":    xpos[n].X,
                "y":    ypos[n].X,
                "w":    widths[item],
                "h":    heights[item],
            })

    status_map = {
        GRB.OPTIMAL:    "OPTIMAL",
        GRB.TIME_LIMIT: "FEASIBLE",
        GRB.INFEASIBLE: "INFEASIBLE",
    }

    # bounds info
    obj_val   = sub.ObjVal   if sub.SolCount > 0 else None
    obj_bound = sub.ObjBound if sub.SolCount > 0 else None
    mip_gap   = abs(obj_val - obj_bound) / (abs(obj_val) + 1e-10) * 100 if (obj_val is not None and obj_bound is not None) else None

    return {
        "rc":          rc,
        "pattern":     pattern,
        "layout":      layout,
        "status":      status_map.get(sub.Status, "UNKNOWN"),
        "solve_time":  solve_time,
        "obj_val":     obj_val,
        "obj_bound":   obj_bound,
        "mip_gap_pct": mip_gap,
    }


# ============================================================
# SOLVER 2 — CP-SAT (NoOverlap2D formulation)
# ============================================================

def solve_subproblem_cpsat(duals, items_data, W, H,
                            time_limit=TIME_LIMIT,
                            verbose=False):
    """
    Solve pricing subproblem with OR-Tools CP-SAT.

    Parameters
    ----------
    duals      : dict {item: dual_value}  -- objective coefficients
    items_data : dict with 'w','h','demand' per item
    W, H       : sheet dimensions
    time_limit : solver time limit in seconds
    verbose    : show CP-SAT log

    Returns
    -------
    dict with keys: rc, pattern, layout, status, solve_time
    """

    items = list(items_data.keys())
    widths  = {k: v['w'] for k, v in items_data.items()}
    heights = {k: v['h'] for k, v in items_data.items()}

    expanded_items, copy_to_item = build_expanded_items(items_data, W, H)

    # Adaptive scaling — keeps objective in meaningful integer range
    max_dual = max(abs(v) for v in duals.values()) if duals else 1.0
    SCALE    = max(min(int(1e6 / max_dual), 10**6), 1000) if max_dual > 0 else 10**6

    model  = cp_model.CpModel()
    solver = cp_model.CpSolver()

    solver.parameters.max_time_in_seconds  = time_limit
    solver.parameters.num_search_workers   = 4
    solver.parameters.log_search_progress  = verbose

    # --------------------------------------------------------
    # Variables
    # --------------------------------------------------------

    s    = {}   # binary: item copy placed
    xvar = {}   # x position (integer)
    yvar = {}   # y position (integer)
    x_iv = {}   # optional x interval
    y_iv = {}   # optional y interval

    for n in expanded_items:
        item = copy_to_item[n]
        w, h = widths[item], heights[item]

        s[n]    = model.NewBoolVar(f"s_{n}")
        xvar[n] = model.NewIntVar(0, W - w, f"x_{n}")
        yvar[n] = model.NewIntVar(0, H - h, f"y_{n}")

        # optional intervals — active only when s[n] = 1
        x_iv[n] = model.NewOptionalFixedSizeIntervalVar(
            xvar[n], w, s[n], f"xi_{n}")
        y_iv[n] = model.NewOptionalFixedSizeIntervalVar(
            yvar[n], h, s[n], f"yi_{n}")

    # --------------------------------------------------------
    # Constraints
    # --------------------------------------------------------

    # Single no-overlap constraint replaces all Big-M constraints
    model.AddNoOverlap2D(list(x_iv.values()), list(y_iv.values()))

    # Symmetry breaking — same type copies ordered
    for item in items:
        same = [n for n in expanded_items if copy_to_item[n] == item]
        for k in range(len(same) - 1):
            # z_i >= z_j : earlier index used before later
            model.Add(s[same[k]] >= s[same[k+1]])
            # lexicographic x ordering
            model.Add(xvar[same[k]] <=
                      xvar[same[k+1]]).OnlyEnforceIf(s[same[k+1]])

    # --------------------------------------------------------
    # Objective : maximize sum(scaled_dual * s_i)
    # Equivalent to minimizing reduced cost = 1 - sum(dual*z)
    # --------------------------------------------------------

    obj_terms = []
    for n in expanded_items:
        dual_int = round(duals[copy_to_item[n]] * SCALE)
        obj_terms.append(dual_int * s[n])

    model.Maximize(cp_model.LinearExpr.Sum(obj_terms))

    # --------------------------------------------------------
    # Solve
    # --------------------------------------------------------

    t0     = time.time()
    status = solver.Solve(model)
    solve_time = time.time() - t0

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return None

    rc = 1.0 - solver.ObjectiveValue() / SCALE

    # --------------------------------------------------------
    # Extract pattern and layout
    # --------------------------------------------------------

    pattern = {item: 0 for item in items}
    layout  = []

    for n in expanded_items:
        if solver.Value(s[n]) == 1:
            item = copy_to_item[n]
            pattern[item] += 1
            layout.append({
                "item": item,
                "x":    solver.Value(xvar[n]),
                "y":    solver.Value(yvar[n]),
                "w":    widths[item],
                "h":    heights[item],
            })

    status_str = "OPTIMAL" if status == cp_model.OPTIMAL else "FEASIBLE"

    # bounds info
    obj_val   = solver.ObjectiveValue()
    obj_bound = solver.BestObjectiveBound()
    # CP-SAT maximizes, convert both to RC space
    rc_val    = 1.0 - obj_val   / SCALE
    rc_bound  = 1.0 - obj_bound / SCALE
    gap_pct   = abs(obj_val - obj_bound) / (abs(obj_bound) + 1e-10) * 100 if obj_bound != 0 else 0.0

    return {
        "rc":          rc,
        "pattern":     pattern,
        "layout":      layout,
        "status":      status_str,
        "solve_time":  solve_time,
        "obj_val":     rc_val,
        "obj_bound":   rc_bound,
        "mip_gap_pct": gap_pct,
    }


# ============================================================
# MAIN — compare both solvers on same duals
# ============================================================

if __name__ == "__main__":

    print("=" * 60)
    print("SUBPROBLEM COMPARISON")
    print("=" * 60)
    print(f"\nDuals (objective coefficients): {duals}")
    print(f"Sheet: {W} x {H}")

    expanded, c2i = build_expanded_items(items_data, W, H)
    print(f"Expanded items: {len(expanded)}")
    print(f"Pairs: {len(expanded)*(len(expanded)-1)//2}")

    # --------------------------------------------------------
    # Run Gurobi
    # --------------------------------------------------------

    print("\n")
    print("=" * 60)
    print("GUROBI MIP SOLVER")
    print("=" * 60)

    result_gurobi = solve_subproblem_gurobi(
        duals      = duals,
        items_data = items_data,
        W          = W,
        H          = H,
        time_limit = TIME_LIMIT,
        mip_gap    = 0.0,
        verbose    = True,
    )

    if result_gurobi:
        print(f"\nStatus      : {result_gurobi['status']}")
        print(f"Reduced Cost: {result_gurobi['rc']:.6f}")
        print(f"Pattern     : {result_gurobi['pattern']}")
        print(f"Solve Time  : {result_gurobi['solve_time']:.4f}s")
        if result_gurobi['status'] != 'OPTIMAL':
            print(f"\n-- NOT OPTIMAL --")
            print(f"  Incumbent (best found) RC : {result_gurobi['obj_val']:.6f}")
            print(f"  Best Bound RC             : {result_gurobi['obj_bound']:.6f}")
            print(f"  MIP Gap                   : {result_gurobi['mip_gap_pct']:.2f}%")
            print(f"  Gap meaning: true optimal RC could be as low as {result_gurobi['obj_bound']:.6f}")
    else:
        print("No feasible solution found")

    # --------------------------------------------------------
    # Run CP-SAT
    # --------------------------------------------------------

    print("\n")
    print("=" * 60)
    print("CP-SAT SOLVER")
    print("=" * 60)

    result_cpsat = solve_subproblem_cpsat(
        duals      = duals,
        items_data = items_data,
        W          = W,
        H          = H,
        time_limit = TIME_LIMIT,
        verbose    = True,
    )

    if result_cpsat:
        print(f"\nStatus      : {result_cpsat['status']}")
        print(f"Reduced Cost: {result_cpsat['rc']:.6f}")
        print(f"Pattern     : {result_cpsat['pattern']}")
        print(f"Solve Time  : {result_cpsat['solve_time']:.4f}s")
        if result_cpsat['status'] != 'OPTIMAL':
            print(f"\n-- NOT OPTIMAL --")
            print(f"  Incumbent (best found) RC : {result_cpsat['obj_val']:.6f}")
            print(f"  Best Bound RC             : {result_cpsat['obj_bound']:.6f}")
            print(f"  Gap                       : {result_cpsat['mip_gap_pct']:.2f}%")
            print(f"  Gap meaning: true optimal RC could be as low as {result_cpsat['obj_bound']:.6f}")
    else:
        print("No feasible solution found")

    # --------------------------------------------------------
    # Comparison
    # --------------------------------------------------------
    print("\n")
    print("=" * 60)
    print("INPUT DATA")
    print("=" * 60)

    for item, data in items_data.items():
        print(
            f"{item}: "
            f"demand={data['demand']}, "
            f"w={data['w']}, "
            f"h={data['h']}, "
            f"dual={duals.get(item,0)}"
        )
    print("\n")
    print("=" * 60)
    print("COMPARISON SUMMARY")
    print("=" * 60)

    if result_gurobi and result_cpsat:
        print(f"\n{'Metric':<25} {'Gurobi MIP':>15} {'CP-SAT':>15}")
        print("-" * 57)
        print(f"{'Status':<25} {result_gurobi['status']:>15} {result_cpsat['status']:>15}")
        print(f"{'Reduced Cost':<25} {result_gurobi['rc']:>15.6f} {result_cpsat['rc']:>15.6f}")
        print(f"{'Incumbent RC':<25} {result_gurobi['obj_val']:>15.6f} {result_cpsat['obj_val']:>15.6f}")
        print(f"{'Best Bound RC':<25} {result_gurobi['obj_bound']:>15.6f} {result_cpsat['obj_bound']:>15.6f}")
        print(f"{'Gap %':<25} {result_gurobi['mip_gap_pct']:>14.2f}% {result_cpsat['mip_gap_pct']:>14.2f}%")
        print(f"{'Solve Time (s)':<25} {result_gurobi['solve_time']:>15.4f} {result_cpsat['solve_time']:>15.4f}")
        print(f"{'Items Packed':<25} {sum(result_gurobi['pattern'].values()):>15} {sum(result_cpsat['pattern'].values()):>15}")

        print()
        rc_diff = abs(result_gurobi['rc'] - result_cpsat['rc'])
        print(f"RC Difference : {rc_diff:.8f}")
        if rc_diff < 1e-4:
            print("-> RC values match within 1e-4 tolerance")
        else:
            print("-> RC values differ — different patterns found")

        # optimality summary
        print()
        for name, res in [("Gurobi", result_gurobi), ("CP-SAT", result_cpsat)]:
            if res['status'] == 'OPTIMAL':
                print(f"{name}: PROVEN OPTIMAL — RC = {res['rc']:.6f}")
            else:
                print(f"{name}: NOT PROVEN OPTIMAL")
                print(f"  Stopped at RC = {res['obj_val']:.6f}")
                print(f"  Best bound   = {res['obj_bound']:.6f}")
                print(f"  Gap          = {res['mip_gap_pct']:.2f}%")
                print(f"  True optimal RC is somewhere in [{res['obj_bound']:.6f}, {res['obj_val']:.6f}]")