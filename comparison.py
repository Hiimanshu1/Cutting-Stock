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
'A': {'demand': 10, 'w': 2, 'h': 3},
'B': {'demand': 6, 'w': 5, 'h': 4},
'C': {'demand': 9, 'w': 5, 'h': 5},
'D': {'demand': 11, 'w': 5, 'h': 2},
'E': {'demand': 3, 'w': 8, 'h': 1},
}

duals = {
'A': 0.012,
'B': 0.018,
'C': 0.009,
'D': 0.021,
'E': 0.015,

}

W, H = 20, 15


# sheet dimensions

TIME_LIMIT = 60 * 60  # seconds per solver

# Example dual variables from master LP
# (replace with actual duals from your master solve)


# ============================================================
# SHARED SETUP -- expanded items
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
# SOLVER 1 -- GUROBI MIP (Big-M disjunctive formulation)
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
        item_n  = copy_to_item[n]
        w_n, h_n = widths[item_n], heights[item_n]
        z[n]    = sub.addVar(vtype=GRB.BINARY, name=f"z_{n}")
        xpos[n] = sub.addVar(lb=0, ub=W - w_n, name=f"x_{n}")
        ypos[n] = sub.addVar(lb=0, ub=H - h_n, name=f"y_{n}")

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

    # Boundary -- item stays inside sheet
    for n in expanded_items:
        item = copy_to_item[n]
        w, h = widths[item], heights[item]
        Mx_i = W - w
        My_i = H - h
        sub.addConstr(xpos[n] + w <= W + Mx_i * (1 - z[n]))
        sub.addConstr(ypos[n] + h <= H + My_i * (1 - z[n]))

    # Non-overlap -- disjunctive constraints
    # Proof of correct M values:
    #   L[i,j] active (=1): x_i + wi <= x_j              (i left of j)
    #   L[i,j] inactive (=0): constraint must be trivially satisfied for ALL valid x_i, x_j
    #     Max LHS = (W - wi) + wi = W   (since xpos[i] <= W-wi)
    #     Min RHS = xpos[j] + M*1 >= 0 + M
    #     Need M >= W  =>  M = W  (not W-wi, which fails when wi is small)
    #   Same logic: R->W, B->H, T->H
    for (i, j) in pairs:
        wi = widths[copy_to_item[i]];  hi = heights[copy_to_item[i]]
        wj = widths[copy_to_item[j]];  hj = heights[copy_to_item[j]]

        sub.addConstr(xpos[i] + wi <= xpos[j] + W * (1 - L[(i,j)]))
        sub.addConstr(xpos[j] + wj <= xpos[i] + W * (1 - R[(i,j)]))
        sub.addConstr(ypos[i] + hi <= ypos[j] + H * (1 - B[(i,j)]))
        sub.addConstr(ypos[j] + hj <= ypos[i] + H * (1 - T[(i,j)]))

        # at least one direction active when both placed
        sub.addConstr(L[(i,j)] + R[(i,j)] + B[(i,j)] + T[(i,j)]
                      >= z[i] + z[j] - 1)

        # directional exclusivity: x-directions exclusive, y-directions exclusive
        # but x and y can BOTH be active simultaneously (valid geometry)
        sub.addConstr(L[(i,j)] + R[(i,j)] <= 1)
        sub.addConstr(B[(i,j)] + T[(i,j)] <= 1)

    # Symmetry breaking -- same type copies ordered
    for item in items:
        same = [n for n in expanded_items if copy_to_item[n] == item]
        for k in range(len(same) - 1):
            sub.addConstr(z[same[k]] >= z[same[k+1]])
            # lexicographic x ordering
            sub.addConstr(xpos[same[k]] <=
                          xpos[same[k+1]] + W * (1 - z[same[k+1]]))

    # Pairwise infeasibility -- BOTH dimensions must be too wide simultaneously
    for (i, j) in pairs:
        wi = widths[copy_to_item[i]];  hi = heights[copy_to_item[i]]
        wj = widths[copy_to_item[j]];  hj = heights[copy_to_item[j]]
        if wi + wj > W and hi + hj > H:
            sub.addConstr(z[i] + z[j] <= 1)

    # Area upper bound (valid inequality -- tightens LP relaxation correctly)
    total_area = W * H
    sub.addConstr(
        gp.quicksum(
            widths[copy_to_item[n]] * heights[copy_to_item[n]] * z[n]
            for n in expanded_items
        ) <= total_area
    )



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
    # FIX: use sub.ObjBound as denominator (not obj_val) -- matches Gurobi's own gap def
    obj_val   = sub.ObjVal   if sub.SolCount > 0 else None
    obj_bound = sub.ObjBound if sub.SolCount > 0 else None
    mip_gap   = abs(obj_val - obj_bound) / (abs(obj_bound) + 1e-10) * 100 \
                if (obj_val is not None and obj_bound is not None) else None

    # FIX: convert obj_val / obj_bound from raw Gurobi obj space -> RC space
    # Gurobi minimizes: obj = 1 - sum(dual*z), so RC = obj_val directly
    rc_val   = obj_val   # already in RC space (Gurobi minimizes RC)
    rc_bound = obj_bound # lower bound on RC

    # area covered
    area_covered = sum(
        items_data[copy_to_item[n]]['w'] * items_data[copy_to_item[n]]['h']
        for n in expanded_items
        if z[n].X > 0.5
    )

    return {
        "rc":          rc,
        "pattern":     pattern,
        "layout":      layout,
        "status":      status_map.get(sub.Status, "UNKNOWN"),
        "solve_time":  solve_time,
        "obj_val":     rc_val,
        "obj_bound":   rc_bound,
        "mip_gap_pct": mip_gap,
        "area_covered": area_covered,
        "sheet_area":   W * H,
    }


# ============================================================
# SOLVER 2 -- CP-SAT (NoOverlap2D formulation)
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

    # Adaptive scaling -- keeps objective in meaningful integer range
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

        # optional intervals -- active only when s[n] = 1
        x_iv[n] = model.NewOptionalFixedSizeIntervalVar(
            xvar[n], w, s[n], f"xi_{n}")
        y_iv[n] = model.NewOptionalFixedSizeIntervalVar(
            yvar[n], h, s[n], f"yi_{n}")

    # --------------------------------------------------------
    # Constraints
    # --------------------------------------------------------

    # Single no-overlap constraint replaces all Big-M constraints
    model.AddNoOverlap2D(list(x_iv.values()), list(y_iv.values()))

    # Symmetry breaking -- same type copies ordered
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

    # area covered
    area_covered = sum(
        items_data[copy_to_item[n]]['w'] * items_data[copy_to_item[n]]['h']
        for n in expanded_items
        if solver.Value(s[n]) == 1
    )

    return {
        "rc":          rc,
        "pattern":     pattern,
        "layout":      layout,
        "status":      status_str,
        "solve_time":  solve_time,
        "obj_val":     rc_val,
        "obj_bound":   rc_bound,
        "mip_gap_pct": gap_pct,
        "area_covered": area_covered,
        "sheet_area":   W * H,
    }


# ============================================================
# MAIN -- compare both solvers on same duals
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

    if result_gurobi and result_cpsat:
        sheet_area = W * H
        g_area  = result_gurobi['area_covered']
        c_area  = result_cpsat['area_covered']
        g_util  = g_area / sheet_area * 100
        c_util  = c_area / sheet_area * 100
        all_items = list(items_data.keys())

        SEP = "=" * 60
        sep = "-" * 60
        W60 = 30  # label column width

        def fmt_val(v, decimals=6):
            """Format a float or None."""
            if v is None:
                return "N/A"
            return f"{v:.{decimals}f}"

        def fmt_pct(v):
            if v is None:
                return "N/A"
            return f"{v:.2f}%"

        print(f"\n{SEP}")
        print("INPUT DATA")
        print(SEP)
        for item, data in items_data.items():
            print(
                f"{item}: demand={data['demand']}, "
                f"w={data['w']}, h={data['h']}, "
                f"dual={duals.get(item, 0)}"
            )

        print(f"\n{SEP}")
        print("COMPARISON SUMMARY")
        print(SEP)
        print(f"\n{'Metric':<{W60}} {'Gurobi MIP':>14} {'CP-SAT':>14}")
        print(sep)

        # --- solver status & RC block (matches image order) ---
        print(f"{'Status':<{W60}} {result_gurobi['status']:>14} {result_cpsat['status']:>14}")
        print(f"{'Reduced Cost':<{W60}} {fmt_val(result_gurobi['rc']):>14} {fmt_val(result_cpsat['rc']):>14}")
        print(f"{'Incumbent RC':<{W60}} {fmt_val(result_gurobi['obj_val']):>14} {fmt_val(result_cpsat['obj_val']):>14}")
        print(f"{'Best Bound RC':<{W60}} {fmt_val(result_gurobi['obj_bound']):>14} {fmt_val(result_cpsat['obj_bound']):>14}")
        print(f"{'Gap %':<{W60}} {fmt_pct(result_gurobi['mip_gap_pct']):>14} {fmt_pct(result_cpsat['mip_gap_pct']):>14}")
        print(f"{'Solve Time (s)':<{W60}} {result_gurobi['solve_time']:>14.4f} {result_cpsat['solve_time']:>14.4f}")
        print(sep)

        # --- pattern block ---
        g_total = sum(result_gurobi['pattern'].values())
        c_total = sum(result_cpsat['pattern'].values())
        print(f"{'Items Packed (total)':<{W60}} {g_total:>14} {c_total:>14}")
        for item in all_items:
            g_cnt = result_gurobi['pattern'].get(item, 0)
            c_cnt = result_cpsat['pattern'].get(item,  0)
            label = f"  Pattern [{item}] (w={items_data[item]['w']} h={items_data[item]['h']})"
            print(f"{label:<{W60}} {g_cnt:>14} {c_cnt:>14}")
        print(sep)

        # --- area block ---
        print(f"{'Sheet Area (W x H)':<{W60}} {sheet_area:>14} {sheet_area:>14}")
        print(f"{'Area Covered':<{W60}} {g_area:>14} {c_area:>14}")
        print(f"{'Utilization %':<{W60}} {g_util:>13.2f}% {c_util:>13.2f}%")
        print(sep)

        # --- bounds block (only shown when NOT optimal) ---
        g_not_opt = result_gurobi['status'] != 'OPTIMAL'
        c_not_opt = result_cpsat['status']  != 'OPTIMAL'
        if g_not_opt or c_not_opt:
            print(f"\n{'-- BOUNDS (non-optimal solvers only) --':^60}")
            print(sep)
            # Lower bound = best proven lower bound on RC (= obj_bound for minimization)
            # Upper bound = incumbent RC (best feasible solution found)
            g_lb = fmt_val(result_gurobi['obj_bound']) if g_not_opt else "  (optimal)"
            c_lb = fmt_val(result_cpsat['obj_bound'])  if c_not_opt else "  (optimal)"
            g_ub = fmt_val(result_gurobi['obj_val'])   if g_not_opt else "  (optimal)"
            c_ub = fmt_val(result_cpsat['obj_val'])    if c_not_opt else "  (optimal)"
            print(f"{'Lower Bound RC (best proven)':<{W60}} {g_lb:>14} {c_lb:>14}")
            print(f"{'Upper Bound RC (incumbent)':<{W60}} {g_ub:>14} {c_ub:>14}")
            print(f"{'Optimality Gap %':<{W60}} {fmt_pct(result_gurobi['mip_gap_pct']) if g_not_opt else '  (optimal)':>14} {fmt_pct(result_cpsat['mip_gap_pct']) if c_not_opt else '  (optimal)':>14}")
            print(f"{'True optimal RC in range':<{W60}}", end="")
            for res, not_opt in [(result_gurobi, g_not_opt), (result_cpsat, c_not_opt)]:
                if not_opt:
                    rng = f"[{fmt_val(res['obj_bound'])}, {fmt_val(res['obj_val'])}]"
                    print(f" {rng:>14}", end="")
                else:
                    print(f" {'  (proven)':>14}", end="")
            print()
            print(sep)

        # --- RC diff & verdict ---
        print()
        rc_diff = abs(result_gurobi['rc'] - result_cpsat['rc'])
        print(f"RC Difference : {rc_diff:.8f}")
        if rc_diff < 1e-4:
            print("-> RC values match within 1e-4 tolerance")
        else:
            print("-> RC values differ -- different patterns found")

        # --- optimality verdict per solver ---
        print()
        for name, res in [("Gurobi", result_gurobi), ("CP-SAT", result_cpsat)]:
            if res['status'] == 'OPTIMAL':
                print(f"{name}: PROVEN OPTIMAL -- RC = {res['rc']:.6f}")
            else:
                print(f"{name}: NOT PROVEN OPTIMAL")
                print(f"  Incumbent RC  = {fmt_val(res['obj_val'])}")
                print(f"  Lower Bound   = {fmt_val(res['obj_bound'])}")
                print(f"  Upper Bound   = {fmt_val(res['obj_val'])}")
                print(f"  Gap           = {fmt_pct(res['mip_gap_pct'])}")
                print(f"  True optimal RC in [{fmt_val(res['obj_bound'])}, {fmt_val(res['obj_val'])}]")
