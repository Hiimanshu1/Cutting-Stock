import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import csv
import os
from datetime import datetime

# ============================================================
# OUTPUT FOLDER
# ============================================================

script_dir  = os.path.dirname(os.path.abspath(__file__))
run_stamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir  = os.path.join(script_dir, f"cg_results_{run_stamp}")
os.makedirs(output_dir, exist_ok=True)

print(f"Output folder : {output_dir}")

# ============================================================
# INPUT DATA
# ============================================================


items_data = {

    'A': {'demand': 160, 'w': 3, 'h': 2},

    'B': {'demand': 150, 'w': 5, 'h': 3},

    'C': {'demand': 170, 'w': 2, 'h': 2},

    'D': {'demand': 130, 'w': 6, 'h': 4},

    'E': {'demand': 120, 'w': 7, 'h': 5},
}

W, H = 60, 30

# ============================================================
# PRINT DATASET USED
# ============================================================

print("\n")
print("=" * 70)
print("DATASET USED")
print("=" * 70)

print(f"\nSheet size: W={W}, H={H}")
print("\nitems_data = {")
for k, v in items_data.items():
    print(f"    '{k}': {{'demand': {v['demand']}, 'w': {v['w']}, 'h': {v['h']}}},")
print("}")
# ============================================================
# TOTAL TIMER
# ============================================================

total_start = time.time()

MAX_RUNTIME = 6 * 60 * 60 

# ============================================================
# DATA
# ============================================================

items = list(items_data.keys())

demands = {}
widths = {}
heights = {}

for item in items:

    demands[item] = items_data[item]["demand"]

    widths[item] = items_data[item]["w"]

    heights[item] = items_data[item]["h"]

# ============================================================
# INITIAL PATTERNS
# ============================================================

patterns = []

a = {}

for idx, item in enumerate(items):

    p = f"p{idx+1}"

    patterns.append(p)

    for j in items:

        if item == j:
            a[(j,p)] = 1
        else:
            a[(j,p)] = 0

# ============================================================
# MASTER PROBLEM
# ============================================================

master = gp.Model("Master")

master.Params.OutputFlag = 1

x = {}

for p in patterns:

    x[p] = master.addVar(

        lb=0,

        vtype=GRB.CONTINUOUS,

        name=f"x_{p}"
    )

master.update()

demand_constraints = {}

for item in items:

    demand_constraints[item] = master.addConstr(

        gp.quicksum(

            a[(item,p)] * x[p]

            for p in patterns
        )

        >=

        demands[item]
    )

master.setObjective(

    gp.quicksum(

        x[p]

        for p in patterns
    ),

    GRB.MINIMIZE
)

# ============================================================
# VISUALIZATION STORAGE
# ============================================================

pattern_visualizations = {}

# ============================================================
# WARM START STORAGE
# ============================================================

prev_sol = None

# ============================================================
# LOG STORAGE
# ============================================================

log = []

# ============================================================
# COLUMN GENERATION
# ============================================================

iteration = 1

while True:

    runtime = time.time() - total_start

    if runtime >= MAX_RUNTIME:

        print("\nMAXIMUM RUNTIME REACHED")

        break

    print("\n")
    print("=" * 70)
    print(f"ITERATION {iteration}")
    print("=" * 70)

    # ========================================================
    # MASTER SOLVE
    # ========================================================

    print("\nMASTER PROBLEM SOLVE")

    master_start = time.time()

    master.optimize()

    master_end = time.time()

    master_time = master_end - master_start

    master_obj = master.ObjVal

    # ========================================================
    # DUAL VALUES
    # ========================================================

    duals = {}

    for item in items:

        duals[item] = demand_constraints[item].Pi

    print("\nDUALS")

    print(duals)

    # ========================================================
    # COPY REDUCTION
    # ========================================================

    expanded_items = []

    copy_to_item = {}

    counter = 1

    for item in items:

        demand = demands[item]

        w = widths[item]

        h = heights[item]

        max_by_area = (W * H) // (w * h)

        max_copies = min(
            demand,
            max_by_area
        )

        for _ in range(max_copies):

            expanded_items.append(counter)

            copy_to_item[counter] = item

            counter += 1

    print("\nNUMBER OF EXPANDED ITEMS")

    print(len(expanded_items))

    # ========================================================
    # SUBPROBLEM
    # ========================================================

    sub = gp.Model("Pricing")

    sub.Params.OutputFlag = 1

    sub.Params.MIPGap = 0.20

    sub.Params.Heuristics = 0.8

    sub.Params.MIPFocus = 1

    sub.Params.Threads = 1

    # ========================================================
    # VARIABLES
    # ========================================================

    z = {}
    xpos = {}
    ypos = {}

    for n in expanded_items:

        z[n] = sub.addVar(
            vtype=GRB.BINARY,
            name=f"z_{n}"
        )

        xpos[n] = sub.addVar(
            lb=0,
            name=f"x_{n}"
        )

        ypos[n] = sub.addVar(
            lb=0,
            name=f"y_{n}"
        )

    # ========================================================
    # RELATIVE POSITION VARIABLES
    # ========================================================

    L = {}
    R = {}
    B = {}
    T = {}

    pairs = []

    for i_idx in range(len(expanded_items)):

        for j_idx in range(i_idx+1, len(expanded_items)):

            i = expanded_items[i_idx]

            j = expanded_items[j_idx]

            pairs.append((i,j))

            L[(i,j)] = sub.addVar(vtype=GRB.BINARY)

            R[(i,j)] = sub.addVar(vtype=GRB.BINARY)

            B[(i,j)] = sub.addVar(vtype=GRB.BINARY)

            T[(i,j)] = sub.addVar(vtype=GRB.BINARY)

    sub.update()

    # ========================================================
    # WARM START FROM PREVIOUS SOLUTION
    # ========================================================

    if prev_sol is not None:

        sub.NumStart = 1

        sub.Params.StartNumber = 0

        for n in expanded_items:

            if n in prev_sol:

                z[n].Start    = prev_sol[n]["z"]
                xpos[n].Start = prev_sol[n]["x"]
                ypos[n].Start = prev_sol[n]["y"]

            else:

                z[n].Start    = 0
                xpos[n].Start = 0
                ypos[n].Start = 0

        for (i, j) in pairs:

            item_i = copy_to_item[i]
            item_j = copy_to_item[j]

            zi = prev_sol[i]["z"] if i in prev_sol else 0
            zj = prev_sol[j]["z"] if j in prev_sol else 0

            xi = prev_sol[i]["x"] if i in prev_sol else 0
            yi = prev_sol[i]["y"] if i in prev_sol else 0
            xj = prev_sol[j]["x"] if j in prev_sol else 0
            yj = prev_sol[j]["y"] if j in prev_sol else 0

            wi = widths[item_i]
            hi = heights[item_i]

            # infer relative positions from coordinates
            if xi + wi <= xj + 1e-6:
                L[(i,j)].Start = 1
                R[(i,j)].Start = 0
                B[(i,j)].Start = 0
                T[(i,j)].Start = 0
            elif xj + widths[item_j] <= xi + 1e-6:
                L[(i,j)].Start = 0
                R[(i,j)].Start = 1
                B[(i,j)].Start = 0
                T[(i,j)].Start = 0
            elif yi + hi <= yj + 1e-6:
                L[(i,j)].Start = 0
                R[(i,j)].Start = 0
                B[(i,j)].Start = 1
                T[(i,j)].Start = 0
            else:
                L[(i,j)].Start = 0
                R[(i,j)].Start = 0
                B[(i,j)].Start = 0
                T[(i,j)].Start = 1

        print("\nWARM START INJECTED FROM PREVIOUS SOLUTION")

    # ========================================================
    # BIG-M VALUES
    # ========================================================

    Mx = W

    My = H

    # ========================================================
    # BOUNDARY CONSTRAINTS
    # ========================================================

    for n in expanded_items:

        item = copy_to_item[n]

        sub.addConstr(

            xpos[n] + widths[item]

            <=

            W + Mx*(1-z[n])
        )

        sub.addConstr(

            ypos[n] + heights[item]

            <=

            H + My*(1-z[n])
        )

    # ========================================================
    # NON OVERLAP CONSTRAINTS
    # ========================================================

    for (i,j) in pairs:

        item_i = copy_to_item[i]

        item_j = copy_to_item[j]

        wi = widths[item_i]

        hi = heights[item_i]

        wj = widths[item_j]

        hj = heights[item_j]

        sub.addConstr(

            xpos[i] + wi

            <=

            xpos[j] + Mx*(1-L[(i,j)])
        )

        sub.addConstr(

            xpos[j] + wj

            <=

            xpos[i] + Mx*(1-R[(i,j)])
        )

        sub.addConstr(

            ypos[i] + hi

            <=

            ypos[j] + My*(1-B[(i,j)])
        )

        sub.addConstr(

            ypos[j] + hj

            <=

            ypos[i] + My*(1-T[(i,j)])
        )

        sub.addConstr(

            L[(i,j)]
            +
            R[(i,j)]
            +
            B[(i,j)]
            +
            T[(i,j)]

            >=

            z[i] + z[j] - 1
        )

    # ========================================================
    # SYMMETRY BREAKING
    # ========================================================

    for item in items:

        same_type = []

        for n in expanded_items:

            if copy_to_item[n] == item:

                same_type.append(n)

        for k in range(len(same_type)-1):

            i = same_type[k]

            j = same_type[k+1]

            sub.addConstr(

                z[i] >= z[j]
            )

    # ========================================================
    # OBJECTIVE
    # ========================================================

    sub.setObjective(

        1
        -
        gp.quicksum(

            duals[copy_to_item[n]] * z[n]

            for n in expanded_items
        ),

        GRB.MINIMIZE
    )

    # ========================================================
    # PHASE 1 : 10 SECOND HEURISTIC SEARCH
    # ========================================================

    print("\nPHASE 1 : 10 SECOND SEARCH")

    sub.Params.TimeLimit = 10

    sub_start = time.time()

    sub.optimize()

    # ========================================================
    # CASE 1 : IMPROVING COLUMN FOUND
    # ========================================================

    if sub.SolCount > 0 and sub.ObjVal < -1e-6:

        print("\nNEGATIVE REDUCED COST FOUND")

        print(f"\nREDUCED COST : {sub.ObjVal}")

        rc = sub.ObjVal

    # ========================================================
    # CASE 2 : NO IMPROVING COLUMN - PHASE 2 SINGLE SOLVE WITH CALLBACK
    # ========================================================

    else:

        print("\nNO IMPROVING COLUMN IN 10 SECONDS")

        print("\nPHASE 2 : CONTINUING SOLVE WITH REMAINING TIME BUDGET")

        CHECKPOINT_INTERVAL = 5 * 60  # 5 minutes

        elapsed = time.time() - total_start

        remaining = MAX_RUNTIME - elapsed

        if remaining <= 0:

            print("\nMAXIMUM RUNTIME REACHED")

            sub_end  = time.time()
            sub_time = sub_end - sub_start

            log.append({
                "iteration":    iteration,
                "time":         time.time() - total_start,
                "reduced_cost": sub.ObjVal if sub.SolCount > 0 else float('inf'),
                "patterns":     len(patterns),
                "master_obj":   master_obj,
                "master_time":  master_time,
                "pricing_time": sub_time,
            })

            break

        # Track state across callback calls
        cb_state = {
            "best_obj":        sub.ObjVal if sub.SolCount > 0 else float('inf'),
            "last_check_time": time.time(),
            "terminate":       False,
        }

        def phase2_callback(model, where):

            if where == GRB.Callback.MIPNODE:

                now = time.time()

                # Global time check
                if now - total_start >= MAX_RUNTIME:

                    print("\nMAXIMUM RUNTIME REACHED IN CALLBACK")

                    cb_state["terminate"] = True

                    model.terminate()

                    return

                # 5-min checkpoint
                if now - cb_state["last_check_time"] >= CHECKPOINT_INTERVAL:

                    cb_state["last_check_time"] = now

                    incumbent  = model.cbGet(GRB.Callback.MIPNODE_OBJBST)
                    best_bound = model.cbGet(GRB.Callback.MIPNODE_OBJBND)

                    gap = abs(incumbent - best_bound) / (abs(incumbent) + 1e-10)

                    print(f"\n5-MIN CHECKPOINT | Incumbent: {incumbent:.6f} | Best so far: {cb_state['best_obj']:.6f} | Gap: {gap*100:.2f}%")

                    if incumbent < cb_state["best_obj"] - 1e-6:

                        cb_state["best_obj"] = incumbent

                        # Only terminate early if we have a negative RC
                        if incumbent < -1e-6:

                            print("\nIMPROVED AND NEGATIVE RC - TERMINATING PHASE 2, SENDING TO MASTER")

                            cb_state["terminate"] = False

                            model.terminate()

                        else:

                            print("\nIMPROVED BUT RC STILL POSITIVE - CONTINUING TO FIND NEGATIVE RC")

                    else:

                        # No improvement - keep going
                        print("\nNO IMPROVEMENT - CONTINUING TO NEXT 5-MIN CHECKPOINT")

        # Single solve - continues from where Phase 1 left off
        sub.Params.TimeLimit = remaining

        sub.optimize(phase2_callback)

        if sub.SolCount == 0:

            print("\nNO FEASIBLE SOLUTION")

            break

        rc = sub.ObjVal

        print(f"\nPHASE 2 REDUCED COST : {rc}")

        # Only hard-stop if max time hit
        if cb_state["terminate"]:

            print("\nMAXIMUM RUNTIME REACHED - EXITING")

            sub_end  = time.time()
            sub_time = sub_end - sub_start

            log.append({
                "iteration":    iteration,
                "time":         time.time() - total_start,
                "reduced_cost": rc,
                "patterns":     len(patterns),
                "master_obj":   master_obj,
                "master_time":  master_time,
                "pricing_time": sub_time,
            })

            break

    sub_end = time.time()

    sub_time = sub_end - sub_start

    # ========================================================
    # SAVE SOLUTION FOR WARM START
    # ========================================================

    if sub.SolCount > 0:

        prev_sol = {}

        for n in expanded_items:

            prev_sol[n] = {
                "z": z[n].X,
                "x": xpos[n].X,
                "y": ypos[n].X,
            }

    # ========================================================
    # TERMINATION
    # ========================================================

    if rc >= -1e-6:

        print("\nNO IMPROVING COLUMN")

        # log final iteration before break
        log.append({
            "iteration":    iteration,
            "time":         time.time() - total_start,
            "reduced_cost": rc,
            "patterns":     len(patterns),
            "master_obj":   master_obj,
            "master_time":  master_time,
            "pricing_time": sub_time,
        })

        break

    # ========================================================
    # EXTRACT PATTERN
    # ========================================================

    new_pattern = {}

    for item in items:

        new_pattern[item] = 0

    rectangles = []

    for n in expanded_items:

        if z[n].X > 0.5:

            item = copy_to_item[n]

            new_pattern[item] += 1

            rectangles.append({

                "item": item,

                "x": xpos[n].X,

                "y": ypos[n].X,

                "w": widths[item],

                "h": heights[item]
            })

    print("\nNEW PATTERN")

    print(new_pattern)

    # ========================================================
    # ADD COLUMN
    # ========================================================

    new_p = f"p{len(patterns)+1}"

    patterns.append(new_p)

    pattern_visualizations[new_p] = rectangles

    column = gp.Column()

    for item in items:

        coeff = new_pattern[item]

        a[(item,new_p)] = coeff

        column.addTerms(

            coeff,

            demand_constraints[item]
        )

    x[new_p] = master.addVar(

        obj=1,

        lb=0,

        vtype=GRB.CONTINUOUS,

        column=column,

        name=f"x_{new_p}"
    )

    master.update()

    # ========================================================
    # TIMING
    # ========================================================

    iteration_time = time.time() - master_start

    print("\nTIMING")

    print(f"Master Time     : {master_time:.4f}")

    print(f"Subproblem Time : {sub_time:.4f}")

    print(f"Iteration Time  : {iteration_time:.4f}")

    # ========================================================
    # LOG
    # ========================================================

    log.append({
        "iteration":    iteration,
        "time":         time.time() - total_start,
        "reduced_cost": rc,
        "patterns":     len(patterns),
        "master_obj":   master_obj,
        "master_time":  master_time,
        "pricing_time": sub_time,
    })

    iteration += 1

# ============================================================
# SAVE LP SOLUTION BEFORE INTEGER CONVERSION
# ============================================================

lp_solution = {}

if master.SolCount > 0:

    for p in patterns:

        if x[p].X > 1e-6:

            lp_solution[p] = x[p].X

# ============================================================
# FINAL INTEGER SOLVE
# ============================================================

print("\n")
print("=" * 70)
print("FINAL INTEGER MASTER SOLVE")
print("=" * 70)

for p in patterns:

    x[p].VType = GRB.INTEGER

master.update()

remaining_for_final = max(1.0, MAX_RUNTIME - (time.time() - total_start))

master.Params.TimeLimit = remaining_for_final

master.optimize()

# ============================================================
# FINAL SOLUTION
# ============================================================

print("\n")
print("=" * 70)
print("FINAL SOLUTION")
print("=" * 70)

used_patterns = []

if master.SolCount > 0:

    for p in patterns:

        if x[p].X > 1e-6:

            used_patterns.append(p)

            print(f"\nPattern {p} used {x[p].X}")

            for item in items:

                print(f"{item}: {a[(item,p)]}")

else:

    print("\nNO FEASIBLE INTEGER SOLUTION - SHOWING ALL GENERATED PATTERNS")

# ============================================================
# OPTIMALITY CHECK + BOTH SOLUTIONS
# ============================================================

import math

lp_bound    = log[-1]["master_obj"] if log else None
lp_ceil     = math.ceil(lp_bound)  if lp_bound else None
integer_obj = master.ObjVal        if master.SolCount > 0 else None

# ============================================================
# PATTERN-LEVEL ROUNDING SOLUTION
# ============================================================

print("\n")
print("=" * 70)
print("PATTERN-LEVEL ROUNDING SOLUTION")
print("=" * 70)

rounded_patterns = {}
rounded_total    = 0

for p, lp_val in lp_solution.items():

    rounded = math.ceil(lp_val)

    rounded_patterns[p] = rounded

    rounded_total += rounded

    print(f"\nPattern {p} : LP={lp_val:.4f} -> rounded up to {rounded}")

    for item in items:

        print(f"  {item}: {a[(item, p)]}")

print(f"\nTotal sheets (pattern rounding) : {rounded_total}")

# ============================================================
# SOLUTION SUMMARY
# ============================================================

print("\n")
print("=" * 70)
print("SOLUTION SUMMARY")
print("=" * 70)

if lp_bound is not None:
    print(f"\nLP Relaxation Bound          : {lp_bound:.4f}")
    print(f"Ceiling of LP Bound          : {lp_ceil} sheets")

print(f"Pattern-Level Rounding Total : {rounded_total} sheets")

if integer_obj is not None:
    print(f"Integer Solve Solution       : {int(integer_obj)} sheets")
    if integer_obj <= lp_ceil:
        print(f"\nGLOBALLY OPTIMAL : {int(integer_obj)} sheets PROVEN OPTIMAL (IRUP satisfied)")
    elif integer_obj <= lp_ceil + 1:
        print(f"\nNEAR-OPTIMAL : {int(integer_obj)} sheets (MIRUP - at most 1 from optimal)")
    else:
        gap = (integer_obj - lp_bound) / lp_bound * 100
        print(f"\nOPTIMALITY GAP : {gap:.2f}%")

# Always show all patterns that have visualization data,
# plus any used patterns from integer solve
all_display_patterns = used_patterns if used_patterns else list(pattern_visualizations.keys())

# ============================================================
# TOTAL RUNTIME
# ============================================================

total_runtime = time.time() - total_start

print("\n")
print("=" * 70)
print("TOTAL EXECUTION TIME")
print("=" * 70)

print(f"\nTotal Runtime : {total_runtime:.4f} sec")

# ============================================================
# CSV EXPORT
# ============================================================

if log:

    # add dataset info as rows
    log.append({
        "iteration":    "DATASET",
        "time":         "",
        "reduced_cost": "",
        "patterns":     "",
        "master_obj":   f"W={W}",
        "master_time":  f"H={H}",
        "pricing_time": "",
    })

    for k, v in items_data.items():
        log.append({
            "iteration":    "DATASET",
            "time":         "",
            "reduced_cost": "",
            "patterns":     k,
            "master_obj":   f"demand={v['demand']}",
            "master_time":  f"w={v['w']}",
            "pricing_time": f"h={v['h']}",
        })

    # add ceiling and integer solution as final rows
    if lp_bound is not None:
        log.append({
            "iteration":    "CEILING",
            "time":         time.time() - total_start,
            "reduced_cost": "-",
            "patterns":     len(patterns),
            "master_obj":   lp_bound,
            "master_time":  "-",
            "pricing_time": f"ceil={lp_ceil}",
        })

    if integer_obj is not None:
        log.append({
            "iteration":    "INTEGER",
            "time":         time.time() - total_start,
            "reduced_cost": "-",
            "patterns":     len(patterns),
            "master_obj":   integer_obj,
            "master_time":  "-",
            "pricing_time": f"sheets={int(integer_obj)}",
        })

    log.append({
        "iteration":    "ROUNDING",
        "time":         time.time() - total_start,
        "reduced_cost": "-",
        "patterns":     len(patterns),
        "master_obj":   rounded_total,
        "master_time":  "-",
        "pricing_time": f"sheets={rounded_total}",
    })

    fieldnames = ["iteration", "time", "reduced_cost", "patterns",
                  "master_obj", "master_time", "pricing_time"]

    final_csv = os.path.join(output_dir, f"column_generation_log_{run_stamp}.csv")

    with open(final_csv, "w", newline="") as f:

        writer = csv.DictWriter(f, fieldnames=fieldnames)

        writer.writeheader()

        writer.writerows(log)

    print(f"\nCSV saved : column_generation_log_{run_stamp}.csv")

else:

    print("\nNO ITERATIONS COMPLETED - CSV NOT SAVED")

# ============================================================
# ANALYTICS GRAPHS
# ============================================================

if len(log) >= 2:

    numeric_log = [r for r in log if isinstance(r["iteration"], int)]

    if len(numeric_log) < 2:
        numeric_log = []

    iters       = [r["iteration"]    for r in numeric_log]
    times       = [r["time"]         for r in numeric_log]
    red_costs   = [r["reduced_cost"] for r in numeric_log]
    pat_counts  = [r["patterns"]     for r in numeric_log]
    master_objs = [r["master_obj"]   for r in numeric_log]
    m_times     = [r["master_time"]  for r in numeric_log]
    p_times     = [r["pricing_time"] for r in numeric_log]

    cum_master  = []
    cum_pricing = []
    cum_m = 0
    cum_p = 0

    for m, p in zip(m_times, p_times):
        cum_m += m
        cum_p += p
        cum_master.append(cum_m)
        cum_pricing.append(cum_p)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Column Generation Analytics", fontsize=16)

    axes[0][0].plot(times, pat_counts, marker='o', color='steelblue')
    axes[0][0].set_xlabel("Time (s)")
    axes[0][0].set_ylabel("Pattern Count")
    axes[0][0].set_title("Time vs Pattern Count")
    axes[0][0].grid(True)

    axes[0][1].plot(times, red_costs, marker='o', color='tomato')
    axes[0][1].set_xlabel("Time (s)")
    axes[0][1].set_ylabel("Reduced Cost")
    axes[0][1].set_title("Time vs Reduced Cost")
    axes[0][1].grid(True)

    axes[0][2].plot(iters, master_objs, marker='s', color='mediumseagreen')
    axes[0][2].set_xlabel("Iteration")
    axes[0][2].set_ylabel("Master Objective")
    axes[0][2].set_title("Iteration vs Master Objective")
    axes[0][2].grid(True)

    axes[1][0].plot(times, master_objs, marker='s', color='darkorange')
    axes[1][0].set_xlabel("Time (s)")
    axes[1][0].set_ylabel("Master Objective")
    axes[1][0].set_title("Time vs Master Objective")
    axes[1][0].grid(True)

    axes[1][1].stackplot(
        iters, cum_master, cum_pricing,
        labels=["Master (cumul.)", "Pricing (cumul.)"],
        colors=["cornflowerblue", "salmon"], alpha=0.8
    )
    axes[1][1].set_xlabel("Iteration")
    axes[1][1].set_ylabel("Cumulative Time (s)")
    axes[1][1].set_title("Cumulative Runtime Breakdown")
    axes[1][1].legend(loc="upper left")
    axes[1][1].grid(True)

    axes[1][2].set_visible(False)

    plt.tight_layout()
    fig.canvas.draw()
    fig.savefig(os.path.join(output_dir, "analytics.png"), dpi=150)
    plt.show(block=False)
    plt.pause(0.1)

    print("\nAnalytics graph saved + displayed")

# ============================================================
# PATTERN VISUALIZATION
# ============================================================

for p in all_display_patterns:

    rectangles = pattern_visualizations.get(p)

    # For initial patterns not stored in visualizations,
    # draw schematic boxes stacked by item count
    if not rectangles:

        rectangles = []

        x_cursor = 0
        y_cursor = 0
        row_h    = 0

        for item in items:

            count = a.get((item, p), 0)

            w = widths[item]
            h = heights[item]

            for _ in range(int(count)):

                if x_cursor + w > W:
                    x_cursor  = 0
                    y_cursor += row_h
                    row_h     = 0

                rectangles.append({
                    "item": item,
                    "x":    x_cursor,
                    "y":    y_cursor,
                    "w":    w,
                    "h":    h,
                })

                x_cursor += w
                row_h     = max(row_h, h)

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.add_patch(patches.Rectangle(
        (0, 0), W, H,
        linewidth=3, edgecolor='black', facecolor='lightgray'
    ))

    for rect_data in rectangles:

        ax.add_patch(patches.Rectangle(
            (rect_data["x"], rect_data["y"]),
            rect_data["w"], rect_data["h"],
            linewidth=2, edgecolor='blue', facecolor='skyblue'
        ))

        ax.text(
            rect_data["x"] + rect_data["w"] / 2,
            rect_data["y"] + rect_data["h"] / 2,
            rect_data["item"],
            ha='center', va='center'
        )

    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect('equal')
    ax.set_title(f"Pattern {p}")
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f"pattern_{p}.png"), dpi=150)
    plt.show(block=False)
    plt.pause(0.001)

    print(f"Pattern {p} saved + displayed")