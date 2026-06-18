import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import csv
import os
import sys
import threading
from datetime import datetime
from ortools.sat.python import cp_model

# ============================================================
# OUTPUT FOLDER
# ============================================================

script_dir  = os.path.dirname(os.path.abspath(__file__))
run_stamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir  = os.path.join(script_dir, f"cg_results_{run_stamp}")
os.makedirs(output_dir, exist_ok=True)

print(f"Output folder : {output_dir}")

# ============================================================
# TERMINAL LOG - TEE STDOUT + STDERR TO FILE
# ============================================================

class Tee:
    """Writes to both terminal and log file simultaneously."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            try:
                s.write(data)
                s.flush()
            except Exception:
                pass

    def flush(self):
        for s in self.streams:
            try:
                s.flush()
            except Exception:
                pass

terminal_log_path = os.path.join(output_dir, "terminal.log")
terminal_log_file = open(terminal_log_path, "w", buffering=1)

sys.stdout = Tee(sys.__stdout__, terminal_log_file)
sys.stderr = Tee(sys.__stderr__, terminal_log_file)

print(f"Terminal log  : {terminal_log_path}")

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
# WATCHDOG THREAD - hard kills process if MAX_RUNTIME exceeded
# ============================================================

def _watchdog():
    while True:
        time.sleep(10)
        if time.time() - total_start >= MAX_RUNTIME + 60:
            print("\nWATCHDOG: MAX_RUNTIME + 60s exceeded - force exit")
            sys.stdout.flush()
            os._exit(1)

_wd = threading.Thread(target=_watchdog, daemon=True)
_wd.start()

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
    # CP-SAT SUBPROBLEM
    # ========================================================

    # adaptive scaling based on dual magnitude - capped to avoid overflow
    max_dual = max(abs(v) for v in duals.values()) if duals else 1.0

    if max_dual > 0:

        SCALE = max(min(int(1e6 / max_dual), 10**6), 1000)

    else:

        SCALE = 10**6

    print(f"\nSCALE : {SCALE} (max dual : {max_dual:.6f})")

    cp    = cp_model.CpModel()

    # ========================================================
    # CP-SAT VARIABLES
    # ========================================================

    s    = {}   # binary: item selected
    xvar = {}   # x position (integer)
    yvar = {}   # y position (integer)
    x_iv = {}   # x interval (optional)
    y_iv = {}   # y interval (optional)

    for n in expanded_items:

        item = copy_to_item[n]
        w    = widths[item]
        h    = heights[item]

        s[n]    = cp.NewBoolVar(f"s_{n}")

        xvar[n] = cp.NewIntVar(0, W - w, f"x_{n}")
        yvar[n] = cp.NewIntVar(0, H - h, f"y_{n}")

        # optional intervals - active only when s[n] = 1
        x_iv[n] = cp.NewOptionalFixedSizeIntervalVar(
            xvar[n], w, s[n], f"xi_{n}"
        )
        y_iv[n] = cp.NewOptionalFixedSizeIntervalVar(
            yvar[n], h, s[n], f"yi_{n}"
        )

    # ========================================================
    # NO OVERLAP - SINGLE CONSTRAINT REPLACES ALL BIG-M
    # ========================================================

    cp.AddNoOverlap2D(
        list(x_iv.values()),
        list(y_iv.values())
    )

    # ========================================================
    # SYMMETRY BREAKING
    # ========================================================

    for item in items:

        same_type = [n for n in expanded_items if copy_to_item[n] == item]

        for k in range(len(same_type) - 1):

            i = same_type[k]
            j = same_type[k + 1]

            # z_i >= z_j : earlier index used before later
            cp.Add(s[i] >= s[j])

            # lexicographic x ordering among same type
            cp.Add(xvar[i] <= xvar[j]).OnlyEnforceIf([s[j]])

    # ========================================================
    # OBJECTIVE : maximize sum(scaled_dual * s_i)
    # ========================================================

    obj_terms = []

    for n in expanded_items:

        dual_int = round(duals[copy_to_item[n]] * SCALE)

        obj_terms.append(dual_int * s[n])

    cp.Maximize(cp_model.LinearExpr.Sum(obj_terms))

    # ========================================================
    # SOLVER SETUP
    # ========================================================

    solver = cp_model.CpSolver()

    solver.parameters.num_search_workers = 4
    solver.parameters.log_search_progress = True

    sub_start = time.time()

    # ========================================================
    # PHASE 1 : 10 SECOND HEURISTIC SEARCH
    # ========================================================

    print("\nPHASE 1 : 10 SECOND SEARCH")

    phase1_budget = max(1.0, MAX_RUNTIME - (time.time() - total_start))
    phase1_time   = min(10.0, phase1_budget)

    solver.parameters.max_time_in_seconds = phase1_time

    status1 = solver.Solve(cp)

    rc = None

    if status1 in (cp_model.OPTIMAL, cp_model.FEASIBLE):

        rc = 1.0 - solver.ObjectiveValue() / SCALE

        print(f"\nPHASE 1 STATUS : {'OPTIMAL' if status1 == cp_model.OPTIMAL else 'FEASIBLE'}")
        print(f"REDUCED COST   : {rc:.6f}")

    else:

        print("\nPHASE 1 : NO SOLUTION FOUND")

        rc = float('inf')

    # ========================================================
    # CASE 1 : IMPROVING COLUMN FOUND IN PHASE 1
    # ========================================================

    if rc is not None and rc < -1e-4:

        print("\nNEGATIVE REDUCED COST FOUND IN PHASE 1")

        cp_status     = status1
        cp_solver_ref = solver

    # ========================================================
    # CASE 2 : NO IMPROVING COLUMN - PHASE 2
    # ========================================================

    else:

        print("\nNO IMPROVING COLUMN IN PHASE 1")
        print("\nPHASE 2 : CONTINUING SOLVE WITH REMAINING TIME BUDGET")

        elapsed   = time.time() - total_start
        remaining = MAX_RUNTIME - elapsed

        if remaining <= 0:

            print("\nMAXIMUM RUNTIME REACHED")

            sub_end  = time.time()
            sub_time = sub_end - sub_start

            log.append({
                "iteration":    iteration,
                "time":         time.time() - total_start,
                "reduced_cost": rc if rc is not None else float('inf'),
                "patterns":     len(patterns),
                "master_obj":   master_obj,
                "master_time":  master_time,
                "pricing_time": sub_time,
            })

            break

        CHECKPOINT_INTERVAL = 5 * 60  # 5 minutes

        best_obj_so_far = rc if rc is not None else float('inf')

        # ====================================================
        # PHASE 2 CHECKPOINT CALLBACK
        # ====================================================

        class Phase2Callback(cp_model.CpSolverSolutionCallback):

            def __init__(self):
                cp_model.CpSolverSolutionCallback.__init__(self)
                self._best        = best_obj_so_far
                self._last_check  = time.time()
                self._terminate   = False
                self._send_master = False

            def on_solution_callback(self):

                now = time.time()

                # global time check
                if now - total_start >= MAX_RUNTIME:

                    print("\nMAXIMUM RUNTIME REACHED IN CALLBACK")

                    self._terminate = True

                    self.StopSearch()

                    return

                # recover current RC
                current_rc = 1.0 - self.ObjectiveValue() / SCALE

                # 5-min checkpoint
                if now - self._last_check >= CHECKPOINT_INTERVAL:

                    self._last_check = now

                    print(f"\n5-MIN CHECKPOINT | RC: {current_rc:.6f} | Best so far: {self._best:.6f}")

                    if current_rc < self._best - 1e-4:

                        self._best = current_rc

                        if current_rc < -1e-4:

                            print("\nIMPROVED AND NEGATIVE RC - SENDING TO MASTER")

                            self._send_master = True

                            self.StopSearch()

                        else:

                            print("\nIMPROVED BUT RC STILL POSITIVE - CONTINUING")

                    else:

                        print("\nNO IMPROVEMENT - CONTINUING TO NEXT 5-MIN CHECKPOINT")

        cb = Phase2Callback()

        # recalculate remaining AFTER Phase 1 consumed time
        remaining = max(1.0, MAX_RUNTIME - (time.time() - total_start))

        solver.parameters.max_time_in_seconds = remaining

        print(f"\nPHASE 2 TIME BUDGET : {remaining:.1f}s")

        status2 = solver.Solve(cp, cb)

        cp_status     = status2
        cp_solver_ref = solver

        # hard global time check after phase 2 returns
        if time.time() - total_start >= MAX_RUNTIME:

            print("\nMAXIMUM RUNTIME REACHED AFTER PHASE 2 - EXITING")

            sub_end  = time.time()
            sub_time = sub_end - sub_start

            rc_logged = 1.0 - solver.ObjectiveValue() / SCALE if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE') else float('inf')

            log.append({
                "iteration":    iteration,
                "time":         time.time() - total_start,
                "reduced_cost": rc_logged,
                "patterns":     len(patterns),
                "master_obj":   master_obj,
                "master_time":  master_time,
                "pricing_time": sub_time,
            })

            break

        if cb._terminate:

            print("\nMAXIMUM RUNTIME REACHED - EXITING")

            sub_end  = time.time()
            sub_time = sub_end - sub_start

            rc_logged = 1.0 - solver.ObjectiveValue() / SCALE if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE') else float('inf')

            log.append({
                "iteration":    iteration,
                "time":         time.time() - total_start,
                "reduced_cost": rc_logged,
                "patterns":     len(patterns),
                "master_obj":   master_obj,
                "master_time":  master_time,
                "pricing_time": sub_time,
            })

            break

        rc = 1.0 - solver.ObjectiveValue() / SCALE if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE') else float('inf')

        print(f"\nPHASE 2 STATUS        : {solver.StatusName(status2)}")
        print(f"PHASE 2 REDUCED COST  : {rc:.6f}")

    sub_end  = time.time()
    sub_time = sub_end - sub_start

    # ========================================================
    # SAVE SOLUTION FOR REFERENCE (no Gurobi warm start needed)
    # ========================================================

    prev_sol = {}

    if cp_solver_ref.StatusName(cp_status) in ('OPTIMAL', 'FEASIBLE'):

        for n in expanded_items:

            prev_sol[n] = {
                "z": cp_solver_ref.Value(s[n]),
                "x": cp_solver_ref.Value(xvar[n]),
                "y": cp_solver_ref.Value(yvar[n]),
            }

    # ========================================================
    # TERMINATION
    # ========================================================

    if rc >= -1e-4:

        # log final iteration before breaking
        log.append({
            "iteration":    iteration,
            "time":         time.time() - total_start,
            "reduced_cost": rc,
            "patterns":     len(patterns),
            "master_obj":   master_obj,
            "master_time":  master_time,
            "pricing_time": sub_time,
        })

        if cp_status == cp_model.OPTIMAL:

            print("\nPROVEN NO IMPROVING COLUMN - OPTIMAL")

            break

        elif time.time() - total_start >= MAX_RUNTIME:

            print("\nTIME LIMIT - STOPPING")

            break

        else:

            print("\nRC POSITIVE BUT NOT PROVEN OPTIMAL - CONTINUING")

    # ========================================================
    # EXTRACT PATTERN
    # ========================================================

    new_pattern = {item: 0 for item in items}

    rectangles = []

    for n in expanded_items:

        if cp_solver_ref.Value(s[n]) == 1:

            item = copy_to_item[n]

            new_pattern[item] += 1

            rectangles.append({
                "item": item,
                "x":    cp_solver_ref.Value(xvar[n]),
                "y":    cp_solver_ref.Value(yvar[n]),
                "w":    widths[item],
                "h":    heights[item],
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
    # LOG + LIVE SAVE
    # ========================================================

    log_entry = {
        "iteration":    iteration,
        "time":         time.time() - total_start,
        "reduced_cost": rc,
        "patterns":     len(patterns),
        "master_obj":   master_obj,
        "master_time":  master_time,
        "pricing_time": sub_time,
    }

    log.append(log_entry)

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

remaining_for_final = max(5 * 60, MAX_RUNTIME - (time.time() - total_start))

master.Params.TimeLimit = remaining_for_final

print(f"\nFinal integer solve budget : {remaining_for_final:.1f}s")

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

    # add dataset info as header rows
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

    # filter only numeric iteration rows (exclude CEILING/INTEGER summary rows)
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

plt.show(block=True)

# ============================================================
# RESTORE STDOUT AND CLOSE LOG FILE
# ============================================================

sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__
terminal_log_file.close()

print(f"\nTerminal log saved : {terminal_log_path}")