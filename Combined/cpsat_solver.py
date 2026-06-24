import gurobipy as gp
from gurobipy import GRB
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import csv
import os
import sys
import math
import threading
from datetime import datetime
from ortools.sat.python import cp_model

# ============================================================
# ENTRY POINT  (called from main.py)
# ============================================================

def run(items_data, W, H, MAX_RUNTIME=6*60*60, instance_id=0):

    # ============================================================
    # OUTPUT FOLDERS
    # ============================================================

    script_dir  = os.path.dirname(os.path.abspath(__file__))
    run_stamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
    inst_tag    = f"instance_{instance_id}"

    base_dir     = os.path.join(script_dir, "results", "cpsat", inst_tag)
    logs_dir     = os.path.join(base_dir, "logs")
    reports_dir  = os.path.join(base_dir, "reports")
    plots_dir    = os.path.join(base_dir, "plots")
    patterns_dir = os.path.join(base_dir, "patterns")

    for d in [logs_dir, reports_dir, plots_dir, patterns_dir]:
        os.makedirs(d, exist_ok=True)

    # ============================================================
    # TERMINAL LOG - TEE STDOUT + STDERR TO FILE
    # ============================================================

    class Tee:
        def __init__(self, *streams):
            self.streams = streams
        def write(self, data):
            for s in self.streams:
                try: s.write(data); s.flush()
                except Exception: pass
        def flush(self):
            for s in self.streams:
                try: s.flush()
                except Exception: pass

    terminal_log_path = os.path.join(logs_dir, f"cpsat_instance_{instance_id}_{run_stamp}.log")
    terminal_log_file = open(terminal_log_path, "w", buffering=1)

    _orig_stdout = sys.stdout
    _orig_stderr = sys.stderr
    sys.stdout = Tee(sys.__stdout__, terminal_log_file)
    sys.stderr = Tee(sys.__stderr__, terminal_log_file)

    print(f"Solver        : CP-SAT")
    print(f"Logs dir      : {logs_dir}")
    print(f"Reports dir   : {reports_dir}")
    print(f"Plots dir     : {plots_dir}")
    print(f"Patterns dir  : {patterns_dir}")

    # ============================================================
    # PRINT DATASET
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

    # ============================================================
    # WATCHDOG THREAD
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
    widths  = {}
    heights = {}

    for item in items:
        demands[item] = items_data[item]["demand"]
        widths[item]  = items_data[item]["w"]
        heights[item] = items_data[item]["h"]

    # ============================================================
    # INITIAL PATTERNS
    # ============================================================

    patterns = []
    a        = {}

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
        x[p] = master.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"x_{p}")

    master.update()

    demand_constraints = {}
    for item in items:
        demand_constraints[item] = master.addConstr(
            gp.quicksum(a[(item,p)] * x[p] for p in patterns) >= demands[item]
        )

    master.setObjective(gp.quicksum(x[p] for p in patterns), GRB.MINIMIZE)

    # ============================================================
    # STORAGE
    # ============================================================

    pattern_visualizations = {}
    prev_sol               = None
    log                    = []
    iteration              = 1

    # ============================================================
    # COLUMN GENERATION
    # ============================================================

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
        master_time  = time.time() - master_start
        master_obj   = master.ObjVal

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
        copy_to_item   = {}
        counter        = 1

        for item in items:
            demand      = demands[item]
            w           = widths[item]
            h           = heights[item]
            max_by_area = (W * H) // (w * h)
            max_copies  = min(demand, max_by_area)
            for _ in range(max_copies):
                expanded_items.append(counter)
                copy_to_item[counter] = item
                counter += 1

        print("\nNUMBER OF EXPANDED ITEMS")
        print(len(expanded_items))

        # ========================================================
        # CP-SAT SUBPROBLEM
        # ========================================================

        # adaptive scaling based on dual magnitude
        max_dual = max(abs(v) for v in duals.values()) if duals else 1.0
        if max_dual > 0:
            SCALE = max(min(int(1e6 / max_dual), 10**6), 1000)
        else:
            SCALE = 10**6

        print(f"\nSCALE : {SCALE} (max dual : {max_dual:.6f})")

        cp = cp_model.CpModel()

        # ========================================================
        # CP-SAT VARIABLES
        # ========================================================

        s    = {}
        xvar = {}
        yvar = {}
        x_iv = {}
        y_iv = {}

        for n in expanded_items:
            item = copy_to_item[n]
            w    = widths[item]
            h    = heights[item]

            s[n]    = cp.NewBoolVar(f"s_{n}")
            xvar[n] = cp.NewIntVar(0, W - w, f"x_{n}")
            yvar[n] = cp.NewIntVar(0, H - h, f"y_{n}")

            x_iv[n] = cp.NewOptionalFixedSizeIntervalVar(xvar[n], w, s[n], f"xi_{n}")
            y_iv[n] = cp.NewOptionalFixedSizeIntervalVar(yvar[n], h, s[n], f"yi_{n}")

        # ========================================================
        # NO OVERLAP
        # ========================================================

        cp.AddNoOverlap2D(list(x_iv.values()), list(y_iv.values()))

        # ========================================================
        # SYMMETRY BREAKING
        # ========================================================

        for item in items:
            same_type = [n for n in expanded_items if copy_to_item[n] == item]
            for k in range(len(same_type) - 1):
                i = same_type[k]
                j = same_type[k + 1]
                cp.Add(s[i] >= s[j])
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
        solver.parameters.num_search_workers  = 4
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
                sub_time = time.time() - sub_start
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

            CHECKPOINT_INTERVAL = 5 * 60

            best_obj_so_far = rc if rc is not None else float('inf')

            class Phase2Callback(cp_model.CpSolverSolutionCallback):

                def __init__(self):
                    cp_model.CpSolverSolutionCallback.__init__(self)
                    self._best        = best_obj_so_far
                    self._last_check  = time.time()
                    self._terminate   = False
                    self._send_master = False

                def on_solution_callback(self):
                    now = time.time()
                    if now - total_start >= MAX_RUNTIME:
                        print("\nMAXIMUM RUNTIME REACHED IN CALLBACK")
                        self._terminate = True
                        self.StopSearch()
                        return
                    current_rc = 1.0 - self.ObjectiveValue() / SCALE
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

            remaining = max(1.0, MAX_RUNTIME - (time.time() - total_start))
            solver.parameters.max_time_in_seconds = remaining

            print(f"\nPHASE 2 TIME BUDGET : {remaining:.1f}s")

            status2       = solver.Solve(cp, cb)
            cp_status     = status2
            cp_solver_ref = solver

            if time.time() - total_start >= MAX_RUNTIME:
                print("\nMAXIMUM RUNTIME REACHED AFTER PHASE 2 - EXITING")
                sub_time  = time.time() - sub_start
                rc_logged = (1.0 - solver.ObjectiveValue() / SCALE
                             if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE')
                             else float('inf'))
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
                sub_time  = time.time() - sub_start
                rc_logged = (1.0 - solver.ObjectiveValue() / SCALE
                             if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE')
                             else float('inf'))
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

            rc = (1.0 - solver.ObjectiveValue() / SCALE
                  if solver.StatusName(status2) in ('OPTIMAL', 'FEASIBLE')
                  else float('inf'))

            print(f"\nPHASE 2 STATUS        : {solver.StatusName(status2)}")
            print(f"PHASE 2 REDUCED COST  : {rc:.6f}")

        sub_time = time.time() - sub_start

        # ========================================================
        # SAVE SOLUTION FOR REFERENCE
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
        rectangles  = []

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
            column.addTerms(coeff, demand_constraints[item])

        x[new_p] = master.addVar(obj=1, lb=0, vtype=GRB.CONTINUOUS,
                                  column=column, name=f"x_{new_p}")
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
    # OPTIMALITY CHECK
    # ============================================================

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
        rounded_total       += rounded
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
    # CSV EXPORT  →  results/cpsat/reports/
    # ============================================================

    if log:

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

        csv_path = os.path.join(reports_dir, f"cpsat_instance_{instance_id}_{run_stamp}.csv")

        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(log)

        print(f"\nCSV saved : {csv_path}")

    else:
        print("\nNO ITERATIONS COMPLETED - CSV NOT SAVED")

    # ============================================================
    # ANALYTICS PLOTS  →  results/cpsat/plots/
    # ============================================================

    numeric_log = [r for r in log if isinstance(r["iteration"], int)]

    if len(numeric_log) >= 2:

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
        fig.suptitle("CP-SAT Column Generation Analytics", fontsize=16)

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
        analytics_path = os.path.join(plots_dir, f"cpsat_instance_{instance_id}_analytics_{run_stamp}.png")
        fig.savefig(analytics_path, dpi=150)
        plt.close(fig)

        print(f"\nAnalytics plot saved : {analytics_path}")

    # ============================================================
    # PATTERN VISUALIZATIONS  →  results/cpsat/patterns/
    # ============================================================

    for p in all_display_patterns:

        rectangles = pattern_visualizations.get(p)

        if not rectangles:
            rectangles = []
            x_cursor = 0
            y_cursor = 0
            row_h    = 0
            for item in items:
                count = a.get((item, p), 0)
                w     = widths[item]
                h     = heights[item]
                for _ in range(int(count)):
                    if x_cursor + w > W:
                        x_cursor  = 0
                        y_cursor += row_h
                        row_h     = 0
                    rectangles.append({"item": item, "x": x_cursor, "y": y_cursor, "w": w, "h": h})
                    x_cursor += w
                    row_h     = max(row_h, h)

        fig, ax = plt.subplots(figsize=(8, 8))

        ax.add_patch(patches.Rectangle((0, 0), W, H,
                                       linewidth=3, edgecolor='black', facecolor='lightgray'))

        for rect_data in rectangles:
            ax.add_patch(patches.Rectangle(
                (rect_data["x"], rect_data["y"]),
                rect_data["w"], rect_data["h"],
                linewidth=2, edgecolor='darkgreen', facecolor='lightgreen'
            ))
            ax.text(
                rect_data["x"] + rect_data["w"] / 2,
                rect_data["y"] + rect_data["h"] / 2,
                rect_data["item"], ha='center', va='center'
            )

        ax.set_xlim(0, W)
        ax.set_ylim(0, H)
        ax.set_aspect('equal')
        ax.set_title(f"CP-SAT — Pattern {p}")
        plt.grid(True)

        pattern_path = os.path.join(patterns_dir, f"cpsat_instance_{instance_id}_pattern_{p}_{run_stamp}.png")
        fig.savefig(pattern_path, dpi=150)
        plt.close(fig)

        print(f"Pattern {p} saved : {pattern_path}")

    # ============================================================
    # PDF REPORT  →  results/cpsat/reports/
    # ============================================================

    _write_pdf_report(
        pdf_path      = os.path.join(reports_dir, f"cpsat_instance_{instance_id}_{run_stamp}.pdf"),
        solver_name   = "CP-SAT",
        items_data    = items_data,
        W             = W,
        H             = H,
        log           = log,
        lp_bound      = lp_bound,
        lp_ceil       = lp_ceil,
        integer_obj   = integer_obj,
        rounded_total = rounded_total,
        lp_solution   = lp_solution,
        patterns      = patterns,
        a             = a,
        run_stamp     = run_stamp,
        plots_dir     = plots_dir,
        patterns_dir  = patterns_dir,
        instance_id   = instance_id,
    )

    # ============================================================
    # RESTORE STDOUT
    # ============================================================

    sys.stdout = _orig_stdout
    sys.stderr = _orig_stderr
    terminal_log_file.close()

    print(f"\nCP-SAT run complete. All outputs in: {base_dir}")


# ============================================================
# PDF WRITER (self-contained, uses matplotlib)
# ============================================================

def _write_pdf_report(pdf_path, solver_name, items_data, W, H, log,
                      lp_bound, lp_ceil, integer_obj, rounded_total,
                      lp_solution, patterns, a, run_stamp,
                      plots_dir, patterns_dir, instance_id=0):

    try:
        from matplotlib.backends.backend_pdf import PdfPages
        import matplotlib.pyplot as plt
        from datetime import datetime
    except ImportError:
        print("[PDF] matplotlib not available - skipping PDF")
        return

    items    = list(items_data.keys())
    now_str  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    numeric_log = [r for r in log if isinstance(r.get("iteration"), int)]

    def _text_page(pdf, title, lines, fontsize=9):
        fig = plt.figure(figsize=(8.5, 11))
        ax  = fig.add_axes([0.05, 0.05, 0.90, 0.90])
        ax.axis("off")
        body = f"{title}\n{'='*80}\n" + "\n".join(str(l) for l in lines)
        ax.text(0, 1, body, transform=ax.transAxes, va="top", ha="left",
                fontsize=fontsize, fontfamily="monospace")
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)

    def _img_page(pdf, img_path, title=""):
        if not os.path.isfile(img_path):
            return
        try:
            img = plt.imread(img_path)
        except Exception:
            return
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.imshow(img); ax.axis("off")
        if title: ax.set_title(title, fontsize=11)
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)

    with PdfPages(pdf_path) as pdf:

        # cover
        fig = plt.figure(figsize=(8.5, 11))
        ax  = fig.add_axes([0.1, 0.3, 0.8, 0.5])
        ax.axis("off")
        ax.text(0.5, 0.85, "2D Cutting Stock Problem", ha="center", fontsize=18, fontweight="bold")
        ax.text(0.5, 0.72, f"Solver: {solver_name}", ha="center", fontsize=14)
        ax.text(0.5, 0.58, now_str, ha="center", fontsize=10, color="gray")
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)

        # input data
        lines = [f"Sheet size: W={W}, H={H}", "", "items_data = {"]
        for k, v in items_data.items():
            lines.append(f"    '{k}': demand={v['demand']}, w={v['w']}, h={v['h']}")
        lines.append("}")
        _text_page(pdf, "INPUT DATA", lines, fontsize=10)

        # solution summary
        s_lines = []
        if lp_bound is not None:
            s_lines.append(f"LP Relaxation Bound          : {lp_bound:.4f}")
            s_lines.append(f"Ceiling of LP Bound          : {lp_ceil} sheets")
        s_lines.append(f"Pattern-Level Rounding Total : {rounded_total} sheets")
        if integer_obj is not None:
            s_lines.append(f"Integer Solve Solution       : {int(integer_obj)} sheets")
            if integer_obj <= lp_ceil:
                s_lines.append("GLOBALLY OPTIMAL : IRUP satisfied")
            elif integer_obj <= lp_ceil + 1:
                s_lines.append("NEAR-OPTIMAL : MIRUP (at most 1 from optimal)")
            else:
                gap = (integer_obj - lp_bound) / lp_bound * 100
                s_lines.append(f"OPTIMALITY GAP : {gap:.2f}%")
        _text_page(pdf, "SOLUTION SUMMARY", s_lines, fontsize=10)

        # iteration log
        if numeric_log:
            hdr = f"{'Iter':>4}  {'Time(s)':>9}  {'RC':>12}  {'Pats':>5}  {'MasterObj':>10}  {'MasterT':>8}  {'PricingT':>9}"
            it_lines = [hdr, "-" * len(hdr)]
            for r in numeric_log:
                rc_s = f"{r['reduced_cost']:.6f}" if isinstance(r['reduced_cost'], float) else str(r['reduced_cost'])
                it_lines.append(
                    f"{r['iteration']:>4}  {r['time']:>9.2f}  {rc_s:>12}  {r['patterns']:>5}  "
                    f"{r['master_obj']:>10.4f}  {r['master_time']:>8.4f}  {r['pricing_time']:>9.4f}"
                )
            _text_page(pdf, "ITERATION LOG", it_lines, fontsize=8)

        # pattern counts
        pc_lines = []
        for p in patterns:
            lp_val = lp_solution.get(p, 0)
            if lp_val < 1e-6: continue
            pc_lines.append(f"Pattern {p}  (LP usage = {lp_val:.4f}, rounded = {math.ceil(lp_val)})")
            area = 0
            for item in items:
                cnt = a.get((item, p), 0)
                if cnt > 0:
                    area += cnt * items_data[item]['w'] * items_data[item]['h']
                    pc_lines.append(f"  {item}: {cnt}")
            util = area / (W * H) * 100 if W * H else 0
            pc_lines.append(f"  Utilization: {util:.1f}%")
            pc_lines.append("")
        _text_page(pdf, "PATTERN COUNTS", pc_lines)

        # analytics plot
        analytics_path = os.path.join(plots_dir, f"cpsat_instance_{instance_id}_analytics_{run_stamp}.png")
        _img_page(pdf, analytics_path, "Analytics")

        # pattern images
        for p in patterns:
            pat_img = os.path.join(patterns_dir, f"cpsat_instance_{instance_id}_pattern_{p}_{run_stamp}.png")
            _img_page(pdf, pat_img, f"Pattern {p}")

    print(f"\nPDF report saved : {pdf_path}")
