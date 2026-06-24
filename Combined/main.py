# ============================================================
# main.py
# Single entry point for the 2D Cutting Stock solver.
# Run:  python main.py
# ============================================================

from input_data import INSTANCES


# ============================================================
# PROMPTS
# ============================================================

def _ask_solver():
    print("\n" + "=" * 50)
    print("  2D CUTTING STOCK — SOLVER SELECTION")
    print("=" * 50)
    print("  1. GUROBI")
    print("  2. CPSAT")
    print("=" * 50)
    while True:
        choice = input("\nSelect solver (1 or 2): ").strip()
        if choice == "1":
            return "GUROBI"
        elif choice == "2":
            return "CPSAT"
        else:
            print("  Invalid — enter 1 or 2.")


def _ask_instance():
    print("\n" + "=" * 50)
    print("  AVAILABLE INSTANCES")
    print("=" * 50)
    for k, v in INSTANCES.items():
        print(f"  {k:>2}. {v['description']}  (W={v['W']}, H={v['H']})")
    print(f"   0. RUN ALL INSTANCES")
    print("=" * 50)
    valid = set(INSTANCES.keys())
    while True:
        raw = input(f"\nSelect instance (0 = all, 1–{max(valid)}): ").strip()
        if raw.isdigit():
            val = int(raw)
            if val == 0:
                return 0        # run all
            elif val in valid:
                return val
        print(f"  Invalid — enter 0 for all, or 1–{max(valid)}.")


def _ask_time_limit():
    print("\n" + "=" * 50)
    print("  TIME LIMIT PER INSTANCE")
    print("=" * 50)
    print("  Enter max runtime for EACH instance individually.")
    print("  Examples: 300 = 5 min | 3600 = 1 hr | 21600 = 6 hr")
    print("=" * 50)
    while True:
        raw = input("\nTime limit per instance (seconds): ").strip()
        if raw.isdigit() and int(raw) > 0:
            return int(raw)
        else:
            print("  Invalid — enter a positive integer number of seconds.")


# ============================================================
# RUNNER
# ============================================================

def _run_one(solver, instance_id, time_limit):
    tc         = INSTANCES[instance_id]
    items_data = tc["items_data"]
    W          = tc["W"]
    H          = tc["H"]

    print("\n")
    print("#" * 60)
    print(f"#  Instance {instance_id} — {tc['description']}")
    print(f"#  Sheet: W={W}, H={H}  |  Time limit: {time_limit}s")
    print("#" * 60)

    if solver == "GUROBI":
        import gurobi_solver
        gurobi_solver.run(items_data, W, H, time_limit, instance_id)

    elif solver == "CPSAT":
        import cpsat_solver
        cpsat_solver.run(items_data, W, H, time_limit, instance_id)


def main():

    SOLVER     = _ask_solver()
    INSTANCE   = _ask_instance()
    TIME_LIMIT = _ask_time_limit()

    print(f"\nSolver     : {SOLVER}")
    print(f"Instance   : {'ALL' if INSTANCE == 0 else INSTANCE}")
    print(f"Time limit : {TIME_LIMIT}s per instance")

    if INSTANCE == 0:
        # run every instance, each gets its own independent time limit
        ids = sorted(INSTANCES.keys())
        print(f"\nRunning all {len(ids)} instances with {TIME_LIMIT}s each...\n")
        for i, instance_id in enumerate(ids, 1):
            print(f"\n[{i}/{len(ids)}] Starting instance {instance_id}...")
            try:
                _run_one(SOLVER, instance_id, TIME_LIMIT)
            except Exception as e:
                print(f"\n[ERROR] Instance {instance_id} failed: {e}")
                print("  Continuing to next instance...\n")
        print("\n" + "=" * 60)
        print("  ALL INSTANCES COMPLETE")
        print("=" * 60)
    else:
        _run_one(SOLVER, INSTANCE, TIME_LIMIT)


if __name__ == "__main__":
    main()
