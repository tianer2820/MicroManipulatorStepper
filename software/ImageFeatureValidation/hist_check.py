import csv
import re
from collections import defaultdict
import numpy as np

SKIP_FIRST_MOVE = True
SKIP_COMBINED_MOVE = True

def parse_position(pos_str):
    """Parse '(x,y)' string into floats."""
    nums = re.findall(r"[-+]?\d*\.\d+|\d+", pos_str)
    return float(nums[0]), float(nums[1])

def analyze_csv(path):
    x_data = defaultdict(list)
    y_data = defaultdict(list)
    combined_data  = defaultdict(list)


    # Track per-segment axis usage
    x_seen = False
    y_seen = False

    with open(path, newline='') as f:
        reader = csv.DictReader(f, delimiter=',')

        for row in reader:
            from_pos = parse_position(row["From Position"])
            to_pos = parse_position(row["To Position"])

            # Detect new segment (starting from origin)
            dx = to_pos[0] - from_pos[0]
            dy = to_pos[1] - from_pos[1]


            # Only pure axis moves
            is_x_move = abs(dx) > 0 and abs(dy) == 0
            is_y_move = abs(dy) > 0 and abs(dx) == 0

            if SKIP_COMBINED_MOVE and (is_x_move and is_y_move):
                x_seen = False
                y_seen = False
                continue

            if SKIP_FIRST_MOVE and is_x_move and not x_seen:
                x_seen = True
                continue
            if SKIP_FIRST_MOVE and is_y_move and not y_seen:
                y_seen = True
                continue

            axis = "x" if is_x_move else "y"

            expected = round(float(row["Expected (um)"]))
            actual = round(float(row["Actual (um)"]))

            if is_x_move and not is_y_move:
                x_data[expected].append(actual)
            elif is_y_move and not is_x_move:
                y_data[expected].append(actual)
            else:
                combined_data[expected].append(actual)

    # Compute important info
    x_stats = {step: (np.mean(vals), np.std(vals), len(vals)) for step, vals in x_data.items() if vals}
    y_stats = {step: (np.mean(vals), np.std(vals), len(vals)) for step, vals in y_data.items() if vals}

    for step, vals in x_data.items():
        combined_data[step] = vals
    for step, vals in y_data.items():
        if step in combined_data:
            combined_data[step].extend(vals)
        else:
            combined_data[step] = vals
        
    combined_stats = {step: (np.mean(vals), np.std(vals), len(vals)) for step, vals in combined_data.items() if vals}


    return x_stats, y_stats, combined_stats


if __name__ == "__main__":
    path = "move_data.csv"
    x_results, y_results, combined_results = analyze_csv(path)
    
    print("X Axis:")
    for step, (avg, std, count) in sorted(x_results.items()):
        print(f"  Step size {step:9.3f} µm -> Avg actual: {avg:11.6f} µm (Std: {std:11.6f} µm) [Count: {count}]")

    print("\nY Axis:")
    for step, (avg, std, count) in sorted(y_results.items()):
        print(f"  Step size {step:9.3f} µm -> Avg actual: {avg:11.6f} µm (Std: {std:11.6f} µm) [Count: {count}]")

    print("\nCombined:")
    for step, (avg, std, count) in sorted(combined_results.items()):
        print(f"  Step size {step:9.3f} µm -> Avg actual: {avg:11.6f} µm (Std: {std:11.6f} µm) [Count: {count}]")