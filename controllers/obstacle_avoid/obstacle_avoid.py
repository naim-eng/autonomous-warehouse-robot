from controller import Robot
import itertools
import math

# --- Configuration & Initialization ---
MAX_SPEED = 6.28
TIME_STEP = 64

# ✅ CHANGED (was 1.8)
MOVE_DURATION = 1.4   

# ✅ CHANGED (was 0.61)
TURN_90_DURATION = 0.5  

# keep same
TURN_180_DURATION = 1.22  

robot = Robot()
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Environment Setup ---
grid = [[0]*10 for _ in range(10)]

obstacles = [(4,3), (4,4), (4,5), (4,6), (2,6), (3,6)]
for r, c in obstacles:
    grid[r][c] = 1

start_pos = (0, 0)
stations = [(2, 2), (8, 2), (5, 8)]

# --- A* ---
def a_star(start, goal, grid):
    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: abs(start[0]-goal[0]) + abs(start[1]-goal[1])}

    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        open_set.remove(current)

        for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
            neighbor = (current[0]+dr, current[1]+dc)

            if 0 <= neighbor[0] < 10 and 0 <= neighbor[1] < 10 and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + abs(neighbor[0]-goal[0]) + abs(neighbor[1]-goal[1])
                    open_set.add(neighbor)

    return None

def get_total_path(order, grid):
    full_path = []
    total_cost = 0
    curr = order[0]

    for target in order[1:]:
        segment = a_star(curr, target, grid)
        if segment:
            full_path.extend(segment)
            total_cost += len(segment)
            curr = target

    return total_cost, full_path

# --- Optimal Routing ---
best_cost = float('inf')
best_order = []

for p in itertools.permutations(stations):
    cost, _ = get_total_path([start_pos] + list(p), grid)
    if cost < best_cost:
        best_cost = cost
        best_order = [start_pos] + list(p)

# --- Greedy Routing ---
greedy_order = [start_pos]
remaining = list(stations)
curr = start_pos
greedy_total_cost = 0

while remaining:
    nxt = min(remaining, key=lambda x: len(a_star(curr, x, grid) or [99]))
    greedy_total_cost += len(a_star(curr, nxt, grid))
    greedy_order.append(nxt)
    remaining.remove(nxt)
    curr = nxt

# --- Results ---
_, final_path = get_total_path(best_order, grid)

print(f"Optimal Path Cost: {best_cost} | Order: {best_order}")
print(f"Greedy Path Cost: {greedy_total_cost} | Order: {greedy_order}")
print(f"Full Path: {final_path}")

# --- Movement ---
current_r, current_c = start_pos
current_dir = (1, 0)

def move_robot(duration, left_vel, right_vel):
    start_time = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time >= duration:
            break
        left_motor.setVelocity(left_vel)
        right_motor.setVelocity(right_vel)

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# --- Execute ---
for step_pos in final_path:
    target_r, target_c = step_pos
    dr, dc = target_r - current_r, target_c - current_c
    target_dir = (dr, dc)

    print(f"MOVE to {step_pos}")

    if target_dir != current_dir:
        cp = current_dir[0]*target_dir[1] - current_dir[1]*target_dir[0]
        dot = current_dir[0]*target_dir[0] + current_dir[1]*target_dir[1]

        if dot == -1:
            move_robot(TURN_180_DURATION, MAX_SPEED, -MAX_SPEED)
        elif cp == 1:
            move_robot(TURN_90_DURATION, MAX_SPEED, -MAX_SPEED)
        elif cp == -1:
            move_robot(TURN_90_DURATION, -MAX_SPEED, MAX_SPEED)

    move_robot(MOVE_DURATION, MAX_SPEED, MAX_SPEED)

    current_dir = target_dir
    current_r, current_c = target_r, target_c