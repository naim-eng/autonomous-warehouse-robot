from controller import Robot
import heapq
import itertools

# =========================
# A* PATHFINDING
# =========================
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        neighbors = [
            (current[0]+1, current[1]),
            (current[0]-1, current[1]),
            (current[0], current[1]+1),
            (current[0], current[1]-1),
        ]

        for n in neighbors:
            r, c = n
            if 0 <= r < rows and 0 <= c < cols:
                if grid[r][c] == 1:
                    continue

                temp = g_score[current] + 1
                if n not in g_score or temp < g_score[n]:
                    g_score[n] = temp
                    f = temp + heuristic(n, goal)
                    heapq.heappush(open_list, (f, n))
                    came_from[n] = current

    return []

# =========================
# SMALL GRID (6x6)
# =========================
grid = [
    [0,0,0,0,0,0],
    [0,1,1,0,0,0],
    [0,0,0,0,1,0],
    [0,0,1,0,1,0],
    [0,0,0,0,0,0],
    [0,0,0,1,0,0]
]

start = (0,0)

stations = [
    (5,0),
    (2,5),
    (5,5)
]

# =========================
# OPTIMAL ROUTE
# =========================
best_path = []
best_cost = float('inf')

for order in itertools.permutations(stations):
    current_position = start
    total_cost = 0
    temp_path = []

    for target in order:
        path = astar(grid, current_position, target)
        if not path:
            break

        if temp_path:
            path = path[1:]

        temp_path.extend(path)
        total_cost += len(path)
        current_position = target

    if total_cost < best_cost:
        best_cost = total_cost
        best_path = temp_path

full_path = best_path

print("OPTIMAL PATH:", full_path)
print("TOTAL COST:", best_cost)

# =========================
# ROBOT SETUP (PIONEER)
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left = robot.getDevice('left wheel')
right = robot.getDevice('right wheel')

left.setPosition(float('inf'))
right.setPosition(float('inf'))

# 🔧 FIXED SPEED
MAX_SPEED = 2.0

# =========================
# SONAR SENSORS
# =========================
sonar = []
for i in range(8):
    s = robot.getDevice(f"so{i}")
    s.enable(timestep)
    sonar.append(s)

# =========================
# OBSTACLE DETECTION
# =========================
def obstacle_detected():
    front = sonar[3].getValue() + sonar[4].getValue()
    left_s = sonar[6].getValue()
    right_s = sonar[1].getValue()

    return front > 400 or left_s > 250 or right_s > 250

# =========================
# MOVEMENT (SCALED DOWN)
# =========================
def get_position():
    values = gps.getValues()
    return values[0], values[2]

def get_heading():
    comp = compass.getValues()
    return -math.atan2(comp[0], comp[2])

def go_to(target_cell):
    target_x, target_z = grid_to_world(target_cell)

    while robot.step(timestep) != -1:
        x, z = get_position()

        dx = target_x - x
        dz = target_z - z
        distance = math.sqrt(dx*dx + dz*dz)

        if distance < 0.05:
            break

        target_angle = math.atan2(dx, dz)
        heading = get_heading()
        error = target_angle - heading

        # normalize angle
        while error > math.pi:
            error -= 2*math.pi
        while error < -math.pi:
            error += 2*math.pi

        left_speed = MAX_SPEED * (1 - error)
        right_speed = MAX_SPEED * (1 + error)

        left.setVelocity(left_speed)
        right.setVelocity(right_speed)

# =========================
# FOLLOW PATH
# =========================
direction = (1, 0)

for i in range(len(full_path)-1):
    current = full_path[i]
    nxt = full_path[i+1]

    move = (nxt[0] - current[0], nxt[1] - current[1])

    if move != direction:
        if move == (-1,0):
            turn_back()
        elif move == (0,1):
            turn_right()
        elif move == (0,-1):
            turn_left()

        direction = move

    go_to(nxt)

# =========================
# STOP
# =========================
left.setVelocity(0)
right.setVelocity(0)

while robot.step(timestep) != -1:
    pass