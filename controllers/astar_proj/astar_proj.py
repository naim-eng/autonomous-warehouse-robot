from controller import Robot
import heapq
import itertools

# ---------- A* ----------
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

# ---------- GRID ----------
grid = [
    [0,0,0,0,0],
    [0,1,1,0,0],
    [0,0,0,0,0],
    [0,0,1,0,0],
    [0,0,0,0,0]
]

start = (3,3)

stations = [
    (4,0),
    (2,4),
    (4,4)
]

# ---------- FIND OPTIMAL ORDER ----------
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

# ---------- ROBOT ----------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left = robot.getDevice('left wheel motor')
right = robot.getDevice('right wheel motor')

left.setPosition(float('inf'))
right.setPosition(float('inf'))

MAX_SPEED = 1.5

# ---------- MOVEMENT ----------
def move_forward(duration=300):
    left.setVelocity(MAX_SPEED)
    right.setVelocity(MAX_SPEED)
    for _ in range(duration):
        robot.step(timestep)

def turn_left(duration=200):
    left.setVelocity(-MAX_SPEED)
    right.setVelocity(MAX_SPEED)
    for _ in range(duration):
        robot.step(timestep)

def turn_right(duration=200):
    left.setVelocity(MAX_SPEED)
    right.setVelocity(-MAX_SPEED)
    for _ in range(duration):
        robot.step(timestep)

# ---------- FOLLOW PATH ----------
direction = (1, 0)

for i in range(len(full_path)-1):
    current = full_path[i]
    nxt = full_path[i+1]

    move = (nxt[0] - current[0], nxt[1] - current[1])

    if move != direction:
        if move == (0,1):
            turn_right()
        elif move == (0,-1):
            turn_left()
        elif move == (-1,0):
            turn_left()
            turn_left()

        direction = move

    move_forward()

# ---------- STOP ----------
left.setVelocity(0)
right.setVelocity(0)

while robot.step(timestep) != -1:
    pass