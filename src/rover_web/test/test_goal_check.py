from rover_web.goal_check import goal_problem

RES = 0.05


def room():
    """Build a 4 x 3 m map: walls round the edge, a pillar, an unmapped corner."""
    w, h = 80, 60
    grid = [[0] * w for _ in range(h)]
    for i in range(w):
        grid[0][i] = grid[h - 1][i] = 100
    for j in range(h):
        grid[j][0] = grid[j][w - 1] = 100
    for j in range(28, 32):
        for i in range(38, 42):
            grid[j][i] = 100          # pillar around (2.0, 1.5)
    for j in range(45, 59):
        for i in range(65, 79):
            grid[j][i] = -1           # unmapped corner
    return w, h, [v for row in grid for v in row]


def check(x, y, clearance=0.27):
    w, h, data = room()
    return goal_problem(RES, w, h, 0.0, 0.0, data, x, y, clearance)


def test_open_floor_is_fine():
    assert check(1.0, 1.0) is None
    assert check(3.0, 0.8) is None


def test_off_the_map():
    assert 'kənar' in check(-0.5, 1.0)
    assert 'kənar' in check(1.0, 3.5)


def test_on_a_wall_or_obstacle():
    assert 'üstündədir' in check(2.0, 1.5)
    assert 'üstündədir' in check(0.01, 1.0)


def test_unmapped():
    assert 'naməlum' in check(3.6, 2.6)


def test_too_close_to_a_wall():
    msg = check(0.2, 1.0)            # 15 cm from the wall's inner face
    assert msg is not None and 'yaxındır' in msg
    assert check(0.4, 1.0) is None   # 35 cm: enough for a 27 cm half-width
    assert 'yaxındır' in check(2.0, 1.25)   # next to the pillar
