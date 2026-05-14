import random

SLOTS = {
    "RED_1": (-0.15, -3.0),
    "RED_2": (-0.15, -3.0),

    "GREEN_1": (-0.15, -1.0),
    "GREEN_2": (-0.15, -1.0),

    "BLUE_1": (-0.15, 1.0),
    "BLUE_2": (-0.15, 1.0),

    "YELLOW_1": (-0.15, 3.0),
    "YELLOW_2": (-0.15, 3.0),
}

target = random.choice(list(SLOTS.keys()))

print("TARGET SLOT:")
print(target)

x, y = SLOTS[target]

print("GO TO:")
print(f"x={x}, y={y}")