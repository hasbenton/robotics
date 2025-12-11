from controller import Robot, Supervisor, Emitter, Receiver
from map_fusion import fuse, naieveFuse
import os

TIME_STEP = 32

robot = Robot()

names = ["FirstRobot"]
rec = robot.getDevice("receiver")
emit = robot.getDevice("emitter")
rec.enable(5)
count = 0
x = 0
maps = {}

while robot.step(TIME_STEP) != -1:
    count += 1
    if count > 100:
        emit.send("maps")
        count = 0

    if rec.getQueueLength() > 0:
        result = str(rec.getString())
        name, result = result.split("|")
        result = result[2:-2].split("], [")
        for i in range(len(result)):
            splitres = result[i].split(", ")
            result[i] = list(map(int, splitres))
        maps[name] = result
        rec.nextPacket()

    if len(maps) > 1:
        print("Fusing")
        bot1 = list(maps)[0]
        bot2 = list(maps)[1]
        fused = naieveFuse(maps.pop(bot1), maps.pop(bot2))
        emit.send(bot1 + "|" + bot2 + "|" + str(fused))

        x += 1

        file_path = os.path.join(os.path.dirname(__file__), f"output{x}.txt")
        with open(file_path, "w", encoding="utf-8") as f:
            for row in fused:
                f.write(" ".join(map(str, row)) + "\n")
        print(f"✅ Grid map saved: {os.path.abspath(file_path)}")
