#only suitable for socket package
from pymorse import Morse

print("Use WASD to control the Ranger")

with Morse() as simu:

  # motion = simu.robot.motion
  motion = simu.robot.motion

  v = 0.0
  w = 0.0

  while True:
      key = input("WASD?")

      if key.lower() == "w":
          v += 0.1
      elif key.lower() == "s":
          v -= 0.1
      elif key.lower() == "a":
          w += 0.1
      elif key.lower() == "d":
          w -= 0.1
      else:
          continue

      motion.publish({"v": v, "w": w})