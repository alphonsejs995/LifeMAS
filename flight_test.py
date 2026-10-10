import airsim
import time
import traceback

client = airsim.MultirotorClient(
    ip="127.0.0.1", port=41451, timeout_value=10
)
drone = "Drone1"

try:
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=drone)

    print("Arming drone...")
    if not client.armDisarm(True, vehicle_name=drone):
        raise RuntimeError("Arming failed.")

    print("Taking off...")
    client.takeoffAsync(
        timeout_sec=10, vehicle_name=drone
    ).join()

    print("Moving forward for 2 seconds...")
    client.moveByVelocityAsync(
        2, 0, 0, 2, vehicle_name=drone
    ).join()

    print("Landing...")
    client.landAsync(
        timeout_sec=15, vehicle_name=drone
    ).join()

    client.armDisarm(False, vehicle_name=drone)
    print("SUCCESS: Arm, takeoff, movement, and landing completed.")

except Exception as e:
    print("FLIGHT TEST FAILED:", repr(e))
    traceback.print_exc()
    print("If airborne, use the simulator's normal landing/emergency controls.")
