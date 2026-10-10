import airsim
import traceback

try:
    print("1. Connecting to AirSimNH at 127.0.0.1:41451...")
    client = airsim.MultirotorClient(
        ip="127.0.0.1",
        port=41451,
        timeout_value=5
    )

    client.confirmConnection()
    print("SUCCESS: AirSim API responded.")

    print("\n2. Checking configured drones...")
    vehicles = client.listVehicles()
    print("Vehicles:", vehicles)

    drone = "Drone1" if "Drone1" in vehicles else ""
    print("Testing drone:", drone or "(default vehicle)")

    print("\n3. Reading drone state...")
    state = client.getMultirotorState(vehicle_name=drone)
    p = state.kinematics_estimated.position
    print(f"Position: x={p.x_val:.2f}, y={p.y_val:.2f}, z={p.z_val:.2f}")
    print("Landed state:", state.landed_state)

    print("\n4. Testing API-control ownership...")
    client.enableApiControl(True, vehicle_name=drone)
    print("API control enabled request sent.")
    print("API control active:", client.isApiControlEnabled(vehicle_name=drone))

    print("\nPASS: AirSim is responding to Python.")

except Exception as e:
    print("\nFAIL: AirSim API test failed.")
    print(type(e).__name__ + ":", e)
    traceback.print_exc()
