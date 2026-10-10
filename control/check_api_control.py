import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)

client.confirmConnection()

print("\nRequesting API control...")
client.enableApiControl(True, vehicle_name="")

print("API control enabled:", client.isApiControlEnabled(vehicle_name=""))

state = client.getMultirotorState()
print("Position:", state.kinematics_estimated.position)
print("Velocity:", state.kinematics_estimated.linear_velocity)
print("Landed state:", state.landed_state)
print("State ready:", state.ready)
