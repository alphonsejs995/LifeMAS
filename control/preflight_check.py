import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)
client.confirmConnection()

state = client.getMultirotorState()
print("State ready:", state.ready)
print("Landed state:", state.landed_state)
print("Position:", state.kinematics_estimated.position)
print("Velocity:", state.kinematics_estimated.linear_velocity)
print("Collision:", client.simGetCollisionInfo().has_collided)
