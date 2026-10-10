import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)

client.confirmConnection()

print("API control enabled:", client.isApiControlEnabled())
print("Armed:", client.getMultirotorState().ready)
print("Position:", client.getMultirotorState().kinematics_estimated.position)
print("Landed state:", client.getMultirotorState().landed_state)
