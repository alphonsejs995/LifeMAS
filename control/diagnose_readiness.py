import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)
client.confirmConnection()

state = client.getMultirotorState()
collision = client.simGetCollisionInfo()

print("\n--- VEHICLE READINESS ---")
print("Ready:", state.ready)
print("Ready message:", repr(state.ready_message))
print("Can arm:", state.can_arm)
print("Landed state:", state.landed_state)
print("API control enabled:", client.isApiControlEnabled())

print("\n--- COLLISION DETAILS ---")
print("Has collided:", collision.has_collided)
print("Collision timestamp:", collision.time_stamp)
print("Object name:", repr(collision.object_name))
print("Object ID:", collision.object_id)
print("Impact point:", collision.impact_point)
print("Penetration depth:", collision.penetration_depth)

print("\n--- VEHICLE POSITION ---")
print("Position:", state.kinematics_estimated.position)
