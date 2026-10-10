import time
import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)

client.confirmConnection()
client.enableApiControl(True)

print("API control:", client.isApiControlEnabled())

try:
    print("Initial position:")
    print(client.getMultirotorState().kinematics_estimated.position)

    print("\nSending a brief zero-velocity command...")
    client.moveByVelocityAsync(
        0, 0, 0,
        duration=1,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(False, 0)
    ).join()

    time.sleep(0.5)

    print("\nPosition after command:")
    print(client.getMultirotorState().kinematics_estimated.position)

    print("Velocity:")
    print(client.getMultirotorState().kinematics_estimated.linear_velocity)

finally:
    client.hoverAsync().join()
    print("\nHover command completed.")
