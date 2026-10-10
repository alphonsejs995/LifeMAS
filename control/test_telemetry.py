import time
import airsim

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=41451,
    timeout_value=5
)

print("Connecting to AirSimNH...", flush=True)

try:
    client.confirmConnection()
    print("API connection successful!", flush=True)

    while True:
        state = client.getMultirotorState()
        position = state.kinematics_estimated.position
        velocity = state.kinematics_estimated.linear_velocity
        collision = client.simGetCollisionInfo()

        print(
            f"Position: X={position.x_val:.2f}, "
            f"Y={position.y_val:.2f}, "
            f"Z={position.z_val:.2f} | "
            f"Altitude relative to origin={-position.z_val:.2f} m | "
            f"Velocity: X={velocity.x_val:.2f}, "
            f"Y={velocity.y_val:.2f}, "
            f"Z={velocity.z_val:.2f} | "
            f"Collision={collision.has_collided}",
            flush=True
        )

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTelemetry test stopped.")

except Exception as exc:
    print(f"\nAPI test failed: {type(exc).__name__}: {exc}")
