import airsim
import numpy as np
import time

def test_lidar():
    try:
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451, timeout_value=3)
        client.confirmConnection()
        print("Connected to AirSim API.")
        vehicles = client.listVehicles()
        print("Vehicles:", vehicles)
        
        for vehicle in vehicles:
            lidar_data = client.getLidarData(lidar_name="Lidar1", vehicle_name=vehicle)
            print(f"Lidar data for {vehicle}: points len = {len(lidar_data.point_cloud)}")
            if len(lidar_data.point_cloud) >= 3:
                pts = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
                print(f"  Sample point 0: {pts[0]}")
                print(f"  Pose position: {lidar_data.pose.position}")
                print(f"  Pose orientation: {lidar_data.pose.orientation}")
    except Exception as e:
        print("AirSim connection test note:", e)

if __name__ == "__main__":
    test_lidar()
