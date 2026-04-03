import asyncio, sys
from mavsdk import System


async def fault_gps_drift(target_id, drift_value):
    drone = System()
    await drone.connect(system_address=f"udp://:{14540 + target_id}")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone {target_id} connected")
            break

    param_server = drone.param

    await param_server.set_param_float("SIM_GPS_FAULT_S", 20.0)
    await param_server.set_param_float("SIM_GPS_DRIFT_E", drift_value)

    print(f"GPS drift fault injected to drone {target_id}")

if __name__ == "__main__":
    # get argv for target_id and drift_value
    if len(sys.argv) == 3:
        target_id = int(sys.argv[1])
        drift_value = float(sys.argv[2])
        asyncio.run(fault_gps_drift(target_id, drift_value))
    else:
        print("Usage: python gps_drift.py <target_id> <drift_value>")