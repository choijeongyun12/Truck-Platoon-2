import carla
import time

TRUCK_TO_TRAILER_OFFSET_M = 2.031

def fix_all_trailers(target_offset=TRUCK_TO_TRAILER_OFFSET_M):
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        while True:
            actors = world.get_actors()
            vehicles = actors.filter('vehicle.*')
            
            trucks = {v.attributes.get('role_name'): v for v in vehicles if 'truck' in v.attributes.get('role_name', '')}
            trailers = {v.attributes.get('role_name').replace('trailer', 'truck'): v for v in vehicles if 'trailer' in v.attributes.get('role_name', '')}

            for role, truck in trucks.items():
                trailer = trailers.get(role)
                if trailer:
                    t_trans = truck.get_transform()
                    tr_loc = trailer.get_location()
                    
                    # 현재 오프셋 계산
                    t_fwd = t_trans.get_forward_vector()
                    rel_vec = tr_loc - t_trans.location
                    current_offset = abs(rel_vec.x * t_fwd.x + rel_vec.y * t_fwd.y + rel_vec.z * t_fwd.z)
                    
                    # 오차가 0.05m 이상이면 보정
                    if abs(current_offset - target_offset) > 0.05:
                        print(f"[{role}] Correcting offset: {current_offset:.3f}m -> {target_offset}m")
                        new_loc = t_trans.location - carla.Location(
                            x = t_fwd.x * target_offset,
                            y = t_fwd.y * target_offset,
                            z = 0.0
                        )
                        trailer.set_transform(carla.Transform(new_loc, t_trans.rotation))
            
            time.sleep(1.0) # 1초마다 체크

    except KeyboardInterrupt:
        print("\nWatcher stopped.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    fix_all_trailers()
