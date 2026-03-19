import carla
import time

# CARLA 클라이언트 연결
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
spectator = world.get_spectator()

print("카메라(Spectator)를 원하는 위치로 이동시킨 후 터미널의 좌표를 확인하세요.")
print("종료하려면 Ctrl+C를 누르세요.\n")

try:
    while True:
        # transform 객체를 통해 location과 rotation을 한 번에 가져옴
        transform = spectator.get_transform()
        loc = transform.location
        rot = transform.rotation
        
        # 터미널 한 줄에 실시간 업데이트 출력
        print(f"X: {loc.x:6.1f} | Y: {loc.y:6.1f} | Z: {loc.z:5.1f} | Yaw: {rot.yaw:6.1f}", end='\r')
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n\n좌표 측정을 종료합니다.")