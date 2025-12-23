# f1_tenth
2026 미래모빌리티 자율주행 Trial Lab 5팀으로 f1_tenth 참가함.

## 표준 개발 환경
우리 팀의 표준 개발 환경입니다.  
이 Docker 환경을 사용하면 **Ubuntu 버전 차이, 라이브러리 충돌, ROS 2 설치 문제** 없이 모든 팀원이 100% 동일한 환경에서 개발할 수 있습니다.

* **Base Image:** ROS 2 Humble (Desktop)
* **OS:** Ubuntu 22.04 LTS 기반
* **Python:** 3.10
* **포함된 툴:** F1TENTH Gym, RViz2, NumPy, SciPy, Pandas 등

## Dockerfile.sim 사용법


### 1. 도커 설치하기
   
 ```  sudo apt update
  sudo apt install -y docker.io
  sudo usermod -aG docker $USER
  newgrp docker```

### 2. 작업 이미지 생성

   ```docker build -t f1tenth:sim -f Dockerfile.sim .```

### 3. 화면 권한 허용

   ```xhost +local:docker```

### 4. 컨테이너 실행

   ```docker run -it \
    --net=host \
    --privileged \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/f1_team_5" \
    --name my_f1_sim \
    --rm \
    f1tenth:sim```
