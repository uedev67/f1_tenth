# f1_tenth
2026 미래모빌리티 자율주행 Trial Lab 5팀으로 f1_tenth 참가함.

## 표준 개발 환경
우리 팀의 표준 개발 환경입니다.  
이 Docker 환경을 사용하면 **Ubuntu 버전 차이, 라이브러리 충돌, ROS 2 설치 문제** 없이 모든 팀원이 100% 동일한 환경에서 개발할 수 있습니다.

* **Base Image:** ROS 2 Humble (Desktop)
* **OS:** Ubuntu 22.04 LTS 기반
* **Python:** 3.10
* **포함된 툴:** F1TENTH Gym, RViz2, NumPy, SciPy, Pandas 등

## Dockerfile.sim 사용법(우분투 사용자)


### 1. 도커 설치하기
   
 ```
  sudo apt update
  sudo apt install -y docker.io
  sudo usermod -aG docker $USER
  newgrp docker
```

### 2. 작업 이미지 생성

   ```
   docker build -t f1tenth:sim -f Dockerfile.sim .
   ```

### 3. 화면 권한 허용

   ```
   xhost +local:docker
```

### 4. 컨테이너 실행

   ```
    docker run -it \
    --net=host \
    --privileged \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/f1_team_5" \
    --name my_f1_sim \
    --rm \
    f1tenth:sim
```

## Dockerfile.sim 사용법(윈도우 사용자)

1. [VcXsrv 다운로드 링크](https://sourceforge.net/projects/vcxsrv/)에서 설치 파일을 받아 설치합니다.
2. **XLaunch** 프로그램을 실행합니다.
3. 설정을 다음과 같이 변경하고 다음(Next)을 누릅니다.
   * **Extra settings** 단계에서 **"Disable access control"** 체크박스를 **반드시 체크**하세요. (이거 안 하면 화면 안 뜹니다!)
   * 마침(Finish)을 누르면 트레이 아이콘에 X 아이콘이 생깁니다.
  
### powershell 열기
```
docker build -t f1tenth:sim -f Dockerfile.sim .
```

### 이미지 실행
```
docker run -it `
    --net=host `
    --privileged `
    --env="DISPLAY=host.docker.internal:0.0" `
    --volume="${PWD}:/root/f1_team_5" `
    --name my_f1_sim `
    --rm `
    f1tenth:sim
```
