import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import os


# ==========================================
# 1. 설정 파라미터 (TUNING PARAMETERS)
# ==========================================
INPUT_FILENAME = 'clicked_waypoints.csv'  # 입력 파일 이름
OUTPUT_FILENAME = 'raceline.csv' # 출력 파일 이름


# 차량 물리 설정 (Serpent 1/8 Scale)
MU = 1.2           # 타이어 마찰 계수 (높음)
GRAVITY = 9.81     # 중력 가속도
MAX_SPEED = 10.0   # 직선 최대 속도 (m/s)
LOOKAHEAD_POINTS = 550  # 생성할 전체 경로의 점 개수 (해상도)


# 방향 설정
FORCE_CLOCKWISE = True  # True면 강제로 시계 방향으로 경로를 뒤집음


# ==========================================
# 2. 함수 정의
# ==========================================


def load_waypoints(filename):
    """CSV 파일을 읽어옵니다. 헤더 유무에 따라 유연하게 대처합니다."""
    if not os.path.exists(filename):
        print(f"[Error] '{filename}' 파일이 없습니다. 경로를 확인하세요.")
        return None
    
    try:
        # 헤더가 있는 경우와 없는 경우를 모두 고려
        df = pd.read_csv(filename)
        # 만약 첫 줄이 'x'가 아니라 숫자라면 헤더가 없는 것으로 간주
        if not isinstance(df.columns[0], str) or 'x' not in df.columns:
            df = pd.read_csv(filename, header=None)
            df.columns = ['x', 'y', 'z'][:len(df.columns)] # z가 있을 수도 있으니 처리
        
        points = df[['x', 'y']].values
        return points
    except Exception as e:
        print(f"[Error] 파일 로드 중 오류: {e}")
        return None


def ensure_clockwise(x, y):
    """
    경로가 시계 방향(CW)인지 확인하고, 반시계(CCW)라면 뒤집습니다.
    Shoelace 공식을 사용하여 다각형의 방향(부호)을 판별합니다.
    """
    # 다각형의 면적 공식 (Shoelace Formula)
    # Area > 0 : Counter-Clockwise (CCW)
    # Area < 0 : Clockwise (CW)
    area = 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])
    
    if area > 0: # 반시계 방향(CCW)인 경우
        print("[Info] 경로가 반시계 방향(CCW)입니다. 시계 방향(CW)으로 뒤집습니다.")
        return x[::-1], y[::-1]
    else:
        print("[Info] 경로가 이미 시계 방향(CW)입니다.")
        return x, y


def process_path():
    # 1. 웨이포인트 로드
    waypoints = load_waypoints(INPUT_FILENAME)
    if waypoints is None: return

    # 중복 점 제거 (시작점과 끝점이 너무 가까우면 스플라인 계산 꼬임 방지)
    if np.linalg.norm(waypoints[0] - waypoints[-1]) < 0.1:
        waypoints = waypoints[:-1]

    x_wp = waypoints[:, 0]
    y_wp = waypoints[:, 1]
    
    print(f"[Step 1] {len(waypoints)}개의 웨이포인트를 로드했습니다.")

    # 2. 스플라인 보간 (부드러운 곡선 생성)
    # s=0.0: 점을 정확히 지나감, s=0.5: 부드럽게 근사함 (노이즈 제거 효과)
    try:
        tck, u = splprep([x_wp, y_wp], s=0.5, per=True) 
        u_new = np.linspace(0, 1, LOOKAHEAD_POINTS)
        x_new, y_new = splev(u_new, tck, der=0)
    except Exception as e:
        print(f"[Error] 스플라인 생성 실패 (점의 개수가 너무 적거나 겹침): {e}")
        return

    # 3. 방향 보정 (Clockwise 강제)
    if FORCE_CLOCKWISE:
        x_new, y_new = ensure_clockwise(x_new, y_new)

    # 4. 곡률 및 속도 계산
    dx = np.gradient(x_new)
    dy = np.gradient(y_new)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    # 곡률 공식: k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = np.power(dx**2 + dy**2, 1.5)
    
    # 0으로 나누기 방지
    curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)
    radius = np.divide(1.0, curvature, out=np.inf * np.ones_like(curvature), where=curvature!=0)

    # 속도 공식: v = sqrt(mu * g * R)
    velocity = np.sqrt(MU * GRAVITY * radius)
    velocity = np.clip(velocity, 0, MAX_SPEED) # 최대 속도 제한

    # 속도 스무딩 (급격한 가감속 방지)
    velocity = np.convolve(velocity, np.ones(10)/10, mode='same')

    # 5. 결과 저장
    output_df = pd.DataFrame({
        'x': x_new,
        'y': y_new,
        'velocity': velocity,
        'radius': radius
    })
    
    output_df.to_csv(OUTPUT_FILENAME, index=False)
    print(f"\n[Success] '{OUTPUT_FILENAME}' 파일이 생성되었습니다! (총 {len(output_df)} 포인트)")

    # 6. 시각화 (확인용)
    plt.figure(figsize=(10, 8))
    
    # 경로 그리기 (속도에 따라 색상 변화)
    sc = plt.scatter(x_new, y_new, c=velocity, cmap='plasma', s=5, label='Raceline')
    plt.colorbar(sc, label='Target Speed (m/s)')
    
    # 원본 웨이포인트 표시
    plt.scatter(x_wp, y_wp, c='red', marker='x', s=50, label='Original Clicks')
    
    # 시작점 표시 (방향 확인용)
    plt.arrow(x_new[0], y_new[0], x_new[5]-x_new[0], y_new[5]-y_new[0], 
              head_width=0.5, color='green', label='Start Direction')

    plt.title(f"Optimized Raceline (Max Speed: {MAX_SPEED}m/s)")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    process_path()