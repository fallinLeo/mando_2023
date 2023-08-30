import numpy as np
from sklearn.linear_model import LinearRegression

# 주어진 데이터
pot_value = np.array([132, 239, 342, 436, 543, 643, 743, 841, 940])
ang_FR = np.array([-15.8, -10.5, -6.0, -2.3, 0, 2.3, 6.5, 11.8, 17.8])

# 데이터를 (행, 열)로 변환
pot_value = pot_value.reshape(-1, 1)

# 선형 회귀 모델 생성
model = LinearRegression()

# 모델 피팅 (학습)
model.fit(pot_value, ang_FR)

# 회귀식의 기울기와 절편 출력
slope = model.coef_[0]
intercept = model.intercept_

print("일차함수식: ang = {:.4f} * pot_value + {:.4f}".format(slope, intercept))
