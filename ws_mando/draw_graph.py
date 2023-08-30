import matplotlib.pyplot as plt

pot_value = [132, 239, 342, 436, 543, 643, 743, 841, 940] 
ang_FR = [-15.8, -10.5, -6.0, -2.3, 0, 2.3, 6.5, 11.8, 17.8]   # FR
ang_FL = [-18, -11, -6.8, -2.8, 0, 3.8, 7, 11.5, 16]   # FL

plt.plot(pot_value, ang_FR, marker='o', label='FR')  # 점을 찍어서 표시
plt.plot(pot_value, ang_FL, marker='o', label='FL')  # 점을 찍어서 표시

plt.xlabel('Potentiometer Value')
plt.ylabel('Angle')
plt.title('Potentiometer Value vs Angle')
plt.grid(True)
plt.legend()  # 범례 추가
plt.savefig('angle_vs_pot_value.png')  # 이미지 파일로 저장
plt.show()
