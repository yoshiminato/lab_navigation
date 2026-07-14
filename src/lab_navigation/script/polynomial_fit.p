import numpy as np
import matplotlib.pyplot as plt

# 元のyを入力値にする
x = np.array([
    1.816, 1.790, 1.757, 1.743, 1.707,
    1.677, 1.647, 1.607, 1.566, 1.524
])

# 元のxを出力値にする
y = np.array([
    29.7, 28.6, 27.4, 26.9, 25.7,
    24.7, 23.9, 22.7, 21.7, 20.7
])

degree = 3

coefficients = np.polyfit(x, y, degree)
function = np.poly1d(coefficients)

print(f"{degree}次の近似関数:")
print(function)

print("\n係数:")
print(f"3次係数: {coefficients[0]:.10g}")
print(f"2次係数: {coefficients[1]:.10g}")
print(f"1次係数: {coefficients[2]:.10g}")
print(f"定数項  : {coefficients[3]:.10g}")

x_curve = np.linspace(x.min(), x.max(), 500)
y_curve = function(x_curve)

plt.scatter(x, y, label="Measured data")
plt.plot(x_curve, y_curve, label="Polynomial fit")

plt.xlabel("Measured voltage")
plt.ylabel("Battery value")
plt.title("Battery Calibration")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig("battery_calibration_fit.png", dpi=150)
plt.show()
