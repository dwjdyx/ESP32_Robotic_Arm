from pathlib import Path

# 输入文件：舵机实际角度 alpha1~alpha5
INPUT_FILE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\calibration\angles_init.txt")

# 输出文件：转换后的关节角 theta1~theta5
OUTPUT_FILE = Path(r"D:\ESP32-S3\2-esp_project\6-5DOF_arm\python_code\detection\joints_init.txt")

def alpha_to_theta(alpha1, alpha2, alpha3, alpha4, alpha5, alpha6):
    theta1 = 135 - alpha1
    theta2 = alpha2 - 231
    theta3 = alpha3 - 231
    theta4 = 45 - alpha4
    theta5 = alpha5 - 225
    theta6 = alpha6 - 225

    return theta1, theta2, theta3, theta4, theta5, theta6


def main():
    if not INPUT_FILE.exists():
        print(f"找不到输入文件：{INPUT_FILE.resolve()}")
        return

    result = []

    with open(INPUT_FILE, "r", encoding="utf-8-sig") as file:
        for line_number, line in enumerate(file, start=1):
            #去掉首尾\n
            line = line.strip()

            # 跳过空行
            if not line:
                continue

            # 兼容中英文逗号分隔
            line = line.replace("，", ",")
            alpha = [float(x.strip()) for x in line.split(",")]

            if len(alpha) != 6:
                raise ValueError(
                    f"第 {line_number} 行应包含 6 个角度，实际为 {len(alpha)} 个"
                )

            theta = alpha_to_theta(*alpha)

            # 整数不带小数点，非整数保留 6 位小数
            values = [
                str(int(x)) if x.is_integer()
                else f"{x:.6f}".rstrip("0").rstrip(".")
                for x in theta
            ]

            result.append(",".join(values))

    with open(OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write("\n".join(result) + "\n")

    print(f"转换完成，共处理 {len(result)} 组数据")
    print(f"输出文件：{OUTPUT_FILE.resolve()}")


if __name__ == "__main__":
    main()