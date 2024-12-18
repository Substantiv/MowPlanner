import mu_values_pb2  # 导入通过 protoc 生成的 Python 类
import os
import matplotlib.pyplot as plt

def plot_mu_values(mu_values):
    # 提取 mu 值
    mu_sim_l = mu_values.mu_sim_l
    mu_sim_r = mu_values.mu_sim_r
    mu_model_l = mu_values.mu_model_l
    mu_model_r = mu_values.mu_model_r

    # 绘制
    labels = ['Left Sim', 'Right Sim', 'Left Model', 'Right Model']
    values = [mu_sim_l, mu_sim_r, mu_model_l, mu_model_r]

    plt.bar(labels, values, color=['blue', 'blue', 'green', 'green'])
    plt.xlabel('Mu Type')
    plt.ylabel('Mu Value')
    plt.title('Friction Coefficients')
    plt.show()




# 定义文件路径
bin_file_path = './mu_values.bin'

def parse_mu_values(bin_file_path):
    # 确保文件存在
    if not os.path.exists(bin_file_path):
        print(f"File {bin_file_path} does not exist.")
        return

    # 打开并读取二进制文件
    with open(bin_file_path, 'rb') as f:
        # 创建一个 MuValues 对象
        mu_values = mu_values_pb2.MuValues()

        # 反序列化数据到 MuValues 对象
        mu_values.ParseFromString(f.read())

        # 打印出反序列化的 mu 值
        print(f"mu_sim_l: {mu_values.mu_sim_l}")
        print(f"mu_sim_r: {mu_values.mu_sim_r}")
        print(f"mu_model_l: {mu_values.mu_model_l}")
        print(f"mu_model_r: {mu_values.mu_model_r}")

        # 返回解析的 mu 值（可选）
        return mu_values

# 解析文件并绘制图形
mu_values = parse_mu_values(bin_file_path)
plot_mu_values(mu_values)
