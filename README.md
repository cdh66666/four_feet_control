# four_feet_control
 # matlab四足机器人VMC控制器 - 2024年11月19日

-  在matlab中打开./matlab/startup.m文件并点击运行，即可看到仿真效果。

   ![示例教程图片](https://github.com/cdh66666/four_feet_control/blob/main/image/README/%E7%A4%BA%E4%BE%8B%E6%95%99%E7%A8%8B%E5%9B%BE%E7%89%87.png)





# python 强化学习 环境配置

>
> **懒得配置环境的，可点击[链接](https://suyvt0crm5.feishu.cn/docx/XK72dTuyco6y7PxgA0dcaZZVngd)进入飞书文档，在学习附件一栏找到“*python强化学习环境（vscode打开文件夹直接用，无需配置）*”直接下载RL.zip解压到任意路径，然后用vscode打开RL文件夹即可（文件夹内已经包含配置环境）**

# [【Conda+vsCode】vsCode 中使用 conda 配置虚拟环境](https://blog.csdn.net/weixin_54383080/article/details/138613865?ops_request_misc=%257B%2522request%255Fid%2522%253A%252258837904ede6c1d0d6aab794b6e0da6c%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=58837904ede6c1d0d6aab794b6e0da6c&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-2-138613865-null-null.142^v100^pc_search_result_base9&utm_term=vscode%20conda&spm=1018.2226.3001.4187)



# 安装基本gym库，只含有入门级环境

```
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple gym
```

# 安装Box2D环境支持组件

```
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple gym[box2d]

pip install -i https://pypi.tuna.tsinghua.edu.cn/simple gym[atari]

pip install -i https://pypi.tuna.tsinghua.edu.cn/simple autorom

AutoROM --accept-license

pip install swing
```



# 环境测试

```
import gym

env = gym.make("LunarLander-v2",render_mode="human")
env.action_space.seed(42)

observation,info = env.reset(seed=42)

for _ in range(1000):
    observation,reward,terminated,truncated,info = env.step(env.action_space.sample())

    if terminated or truncated:
        observation,info = env.reset()


env.close()
```

- 出现如下游戏运行结果，表示环境安装成功

  ![python强化学习环境测试图片](https://github.com/cdh66666/four_feet_control/blob/main/image/README/python%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0%E7%8E%AF%E5%A2%83%E6%B5%8B%E8%AF%95.png)
