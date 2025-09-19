# **Uni-Lab 安装**

## 快速开始

1. **配置 Conda 环境**

Uni-Lab-OS 建议使用 `mamba` 管理环境。创建新的环境：

```shell
mamba create -n unilab uni-lab::unilabos -c robostack-staging -c conda-forge
```

2. **安装开发版 Uni-Lab-OS**

```shell
# 配置好conda环境后，克隆仓库
git clone https://github.com/dptech-corp/Uni-Lab-OS.git -b dev
cd Uni-Lab-OS

# 安装 Uni-Lab-OS
pip install -e .
```

3. **安装开发版 ros-humble-unilabos-msgs**

**卸载老版本：**
```shell
conda activate unilab
conda remove --force ros-humble-unilabos-msgs
```
有时相同的安装包版本会由于dev构建得到的md5不一样，触发安全检查，可输入 `config set safety_checks disabled` 来关闭安全检查。

**安装新版本：**

访问 https://github.com/dptech-corp/Uni-Lab-OS/actions/workflows/multi-platform-build.yml 选择最新的构建，下载对应平台的压缩包（仅解压一次，得到.conda文件）使用如下指令：
```shell
conda activate base
conda install ros-humble-unilabos-msgs-<version>-<platform>.conda --offline -n <环境名>
```

4. **启动 Uni-Lab 系统**

请参见{doc}`启动样例 <../boot_examples/index>`或{doc}`启动指南 <launch>`了解详细的启动方法。
