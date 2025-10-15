# Uni-Lab-OS 一键安装快速指南

## 概述

本指南提供最快速的 Uni-Lab-OS 安装方法，使用预打包的 conda 环境，无需手动配置依赖。

## 前置要求

- 已安装 Conda/Miniconda/Miniforge/Mamba
- 至少 10GB 可用磁盘空间
- Windows 10+, macOS 10.14+, 或 Linux (Ubuntu 20.04+)

## 安装步骤

### 第一步：下载预打包环境

1. 访问 [GitHub Actions - Conda Pack Build](https://github.com/dptech-corp/Uni-Lab-OS/actions/workflows/conda-pack-build.yml)

2. 选择最新的成功构建记录（绿色勾号 ✓）

3. 在页面底部的 "Artifacts" 部分，下载对应你操作系统的压缩包：
   - Windows: `unilab-pack-win-64-{branch}.zip`
   - macOS (Intel): `unilab-pack-osx-64-{branch}.tar.gz`
   - macOS (Apple Silicon): `unilab-pack-osx-arm64-{branch}.tar.gz`
   - Linux: `unilab-pack-linux-64-{branch}.tar.gz`

### 第二步：解压并运行安装脚本

#### Windows

```batch
REM 使用 Windows 资源管理器解压下载的 zip 文件
REM 或使用命令行：
tar -xzf unilab-pack-win-64-dev.zip

REM 进入解压后的目录
cd unilab-pack-win-64-dev

REM 双击运行 install_unilab.bat
REM 或在命令行中执行：
install_unilab.bat
```

#### macOS

```bash
# 解压下载的压缩包
tar -xzf unilab-pack-osx-arm64-dev.tar.gz

# 进入解压后的目录
cd unilab-pack-osx-arm64-dev

# 运行安装脚本
bash install_unilab.sh
```

#### Linux

```bash
# 解压下载的压缩包
tar -xzf unilab-pack-linux-64-dev.tar.gz

# 进入解压后的目录
cd unilab-pack-linux-64-dev

# 添加执行权限（如果需要）
chmod +x install_unilab.sh

# 运行安装脚本
./install_unilab.sh
```

### 第三步：激活环境

```bash
conda activate unilab
```

### 第四步：验证安装（推荐）

```bash
# 确保已激活环境
conda activate unilab

# 运行验证脚本
python verify_installation.py
```

如果看到 "✓ All checks passed!"，说明安装成功！

## 常见问题

### Q: 安装脚本找不到 conda？

**A:** 确保你已经安装了 conda/miniconda/miniforge，并且安装在标准位置：

- **Windows**:

  - `%USERPROFILE%\miniforge3`
  - `%USERPROFILE%\miniconda3`
  - `%USERPROFILE%\anaconda3`
  - `C:\ProgramData\miniforge3`

- **macOS/Linux**:
  - `~/miniforge3`
  - `~/miniconda3`
  - `~/anaconda3`
  - `/opt/conda`

如果安装在其他位置，可以先激活 conda base 环境，然后手动运行安装脚本。

### Q: 安装后激活环境提示找不到？

**A:** 尝试以下方法：

```bash
# 方法 1: 使用 conda activate
conda activate unilab

# 方法 2: 使用完整路径激活（Windows）
call C:\Users\{YourUsername}\miniforge3\envs\unilab\Scripts\activate.bat

# 方法 2: 使用完整路径激活（Unix）
source ~/miniforge3/envs/unilab/bin/activate
```

### Q: conda-unpack 失败怎么办？

**A:** 尝试手动运行：

```bash
# Windows
cd %CONDA_PREFIX%\envs\unilab
.\Scripts\conda-unpack.exe

# macOS/Linux
cd $CONDA_PREFIX/envs/unilab
./bin/conda-unpack
```

### Q: 验证脚本报错？

**A:** 首先确认环境已激活：

```bash
# 检查当前环境
conda env list

# 应该看到 unilab 前面有 * 标记
```

如果仍有问题，查看具体报错信息，可能需要：

- 重新运行安装脚本
- 检查磁盘空间
- 查看详细文档

### Q: 环境很大，有办法减小吗？

**A:** 预打包的环境包含所有依赖，通常较大（压缩后 2-5GB）。这是为了确保离线安装和完整功能。如果空间有限，考虑使用手动安装方式，只安装需要的组件。

### Q: 如何更新到最新版本？

**A:** 重新下载最新的预打包环境，运行安装脚本时选择覆盖现有环境。

或者在现有环境中更新：

```bash
conda activate unilab

# 更新 unilabos
cd /path/to/Uni-Lab-OS
git pull
pip install -e . --upgrade

# 更新 ros-humble-unilabos-msgs
mamba update ros-humble-unilabos-msgs -c uni-lab -c robostack-staging -c conda-forge
```

## 下一步

安装完成后，你可以：

1. **查看启动指南**: {doc}`launch`
2. **运行示例**: {doc}`../boot_examples/index`
3. **配置设备**: 编辑 `unilabos_data/startup_config.json`
4. **阅读开发文档**: {doc}`../developer_guide/workstation_architecture`

## 需要帮助？

- **文档**: [docs/user_guide/installation.md](installation.md)
- **问题反馈**: [GitHub Issues](https://github.com/dptech-corp/Uni-Lab-OS/issues)
- **开发版安装**: 参考 {doc}`installation` 的方式二

---

**提示**: 这个预打包环境包含了从指定分支（通常是 `dev`）构建的最新代码。如果需要稳定版本，请使用方式二手动安装 release 版本。
