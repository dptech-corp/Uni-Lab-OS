<div align="center">
  <img src="docs/logo.png" alt="Uni-Lab Logo" width="200"/>
</div>

# Uni-Lab-OS

[![GitHub Stars](https://img.shields.io/github/stars/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/stargazers)
[![GitHub Forks](https://img.shields.io/github/forks/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/network/members)
[![GitHub Issues](https://img.shields.io/github/issues/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/issues)
[![GitHub License](https://img.shields.io/github/license/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/blob/main/LICENSE)

Uni-Lab 操作系统是一个用于实验室自动化的综合平台，旨在连接和控制各种实验设备，实现实验流程的自动化和标准化。

## 核心特点

- 多设备集成管理
- 自动化实验流程
- 云端连接能力
- 灵活的配置系统
- 支持多种实验协议

## 文档

详细文档可在以下位置找到:

- [在线文档](https://readthedocs.dp.tech/Uni-Lab/v0.8.0/)

## 安装指南

请先克隆此仓库:

```bash
git clone https://github.com/dptech-corp/Uni-Lab-OS.git
cd Uni-Lab-OS
```

然后安装 Uni-Lab-OS:

```bash
pip install .
```

## 快速开始

1. 配置Conda环境

Uni-Lab-OS 建议使用 `mamba` 管理环境。根据您的操作系统选择适当的环境文件:

```bash
# 创建新环境
mamba env create -f unilabos-[YOUR_OS].yaml
mamba activate unilab

# 或更新现有环境
conda env update --file unilabos-[YOUR_OS].yml -n 环境名

# 现阶段，需要安装 `unilabos_msgs` 包
# 可以前往 Release 页面下载系统对应的包进行安装
conda install ros-humble-unilabos-msgs-0.8.0-xxxxx.tar.bz2
```

其中 `[YOUR_OS]` 可以是 `win64`, `linux-64`, `osx-64`, 或 `osx-arm64`。


2. 启动 Uni-Lab 系统:

请见[文档-启动样例](https://readthedocs.dp.tech/Uni-Lab/v0.8.0/boot_examples/index.html)

## 消息格式

Uni-Lab-OS 使用预构建的 `unilabos_msgs` 进行系统通信。您可以在 [GitHub Releases](https://github.com/dptech-corp/Uni-Lab-OS/releases) 页面找到已构建的版本。

## 许可证

此项目采用 GPL-3.0 许可 - 详情请参阅 [LICENSE](LICENSE) 文件。

## 项目统计

### Stars 趋势

<a href="https://star-history.com/#dptech-corp/Uni-Lab-OS&Date">
  <img src="https://api.star-history.com/svg?repos=dptech-corp/Uni-Lab-OS&type=Date" alt="Star History Chart" width="600">
</a>

## 联系我们

- GitHub Issues: [https://github.com/dptech-corp/Uni-Lab-OS/issues](https://github.com/dptech-corp/Uni-Lab-OS/issues)