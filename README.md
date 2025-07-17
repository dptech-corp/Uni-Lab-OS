<div align="center">
  <img src="docs/logo.png" alt="Uni-Lab Logo" width="200"/>
</div>

# Uni-Lab-OS

<!-- Language switcher -->
**English** | [中文](README_zh.md)

[![GitHub Stars](https://img.shields.io/github/stars/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/stargazers)
[![GitHub Forks](https://img.shields.io/github/forks/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/network/members)
[![GitHub Issues](https://img.shields.io/github/issues/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/issues)
[![GitHub License](https://img.shields.io/github/license/dptech-corp/Uni-Lab-OS.svg)](https://github.com/dptech-corp/Uni-Lab-OS/blob/main/LICENSE)

Uni-Lab-OS is a platform for laboratory automation, designed to connect and control various experimental equipment, enabling automation and standardization of experimental workflows.

## 🏆 Competition

Join the [Intelligent Organic Chemistry Synthesis Competition](https://bohrium.dp.tech/competitions/1451645258) to explore automated synthesis with Uni-Lab-OS!

## Key Features

- Multi-device integration management
- Automated experimental workflows
- Cloud connectivity capabilities
- Flexible configuration system
- Support for multiple experimental protocols

## Documentation

Detailed documentation can be found at:

- [Online Documentation](https://readthedocs.dp.tech/Uni-Lab/v0.8.0/)

## Quick Start

1. Configure Conda Environment

Uni-Lab-OS recommends using `mamba` for environment management. Choose the appropriate environment file for your operating system:

```bash
# Create new environment
mamba env create -f unilabos-[YOUR_OS].yaml
mamba activate unilab

# Or update existing environment
# Where `[YOUR_OS]` can be `win64`, `linux-64`, `osx-64`, or `osx-arm64`.
conda env update --file unilabos-[YOUR_OS].yml -n environment_name

# Currently, you need to install the `unilabos_msgs` package
# You can download the system-specific package from the Release page
conda install ros-humble-unilabos-msgs-0.9.12-xxxxx.tar.bz2

# Install PyLabRobot and other prerequisites
git clone https://github.com/PyLabRobot/pylabrobot plr_repo
cd plr_repo
pip install .[opentrons]
```

2. Install Uni-Lab-OS:

```bash
# Clone the repository
git clone https://github.com/dptech-corp/Uni-Lab-OS.git
cd Uni-Lab-OS

# Install Uni-Lab-OS
pip install .
```

3. Start Uni-Lab System:

Please refer to [Documentation - Boot Examples](https://readthedocs.dp.tech/Uni-Lab/v0.8.0/boot_examples/index.html)

## Message Format

Uni-Lab-OS uses pre-built `unilabos_msgs` for system communication. You can find the built versions on the [GitHub Releases](https://github.com/dptech-corp/Uni-Lab-OS/releases) page.

## License

This project is licensed under GPL-3.0 - see the [LICENSE](LICENSE) file for details.

## Project Statistics

### Stars Trend

<a href="https://star-history.com/#dptech-corp/Uni-Lab-OS&Date">
  <img src="https://api.star-history.com/svg?repos=dptech-corp/Uni-Lab-OS&type=Date" alt="Star History Chart" width="600">
</a>

## Contact Us

- GitHub Issues: [https://github.com/dptech-corp/Uni-Lab-OS/issues](https://github.com/dptech-corp/Uni-Lab-OS/issues)