<h1 align="center">MagiClaw</h1>

<p align="center">
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white" /></a>
  <a href="https://www.raspberrypi.com/products/raspberry-pi-5/"><img src="https://img.shields.io/badge/Raspberry_Pi-5_-A22846?logo=raspberry-pi&logoColor=white" /></a>
  <a href=""><img src="https://img.shields.io/badge/Docs-latest-4285F4?logo=google-docs&logoColor=white" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-3DA639?logo=open-source-initiative&logoColor=white" /></a>
</p>

## ✨ Overview

MagiClaw is a versatile human-robot interactive manipulation system designed by [Design and Learning Research Group (DLRG)](https://ancorasir.com) at [SUSTech](https://www.sustech.edu.cn/en/).

## 🦾 Hardware

MagiClaw consists of a motor-driven gripper, two [meta-fingers](https://github.com/asMagiClaw/meta-finger), a [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/), an iPhone, and several 3D-printed parts. There are two types of MagiClaws, including in-hand and on-robot types. For more details, please refer to the [docs]().

## 🚀 Installation

This repository mainly includes the code running on the Raspberry Pi 5. To set up MagiClaw, first clone the repository:

```bash
git clone https://github.com/asMagiClaw/magiclaw.git
cd magiclaw
```

It's recommended to create a `conda` environment for this project, and install `magiclaw` using `pip`:

```bash
conda create -n magiclaw python=3.10
conda activate magiclaw
pip install -e .
```

## 🛠️ Configuration

Before running MagiClaw, you need to modify the files in `configs/` according to your setup. For more details, refer to the [docs]().

## 🏃 Quick Start

To run MagiClaw, use the following command:

```bash
run-magiclaw --id <id> --mode <mode> --loop_rate <loop_rate> --phone_host <phone_host> --bilateral_host <bilateral_host>
```

For more details, refer to the [docs]().

## 📖 Documentation

We have comprehensive documentation available to help you get started and understand the features of MagiClaw. You can find it [here]().

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

We welcome contributions to MagiClaw! If you have suggestions or improvements, please open an issue or submit a pull request.

## 📫 Contact

For any questions or feedback, feel free to reach out to us via the [GitHub Issues](https://github.com/asMagiClaw/magiclaw/issues) page.
