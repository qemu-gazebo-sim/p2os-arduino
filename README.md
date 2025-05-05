
<h1 align="center">🔗 P2OS  Arduino</h1>
<p align="center">C++ implementation of the P2OS communication protocol for arduino</p>

## 🎈 Overview

P2OS (Pioneer 2 Operating System) Communication protocol for Arduino/ESP32 based in the [ROS P2OS package](https://github.com/allenh1/p2os). In addition to this ROS package, we used the [Pioner 2 manual](https://www.iri.upc.edu/groups/lrobots/private/Pioneer2/AT_DISK1/DOCUMENTS/p2opman9.pdf) as knowledge base.

If you want to undertand how this protocol works, check the file [p2os_overview.md](./p2os_overview.md)

## 🚀 Implementation example


## ⚡ Quick Start

If you are using platformio, add the library as dependece of your project:

```
[env:esp32doit-devkit-v1]
...
lib_deps = P2OS
```

check the example in the file [platformio.example.ini](./platformio.example.ini)

If you are using arduino ide, install the package from the packages available in the arduino library.

## 🧩 Project UML

![P2OS Arduino UML](./docs/diagramas-test_setup-pioneer_2dx_interface-p2os_comm-Complete.drawio.png)

## 🎨 Format

The project uses `clang-format` to format files, there is a `.clang-format` with the formatting rules for the project. To install it, on Ubuntu, run the following command on the terminal:

```bash
sudo apt install clang-format
```

If you are using windows, check the official documentation [here](https://clang.llvm.org/get_started.html)