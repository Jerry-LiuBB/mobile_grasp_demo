#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
from setuptools import setup, find_packages

# 读取版本信息
with open("__init__.py", "r", encoding="utf-8") as f:
    version_match = re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", f.read(), re.M)
    if version_match:
        version = version_match.group(1)
    else:
        raise RuntimeError("无法从__init__.py中找到版本信息")

# 读取README文件
with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

# 核心依赖项
install_requires = [
    "protobuf==4.21.0",
    "websockets==13.1",
    "asyncio>=3.4.3",
    "typing-extensions>=4.0.0",
    "python-dateutil>=2.8.2",
]

# 开发依赖项
dev_requires = [
    "pytest>=7.0.0",
    "pytest-asyncio>=0.18.0",
    "black>=22.0.0",
    "isort>=5.10.0",
    "mypy>=0.950",
]

setup(
    name="woosh_robot_py",
    version=version,
    description="Python SDK for Woosh Robot Control System - 悟时机器人控制系统SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="woosh",
    author_email="support@wooshrobot.com",
    url="",
    packages=find_packages(include=["woosh", "woosh.*"]),
    py_modules=["woosh_robot", "woosh_interface", "woosh_base"],
    include_package_data=True,
    install_requires=install_requires,
    extras_require={
        "dev": dev_requires,
    },
    python_requires=">=3.7",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: Other/Proprietary License",
        "Natural Language :: Chinese (Simplified)",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Software Development :: Libraries",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    keywords="robot, control, woosh, robotics, automation, navigation",
    project_urls={},
    entry_points={
        "console_scripts": [
            "woosh-robot=cli.main:main",
        ],
    },
)
