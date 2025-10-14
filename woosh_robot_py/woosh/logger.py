"""日志配置模块

提供统一的日志配置和管理功能。
支持文件日志、控制台日志、日志轮转等特性。
"""

import logging
import logging.handlers
import sys
from typing import Optional
from pathlib import Path
import traceback
import json


class LogFormatter(logging.Formatter):
    """自定义日志格式化器"""

    def __init__(self):
        super().__init__(
            fmt="%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

    def formatException(self, exc_info) -> str:
        """格式化异常信息，包含完整的堆栈跟踪"""
        if exc_info:
            return "\n" + "".join(traceback.format_exception(*exc_info))
        return ""

    def format(self, record: logging.LogRecord) -> str:
        """格式化日志记录"""
        if isinstance(record.msg, dict):
            record.msg = json.dumps(record.msg, ensure_ascii=False)

        return super().format(record)


class WooshLogger:
    """Woosh机器人日志管理器"""

    def __init__(
        self,
        name: str = "woosh",
        level: str = "INFO",
        log_dir: Optional[str] = None,
        max_bytes: int = 10 * 1024 * 1024,  # 10MB
        backup_count: int = 5,
        console: bool = True,
        file: bool = True,
    ):
        """初始化日志管理器

        Args:
            name: 日志器名称
            level: 日志级别
            log_dir: 日志文件目录
            max_bytes: 单个日志文件最大字节数
            backup_count: 保留的日志文件数量
            console: 是否输出到控制台
            file: 是否输出到文件
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))

        # 避免重复处理器
        self.logger.handlers = []

        # 创建格式化器
        formatter = LogFormatter()

        # 控制台处理器
        if console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

        # 文件处理器
        if file and log_dir:
            log_dir = Path(log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)

            # 常规日志文件
            log_file = log_dir / f"{name}.log"
            file_handler = logging.handlers.RotatingFileHandler(
                log_file, maxBytes=max_bytes, backupCount=backup_count, encoding="utf-8"
            )
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

            # 错误日志文件
            error_file = log_dir / f"{name}_error.log"
            error_handler = logging.handlers.RotatingFileHandler(
                error_file,
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding="utf-8",
            )
            error_handler.setFormatter(formatter)
            error_handler.setLevel(logging.ERROR)
            self.logger.addHandler(error_handler)

    def get_logger(self) -> logging.Logger:
        """获取日志器实例"""
        return self.logger


def create_logger(
    name: str = "woosh", level: str = "INFO", log_dir: Optional[str] = None, **kwargs
) -> logging.Logger:
    """创建日志器的便捷函数

    Args:
        name: 日志器名称
        level: 日志级别
        log_dir: 日志文件目录
        **kwargs: 其他WooshLogger参数

    Returns:
        logging.Logger: 配置好的日志器实例
    """
    logger_manager = WooshLogger(name=name, level=level, log_dir=log_dir, **kwargs)
    return logger_manager.get_logger()


# 默认日志器
default_logger = create_logger()
