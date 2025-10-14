import asyncio
from typing import Optional

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT
from woosh.proto.map.map_pack_pb2 import (
    SceneList,
    SceneData,
    SceneDataEasy,
    Download,
    Upload,
    Rename,
    Delete,
)

from base_menu import BaseMenu


class MapInfoMenu(BaseMenu):
    """地图信息菜单"""

    prompt = "(map) "

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__(robot, loop)

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "地图查询": [
                ("list", "获取场景列表"),
                ("data", "获取场景数据 (场景名)"),
                ("data_easy", "获取简化场景数据 (场景名)"),
            ],
            "地图操作": [
                ("download", "下载地图 (场景名)"),
                ("upload", "上传地图 (场景名 文件路径)"),
                ("rename", "重命名地图 (旧名称 新名称)"),
                ("delete", "删除地图 (场景名)"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("地图信息菜单帮助", sections)

    def do_list(self, arg):
        """场景列表请求"""
        self.run_async(self._get_scene_list())

    async def _get_scene_list(self):
        req = SceneList()
        result, ok, msg = await self.robot.scene_list_req(req, FULL_PRINT, FULL_PRINT)
        if ok and result:
            print(self.format_success("场景列表请求", result))
        else:
            print(self.format_error("场景列表请求", msg))

    def do_data(self, arg):
        """场景数据请求"""
        if not arg:
            print("用法: data <scene_name>")
            return

        self.run_async(self._get_scene_data(arg))

    async def _get_scene_data(self, scene_name):
        req = SceneData()
        req.name = scene_name

        result, ok, msg = await self.robot.scene_data_req(req, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("场景数据请求", result))
        else:
            print(self.format_error("场景数据请求", msg))

    def do_data_easy(self, arg):
        """场景数据请求(Easy)"""
        if not arg:
            print("用法: data_easy <scene_name>")
            return

        self.run_async(self._get_scene_data_easy(arg))

    async def _get_scene_data_easy(self, scene_name):
        req = SceneDataEasy()
        req.name = scene_name

        result, ok, msg = await self.robot.scene_data_easy_req(
            req, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("场景数据(Easy)请求", result))
        else:
            print(self.format_error("场景数据(Easy)请求", msg))

    def do_download(self, arg):
        """下载地图请求"""
        if not arg:
            print("用法: download <scene_name>")
            return

        self.run_async(self._get_download(arg))

    async def _get_download(self, scene_name):
        req = Download()
        req.scene_name = scene_name

        result, ok, msg = await self.robot.download_map(req, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("下载地图请求", result))
        else:
            print(self.format_error("下载地图请求", msg))

    def do_upload(self, arg):
        """上传地图请求"""
        args = arg.split()
        if len(args) < 2:
            print("用法: upload <scene_name> <file_path>")
            return

        scene_name = args[0]
        file_path = " ".join(args[1:])

        self.run_async(self._upload_map(scene_name, file_path))

    async def _upload_map(self, scene_name, file_path):
        req = Upload()
        req.scene_name = scene_name
        # TODO

        result, ok, msg = await self.robot.upload_map(req, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("上传地图请求", result))
        else:
            print(self.format_error("上传地图请求", msg))

    def do_rename(self, arg):
        """重命名地图请求"""
        args = arg.split()
        if len(args) != 2:
            print("用法: rename <old_name> <new_name>")
            return

        old_name = args[0]
        new_name = args[1]

        self.run_async(self._rename_map(old_name, new_name))

    async def _rename_map(self, old_name, new_name):
        req = Rename()
        req.old_map_name = old_name
        req.new_map_name = new_name

        result, ok, msg = await self.robot.rename_map(req, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("重命名地图请求", result))
        else:
            print(self.format_error("重命名地图请求", msg))

    def do_delete(self, arg):
        """删除地图请求"""
        if not arg:
            print("用法: delete <scene_name>")
            return

        self.run_async(self._delete_map(arg))

    async def _delete_map(self, scene_name):
        req = Delete()
        req.scene_name = scene_name

        result, ok, msg = await self.robot.delete_map(req, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("删除地图请求", result))
        else:
            print(self.format_error("删除地图请求", msg))

    def do_help(self, arg):
        """显示帮助信息"""
        self._print_help()
