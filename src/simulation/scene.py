"""ArUco station geometry placed by the random room generator."""
import xml.etree.ElementTree as ET
import cv2
import mujoco
import numpy as np
from src.perception.camera import DICTIONARY, MARKER_ID, MARKER_SIZE




def add_station(scene):
    asset = scene.find("asset")
    if asset is None:
        asset = ET.SubElement(scene, "asset")
    ET.SubElement(asset, "material", name="tag_white", rgba="1 1 1 1", emission="1", specular="0", shininess="0")
    ET.SubElement(asset, "material", name="tag_black", rgba="0 0 0 1", emission="1", specular="0", shininess="0")
    world = scene.find("worldbody")
    station = ET.SubElement(world, "body", name="charging_station",pos="0 0 .16")
    ET.SubElement(station, "geom", name="dock_board", type="box", pos="0 0 -0.009",
                  size="0.12 0.12 0.008", material="tag_white")
    # 由同一ArUco字典生成黑白格。实体格子由相机真实渲染，未向图像粘贴标记。
    bits = cv2.aruco.generateImageMarker(DICTIONARY, MARKER_ID, 6)
    cell = MARKER_SIZE/6
    for row in range(6):
        for col in range(6):
            if bits[row,col] == 0:
                ET.SubElement(station, "geom", name=f"tag_{row}_{col}", type="box",
                              pos=f"{(col-2.5)*cell} {(2.5-row)*cell} -0.0005",
                              size=f"{cell/2} {cell/2} 0.0005", material="tag_black",
                              contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="dock_support", type="cylinder", pos="0 0 .001",
                  size=".22 .001", rgba="0.5 0.1 0.65 1")


