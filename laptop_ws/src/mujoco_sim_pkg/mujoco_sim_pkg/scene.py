"""Load the MuJoCo track model with assets taken from the DATN package."""

import os

import cv2
import numpy as np

SIGN_TEXTURES = ('turn_left.png', 'speed_limit_20.png', 'stop.png')
MESHES = ('base_link.STL', 'left_Link.STL', 'right_Link.STL', 'free_Link.STL')

# MuJoCo ignores texture alpha, so the transparent corners of the sign PNGs
# would render black and could be read as lane tape. Gazebo discards those
# pixels instead; a light plate keeps them bright, like the scene behind.
PLATE_BGR = (200, 200, 200)


def composite_on_plate(image, plate_bgr=PLATE_BGR):
    """Return a BGR image with any alpha channel blended onto ``plate_bgr``."""
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 3:
        return image
    alpha = image[:, :, 3:4].astype(np.float32) / 255.0
    plate = np.empty_like(image[:, :, :3], dtype=np.float32)
    plate[:] = plate_bgr
    blended = image[:, :, :3].astype(np.float32) * alpha + plate * (1.0 - alpha)
    return np.round(blended).astype(np.uint8)


def load_assets(datn_share):
    """Return the in-memory files referenced by ``datn_track.xml``."""
    assets = {}
    texture_dir = os.path.join(datn_share, 'materials', 'textures')
    for name in SIGN_TEXTURES:
        image = cv2.imread(os.path.join(texture_dir, name), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise FileNotFoundError(os.path.join(texture_dir, name))
        ok, encoded = cv2.imencode('.png', composite_on_plate(image))
        if not ok:
            raise RuntimeError(f'Cannot encode texture {name}')
        assets[name] = encoded.tobytes()
    for name in MESHES:
        with open(os.path.join(datn_share, 'meshes', name), 'rb') as mesh_file:
            assets[name] = mesh_file.read()
    return assets


def load_model(model_path, datn_share):
    """Compile ``model_path`` into an ``mujoco.MjModel``."""
    import mujoco

    with open(model_path, 'r') as model_file:
        xml = model_file.read()
    return mujoco.MjModel.from_xml_string(xml, load_assets(datn_share))
