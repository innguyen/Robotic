"""Trusted lossless UE PNG/EXR decode and depth-tested human visibility."""
import base64
import os


def decode_human_capture(raw):
    os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"
    import cv2
    import numpy as np
    def image(text):
        result = cv2.imdecode(np.frombuffer(base64.b64decode(text,validate=True),np.uint8),cv2.IMREAD_UNCHANGED)
        if result is None: raise ValueError("invalid native image payload")
        return result
    rgb = image(raw["rgb_png"])
    z = image(raw["depth_exr"])
    z = z[:,:,2].astype(np.float64)/100
    valid = np.isfinite(z)&(z>=.01)&(z<=10)
    visible = {}
    for identity,payload in raw["isolated_depth_exr"].items():
        isolated = image(payload)[:,:,2].astype(np.float64)/100
        keep = valid & np.isfinite(isolated) & (isolated>.01)&(isolated<10)&(np.abs(isolated-z)<=np.maximum(.001,np.abs(isolated)*.0007))
        ys,xs = np.where(keep)
        if len(xs): visible[identity] = {"pixels":len(xs),"bbox":[int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]}
    depth = z.flatten().tolist()
    for i,enabled in enumerate(valid.flatten()):
        if not enabled: depth[i] = None
    width,height = raw["camera"]["width"],raw["camera"]["height"]
    if rgb.shape[:2] != (height,width) or z.shape != (height,width): raise ValueError("native image size mismatch")
    return {**raw,"rgb":rgb[:,:,:3][:,:,::-1].flatten().tolist(),"depth_m":depth,"visible":visible}
