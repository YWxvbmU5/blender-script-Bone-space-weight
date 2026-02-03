import math
import bpy
from mathutils import Vector, Euler
from .math_utils import _euler_deg_to_dir, _euler_deg_to_quat

def driver_compute_rotation_weight(
    crx, cry, crz, 
    prx, pry, prz, 
    rrx, rry, rrz,   # 静止旋转 rest_rot
    cone_angle, 
    axis_idx,
    mode=1           # 0: 锥体衰减, 1: 投影映射模式
):
    try:
        # 统一单位：crx/y/z 是驱动器传入的弧度，prx/rrx 是配置的角度
        cur = Vector((crx, cry, crz))
        pose = Vector((math.radians(prx), math.radians(pry), math.radians(prz)))
        rest = Vector((math.radians(rrx), math.radians(rry), math.radians(rrz)))

        # --- 模式 1：投影映射模式
        if mode == 1:
            pose_dir = pose - rest
            cur_rel = cur - rest
            denom = pose_dir.dot(pose_dir)
            
            if denom < 1e-6:
                return 1.0 if cur_rel.length < 1e-3 else 0.0
            
            w = cur_rel.dot(pose_dir) / denom
            
            # 对称衰减逻辑：[0->1] 权重上升，[1->2] 权重回落，其余 0
            if w < 0.0 or w > 2.0:
                return 0.0
            elif w > 1.0:
                return 2.0 - w
            return w

        # --- 模式 0：标准锥体衰减
        else:
            def _get_dir(rot_rad, a_idx):
                e = Euler(rot_rad, 'XYZ')
                return (e.to_matrix() @ Vector(((1,0,0),(0,1,0),(0,0,1))[int(a_idx)])).normalized()

            dir_center = _get_dir(pose, axis_idx)
            dir_cur = _get_dir(cur, axis_idx)
            
            dot = max(-1.0, min(1.0, dir_center.dot(dir_cur)))
            angle_deg = math.degrees(math.acos(dot))
            
            if angle_deg <= cone_angle:
                return 1.0 - (angle_deg / cone_angle) if cone_angle > 0.0 else 1.0
            return 0.0
            
    except Exception as e:
        print(f"PSD Error: {e}")
        return 0.0

def driver_compute_location_weight(
    clx, cly, clz,  # 当前骨骼 Local 位置 (变量传入)
    plx, ply, plz,  # 目标 Pose 位置 
    rlx, rly, rlz,  # 初始 Rest 位置 
    radius,         # 判定半径/范围
    mode            # 0: 投影模式, 1: 轴向立方体模式, 2: 球形衰减模式
):
    try:
        # 转换为 Vector 方便计算
        cur = Vector((clx, cly, clz))
        target = Vector((plx, ply, plz))
        rest = Vector((rlx, rly, rlz))
        
        # 模式 2：球形衰减 (Spherical Falloff)
        if mode == 2:
            if radius <= 0: 
                return 1.0 if (cur - target).length < 0.001 else 0.0
            
            # 计算当前点到目标点的距离
            dist = (cur - target).length
            
            # 线性衰减：1.0 - (距离 / 半径)
            weight = 1.0 - (dist / radius)
            return max(0.0, min(1.0, weight))

        # 模式 1：轴向立方体/判定
        elif mode == 1:
            d = cur - target
            wx = max(0.0, 1.0 - abs(d.x) / radius) if radius > 0 else 1.0
            wy = max(0.0, 1.0 - abs(d.y) / radius) if radius > 0 else 1.0
            wz = max(0.0, 1.0 - abs(d.z) / radius) if radius > 0 else 1.0
            return wx * wy * wz
            
        # 模式 0：轴向投影模式
        else:
            line_vec = target - rest
            if line_vec.length_squared < 1e-9: return 0.0
            t = (cur - rest).dot(line_vec) / line_vec.length_squared
            return 2.0 - t if t > 1.0 else (t if t > 0.0 else 0.0)

    except Exception as e:
        return 0.0
    
def driver_compute_scale_weight(
    csx, csy, csz,  # 当前缩放 (变量)
    psx, psy, psz,  # 目标缩放 (JSON 静态值)
    rsx, rsy, rsz,  # 初始缩放 (通常是 1.0, 1.0, 1.0)
    mode=0          # 0: 投影映射模式, 1: 简单轴向比例
):
    try:
        cur = Vector((csx, csy, csz))
        pose = Vector((psx, psy, psz))
        rest = Vector((rsx, rsy, rsz))
        
        # --- 模式 0：投影映射模式 (w = 2.0 - w) ---
        if mode == 0:
            pose_dir = pose - rest
            cur_rel = cur - rest
            denom = pose_dir.dot(pose_dir)
            
            if denom < 1e-6:
                return 1.0 if cur_rel.length < 1e-3 else 0.0
            
            w = cur_rel.dot(pose_dir) / denom
            
            # 对称衰减：0->1 增加, 1->2 减少
            if w < 0.0 or w > 2.0:
                return 0.0
            elif w > 1.0:
                return 2.0 - w
            return w
            
        # --- 模式 1：简单距离判定 (类似半径) ---
        else:
            dist = (cur - pose).length
            return max(0.0, 1.0 - dist) # 偏差越大权重越低

    except:
        return 0.0


# --- 注册到命名空间 ---
def register_drivers():

    bpy.app.driver_namespace["psd_rot"] = driver_compute_rotation_weight
    bpy.app.driver_namespace["psd_loc"] = driver_compute_location_weight
    bpy.app.driver_namespace["psd_sca"] = driver_compute_scale_weight