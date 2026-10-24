import math
import bpy
from mathutils import Vector, Euler, Quaternion
from .math_utils import _euler_deg_to_dir, _euler_deg_to_quat
import bpy.app.handlers as handlers

def driver_compute_rotation_weight(
    crx, cry, crz, 
    prx, pry, prz, 
    rrx, rry, rrz,
    cone_angle, 
    axis_idx,
    mode=1,
    sx=1.0, sy=1.0, sz=1.0,
    use_triangle_weights=0
):
    try:
        # 1. 统一转换弧度
        cur_euler = Euler((crx, cry, crz), 'XYZ')
        pose_euler = Euler((math.radians(prx), math.radians(pry), math.radians(prz)), 'XYZ')
        rest_euler = Euler((math.radians(rrx), math.radians(rry), math.radians(rrz)), 'XYZ')

        # --- 模式 2, 3, 4: Swing/Twist 融合驱动模式 ---
        if mode >= 2:
            # 1. 确定扭转轴
            t_axis = Vector((1,0,0)) if mode == 2 else (Vector((0,1,0)) if mode == 3 else Vector((0,0,1)))
            
            # 2. 获取四元数增量 (相对于 Rest 姿态)
            dq_cur = cur_euler.to_quaternion() @ rest_euler.to_quaternion().inverted()
            dq_tar = pose_euler.to_quaternion() @ rest_euler.to_quaternion().inverted()
            
            # 3. 内部辅助：Swing/Twist 分解
            def decompose_st(q, axis):
                q_vec = Vector((q.x, q.y, q.z))
                proj = axis * q_vec.dot(axis)
                tw = Quaternion((q.w, proj.x, proj.y, proj.z))
                # try: tw.normalize()
                # except: tw = Quaternion((1,0,0,0))
                if tw.magnitude > 1e-4:
                    tw.normalize()
                else:
                    # 触发奇点反推逻辑 (Singularity)
                    # 利用非扭曲轴的正交分量来硬性构建 Twist，锁定 Swing 通道
                    if abs(axis.y) > 0.5:   # Y 轴扭曲
                        tw = Quaternion((q.x, 0.0, q.z, 0.0))
                    elif abs(axis.x) > 0.5: # X 轴扭曲
                        tw = Quaternion((q.y, -q.z, 0.0, 0.0))
                    else:                   # Z 轴扭曲
                        tw = Quaternion((q.x, 0.0, 0.0, -q.y))
                        
                    if tw.magnitude > 1e-8:
                        tw.normalize()
                    else:
                        tw = Quaternion((1.0, 0.0, 0.0, 0.0))
                        
                # 统一符号，防止驱动器插值跳变
                if tw.w < 0.0:
                    tw.negate()
                    
                sw = q @ tw.inverted()
                if sw.w < 0.0:
                    sw.negate()
                    
                return sw, tw
                # sw = q @ tw.inverted()
                # return sw, tw

            # 4. 执行分解
            sw_cur, tw_cur = decompose_st(dq_cur, t_axis)
            sw_tar, tw_tar = decompose_st(dq_tar, t_axis)
            
            # 5. 提取 Twist 角度 (弧度)
            def get_angle_rad(q, axis):
                if q.w < 0: q = q.copy(); q.negate()
                sin_h = Vector((q.x, q.y, q.z)).dot(axis)
                return 2.0 * math.atan2(sin_h, q.w)

            cur_a = get_angle_rad(tw_cur, t_axis)
            tar_a = get_angle_rad(tw_tar, t_axis)
            
            # 6. 计算各自的权重分量
            # --- Twist 权重 ---
            twist_w = (cur_a / tar_a) if abs(tar_a) > 1e-6 else 0.0



            
            # # --- Swing 权重四元数虚部进行向量投影 ---
            # v_cur = Vector((sw_cur.x, sw_cur.y, sw_cur.z))
            # v_tar = Vector((sw_tar.x, sw_tar.y, sw_tar.z))
            # mag_tar_sq = v_tar.length_squared
            # swing_w = (v_cur.dot(v_tar) / mag_tar_sq) if mag_tar_sq > 1e-6 else 0.0
            
            #==========提取角度与轴向的余弦投影，真实角度==========
            axis_cur = sw_cur @ t_axis
            axis_tar = sw_tar @ t_axis

            # 2. 计算真实的摆动幅度（绝对值角度，单位：度）
            swing_angle_cur = math.degrees(t_axis.angle(axis_cur))
            swing_angle_tar = math.degrees(t_axis.angle(axis_tar))

            # 3. 计算当前摆动相对于目标摆动的“正负方向”
            if swing_angle_tar > 1e-6 and swing_angle_cur > 1e-6:
                # 提取当前和目标轴在垂直于 t_axis 平面上的分量（即摆动方向向量）
                # 向量减去在 t_axis 上的投影，得到的就是平面上的二维朝向
                proj_cur = (axis_cur - t_axis * axis_cur.dot(t_axis)).normalized()
                proj_tar = (axis_tar - t_axis * axis_tar.dot(t_axis)).normalized()
                
                # 计算这两个平面向量的夹角余弦值
                cos_theta = max(-1.0, min(1.0, proj_cur.dot(proj_tar)))

                # 4. 计算 swing 权重
                # 物理意义：当前摆动在目标摆动方向上的投影长度占比
                swing_w = (swing_angle_cur / swing_angle_tar) * cos_theta
                
            else:
                swing_w = 0.0

            # 7. ⭐ 权重融合因子 (Hybrid Factor)
            # 这里的 10 度是一个甜点值，表示在目标扭转角度较小时主要依赖 Swing，较大时逐渐过渡到 Twist
            threshold = math.radians(10.0)
            twist_factor = min(1.0, abs(tar_a) / threshold)
            
            # 8. 混合并输出
            w = (1.0 - twist_factor) * swing_w + twist_factor * twist_w
            

        # --- 模式 1：基础投影映射 (欧拉空间) ---
        elif mode == 1:
            pose_vec = Vector(pose_euler)
            rest_vec = Vector(rest_euler)
            cur_vec = Vector(cur_euler)
            
            pose_dir = pose_vec - rest_vec
            cur_rel = cur_vec - rest_vec
            denom = pose_dir.dot(pose_dir)
            
            if denom < 1e-6:
                w = 1.0 if cur_rel.length < 1e-3 else 0.0
            else:
                w = cur_rel.dot(pose_dir) / denom

        # --- 模式 0：椭圆锥体衰减 (Elliptical Cone) ---
        else:
            # 1. 提取 Pose 和 Cur 的完整旋转矩阵
            pose_mat = pose_euler.to_matrix()
            cur_mat = cur_euler.to_matrix()
            
            # 2. 提取 Pose 姿态下的真实局部坐标轴
            axis_x = pose_mat @ Vector((1, 0, 0))
            axis_y = pose_mat @ Vector((0, 1, 0))
            axis_z = pose_mat @ Vector((0, 0, 1))
            
            # 3. 根据指定的 axis_idx (0=X, 1=Y, 2=Z)，将当前向量投影到真实的 Pose 轴上进行缩放
            if axis_idx == 0: # X 轴为中心
                dir_cur = cur_mat @ Vector((1, 0, 0))
                p_lat_1 = dir_cur.dot(axis_y) / max(0.01, sy)
                p_lat_2 = dir_cur.dot(axis_z) / max(0.01, sz)
                p_cen   = dir_cur.dot(axis_x)
            elif axis_idx == 1: # Y 轴为中心
                dir_cur = cur_mat @ Vector((0, 1, 0))
                p_lat_1 = dir_cur.dot(axis_x) / max(0.01, sx)
                p_lat_2 = dir_cur.dot(axis_z) / max(0.01, sz)
                p_cen   = dir_cur.dot(axis_y)
            else: # Z 轴为中心
                dir_cur = cur_mat @ Vector((0, 0, 1))
                p_lat_1 = dir_cur.dot(axis_x) / max(0.01, sx)
                p_lat_2 = dir_cur.dot(axis_y) / max(0.01, sy)
                p_cen   = dir_cur.dot(axis_z)
                
            # 4. 重组并归一化
            mod_v = Vector((p_lat_1, p_cen, p_lat_2))
            mod_v_n = mod_v.normalized()
            
            # p_cen 统一放在了 index 1 (Y) 的位置，直接取 mod_v_n[1] 计算点乘
            dot = max(-1.0, min(1.0, mod_v_n[1]))
            angle_deg = math.degrees(math.acos(dot))
            
            w = (1.0 - (angle_deg / cone_angle)) if angle_deg <= cone_angle and cone_angle > 0.0 else (1.0 if angle_deg <= 0.0 else 0.0)
            return max(0.0, min(1.0, w))

        # 统一的 0->1->2 衰减逻辑处理
        #如果启动了use_triangle_weights才是使用这种衰减方式，否则就是线性衰减
        if use_triangle_weights == 1:
            if w < 0.0 or w > 2.0: return 0.0
            return float(max(0.0, min(1.0, (2.0 - w) if w > 1.0 else w)))
        else:
            return max(0.0, min(1.0, w))
            
    except Exception as e:
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


def driver_compute_distance_trigger(
    t_wx, t_wy, t_wz,  # 触发骨骼(Trigger)的世界坐标 (变量传入)
    g_wx, g_wy, g_wz,  # 目标骨骼(Target)的世界坐标 (变量传入)
    radius,            # 判定半径
    falloff_mode=0     # 0: LINEAR (线性), 1: SMOOTH (平滑平滑)
):
    """
    计算两个世界空间点之间的距离权重
    """
    try:
        t_pos = Vector((t_wx, t_wy, t_wz))
        g_pos = Vector((g_wx, g_wy, g_wz))
        
        # 计算欧几里得距离
        dist = (t_pos - g_pos).length
        r = max(1e-6, radius)
        
        # 计算原始比例 [0, 1]
        t = min(max(dist / r, 0.0), 1.0)
        
        if falloff_mode == 1: # SMOOTH
            # 使用 SmoothStep 公式: 1 - (3t^2 - 2t^3)
            weight = 1.0 - (3.0 * t * t - 2.0 * t * t * t)
        else: # LINEAR
            weight = 1.0 - t
            
        return max(0.0, min(1.0, weight))
    except:
        return 0.0



def calc_pure_twist_from_euler(ux, uy, uz, fx, fy, fz):
    # 1. 将驱动器传进来的欧拉角（弧度）重建成四元数
    # 注意：这里的 'XYZ' 需要与你骨骼属性面板里的旋转顺规（Rotation Mode）一致
    # 如果你的骨骼是 XYZ Euler，这里就用 'XYZ'
    q_u = Euler((ux, uy, uz), 'YZX').to_quaternion()
    q_f = Euler((fx, fy, fz), 'YZX').to_quaternion()
    
    # 2. 计算相对四元数 Q_rel = Q_upper^-1 * Q_forearm
    # 四元数的逆就是共轭 (conjugated)
    q_rel = q_u.conjugated() @ q_f
    
    # 3. 提取 Y 轴的纯 Twist 投影
    q_twist = Quaternion((q_rel.w, 0.0, q_rel.y, 0.0))
    
    # 防止极小值归一化导致除以零
    if q_twist.magnitude > 1e-6:
        q_twist.normalize()
    else:
        q_twist = Quaternion((1.0, 0.0, 0.0, 0.0))
        
    # 4. 返回带有正负号的 Twist 弧度值
    return 2.0 * math.atan2(q_twist.y, q_twist.w)


# Converts difference to a 0 to 1 range
# TO DO: Convert to Follow the rate similiar to Daz Studio
def erc_keyed(var, min, max, normalized_dist, dist):
    if dist < 0:
        if max <= var <= min:
            return abs((var - min) / dist)
        elif max >= var:
            return 1
        else:
            return 0
    if min <= var <= max:
        return abs((var - min * normalized_dist) / dist)
    elif max <= var:
        return 1
    else:
        return 0

# --- 注册到命名空间 ---
@handlers.persistent
def register_drivers(dummy):

    bpy.app.driver_namespace["psd_rot"] = driver_compute_rotation_weight
    bpy.app.driver_namespace["psd_loc"] = driver_compute_location_weight
    bpy.app.driver_namespace["psd_sca"] = driver_compute_scale_weight
    bpy.app.driver_namespace["psd_twist"] = calc_pure_twist_from_euler
    bpy.app.driver_namespace["psd_trig"] = driver_compute_distance_trigger
    bpy.app.driver_namespace["erc_keyed"] = erc_keyed