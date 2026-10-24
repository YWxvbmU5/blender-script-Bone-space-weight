import math
from mathutils import Vector, Euler, Quaternion

# 根据欧拉角（度）和指定轴向计算方向向量 ===============================
def _euler_deg_to_dir(rot_deg, axis='Z'):
    
    ex, ey, ez = rot_deg
    e = Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ')
    mat = e.to_matrix()
    if axis == 'X':
        base = Vector((1.0, 0.0, 0.0))
    elif axis == 'Y':
        base = Vector((0.0, 1.0, 0.0))
    else:
        base = Vector((0.0, 0.0, 1.0))
    dir_vec = mat @ base
    if dir_vec.length == 0:
        return Vector((0.0, 0.0, 1.0))
    return dir_vec.normalized()

# 将欧拉角（度）转换为四元数 ==========================================
def _euler_deg_to_quat(rot_deg):
    
    ex, ey, ez = rot_deg
    return Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ').to_quaternion()

# 将旋转四元数分解为摆动（Swing）和扭转（Twist）两个分量 ================
# def _swing_twist_decompose(q: Quaternion, twist_axis: Vector):
    
#     if twist_axis.length == 0:
#         return q.copy(), Quaternion((1.0, 0.0, 0.0, 0.0))
#     axis = twist_axis.normalized()
#     q_vec = Vector((q.x, q.y, q.z))
#     proj = axis * q_vec.dot(axis)
#     twist = Quaternion((q.w, proj.x, proj.y, proj.z))
#     try:
#         twist.normalize()
#     except Exception:
#         twist = Quaternion((1.0, 0.0, 0.0, 0.0))
#     swing = q @ twist.inverted()
#     return swing, twist

# def _swing_twist_decompose(q: Quaternion, axis: Vector):
#     if axis.length == 0:
#         return q.copy(), Quaternion((1,0,0,0))

#     axis = axis.normalized()

#     # 👉 提取旋转轴（更稳定）
#     rot_axis = Vector((q.x, q.y, q.z))

#     if rot_axis.length < 1e-8:
#         # 几乎没有旋转
#         return Quaternion((1,0,0,0)), q.copy()

#     # 👉 twist 角度 = 投影后的轴
#     proj = axis * rot_axis.dot(axis)

#     twist = Quaternion((q.w, proj.x, proj.y, proj.z))
    
#     if twist.magnitude < 1e-8:
#         twist = Quaternion((1,0,0,0))
#     else:
#         twist.normalize()

#     # ✅ 关键：统一符号（防跳变）
#     if twist.w < 0:
#         twist = -twist

#     swing = q @ twist.inverted()

#     # 同样处理 swing
#     if swing.w < 0:
#         swing = -swing

#     return swing, twist
def _swing_twist_decompose(q: Quaternion, axis: Vector):
    axis = axis.normalized()
    p = Vector((q.x, q.y, q.z))
    proj = axis * p.dot(axis)
    twist = Quaternion((q.w, proj.x, proj.y, proj.z))
    
    # 【核心修复】：将阈值从 1e-8 提高到 1e-4！
    # 用于吸收 Matrix -> Euler -> Quat 转换时，180度变成 179.9999度 产生的 Gimbal Lock 浮点误差
    if twist.magnitude > 1e-4:
        twist.normalize()
    else:
        # 触发奇点反推逻辑 (Singularity)
        # 利用非扭曲轴的正交分量来硬性构建 Twist，锁定 Swing 通道
        if abs(axis.y) > 0.5:   # Y 轴扭曲
            twist = Quaternion((q.x, 0.0, q.z, 0.0))
        elif abs(axis.x) > 0.5: # X 轴扭曲
            twist = Quaternion((q.y, -q.z, 0.0, 0.0))
        else:                   # Z 轴扭曲
            twist = Quaternion((q.x, 0.0, 0.0, -q.y))
            
        if twist.magnitude > 1e-8:
            twist.normalize()
        else:
            twist = Quaternion((1.0, 0.0, 0.0, 0.0))
            
    # 统一符号，防止驱动器插值跳变
    if twist.w < 0.0:
        twist.negate()
        
    swing = q @ twist.inverted()
    if swing.w < 0.0:
        swing.negate()
        
    return swing, twist


# 从四元数中提取绕特定轴的有符号旋转角度 ================================
def _signed_angle_from_quat_acos(q: Quaternion, axis: Vector):
    
    q = q.copy()
    if q.w < 0:
        q.negate()
    theta = 2.0 * math.acos(q.w)
    vec = Vector((q.x, q.y, q.z))
    sign = 1.0 if vec.dot(axis.normalized()) >= 0 else -1.0
    return sign * math.degrees(theta)

def _signed_angle_from_quat_atan2(q: Quaternion, axis: Vector):
    """
    使用 atan2 从四元数中提取绕特定轴的有符号旋转角度。
    
    :param q: 旋转四元数
    :param axis: 参考轴向 (Vector((1,0,0)), (0,1,0) 或 (0,0,1))
    :return: 有符号角度（度）范围 [-180, 180]
    """
    # 确保四元数处于正半球，避免双倍覆盖带来的角度翻转
    q_calc = q.copy()
    if q_calc.w < 0:
        q_calc.negate()

    # 将四元数虚部向量投影到目标轴上，得到 sin(theta/2) 的缩放值
    q_vec = Vector((q_calc.x, q_calc.y, q_calc.z))
    # axis.normalized().dot(q_vec) 提取了 sin(theta/2) 在该轴上的分量
    sin_half = q_vec.dot(axis.normalized())
    cos_half = q_calc.w

    # 使用 atan2 计算 theta/2，结果范围 [-pi, pi]
    angle_rad = 2.0 * math.atan2(sin_half, cos_half)

    # 将弧度转换为角度，并规范化到 [-180, 180] 之间
    angle_deg = math.degrees(angle_rad)
    
    if angle_deg > 180.0:
        angle_deg -= 360.0
    elif angle_deg < -180.0:
        angle_deg += 360.0
        
    return angle_deg



# 计算当前值相对于目标值的三角形映射比例（暂时没用） =====================
def _triangular_ratio(cur, target):
    
    if target is None:
        return 0.0
    if abs(target) < 1e-6:
        return 1.0 if abs(cur) < 1e-3 else 0.0
    r = float(cur) / float(target)
    if r < 0.0:
        return 0.0
    if r > 2.0:
        return 0.0
    if r > 1.0:
        return 2.0 - r
    return r

#==========权重算法================================
#RBF
# def compute_rbf_weights_batch(current_q: Quaternion, sample_qs: list, epsilons: list):
#     """
#     RBF 批处理计算：输入当前姿态和一组样本，返回归一化的权重列表
#     """
#     if not sample_qs:
#         return []
    
#     weights = []
#     for i, q_s in enumerate(sample_qs):
#         # 计算四元数之间的夹角距离 (弧度)
#         # Quaternion.angle 算出的是两个旋转之间的最短弧度差
#         dist = current_q.angle(q_s)
        
#         # 高斯核计算: exp(-(eps * dist)^2)
#         # eps 越大，在这个 Pose 附近的触发越敏锐（范围越小）
#         eps = epsilons[i]
#         w = math.exp(-(eps * dist)**2)
#         weights.append(w)
    
#     # 归一化：保证所有 RBF 姿态总和为 1.0，防止形变叠加过度
#     total = sum(weights)
#     if total > 1e-6:
#         return [w / total for w in weights]
    
#     return [0.0] * len(weights)

# def compute_rbf_weights_interpolation(q_cur, entries, matrix_lambda):
#     """
#     代替原有的归一化计算，使用预计算的系数矩阵进行插值
#     """
#     import math
#     n = len(entries)
#     if not matrix_lambda or len(matrix_lambda) != n:
#         return [0.0] * n

#     # 1. 计算当前的基础核函数值向量 (Phi)
#     phi_vector = []
#     for e in entries:
#         q_pose = _euler_deg_to_quat(e.pose_rot)
#         # 记得带上你之前的 Swing-Twist 距离逻辑
#         dist = q_cur.rotation_difference(q_pose).angle 
        
#         eps = getattr(e, 'rbf_epsilon', 1.0)
#         phi = math.exp(-((dist * eps) ** 2))
#         phi_vector.append(phi)

#     # 2. 线性组合：利用系数矩阵计算最终权重
#     results = []
#     for i in range(n):
#         # 第 i 个姿态的权重 = 系数行[i] 点乘 Phi向量
#         row_coeffs = matrix_lambda[i]
#         w = sum(row_coeffs[j] * phi_vector[j] for j in range(n))
        
#         # 工业级保险：钳制到 0-1
#         results.append(max(0.0, min(1.0, w)))
        
#     return results



# 计算指定通道（X/Y/Z）的直接旋转权重，支持 Swing/Twist 分解=============
def compute_direct_channel_weight(
    entry, bone_to_cur_rot, arm, bn,
    psd_set_result_cache_only, PREFIX_RESULT, _safe_name
):
    """计算 Direct Channel 权重并写入缓存"""
    try:
        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        cur_rot = bone_to_cur_rot.get(bn)

        if cur_rot is None:
            psd_set_result_cache_only(arm, key_rot, 0.0, verbose=False)
            return

        mode = getattr(entry, 'record_rot_channel_mode', 'NONE')
        ch_axis = getattr(entry, 'channel_axis', 'X')
        axis_idx_map = {'X': 0, 'Y': 1, 'Z': 2}
        axis_idx = axis_idx_map.get(ch_axis, 0)
        cur_rot_value = float(cur_rot[axis_idx])
        q_cur = _euler_deg_to_quat(tuple(cur_rot))

        w_deg = cur_rot_value

        if mode in ('record_rot_SWING_X_TWIST', 'record_rot_SWING_Y_TWIST', 'record_rot_SWING_Z_TWIST'):
            if mode == 'record_rot_SWING_X_TWIST':
                twist_axis = Vector((1.0, 0.0, 0.0))
            elif mode == 'record_rot_SWING_Y_TWIST':
                twist_axis = Vector((0.0, 1.0, 0.0))
            else:
                twist_axis = Vector((0.0, 0.0, 1.0))

            swing_q, twist_q = _swing_twist_decompose(q_cur, twist_axis)

            def _clamp1(v):
                if v is None:
                    return 0.0
                v = float(v)
                return max(-1.0, min(1.0, v))

            if ch_axis == 'X':
                if mode == 'record_rot_SWING_X_TWIST':
                    w_deg = _signed_angle_from_quat_atan2(twist_q, twist_axis)
                else:
                    sx = _clamp1(swing_q.x)
                    w_deg = math.degrees(2.0 * math.asin(sx))
            elif ch_axis == 'Y':
                if mode == 'record_rot_SWING_Y_TWIST':
                    w_deg = _signed_angle_from_quat_atan2(twist_q, twist_axis)
                else:
                    sy = _clamp1(swing_q.y)
                    w_deg = math.degrees(2.0 * math.asin(sy))
            elif ch_axis == 'Z':
                if mode == 'record_rot_SWING_Z_TWIST':
                    w_deg = _signed_angle_from_quat_atan2(twist_q, twist_axis)
                else:
                    sz = _clamp1(swing_q.z)
                    w_deg = math.degrees(2.0 * math.asin(sz))

        w = math.radians(w_deg)
        if math.isnan(w):
            w = 0.0
        psd_set_result_cache_only(arm, key_rot, w, verbose=False)

    except Exception as e:
        print(f"[Direct Channel] 计算出错 {entry.name}: {e}")
        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_rot, 0.0, verbose=False)

# 根据骨骼旋转计算权重，支持圆锥区域（Cone）判定或相对旋转偏移计算=========
# def compute_rotation_weight(
#     entry, bone_to_cur_rot, bn, arm,
#     psd_set_result_cache_only, PREFIX_RESULT, _safe_name
# ):
#     """计算 Rotation Channel 权重并写入缓存"""
#     try:
#         cur_rot = bone_to_cur_rot.get(bn)
#         if cur_rot is not None:
#             if getattr(entry, 'cone_enabled', False):
#                 dir_center = _euler_deg_to_dir(entry.pose_rot, entry.cone_axis)
#                 dir_cur = _euler_deg_to_dir(tuple(cur_rot), entry.cone_axis)
#                 dot = max(-1.0, min(1.0, dir_center.dot(dir_cur)))
#                 angle_rad = math.acos(dot)
#                 angle_deg = math.degrees(angle_rad)
#                 if angle_deg <= entry.cone_angle:
#                     w = 1.0 - (angle_deg / entry.cone_angle) if entry.cone_angle > 0.0 else 1.0
#                 else:
#                     w = 0.0
#             else:
#                 pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
#                 cur_rel = Vector(cur_rot) - Vector(entry.rest_rot)
#                 denom = pose_dir.dot(pose_dir)
#                 if denom < 1e-6:
#                     w = 1.0 if cur_rel.length < 1e-3 else 0.0
#                 else:
#                     w = cur_rel.dot(pose_dir) / denom
#                     if w < 0.0:
#                         w = 0.0
#                     elif w > 2.0:
#                         w = 0.0
#                     elif w > 1.0:
#                         w = 2.0 - w
#             if math.isnan(w):
#                 w = 0.0
#             w = max(0.0, min(1.0, w))
#         else:
#             w = 0.0

#         key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
#         psd_set_result_cache_only(arm, key_rot, w, verbose=False)

#     except Exception as e:
#         print(f"[Rotation Channel] 计算出错 {entry.name}: {e}")
#RBF
def compute_rbf_group_weights(current_rot, entries, psd_set_result_cache_only, arm, bn, _safe_name, PREFIX_RESULT):
    """
    工业级 RBF 解算器：使用矩阵插值（Matrix Interpolation）实现精准权重
    支持 Swing-Twist 分解与预计算系数点积
    """
    
    # --- 1. 尝试获取预计算的系数矩阵 (核心) ---
    def get_rbf_matrix(arm, bone_name):
    # 这里的查找逻辑要和 operators 存的时候完全对应
        storage = next((s for s in arm.psd_rbf_storage if s.bone_name == bone_name), None)
        if not storage or not storage.coeffs:
            return None
        
        try:
            # 将存储的所有行重新组装成矩阵
            return [[float(v) for v in row.values.split(",")] for row in storage.coeffs]
        except:
            return None

    try:
        n = len(entries)
        q_cur = _euler_deg_to_quat(current_rot)
        
        # 获取系数仓库
        matrix_lambda = get_rbf_matrix(arm, bn)
        
        # 计算所有样本点的基函数值 (Phi Vector)
        phi_vector = []

        for j, entry_j in enumerate(entries):

            q_pose = _euler_deg_to_quat(entry_j.pose_rot)

            mode = getattr(entry_j, 'record_rot_channel_mode', 'NONE')

            # 距离逻辑必须与 Operator 完全一致
            if mode != 'NONE' :

                if 'SWING_X' in mode:
                    t_axis = Vector((1.0, 0.0, 0.0))
                elif 'SWING_Z' in mode:
                    t_axis = Vector((0.0, 0.0, 1.0))
                else:
                    t_axis = Vector((0.0, 1.0, 0.0))

                s1, _ = _swing_twist_decompose(q_cur, t_axis)
                s2, _ = _swing_twist_decompose(q_pose, t_axis)

                dist = s1.rotation_difference(s2).angle

            else:
                dist = q_cur.rotation_difference(q_pose).angle

            eps = float(getattr(entry_j, 'rbf_epsilon', 1.0))

            phi = math.exp(-((dist * eps) ** 2))

            phi_vector.append(phi)

        # --- 2. 逻辑分流：精准路径 vs 兼容路径 ---
        
        # A. 开启【精准路径】：如果矩阵存在且样本数匹配
        if matrix_lambda and len(matrix_lambda) == n:
            # print(f"DEBUG:bone '{bn}' 存储数={len(matrix_lambda) if matrix_lambda else 0}, 当前Entry数={n}")
            for i, entry in enumerate(entries):
                # 核心点积公式：Weight = Σ (Lambda_i_j * Phi_j)
                # print("[RBF] Using precise matrix interpolation for bone '{}', entry '{}', phi_vector: {}".format(bn, entry.name, ["{:.4f}".format(p) for p in phi_vector]))
                row_coeffs = matrix_lambda[i]

                # print(f"DEBUG: Row_Coeffs: {row_coeffs}")
                # print(f"DEBUG: Phi_Vector: {phi_vector}")

                final_w = sum(row_coeffs[j] * phi_vector[j] for j in range(n))
                
                # 线性插值会有极微小的浮点溢出，做 0-1 钳制
                final_w = max(0.0, min(1.0, final_w))
                
                key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
                psd_set_result_cache_only(arm, key, final_w, verbose=False)
                
        # B. 开启【兼容路径】：作为保底，防止用户没点 Update 导致形变失效
        else:
            # print(f"⚠️ [RBF Warning] Bone '{bn}' has {n} entries but no valid coefficient matrix found. Falling back to normalized weights. Please use 'Update RBF Solver' after modifying poses.")
            total_weight = sum(phi_vector)
            for i, entry in enumerate(entries):
                final_w = phi_vector[i] / total_weight if total_weight > 1e-6 else 0.0
                final_w = max(0.0, min(1.0, final_w))
                
                key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
                psd_set_result_cache_only(arm, key, final_w, verbose=False)

    except Exception as e:
        print(f"🌚 [RBF Precise Kernel Error] Bone: {bn} | Info: {e}")
#
def compute_rotation_weight(
    entry, bone_to_cur_rot, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT, _safe_name
):
    """计算 Rotation Channel 权重（支持非均匀缩放椭圆锥）"""
    try:
        cur_rot = bone_to_cur_rot.get(bn)
        if cur_rot is not None:
            if getattr(entry, 'cone_enabled', False):
                # 1. 构建欧拉角，并直接转换为矩阵
                pose_mat = Euler((math.radians(entry.pose_rot[0]), math.radians(entry.pose_rot[1]), math.radians(entry.pose_rot[2])), 'XYZ').to_matrix()
                cur_mat = Euler((math.radians(cur_rot[0]), math.radians(cur_rot[1]), math.radians(cur_rot[2])), 'XYZ').to_matrix()
                
                # 2. 从 Pose 矩阵中提取真实的局部目标坐标系
                axis_x = pose_mat @ Vector((1, 0, 0))
                axis_y = pose_mat @ Vector((0, 1, 0))
                axis_z = pose_mat @ Vector((0, 0, 1))
                
                # 3. 提取UI设定的缩放系数
                sx = getattr(entry, "cone_scale_x", 1.0)
                sy = getattr(entry, "cone_scale_y", 1.0)
                sz = getattr(entry, "cone_scale_z", 1.0)
                
                # 4. 根据设定的主轴，直接向真实局部轴投影并进行非均匀缩放
                if entry.cone_axis == 'X':
                    dir_cur = cur_mat @ Vector((1, 0, 0))
                    p_lat_1 = dir_cur.dot(axis_y) / max(0.01, sy)
                    p_lat_2 = dir_cur.dot(axis_z) / max(0.01, sz)
                    p_cen   = dir_cur.dot(axis_x)
                elif entry.cone_axis == 'Y':
                    dir_cur = cur_mat @ Vector((0, 1, 0))
                    p_lat_1 = dir_cur.dot(axis_x) / max(0.01, sx)
                    p_lat_2 = dir_cur.dot(axis_z) / max(0.01, sz)
                    p_cen   = dir_cur.dot(axis_y)
                else: # Z
                    dir_cur = cur_mat @ Vector((0, 0, 1))
                    p_lat_1 = dir_cur.dot(axis_x) / max(0.01, sx)
                    p_lat_2 = dir_cur.dot(axis_y) / max(0.01, sy)
                    p_cen   = dir_cur.dot(axis_z)
                    
                # 5. 重组局部向量并归一化
                mod_v = Vector((p_lat_1, p_cen, p_lat_2))
                mod_v_n = mod_v.normalized()
                
                # 中心轴分量已统一放在了 Y (index 1) 的位置
                dot = max(-1.0, min(1.0, mod_v_n[1]))
                angle_deg = math.degrees(math.acos(dot))
                
                # 6. 计算最终衰减权重
                if angle_deg <= entry.cone_angle:
                    w = 1.0 - (angle_deg / entry.cone_angle) if entry.cone_angle > 0.0 else 1.0
                else:
                    w = 0.0

            else:
                # --- 增强型投影映射模式 ---
                mode = getattr(entry, 'record_rot_channel_mode', 'NONE')
                use_triangle_weights = getattr(entry, 'use_triangle_weights', False)
                
                # 如果模式为 NONE，走原有的欧拉向量投影逻辑
                if mode == 'NONE':
                    pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
                    cur_rel = Vector(cur_rot) - Vector(entry.rest_rot)
                    denom = pose_dir.dot(pose_dir)
                    if denom < 1e-6:
                        w = 1.0 if cur_rel.length < 1e-3 else 0.0
                    else:
                        w = cur_rel.dot(pose_dir) / denom

                    if use_triangle_weights:
                        if w < 0.0 or w > 2.0:
                            w = 0.0
                        elif w > 1.0:
                            w = 2.0 - w
                    else:
                        w
                
                # 如果开启了 Swing/Twist 分解模式
                else:
                    # 1. 确定扭转轴
                    if mode == 'record_rot_SWING_X_TWIST':
                        t_axis = Vector((1.0, 0.0, 0.0))
                    elif mode == 'record_rot_SWING_Y_TWIST':
                        t_axis = Vector((0.0, 1.0, 0.0))
                    else:
                        t_axis = Vector((0.0, 0.0, 1.0))

                    # 2. 将所有欧拉角转为四元数进行空间分解
                    q_cur = _euler_deg_to_quat(cur_rot)   
                    q_pose = _euler_deg_to_quat(entry.pose_rot)
                    q_rest = _euler_deg_to_quat(entry.rest_rot)

                    q_rest_inv = q_rest.inverted() 
                    # 3. 计算相对于静止姿态的增量旋转
                    dq_cur = q_cur @ q_rest_inv
                    # dq_target = 目标相对于 rest 的变化
                    dq_target = q_pose @ q_rest_inv

                    # 4. 执行分解
                    # 注意：必须同时接收 swing 和 twist
                    swing_cur, twist_cur = _swing_twist_decompose(dq_cur, t_axis)
                    swing_tar, twist_tar = _swing_twist_decompose(dq_target, t_axis)

                    # --- 5. 提取角度并计算 ---
                    # 现在 twist_cur 是真正的 Quaternion 对象
                    cur_angle = _signed_angle_from_quat_atan2(twist_cur, t_axis)
                    tar_angle = _signed_angle_from_quat_atan2(twist_tar, t_axis)

                    # 【Swing 部分：改为真实物理夹角】
                    # 1. 计算扭曲轴在当前和目标摆动下的实际朝向
                    axis_cur = swing_cur @ t_axis
                    axis_tar = swing_tar @ t_axis

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


                    # 利用四元数的虚部向量来代表摆动方向和强度
                    # v_cur = Vector((swing_cur.x, swing_cur.y, swing_cur.z))
                    # v_tar = Vector((swing_tar.x, swing_tar.y, swing_tar.z))

                    # dot_swing = v_cur.dot(v_tar)
                    # mag_tar_sq = v_tar.length_squared
                    
                    # # --- swing 权重 ---
                    # if mag_tar_sq > 1e-6:
                    #     swing_w = dot_swing / mag_tar_sq
                    # else:
                    #     swing_w = 0.0

                    # --- twist 权重 ---
                    if abs(tar_angle) > 1e-6:
                        twist_w = cur_angle / tar_angle
                    else:
                        twist_w = 0.0

                    # --- ：连续权重混合 ---
                    # 根据目标姿态中 twist 的“占比”决定权重
                    twist_factor = min(1.0, abs(tar_angle) / 10.0)
                    #  10° 是经验值，可调

                    w = (1.0 - twist_factor) * swing_w + twist_factor * twist_w

                    # 6. 可选的三角形权重映射
                    if use_triangle_weights:
                        if w < 0.0 or w > 2.0:
                            w = 0.0
                        elif w > 1.0:
                            w = 2.0 - w
                    else:
                        w

        
            if math.isnan(w): w = 0.0
            w = max(0.0, min(1.0, w))
        else:
            w = 0.0

        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_rot, w, verbose=False)

    except Exception as e:
        print(f"🌚 [Rotation] 计算出错 {entry.name}: {e}")

# 根据骨骼位移计算权重，支持空间半径判定或相对位移投影计算=================
def compute_location_weight(
    entry, bone_to_cur_loc, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
):
    """计算 Location Channel 权重并写入缓存"""
    try:
        cur_loc = bone_to_cur_loc.get(bn)
        if cur_loc is not None:
            center = Vector(entry.pose_loc)
            radius = float(getattr(entry, 'loc_radius', 0.0) or 0.0)
            if getattr(entry, 'loc_enabled', False):
                if radius <= 0.0:
                    w_loc = 1.0 if (cur_loc - center).length < 1e-6 else 0.0
                else:
                    d = cur_loc - center
                    wx = max(0.0, 1.0 - abs(d.x) / radius)
                    wy = max(0.0, 1.0 - abs(d.y) / radius)
                    wz = max(0.0, 1.0 - abs(d.z) / radius)
                    w_loc = wx * wy * wz
            else:
                w_loc = 1.0
                rest_vec = Vector(getattr(entry, 'rest_loc', (0.0, 0.0, 0.0)))
                pose_vec = Vector(entry.pose_loc)
                direction = pose_vec - rest_vec
                len2 = direction.length_squared
                if len2 < 1e-12:
                    w_loc = 1.0 if (cur_loc - rest_vec).length < 1e-6 else 0.0
                else:
                    t = (cur_loc - rest_vec).dot(direction) / len2
                    if math.isnan(t):
                        t = 0.0
                    if t < 0.0 or t > 2.0:
                        w_loc = 0.0
                    elif t > 1.0:
                        w_loc = 2.0 - t
                    else:
                        w_loc = t
                    w_loc = max(0.0, min(1.0, w_loc))
            if math.isnan(w_loc):
                w_loc = 0.0
            w_loc = max(0.0, min(1.0, w_loc))
        else:
            w_loc = 0.0

        key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_loc, w_loc, verbose=False)

    except Exception as e:
        print(f"[Location Channel] 计算出错 {entry.name}: {e}")

# 根据骨骼缩放比例计算权重==============================================
def compute_scale_weight(
    entry, bone_to_cur_sca, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT_SCA, _safe_name
):
    """计算 Scale Channel 权重并写入缓存"""
    try:
        cur_sca = bone_to_cur_sca.get(bn)
        if cur_sca is not None:
            rest_vec = Vector(getattr(entry, 'rest_sca', (1.0, 1.0, 1.0)))
            pose_vec = Vector(entry.pose_sca)
            direction = pose_vec - rest_vec
            len2 = direction.length_squared
            if len2 < 1e-12:
                w_sca = 1.0 if (cur_sca - rest_vec).length < 1e-6 else 0.0
            else:
                t = (cur_sca - rest_vec).dot(direction) / len2
                if math.isnan(t):
                    t = 0.0
                if t < 0.0 or t > 2.0:
                    w_sca = 0.0
                elif t > 1.0:
                    w_sca = 2.0 - t
                else:
                    w_sca = t
                w_sca = max(0.0, min(1.0, w_sca))
            if math.isnan(w_sca):
                w_sca = 0.0
        else:
            w_sca = 0.0

        key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_sca, w_sca, verbose=False)

    except Exception as e:
        print(f"[Scale Channel] 计算出错 {entry.name}: {e}")

# 计算骨骼间的距离触发器权重，支持平滑衰减================================
# def compute_triggers(
#     orig_arm,
#     psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
# ):
#     """计算所有 Triggers 并写入缓存"""
#     if not getattr(orig_arm, "psd_triggers", None):
#         return

#     for trig in orig_arm.psd_triggers:
#         if not trig.enabled:
#             continue
#         trig.last_weight = 0.0
#         pb_trigger = orig_arm.pose.bones.get(trig.bone_name)
#         pb_target = orig_arm.pose.bones.get(trig.target_bone)
#         if not pb_trigger or not pb_target:
#             continue
#         try:
#             head_trigger_world = orig_arm.matrix_world @ pb_trigger.head
#             head_target_world = orig_arm.matrix_world @ pb_target.head
#         except Exception:
#             head_trigger_world = orig_arm.matrix_world @ pb_trigger.bone.head_local
#             head_target_world = orig_arm.matrix_world @ pb_target.bone.head_local

#         d = (head_target_world - head_trigger_world).length
#         r = max(1e-6, float(trig.radius))
#         if trig.falloff == 'SMOOTH':
#             t = min(max(d / r, 0.0), 1.0)
#             w = 1.0 - (3.0 * t * t - 2.0 * t * t * t)
#         else:
#             w = max(0.0, min(1.0, 1.0 - d / r))
#         trig.last_weight = w
#         key_base = f"{PREFIX_RESULT_LOC}{_safe_name(trig.target_bone)}_{_safe_name(trig.name)}"
#         psd_set_result_cache_only(orig_arm, f"{key_base}_w", float(w), verbose=False)

def compute_triggers(
    orig_arm,
    psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
):
    """计算所有 Triggers 并写入缓存"""
    if not getattr(orig_arm, "psd_triggers", None):
        return

    for trig in orig_arm.psd_triggers:
        if not trig.enabled:
            continue
        
        trig.last_weight = 0.0
        pb_trigger = orig_arm.pose.bones.get(trig.bone_name)
        pb_target = orig_arm.pose.bones.get(trig.target_bone)
        
        if not pb_trigger or not pb_target:
            continue
            
        try:
            head_trigger_world = orig_arm.matrix_world @ pb_trigger.head
            head_target_world = orig_arm.matrix_world @ pb_target.head
        except Exception:
            head_trigger_world = orig_arm.matrix_world @ pb_trigger.bone.head_local
            head_target_world = orig_arm.matrix_world @ pb_target.bone.head_local

        d = (head_target_world - head_trigger_world).length
        r = max(1e-6, float(trig.radius))
        
        if trig.falloff == 'SMOOTH':
            t = min(max(d / r, 0.0), 1.0)
            w = 1.0 - (3.0 * t * t - 2.0 * t * t * t)
        else:
            w = max(0.0, min(1.0, 1.0 - d / r))
            
        trig.last_weight = w

        # --- 核心修改部分：更新命名规则 ---
        # 目标：psd_loc_目标骨骼_to_触发骨骼_触发器名
        s_target = _safe_name(trig.target_bone)
        s_trigger = _safe_name(trig.bone_name)
        s_name = _safe_name(trig.name)
        
        # 拼接新的 key_base
        key_base = f"{PREFIX_RESULT_LOC}{s_target}_to_{s_trigger}_{s_name}"
        # --------------------------------

        # 写入缓存 (带上 _w 后缀)
        psd_set_result_cache_only(orig_arm, f"{key_base}_w", float(w), verbose=False)