import bpy
import gpu
import math
from .core import _psd_results_cache, _psd_perf_stats
from mathutils import Matrix, Vector, Euler
from gpu_extras.batch import batch_for_shader

_draw_handler = None
_shader = gpu.shader.from_builtin('POLYLINE_UNIFORM_COLOR')
_UNIT_RAY_VERTS = [Vector((0, 0, 0)), Vector((0, 1, 0))]

# 预生成球体线框数据，避免每帧重复计算
def generate_sphere_wireframe(segments=16, rings=8):
    vertices = []
    for ring in range(rings + 1):
        theta = math.pi * ring / rings
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        for seg in range(segments):
            phi = 2 * math.pi * seg / segments
            x = sin_theta * math.cos(phi)
            y = sin_theta * math.sin(phi)
            z = cos_theta
            vertices.append(Vector((x, y, z)))
    return vertices

def generate_sphere_indices(segments, rings):
    indices = []
    for ring in range(rings + 1):
        for seg in range(segments):
            i1 = ring * segments + seg
            i2 = ring * segments + (seg + 1) % segments
            indices.append((i1, i2))
    for seg in range(segments):
        for ring in range(rings):
            i1 = ring * segments + seg
            i2 = (ring + 1) * segments + seg
            indices.append((i1, i2))
    return indices

SPHERE_VERTS = generate_sphere_wireframe()
SPHERE_INDS = generate_sphere_indices(16, 8)


def _generate_cone_geometry(angle_deg, axis='Y', length=0.5, segments=32):
    """生成圆锥的线框数据 (Apex at 0,0,0)"""
    # 限制角度避免数学错误
    angle_deg = max(0.1, min(179.9, angle_deg))
    angle_rad = math.radians(angle_deg)
    
    # 计算底面半径
    # 如果角度很大，限制一下显示长度，避免无穷大
    if angle_deg >= 89.9:
        radius = length * 5 # 最大半径
        height = 0.0
    else:
        radius = length * math.tan(angle_rad)
        height = length

    verts = []
    
    # 1. 确定基础轴向的圆环点
    # 默认生成在 +Y 轴 (Blender 骨骼主轴)
    # 如果 axis 是 X 或 Z，稍后旋转
    ring_verts = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        # 在 XZ 平面上画圆，高度在 Y
        x = radius * math.cos(theta)
        z = radius * math.sin(theta)
        ring_verts.append(Vector((x, height, z)))
    
    # 2. 处理轴向旋转
    rot_mat = Matrix.Identity(3)
    if axis == 'X':
        # Y -> X: Rotate -90 around Z
        rot_mat = Euler((0, 0, math.radians(-90)), 'XYZ').to_matrix()
    elif axis == 'Z':
        # Y -> Z: Rotate +90 around X
        rot_mat = Euler((math.radians(90), 0, 0), 'XYZ').to_matrix()
    
    final_ring = [rot_mat @ v for v in ring_verts]
    apex = Vector((0,0,0))

    # 3. 构建线段列表 (Lines)
    draw_verts = []
    # 3.1 Apex 到 圆环
    for v in final_ring:
        draw_verts.append(apex)
        draw_verts.append(v)
    
    # 3.2 圆环封闭
    for i in range(segments):
        draw_verts.append(final_ring[i])
        draw_verts.append(final_ring[(i + 1) % segments])

    # 3.3 轴心线 (可选)
    center_end = rot_mat @ Vector((0, height, 0))
    draw_verts.append(apex)
    draw_verts.append(center_end)
    
    return draw_verts

def _generate_spherical_sector_geometry(angle_deg, axis='Y', length=0.5, segments=32, scales=(1.0, 1.0, 1.0)):
    """
    生成支持非均匀缩放的球扇形线框
    scales: (sx, sy, sz)
    """
    angle_rad = math.radians(angle_deg)
    angle_rad = max(0.01, min(math.pi, angle_rad))
    
    draw_verts = []
    apex = Vector((0, 0, 0))
    
    # 1. 基础旋转矩阵 (对齐轴向)
    rot_mat = Matrix.Identity(3)
    if axis == 'X':
        rot_mat = Euler((0, 0, math.radians(-90)), 'XYZ').to_matrix()
    elif axis == 'Z':
        rot_mat = Euler((math.radians(90), 0, 0), 'XYZ').to_matrix()

    # --- 新增：构造缩放矩阵 ---
    # 注意：这里的缩放是在旋转之后应用的，以确保缩放轴与 cone_axis 对齐
    scale_mat = Matrix.Diagonal(Vector(scales)).to_3x3()
    # 最终变换矩阵 (局部)
    combined_mat = scale_mat @ rot_mat

    # 2. 生成底部的圆环
    ring_radius = length * math.sin(angle_rad)
    ring_height = length * math.cos(angle_rad)
    
    ring_points = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        p = Vector((ring_radius * math.cos(theta), ring_height, ring_radius * math.sin(theta)))
        # 应用旋转和缩放
        ring_points.append(combined_mat @ p)

    # 绘制圆环封闭线
    for i in range(segments):
        draw_verts.append(ring_points[i])
        draw_verts.append(ring_points[(i + 1) % segments])

    # 3. 生成经线 (Radiating Lines)
    radial_steps = 8 
    for i in range(radial_steps):
        theta = 2.0 * math.pi * i / radial_steps
        target_p = Vector((ring_radius * math.cos(theta), ring_height, ring_radius * math.sin(theta)))
        draw_verts.append(apex)
        draw_verts.append(combined_mat @ target_p)

    # 4. 侧边弧线 (支持非均匀缩放后，弧线会变成椭圆弧)
    if angle_deg > 59:
        arc_segments = 16
        for r in [0, math.pi/2, math.pi, math.pi*1.5]: 
            last_p = None
            for s in range(arc_segments + 1):
                t = angle_rad * s / arc_segments
                p = Vector((length * math.sin(t) * math.cos(r), 
                            length * math.cos(t), 
                            length * math.sin(t) * math.sin(r)))
                curr_p = combined_mat @ p
                if last_p:
                    draw_verts.append(last_p)
                    draw_verts.append(curr_p)
                last_p = curr_p

    # 5. 轴心线
    center_end = combined_mat @ Vector((0, length, 0))
    draw_verts.append(apex)
    draw_verts.append(center_end)

    return draw_verts
# =========================================================================


def draw_psd_visualizers():

    # gpu.matrix.reset()

    context = bpy.context
    scene = context.scene
    if not getattr(scene, "psd_show_saved_poses", True):
        return

    obj = context.active_object
    if not obj or obj.type != 'ARMATURE':
        return

    obj_scale = (obj.scale[0] + obj.scale[1] + obj.scale[2]) / 3.0

    # 1. 在循环外获取一次缓存，避免重复计算指针和查找字典
    from .utils import _arm_key_for_obj, _safe_name, PREFIX_RESULT, PREFIX_RESULT_LOC
    arm_key = _arm_key_for_obj(obj)
    arm_cache = _psd_results_cache.get(arm_key, {})

    gpu.state.depth_test_set('ALWAYS')     # 关键：设置为 ALWAYS，无论深度如何都绘制
    gpu.state.depth_mask_set(False)
    gpu.state.blend_set('ALPHA')        # 开启透明混合
    gpu.state.line_width_set(1.0)       # 强制重置线宽

    
    _shader.bind()

    obj_mat_world = obj.matrix_world  # 缓存矩阵引用

    # =========================================================================
    # 绘制触发器球体 (PSD Triggers)
    # =========================================================================
    if getattr(scene, "psd_show_triggers", True):
        for trig in obj.psd_triggers:
            # 这里的 show_sphere 对应你在 UI 里的 box.prop(trig, "show_sphere_trigger")
            # 建议在 props.py 里统一属性名为 show_sphere
            if not trig.enabled or not getattr(trig, "show_sphere", False):
                continue

            target_pb = obj.pose.bones.get(trig.target_bone)
            if not target_pb:
                continue

            # 获取触发目标骨骼的世界坐标 (通常在骨骼头部)
            target_bone_world_mat = obj_mat_world @ target_pb.matrix
            target_pos = target_bone_world_mat.to_translation()

            # 构造变换矩阵：位移到骨骼位置 @ 缩放半径
            trig_sphere_mat = Matrix.Translation(target_pos) @ Matrix.Scale(trig.radius, 4)
            
            # 变换预生成的球体顶点
            v_trig_sphere = [trig_sphere_mat @ v for v in SPHERE_VERTS]
            
            # 创建 Batch
            trig_batch = batch_for_shader(_shader, 'LINES', {"pos": v_trig_sphere}, indices=SPHERE_INDS)
            
            # 颜色逻辑：使用 trig.last_weight。权重越高越红，低权重偏青色/蓝色
            w = trig.last_weight
            r_trig = math.sin(w * math.pi / 2)
            gb_trig = math.cos(w * math.pi / 2)
            
            _shader.uniform_float("lineWidth", 1.0)
            _shader.uniform_float("color", (r_trig, gb_trig, 1.0, 0.6)) # 加上蓝色通道区分触发器和普通位置球

            trig_batch.draw(_shader)

    # =========================================================================
    for entry in obj.psd_saved_poses:
        pb = obj.pose.bones.get(entry.bone_name)
        if not pb:
            continue

        # 构造通用的 Key 前缀
        safe_bn = _safe_name(entry.bone_name)
        safe_en = _safe_name(entry.name)

        current_head_world = obj_mat_world @ pb.head

        # =========================================================================
        # 2. 绘制位置衰减球体 (Position Sphere)
        # =========================================================================
        if entry.has_loc and entry.loc_enabled and getattr(entry, 'show_sphere', False):
            # 获取位置权重
            loc_w = arm_cache.get(f"{PREFIX_RESULT_LOC}{safe_bn}_{safe_en}", 0.0)
            
            target_local_offset = Vector(entry.pose_loc)
            if pb.parent:
                mat_relative_rest = pb.parent.bone.matrix_local.inverted() @ pb.bone.matrix_local
                base_model_mat = pb.parent.matrix @ mat_relative_rest
            else:
                base_model_mat = pb.bone.matrix_local.copy()

            target_world_pos = obj_mat_world @ (base_model_mat @ target_local_offset)

            # 绘制球体
            sphere_mat = Matrix.Translation(target_world_pos) @ Matrix.Scale(entry.loc_radius * obj_scale, 4)
            v_sphere = [sphere_mat @ v for v in SPHERE_VERTS]
            
            # 注意：频繁创建 batch 比较耗能，如果顶点不变，建议预创建
            s_batch = batch_for_shader(_shader, 'LINES', {"pos": v_sphere}, indices=SPHERE_INDS)
            _shader.uniform_float("lineWidth", 1.0)
            # 变红
            r_loc = math.sin(loc_w * math.pi / 2)
            gb_loc = math.cos(loc_w * math.pi / 2)
            _shader.uniform_float("color", (r_loc, gb_loc, gb_loc, 0.8))

            s_batch.draw(_shader)

        # =========================================================================
        # 3. 绘制旋转衰减圆锥 (Rotation Cone)
        # =========================================================================
        if entry.cone_enabled and entry.show_cone:
            # 获取旋转权重
            rot_w = arm_cache.get(f"{PREFIX_RESULT}{safe_bn}_{safe_en}", 0.0)

            # 绘制指示红线 (当前朝向)
            current_tail_world = obj_mat_world @ pb.tail
            line_batch = batch_for_shader(_shader, 'LINES', {"pos": [current_head_world, current_tail_world]})
            _shader.uniform_float("lineWidth", 3.0) 
            _shader.uniform_float("color", (1.0, 0.0, 0.0, 1.0))
            line_batch.draw(_shader)

            # 获取缩放参数
            scales = (
                getattr(entry, 'cone_scale_x', 1.0),
                getattr(entry, 'cone_scale_y', 1.0),
                getattr(entry, 'cone_scale_z', 1.0)
            )

            # 构造目标时刻旋转
            target_euler = Euler([math.radians(a) for a in entry.pose_rot], 'XYZ')
            target_local_rot_mat = target_euler.to_matrix().to_4x4()

            if pb.parent:
                mat_relative = pb.parent.bone.matrix_local.inverted() @ pb.bone.matrix_local
                model_mat = pb.parent.matrix @ mat_relative @ target_local_rot_mat
            else:
                model_mat = pb.bone.matrix_local @ target_local_rot_mat

            final_cone_mat = obj_mat_world @ model_mat
            final_cone_mat.translation = current_head_world

            viz_len = pb.length * 0.6 if pb.length > 0 else 0.5
            
            # --- 调用修改后的几何生成函数，传入 scales ---
            verts = _generate_spherical_sector_geometry(
                entry.cone_angle, 
                entry.cone_axis, 
                length=viz_len, 
                scales=scales
            )
            
            world_verts = [final_cone_mat @ v for v in verts]
            cone_batch = batch_for_shader(_shader, 'LINES', {"pos": world_verts})

            _shader.uniform_float("lineWidth", 2.0)
            # 变红逻辑
            r_rot = math.sin(rot_w * math.pi / 2)
            gb_rot = math.cos(rot_w * math.pi / 2)
            _shader.uniform_float("color", (r_rot, gb_rot, gb_rot, 0.8))

            cone_batch.draw(_shader)

    # =========================================================================
    # 4. 绘制姿态旋转射线丛 (Pose Rotation Rays)
    # =========================================================================
        # 1. 它是 Scene 属性，直接在最外层（不要嵌套在圆锥的 if 里面）进行判断！
    if getattr(context.scene, "show_ray_verts", False):
        
        pb = context.active_pose_bone
        
        if pb and obj: # 确保当前选了骨骼，且物体存在
            ray_len = pb.length * 0.6 if pb.length > 0 else 0.5

            # 3. 总闸和骨骼都就位，才进来遍历所有的已保存姿态
            for sub_entry in obj.psd_saved_poses:
                # 严格过滤：只有条目属于当前选中的骨骼，才为它发射一条射线
                if sub_entry.bone_name != pb.name:
                    continue

                # ---- 接下来是你完全不乱飞的矩阵计算（完全对齐你的圆锥逻辑） ----
                sub_euler = Euler([math.radians(a) for a in sub_entry.pose_rot], 'XYZ')
                sub_rot_mat = sub_euler.to_matrix().to_4x4()

                if pb.parent:
                    mat_relative = pb.parent.bone.matrix_local.inverted() @ pb.bone.matrix_local
                    model_mat_sub = pb.parent.matrix @ mat_relative @ sub_rot_mat
                else:
                    model_mat_sub = pb.bone.matrix_local @ sub_rot_mat

                final_ray_mat = obj_mat_world @ model_mat_sub

                # 变换到世界空间
                v_start_world = final_ray_mat @ Vector((0.0, 0.0, 0.0))
                v_end_world = final_ray_mat @ Vector((0.0, ray_len, 0.0))

                final_ray_mat.translation = v_start_world

                # Y轴极值梯度渐变色计算
                local_y_radians = sub_euler.y 
                factor = max(0.0, min(1.0, abs(local_y_radians) / math.pi))

                r = 0.1 + 0.9 * factor
                g = 0.9 - 0.8 * factor
                b = 0.1
                alpha = 0.5 + 0.3 * factor 

                # 绘制单条射线
                single_ray_batch = batch_for_shader(_shader, 'LINES', {"pos": [v_start_world, v_end_world]})
                _shader.uniform_float("lineWidth", 2.0)
                _shader.uniform_float("color", (r, g, b, alpha))
                single_ray_batch.draw(_shader)

                # 绘制末端小球
                dot_radius = ray_len * 0.05 
                dot_mat = Matrix.Translation(v_end_world) @ Matrix.Scale(dot_radius, 4)
                v_dot_sphere = [dot_mat @ v for v in SPHERE_VERTS]
                
                dot_batch = batch_for_shader(_shader, 'LINES', {"pos": v_dot_sphere}, indices=SPHERE_INDS)
                _shader.uniform_float("lineWidth", 2.5)
                _shader.uniform_float("color", (r, g, b, min(1.0, alpha + 0.2)))
                dot_batch.draw(_shader)




def register_draw_handler():
    global _draw_handler
    if _draw_handler is None:
        _draw_handler = bpy.types.SpaceView3D.draw_handler_add(draw_psd_visualizers, (), 'WINDOW', 'POST_VIEW')

def unregister_draw_handler():
    global _draw_handler
    if _draw_handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handler, 'WINDOW')
        _draw_handler = None