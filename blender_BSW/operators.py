import bpy
import json
import os
import math
from mathutils import Vector, Euler, Quaternion, Matrix
from bpy_extras.io_utils import ImportHelper, ExportHelper
from .utils import _safe_name, _get_selected_pair_bone, _capture_bone_local_rotation_deg, _capture_bone_local_translation, _capture_bone_local_scale, psd_register_cache_empty, psd_unregister_cache_empty, _arm_key_for_obj, psd_get_registered_empty  # 导入依赖
from .utils import _PSD_CACHE_OBJ_PROP
from .core import psd_invalidate_bone_cache, psd_clear_emply_results_cache  # 导入核心函数
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA
from .math_utils import _euler_deg_to_quat, _swing_twist_decompose
import numpy as np

#同步 UI 列表结构

# 生成形态键和驱动器的自动化
class PSD_OT_EntriesToShapeKey(bpy.types.Operator):
    """Generate shape keys for the selected mesh."""
    bl_idname = "psd.entries_to_shapekey"
    bl_label = "Generate Shape Keys from Saved Poses"
    bl_options = {'REGISTER', 'UNDO'}

    # 定义弹出窗口中让用户选择的网格对象
    def get_mesh_items(self, context):
        items = []
        for obj in context.scene.objects:
            if obj.type == 'MESH':
                items.append((obj.name, obj.name, f"Apply to {obj.name}"))
        return items

    target_mesh_name: bpy.props.EnumProperty(
        name="Target Mesh",
        description="Select the mesh to receive shape keys",
        items=get_mesh_items
    )

    def invoke(self, context, event):
        # 初始检查
        arm = context.object
        if not arm.psd_saved_poses:
            self.report({'ERROR'}, "No saved poses found!")
            return {'CANCELLED'}
            
        # 呼出确认/参数调整对话框
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        arm_db = arm.data
        
        # --- 获取当前选中的 Bone Pair ---
        pair_idx = arm.psd_bone_pairs_index
        if not (0 <= pair_idx < len(arm.psd_bone_pairs)):
            self.report({'ERROR'}, "Please select a Bone Pair in the list first!")
            return {'CANCELLED'}
        
        # 确定当前要处理的骨骼名
        current_bone = arm.psd_bone_pairs[pair_idx].bone_name
        # ---------------------------------------

        target_obj = bpy.data.objects.get(self.target_mesh_name)
        if not target_obj:
            self.report({'ERROR'}, "Must select a target mesh!")
            return {'CANCELLED'}

        # 1. 检查 Empty 缓存对象逻辑 (保持你的修复)
        empty_name = arm_db.get(_PSD_CACHE_OBJ_PROP, "")
        empty_obj = bpy.data.objects.get(empty_name)
        if getattr(arm, "psd_output_mode", 'STORE_TO_EMPTY') != 'STORE_TO_EMPTY' or not empty_obj:
            self.report({'ERROR'}, "Storage mode error or Empty not initialized")
            return {'CANCELLED'}

        # 2. 确保有 Basis
        if not target_obj.data.shape_keys:
            target_obj.shape_key_add(name="Basis")

        # 3. 遍历并筛选条目
        created_count = 0
        for entry in arm.psd_saved_poses:
            # --- 关键过滤逻辑 ---
            if entry.bone_name != current_bone:
                continue
            # ------------------

            safe_bn = _safe_name(entry.bone_name)
            safe_en = _safe_name(entry.name)
            sk_name = f"PSD_{safe_bn}_{safe_en}"
            prop_path = f'["{PREFIX_RESULT}{safe_bn}_{safe_en}"]'

            
            # 冲突检查
            if sk_name in target_obj.data.shape_keys.key_blocks:
                continue

            # 创建形态键 (from_mix=False 保证它是干净的零位)
            skey = target_obj.shape_key_add(name=sk_name, from_mix=False)
            created_count += 1

            # --- 驱动器绑定 ---
            # 你的修复 driver.type = 'SCRIPTED' 和 expression = "var" 非常稳！
            # driver_ptr = target_obj.data.shape_keys.driver_add(f'key_blocks["{sk_name}"].value')
            # driver = driver_ptr.driver
            # driver.type = 'SCRIPTED' 
            # driver.expression = "var"

            # var = driver.variables.new()
            # var.name = "var"
            # var.type = 'SINGLE_PROP'
            
            # target_var = var.targets[0]
            # target_var.id_type = 'OBJECT'
            # target_var.id = empty_obj
            # target_var.data_path = prop_path

        self.report({'INFO'}, f"Generated {created_count} Shape Keys for bone: {current_bone} ✨")
        return {'FINISHED'}

class PSD_OT_ShapeKeyDriver(bpy.types.Operator):
    """bind drivers from the Empty"""
    bl_idname = "psd.shape_key_driver"
    bl_label = "bind drivers from the Empty"
    bl_options = {'REGISTER', 'UNDO'}

    def get_mesh_items(self, context):
        items = []
        for obj in context.scene.objects:
            if obj.type == 'MESH':
                items.append((obj.name, obj.name, f"Apply to {obj.name}"))
        return items

    target_mesh_name: bpy.props.EnumProperty(
        name="Target Mesh",
        description="Select the mesh to receive shape keys",
        items=get_mesh_items
    )

    def invoke(self, context, event):
        # 初始检查
        arm = context.object
        if not arm.psd_saved_poses:
            self.report({'ERROR'}, "No saved poses found!")
            return {'CANCELLED'}
            
        # 呼出确认/参数调整对话框
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        arm_db = arm.data
        
        # --- 获取当前选中的 Bone Pair ---
        pair_idx = arm.psd_bone_pairs_index
        if not (0 <= pair_idx < len(arm.psd_bone_pairs)):
            self.report({'ERROR'}, "Please select a Bone Pair in the list first!")
            return {'CANCELLED'}
        
        # 确定当前要处理的骨骼名
        current_bone = arm.psd_bone_pairs[pair_idx].bone_name
        # ---------------------------------------

        target_obj = bpy.data.objects.get(self.target_mesh_name)
        if not target_obj:
            self.report({'ERROR'}, "Must select a target mesh!")
            return {'CANCELLED'}

        # 1. 检查 Empty 缓存对象逻辑 (保持你的修复)
        empty_name = arm_db.get(_PSD_CACHE_OBJ_PROP, "")
        empty_obj = bpy.data.objects.get(empty_name)
        if getattr(arm, "psd_output_mode", 'STORE_TO_EMPTY') != 'STORE_TO_EMPTY' or not empty_obj:
            self.report({'ERROR'}, "Storage mode error or Empty not initialized")
            return {'CANCELLED'}

        # 2. 确保有 Basis
        if not target_obj.data.shape_keys:
            target_obj.shape_key_add(name="Basis")

        created_count = 0
        for entry in arm.psd_saved_poses:
            # --- 关键过滤逻辑 ---
            if entry.bone_name != current_bone:
                continue
            # ------------------

            safe_bn = _safe_name(entry.bone_name)
            safe_en = _safe_name(entry.name)
            sk_name = f"PSD_{safe_bn}_{safe_en}"
            prop_path = f'["{PREFIX_RESULT}{safe_bn}_{safe_en}"]'
            data_path = f'key_blocks["{sk_name}"].value'

            has_driver = False
            anim_data = target_obj.data.shape_keys.animation_data

            # 冲突检查
            if sk_name not in target_obj.data.shape_keys.key_blocks:
                continue

            if anim_data:
                # 在驱动器列表中查找路径完全匹配的项
                has_driver = any(d.data_path == data_path for d in anim_data.drivers)

            # 创建形态键驱动器
            if not has_driver:
                driver_ptr = target_obj.data.shape_keys.driver_add(data_path)
                driver = driver_ptr.driver
                driver.type = 'SCRIPTED' 
                driver.expression = "var"

                var = driver.variables.new()
                var.name = "var"
                var.type = 'SINGLE_PROP'
                
                target_var = var.targets[0]
                target_var.id_type = 'OBJECT'
                target_var.id = empty_obj
                target_var.data_path = prop_path

                created_count += 1



        self.report({'INFO'}, f"Generated {created_count} Shape Keys Driver for bone: {current_bone} ✨")
        return {'FINISHED'}

#============

#RBF
class PSD_OT_UpdateRBFSolver(bpy.types.Operator):
    """Recalculate the RBF linear interpolation coefficients based on the current pose sample."""
    bl_idname = "psd.update_rbf_solver"
    bl_label = "Update RBF Solver"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        # 参考你提供的 PSDApplySavePose 获取 arm 的方式
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}

        # --- 1. 获取所有开启 RBF 的条目 ---
        # 确认使用 arm.psd_saved_poses，这在你的 Apply 算子中已经验证是正确的
        if not hasattr(arm, "psd_saved_poses"):
            self.report({'ERROR'}, "Property 'psd_saved_poses' not found on Object")
            return {'CANCELLED'}

        all_rbf_entries = [e for e in arm.psd_saved_poses if e.use_rbf]
        
        if not all_rbf_entries:
            self.report({'WARNING'}, "No poses marked with 'Use RBF' found")
            return {'CANCELLED'}

        # --- 2. 按骨骼分组计算 ---
        bone_groups = {}
        for e in all_rbf_entries:
            # 假设你的 entry 里存骨骼名的字段是 bone_name
            bn = e.bone_name 
            bone_groups.setdefault(bn, []).append(e)

        for bn, entries in bone_groups.items():
            n = len(entries)
            if n < 1: continue 

            # 获取旋转数据 (Quaternions)
            sample_qs = [_euler_deg_to_quat(e.pose_rot) for e in entries]
            
            # 构建距离矩阵 M
            matrix_data = []
            for i in range(n):
                row = []
                # 获取第 i 个样本（当前行代表的样本）的基础数据
                q_i = sample_qs[i]
                for j in range(n):
                    q_j = sample_qs[j]
                    # 获取第 j 个样本的算法设置
                    # ！！！重要：使用中心点 j 的配置逻辑 ！！！
                    mode = getattr(entries[j], 'record_rot_channel_mode', 'NONE')
                    eps = float(getattr(entries[j], 'rbf_epsilon', 1.0))
                    
                    # 这里的 self.get_sample_dist_logic 内部一定要和 core.py 完全一致
                    dist = self.get_sample_dist_logic(q_i, q_j, mode)
                    
                    val = math.exp(-((dist * eps) ** 2))
                    row.append(val)
                matrix_data.append(row)


            try:
                # --- Operators.py ---
                Phi = np.array(matrix_data, dtype=np.float64)

                # 防止奇异矩阵
                Phi += np.eye(n) * 1e-8

                try:
                    M_inv = np.linalg.inv(Phi)

                except np.linalg.LinAlgError:

                    self.report(
                        {'ERROR'},
                        f"Bone {bn} RBF matrix is singular"
                    )

                    continue

                M_inv = M_inv.tolist()

                # 1. 找到该骨骼的存储对象
                storage_list = arm.psd_rbf_storage
                # 1. 找到该骨骼的存储对象（不要在 entries 循环里做这个，要在骨骼组循环里做）
                bone_storage = next((s for s in storage_list if s.bone_name == bn), None)
                if not bone_storage:
                    bone_storage = storage_list.add()
                    bone_storage.bone_name = bn

                # 2. 【关键】彻底清空旧行，确保里面没有残留的 1.0
                bone_storage.coeffs.clear() 

                # 3. 依次存入每一行
                for r in range(n):
                    row_item = bone_storage.coeffs.add()
                    # 这里的 M_inv[r] 才是真正的灵魂
                    row_item.values = ",".join([f"{v:.8f}" for v in M_inv[r]])
                
                bone_storage.sample_count = n
                print(f"DEBUG: Bone {bn} matrix update complete; {n} rows of coefficients stored.")

            except Exception as e:
                print(f"Error updating bone {bn}: {e}")

        self.report({'INFO'}, "RBF Coefficients updated successfully ✨")
        return {'FINISHED'}

    def get_sample_dist_logic(self, q1, q2, mode): # <--- 添加了 self
        """
        Core Distance Logic: Calculates the difference between two rotations based on the specified mode.
        Returns the value in radians (Float).
        """
        # 如果 mode 是 NONE，说明是全轴 RBF，直接计算最短旋转角弧度
        if mode == 'NONE' or mode == 'record_rot_ALL':
            return q1.rotation_difference(q2).angle

        # 确定 Twist 轴
        t_axis = Vector((0.0, 1.0, 0.0)) # 默认 Y 轴
        if 'record_rot_SWING_X_TWIST' in mode:
            t_axis = Vector((1.0, 0.0, 0.0))
        elif 'record_rot_SWING_Z_TWIST' in mode:
            t_axis = Vector((0.0, 0.0, 1.0))

        # 执行 Swing-Twist 分解
        # 请确保 _swing_twist_decompose 已经在 math_utils.py 中定义并在此处可用
        s1, _ = _swing_twist_decompose(q1, t_axis)
        s2, _ = _swing_twist_decompose(q2, t_axis)

        return s1.rotation_difference(s2).angle
    
#=======镜像=====
def mirror_rotation_values(src_rot):
    """
    Flip the Y and Z Euler angles. This depends on the skeleton's axis orientation; here, Blender's standard mirroring rules are applied
    """
    # 假设 src_rot 是 Vector((x, y, z))，单位是度
    mirrored = Vector((src_rot.x, -src_rot.y, -src_rot.z))
    return mirrored

def get_mirror_name(name):
    """Determine the .L and .R suffixes and return the mirror name"""
    if name.endswith(".L") or name.endswith(".l"):
        return name[:-2] + (".R" if name.endswith(".L") else ".r")
    if name.endswith(".R") or name.endswith(".r"):
        return name[:-2] + (".L" if name.endswith(".R") else ".l")
    return None

class PSD_OT_CopyPath(bpy.types.Operator): 
    """Copy property property path to clipboard with one click"""
    bl_idname = "psd.copy_path"
    bl_label = "Copy property Path"
    
    path_to_copy: bpy.props.StringProperty()

    def execute(self, context):
        # 使用 Blender 内置的剪贴板接口
        context.window_manager.clipboard = self.path_to_copy
        self.report({'INFO'}, f"Copied: {self.path_to_copy}")
        return {'FINISHED'}

class PSD_OT_SyncMirrorData(bpy.types.Operator):
    """sync L -> R"""
    bl_idname = "psd.sync_mirror_data"
    bl_label = "sync L -> R"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        if not arm or not hasattr(arm, "psd_saved_poses"):
            return {'CANCELLED'}

        poses = arm.psd_saved_poses
        # 1. 建立索引：(骨骼名, 姿态名) -> 条目对象
        pose_map = {(p.bone_name, p.name): p for p in poses}
        
        sync_count = 0
        for p_src in poses:
            # 只处理 .L 作为源
            if not (p_src.bone_name.endswith(".L") or p_src.bone_name.endswith(".l")):
                continue
            
            # 计算目标镜像骨骼名
            base_name = p_src.bone_name[:-2]
            suffix = p_src.bone_name[-1] # L 或 l
            mir_bone = base_name + (".R" if suffix == "L" else ".r")
            
            # 查找对应的 .R 条目
            p_dst = pose_map.get((mir_bone, p_src.name))
            
            if p_dst:
                # --- 开始同步属性 ---
                p_dst.has_rot = p_src.has_rot
                p_dst.has_loc = p_src.has_loc
                p_dst.has_sca = p_src.has_sca

                p_dst.is_direct_channel = p_src.is_direct_channel
                p_dst.use_rbf = p_src.use_rbf
                p_dst.use_triangle_weights = p_src.use_triangle_weights
                p_dst.record_rot_channel_mode = p_src.record_rot_channel_mode
                
                # --- 旋转镜像核心逻辑 ---
                if p_dst.use_rbf:
                    p_dst.rbf_epsilon = p_src.rbf_epsilon

                if p_dst.is_direct_channel:
                    p_dst.channel_axis = p_src.channel_axis

                if p_dst.has_rot:
                    # 1. 镜像静止姿态 (通常是 0,0,0)
                    p_dst.rest_rot = mirror_rotation_values(Vector(p_src.rest_rot))
                    # 2. 镜像目标姿态 (关键数值)
                    p_dst.pose_rot = mirror_rotation_values(Vector(p_src.pose_rot))
                    # 3. 如果是锥形衰减，也要处理轴向
                    # new_pose.cone_axis = mirror_rotation_values(Vector(src_e.cone_axis))
                    p_dst.cone_enabled = p_src.cone_enabled
                    p_dst.cone_angle = p_src.cone_angle
                    p_dst.cone_axis = p_src.cone_axis
                    p_dst.cone_scale_x = p_src.cone_scale_x
                    p_dst.cone_scale_y = p_src.cone_scale_y
                    p_dst.cone_scale_z = p_src.cone_scale_z

                # --- 位置镜像逻辑 ---
                # X通常是横向，镜像时变负
                p_dst.rest_loc = (-p_src.rest_loc[0], p_src.rest_loc[1], p_src.rest_loc[2])
                p_dst.pose_loc = (-p_src.pose_loc[0], p_src.pose_loc[1], p_src.pose_loc[2])
                p_dst.loc_enabled = p_src.loc_enabled
                p_dst.loc_radius = p_src.loc_radius
                
                # --- 缩放通常直接同步 ---
                p_dst.pose_sca = p_src.pose_sca
                
                sync_count += 1

        self.report({'INFO'}, f"Synced {sync_count} mirrored poses")
        return {'FINISHED'}


class PSD_OT_MirrorBoneFilter(bpy.types.Operator):
    """Automatically mirror bone and pose entries (.L <-> .R)"""
    bl_idname = "psd.mirror_bone_filter"
    bl_label = "Image Completion (Filters and Data)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        
        # 1. 获取现有骨骼过滤器列表
        existing_filter_bones = {p.bone_name for p in arm.psd_bone_pairs if p.bone_name}
        new_bones_count = 0
        added_mirror_pairs = [] # 存储新补全的 [源骨骼, 镜像骨骼]

        # 阶段 A: 补全骨骼过滤器
        for bn in list(existing_filter_bones):
            mirror_name = get_mirror_name(bn)
            if mirror_name and mirror_name in arm.data.bones:
                if mirror_name not in existing_filter_bones:
                    item = arm.psd_bone_pairs.add()
                    item.bone_name = mirror_name
                    existing_filter_bones.add(mirror_name)
                    added_mirror_pairs.append((bn, mirror_name))
                    new_bones_count += 1

        # 阶段 B: 补全数据条目 (Saved Poses)
        new_poses_count = 0
        if hasattr(arm, "psd_saved_poses"):
            # 先扫描现有的所有条目，避免重复创建
            # key 为 (骨骼名, 条目名)
            existing_poses = {(e.bone_name, e.name) for e in arm.psd_saved_poses}
            
            # 对每一对镜像骨骼进行数据克隆
            for src_bn, mir_bn in added_mirror_pairs:
                src_entries = [e for e in arm.psd_saved_poses if e.bone_name == src_bn]
                for src_e in src_entries:
                    if (mir_bn, src_e.name) not in existing_poses:
                        new_pose = arm.psd_saved_poses.add()
                        # 基础拷贝
                        new_pose.name = src_e.name
                        new_pose.bone_name = mir_bn

                        # 暂时先保持和源数据一致，因为镜像骨骼通常需要重新 Capture 姿态
                        new_poses_count += 1

        self.report({'INFO'}, f"Completed bones: {new_bones_count}, Completed poses: {new_poses_count}")
        return {'FINISHED'}


class PSD_OT_ToggleAllFilterDetails(bpy.types.Operator):
    """Toggle Details Display"""
    bl_idname = "psd.toggle_all_filter_details"
    bl_label = "Toggle Details Display"

    def execute(self, context):
        arm = context.object
        # 切换全局状态
        arm.psd_filter_show_all_details = not arm.psd_filter_show_all_details
        
        # 同步给所有子条目
        for item in arm.psd_bone_pairs:
            item.show_mirror_details = arm.psd_filter_show_all_details
            
        return {'FINISHED'}

#=======CapturePose=====
class PSDCapturePose(bpy.types.Operator):
    bl_idname = "psd.capture_pose"
    bl_label = "Capture Transformation Data"
    bl_options = {'REGISTER', 'UNDO'}
    
    # 必须明确定义这些属性，UI 传参才能生效
    target_type: bpy.props.EnumProperty(
        items=[('REST', "Rest", ""), ('POSE', "Pose", "")]
    )
    channel: bpy.props.EnumProperty(
        items=[
            ('ROT', "Rotation", ""), 
            ('LOC', "Location", ""), 
            ('SCA', "Scale", ""),
        ],
        default='ROT'
    )

    def execute(self, context):
        arm = context.object
        
        selected_bone_name = _get_selected_pair_bone(context)
        if selected_bone_name == '<NONE>':
            self.report({'ERROR'}, "Please select a bone in the list")
            return {'CANCELLED'}
        
        idx = arm.psd_saved_pose_index
        
        if idx < 0 or idx >= len(arm.psd_saved_poses):
             return {'CANCELLED'}

        entry = arm.psd_saved_poses[idx]

        pose_bone = arm.pose.bones.get(selected_bone_name)

        if not pose_bone:
            self.report({'ERROR'}, f"Bone not found: {selected_bone_name}")
            return {'CANCELLED'}

        # --- 分通道处理，确保不会串位 ---
        
        if self.channel == 'ROT':
            # 旋转捕获：转换为 XYZ 欧拉度数
            if pose_bone.rotation_mode == 'QUATERNION':
                euler = pose_bone.rotation_quaternion.to_euler('XYZ')
            elif pose_bone.rotation_mode == 'AXIS_ANGLE':
                euler = pose_bone.rotation_axis_angle.to_euler('XYZ')
            else:
                euler = pose_bone.rotation_euler.to_matrix().to_euler('XYZ')
            
            rot_deg = (math.degrees(euler.x), math.degrees(euler.y), math.degrees(euler.z))
            
            if self.target_type == 'REST':
                entry.rest_rot = rot_deg
            else:
                entry.pose_rot = rot_deg

        elif self.channel == 'LOC':
            # 位置捕获
            loc_val = pose_bone.location.copy()
            if self.target_type == 'REST':
                entry.rest_loc = loc_val
            else:
                entry.pose_loc = loc_val

        elif self.channel == 'SCA':
            # 缩放捕获
            sca_val = pose_bone.scale.copy()
            if self.target_type == 'REST':
                entry.rest_sca = sca_val
            else:
                entry.pose_sca = sca_val

        # 强制界面刷新
        context.area.tag_redraw()
        
        self.report({'INFO'}, f"Successfully captured {selected_bone_name} {self.channel} ({self.target_type})")
        return {'FINISHED'}

#=======apply=====
class PSDApplySavePose(bpy.types.Operator):
    bl_idname = "psd.apply_save_pose"
    bl_label = "Apply Pose to Bone"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}

        selected_bone_name = _get_selected_pair_bone(context)
        if selected_bone_name == '<NONE>':
            self.report({'ERROR'}, "Please select a bone in the list")
            return {'CANCELLED'}

        # 1. 获取条目
        idx = arm.psd_saved_pose_index
        if idx < 0 or idx >= len(arm.psd_saved_poses):
             return {'CANCELLED'}
        
        entry = arm.psd_saved_poses[idx]

        pose_bone = arm.pose.bones.get(selected_bone_name)
        if not pose_bone:
            return {'CANCELLED'}

        # --- 2. 应用位置 (Location) ---
        if entry.has_loc:
            pose_bone.location = Vector(entry.pose_loc)

        # --- 3. 应用缩放 (Scale) ---
        if entry.has_sca:
            pose_bone.scale = Vector(entry.pose_sca)

        # --- 4. 应用旋转 (Rotation) - 兼容所有模式 ---
        if entry.has_rot:
            # 1. 构造原始的 XYZ 欧拉对象
            saved_euler = Euler((
                math.radians(entry.pose_rot[0]),
                math.radians(entry.pose_rot[1]),
                math.radians(entry.pose_rot[2])
            ), 'XYZ')

            mode = pose_bone.rotation_mode
            if mode == 'QUATERNION':
                pose_bone.rotation_quaternion = saved_euler.to_quaternion()
            elif mode == 'AXIS_ANGLE':
                quat = saved_euler.to_quaternion()
                pose_bone.rotation_axis_angle = quat.to_axis_angle()
            else:
                # 先转成矩阵，再用矩阵转成对应 mode 的欧拉角
                # 这样可以自动处理所有的轴向顺序转换（如 ZYX, YXZ 等）
                pose_bone.rotation_euler = saved_euler.to_matrix().to_euler(mode)

        # 5. 更新视图
        # context.view_layer.update()
        
        self.report({'INFO'}, f"Bone {selected_bone_name} pose restored")
        return {'FINISHED'}


# =======配置导入导出=====

class PSDExportConfig(bpy.types.Operator, ExportHelper):
    """Export Current Armature PSD Configuration"""
    bl_idname = "psd.export_config"
    bl_label = "Export PSD Configuration"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}

        obj_scale = (arm.scale[0] + arm.scale[1] + arm.scale[2]) / 3

        data = {"bone_pairs": [], "saved_by_bone": {}, "orphan_saved_poses": [], "triggers": []}

        # 导出 bone_pairs
        try:
            data["bone_pairs"] = [p.bone_name for p in arm.psd_bone_pairs]
        except:
            pass

        # 导出 saved_poses 并按 bone_name 分组
        try:
            for e in arm.psd_saved_poses:
                item = {
                    "name": e.name,
                    "bone_name": e.bone_name,
                    "rest_rot": list(e.rest_rot),
                    "pose_rot": list(e.pose_rot),
                    "has_rot": bool(e.has_rot),
                    "is_direct_channel": bool(e.is_direct_channel),
                    "channel_axis": getattr(e, "channel_axis", "X"),
                    "record_rot_channel_mode": getattr(e, "record_rot_channel_mode", "NONE"),
                    "cone_enabled": bool(e.cone_enabled),
                    "cone_angle": float(e.cone_angle),
                    "cone_axis": getattr(e, "cone_axis", "Z"),
                    "cone_scale_x": float(getattr(e, "cone_scale_x", 1.0)),
                    "cone_scale_y": float(getattr(e, "cone_scale_y", 1.0)),
                    "cone_scale_z": float(getattr(e, "cone_scale_z", 1.0)),
                    "use_rbf": bool(e.use_rbf),
                    "rbf_epsilon": float(getattr(e, "rbf_epsilon", 1.0)),
                    "use_triangle_weights": bool(getattr(e, "use_triangle_weights", False)),

                    "rest_loc": list(e.rest_loc),
                    "pose_loc": list(e.pose_loc),
                    "has_loc": bool(e.has_loc),
                    "loc_enabled": bool(e.loc_enabled),
                    "loc_radius": float(e.loc_radius),

                    "group_name": getattr(e, "group_name", ""),

                    "rest_sca": list(e.rest_sca),
                    "pose_sca": list(e.pose_sca),
                    "has_sca": bool(e.has_sca)
                }
                if e.bone_name in data["bone_pairs"]:
                    data["saved_by_bone"].setdefault(e.bone_name, []).append(item)
                else:
                    data["orphan_saved_poses"].append(item)
        except Exception as err:
            self.report({'WARNING'}, f"Error exporting items: {err}")

        # 3. 新增：导出骨骼触发器 (Triggers)
        try:
            if hasattr(arm, "psd_triggers"):
                for trig in arm.psd_triggers:
                    trig_item = {
                        "name": trig.name,
                        "enabled": bool(trig.enabled),
                        "bone_name": trig.bone_name,       # 触发骨骼
                        "target_bone": trig.target_bone,   # 目标骨骼
                        "radius": float(trig.radius) / obj_scale,  # 缩放到世界单位
                        "falloff": getattr(trig, "falloff", "LINEAR"),
                        "show_sphere": bool(trig.show_sphere)
                    }
                    data["triggers"].append(trig_item)
        except Exception as err:
            self.report({'WARNING'}, f"Error exporting triggers: {err}")

        # 写文件
        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.report({'INFO'}, f"Successfully exported to {self.filepath}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write: {e}")
            return {'CANCELLED'}

    def invoke(self, context, event):
        arm = context.object
        if arm and arm.type == 'ARMATURE':
            self.filepath = bpy.path.ensure_ext(arm.name + "_psd_config.json", ".json")
        return super().invoke(context, event)

class PSDImportConfig(bpy.types.Operator, ImportHelper):
    """Import PSD configuration from JSON (merge and skip duplicates; do not overwrite existing entries)."""
    bl_idname = "psd.import_config"
    bl_label = "Import PSD Configuration"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object before importing")
            return {'CANCELLED'}

        try:
            with open(self.filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to read config file: {e}")
            return {'CANCELLED'}

        # ---------- 1. 合并 bone_pairs ----------
        existing_pairs = {p.bone_name for p in arm.psd_bone_pairs}
        for bn in data.get("bone_pairs", []):
            if bn not in existing_pairs:
                p = arm.psd_bone_pairs.add()
                p.bone_name = str(bn or "")
                existing_pairs.add(bn)

        # ---------- 2. 导入 PSD 条目 (Rot/Loc/Sca) ----------
        entries = []
        for bone, ents in data.get("saved_by_bone", {}).items():
            for ent in ents: entries.append(ent)
        for ent in data.get("orphan_saved_poses", []):
            entries.append(ent)

        existing_entries = {(e.bone_name, e.name) for e in arm.psd_saved_poses}
        added_entries = 0
        skipped_entries = 0

        for ent in entries:
            bone_name = str(ent.get("bone_name", "") or "")
            base_name = str(ent.get("name", "entry") or "entry")
            key = (bone_name, base_name)
            
            if key in existing_entries:
                skipped_entries += 1
                continue

            new = arm.psd_saved_poses.add()
            try:
                new.name = base_name
                new.bone_name = bone_name
                # 旋转
                rr = ent.get("rest_rot", [0.0, 0.0, 0.0])
                new.rest_rot = (float(rr[0]), float(rr[1]), float(rr[2]))
                pr = ent.get("pose_rot", [0.0, 0.0, 0.0])
                new.pose_rot = (float(pr[0]), float(pr[1]), float(pr[2]))
                new.has_rot = bool(ent.get("has_rot", False))
                new.is_direct_channel = bool(ent.get("is_direct_channel", False))
                new.channel_axis = str(ent.get("channel_axis", "X"))
                new.record_rot_channel_mode = str(ent.get("record_rot_channel_mode", "NONE"))
                new.cone_enabled = bool(ent.get("cone_enabled", False))
                new.cone_angle = float(ent.get("cone_angle", 60.0))
                new.cone_axis = str(ent.get("cone_axis", "Z"))
                new.cone_scale_x = float(ent.get("cone_scale_x", 1.0))
                new.cone_scale_y = float(ent.get("cone_scale_y", 1.0))
                new.cone_scale_z = float(ent.get("cone_scale_z", 1.0))
                new.use_rbf = bool(ent.get("use_rbf", False))
                new.rbf_epsilon = float(ent.get("rbf_epsilon", 1.0))
                new.use_triangle_weights = bool(ent.get("use_triangle_weights", False))
                # 位移
                rl = ent.get("rest_loc", [0.0, 0.0, 0.0])
                new.rest_loc = (float(rl[0]), float(rl[1]), float(rl[2]))
                pl = ent.get("pose_loc", [0.0, 0.0, 0.0])
                new.pose_loc = (float(pl[0]), float(pl[1]), float(pl[2]))
                new.has_loc = bool(ent.get("has_loc", False))
                new.loc_enabled = bool(ent.get("loc_enabled", False))
                new.loc_radius = float(ent.get("loc_radius", 0.1))
                # 缩放
                rs = ent.get("rest_sca", [1.0, 1.0, 1.0])
                new.rest_sca = (float(rs[0]), float(rs[1]), float(rs[2]))
                ps = ent.get("pose_sca", [1.0, 1.0, 1.0])
                new.pose_sca = (float(ps[0]), float(ps[1]), float(ps[2]))
                new.has_sca = bool(ent.get("has_sca", False))
                # 其他
                new.group_name = str(ent.get("group_name", ""))
            except Exception:
                pass
            
            existing_entries.add(key)
            added_entries += 1

        # ---------- 3. [新增] 导入骨骼触发器 (Triggers) ----------
        added_trig = 0
        skipped_trig = 0
        
        if "triggers" in data and hasattr(arm, "psd_triggers"):
            # 唯一性检查：(触发骨骼, 目标骨骼, 触发器名)
            existing_trigs = {(t.bone_name, t.target_bone, t.name) for t in arm.psd_triggers}
            
            for t_data in data["triggers"]:
                t_key = (t_data.get("bone_name"), t_data.get("target_bone"), t_data.get("name"))
                
                if t_key in existing_trigs:
                    skipped_trig += 1
                    continue
                
                new_t = arm.psd_triggers.add()
                try:
                    new_t.name = str(t_data.get("name", "Trigger"))
                    new_t.enabled = bool(t_data.get("enabled", True))
                    new_t.bone_name = str(t_data.get("bone_name", ""))
                    new_t.target_bone = str(t_data.get("target_bone", ""))
                    new_t.radius = float(t_data.get("radius", 0.1))
                    new_t.falloff = str(t_data.get("falloff", "LINEAR"))
                    new_t.show_sphere = bool(t_data.get("show_sphere", True))
                    
                    existing_trigs.add(t_key)
                    added_trig += 1
                except Exception:
                    pass

        # ---------- 4. 报告结果 ----------
        self.report({'INFO'}, 
            f"Import Finished: Added {added_entries} Poses & {added_trig} Triggers. "
            f"Skipped {skipped_entries + skipped_trig} duplicates."
        )
        return {'FINISHED'}

# ======骨骼对管理（增删改）======

class PSD_OT_AddBonePair(bpy.types.Operator):
    bl_idname = "psd.add_bone_pair"
    bl_label = "Add Bone Pair"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        existing = set(p.bone_name for p in arm.psd_bone_pairs if p.bone_name)
        chosen = None
        for b in arm.data.bones:
            if b.name not in existing:
                chosen = b.name
                break
        if chosen is None:
            chosen = arm.data.bones[0].name if len(arm.data.bones) > 0 else ""
        pair = arm.psd_bone_pairs.add()
        pair.bone_name = chosen
        arm.psd_bone_pairs_index = len(arm.psd_bone_pairs) - 1
        return {'FINISHED'}

class PSD_OT_RemoveAllBonePairs(bpy.types.Operator):
    """Remove all bone pairs and their associated saved poses and RBF data"""
    bl_idname = "psd.remove_all_bone_pairs"
    bl_label = "Remove All Bone Pairs"
    bl_options = {'REGISTER', 'UNDO'}  # 支持 Ctrl+Z 撤销，防止误删无法恢复

    @classmethod
    def poll(cls, context):
        arm = context.object
        return (
            arm is not None 
            and arm.type == 'ARMATURE' 
            and len(getattr(arm, "psd_bone_pairs", ())) > 0
        )

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        pair_count = len(arm.psd_bone_pairs)
        if pair_count == 0:
            self.report({'WARNING'}, "No bone pairs to remove")
            return {'CANCELLED'}

        # 1. 收集当前所有骨骼对的名称集合
        target_bone_names = {pair.bone_name for pair in arm.psd_bone_pairs}

        try:
            # 2. 级联清理关联的保存姿态 (psd_saved_poses)
            poses = arm.psd_saved_poses
            removed_poses_count = 0
            for i in range(len(poses) - 1, -1, -1):
                if poses[i].bone_name in target_bone_names:
                    poses.remove(i)
                    removed_poses_count += 1
            # 如果所有姿态均隶属于骨骼对，也可直接调用 poses.clear()

            # 3. 级联清理关联的 RBF 矩阵系数 (psd_rbf_storage)
            storage_list = arm.psd_rbf_storage
            for i in range(len(storage_list) - 1, -1, -1):
                if storage_list[i].bone_name in target_bone_names:
                    storage_list.remove(i)

            # 4. 清空骨骼对列表本身
            arm.psd_bone_pairs.clear()

            # 5. 重置 UI 列表激活索引，防止越界
            arm.psd_bone_pairs_index = -1
            if hasattr(arm, "psd_saved_poses_index"):
                arm.psd_saved_poses_index = (
                    min(arm.psd_saved_poses_index, len(poses) - 1) if len(poses) > 0 else -1
                )

            self.report(
                {'INFO'},
                f"Removed {pair_count} bone pairs and {removed_poses_count} associated poses."
            )

        except Exception as e:
            self.report({'ERROR'}, f"Failed to perform cascading delete: {str(e)}")
            return {'CANCELLED'}

        return {'FINISHED'}


class PSD_OT_RemoveBonePair(bpy.types.Operator):
    bl_idname = "psd.remove_bone_pair"
    bl_label = "Remove Bone Pair"

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        idx = arm.psd_bone_pairs_index
        if not (0 <= idx < len(arm.psd_bone_pairs)):
            self.report({'WARNING'}, "No bone pair selected")
            return {'CANCELLED'}

        # 1. 获取要删除的骨骼名称
        target_bone_name = arm.psd_bone_pairs[idx].bone_name
        
        try:
            
            # 必须从后往前遍历，否则删除时索引会变动导致删错或跳过
            poses = arm.psd_saved_poses
            for i in range(len(poses) - 1, -1, -1):
                if poses[i].bone_name == target_bone_name:
                    poses.remove(i)
            
            
            # 骨骼都没了，对应的 RBF 矩阵系数也必须清理干净
            storage_list = arm.psd_rbf_storage
            for i in range(len(storage_list) - 1, -1, -1):
                if storage_list[i].bone_name == target_bone_name:
                    storage_list.remove(i)

            # 4. 最后删除骨骼对本身
            arm.psd_bone_pairs.remove(idx)
            
            # 5. 更新骨骼对列表的索引
            arm.psd_bone_pairs_index = max(0, min(idx, len(arm.psd_bone_pairs) - 1))
            if len(arm.psd_bone_pairs) == 0:
                arm.psd_bone_pairs_index = -1

            self.report({'INFO'}, f"Deleted bone {target_bone_name} and all its associated poses.")

        except Exception as e:
            self.report({'ERROR'}, f"Failed to perform cascading delete: {str(e)}")
            return {'CANCELLED'}

        return {'FINISHED'}

class PSD_OT_MoveBonePair(bpy.types.Operator):
    bl_idname = "psd.move_bone_pair"
    bl_label = "Move Bone Pair"
    direction: bpy.props.EnumProperty(items=[('UP','Up',''), ('DOWN','Down','')], default='UP')

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            return {'CANCELLED'}
        idx = arm.psd_bone_pairs_index
        if idx < 0 or idx >= len(arm.psd_bone_pairs):
            return {'CANCELLED'}
        if self.direction == 'UP' and idx > 0:
            try:
                arm.psd_bone_pairs.move(idx, idx-1)
                arm.psd_bone_pairs_index = idx-1
            except Exception:
                pass
        elif self.direction == 'DOWN' and idx < len(arm.psd_bone_pairs)-1:
            try:
                arm.psd_bone_pairs.move(idx, idx+1)
                arm.psd_bone_pairs_index = idx+1
            except Exception:
                pass
        return {'FINISHED'}

class PSDCaptureRest(bpy.types.Operator):
    bl_idname = "psd.capture_rest"
    bl_label = "Capture Rest Rotation"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            degs = _capture_bone_local_rotation_deg(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_rest_bone = selected_bone
            context.scene.psd_temp_rest = degs
            self.report({'INFO'}, f"Successfully captured rest rotation for {selected_bone}: {degs}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture rest rotation: {ex}")
            return {'CANCELLED'}

class PSDCaptureRotation(bpy.types.Operator):
    bl_idname = "psd.capture_rotation"
    bl_label = "Capture Pose Rotation"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            degs = _capture_bone_local_rotation_deg(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_pose_bone = selected_bone
            context.scene.psd_temp_pose = degs
            self.report({'INFO'}, f"Successfully captured pose rotation for {selected_bone}: {degs}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture pose rotation: {ex}")
            return {'CANCELLED'}

class PSDCaptureLocationRest(bpy.types.Operator):
    bl_idname = "psd.capture_loc_rest"
    bl_label = "Capture Rest Location"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            rest_loc, _ = _capture_bone_local_translation(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_loc_rest_bone = selected_bone
            context.scene.psd_temp_loc_rest = tuple(rest_loc)
            self.report({'INFO'}, f"Successfully captured rest location for {selected_bone}: {tuple(rest_loc)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture rest location: {ex}")
            return {'CANCELLED'}

class PSDCaptureLocation(bpy.types.Operator):
    bl_idname = "psd.capture_location"
    bl_label = "Capture Pose Location"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            _, pose_loc = _capture_bone_local_translation(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_loc_bone = selected_bone
            context.scene.psd_temp_loc = tuple(pose_loc)
            self.report({'INFO'}, f"Successfully captured pose location for {selected_bone}: {tuple(pose_loc)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture pose location: {ex}")
            return {'CANCELLED'}

class PSDCaptureScaleRest(bpy.types.Operator):
    bl_idname = "psd.capture_sca_rest"
    bl_label = "Capture Rest Scale"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            rest_sca, _ = _capture_bone_local_scale(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_sca_rest_bone = selected_bone
            context.scene.psd_temp_sca_rest = tuple(rest_sca)
            self.report({'INFO'}, f"Successfully captured rest scale for {selected_bone}: {tuple(rest_sca)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture rest scale: {ex}")
            return {'CANCELLED'}

class PSDCaptureScale(bpy.types.Operator):
    bl_idname = "psd.capture_scale"
    bl_label = "Capture Pose Scale"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        try:
            _, pose_sca = _capture_bone_local_scale(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_sca_bone = selected_bone
            context.scene.psd_temp_sca = tuple(pose_sca)
            self.report({'INFO'}, f"Successfully captured pose scale for {selected_bone}: {tuple(pose_sca)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to capture pose scale: {ex}")
            return {'CANCELLED'}

class PSDSaveCapturedRotationEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_rotation"
    bl_label = "Save Captured Rotation Entry"

    entry_name: bpy.props.StringProperty(name="Entry Name", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        bone_r = getattr(scene, 'psd_temp_rest_bone', '')
        bone_p = getattr(scene, 'psd_temp_pose_bone', '')

        has_rot = bool(bone_p and bone_p != '<NONE>')
        if not has_rot:
            self.report({'ERROR'}, "Please capture a rotation first (required)")
            return {'CANCELLED'}

        if bone_r and bone_r != bone_p and bone_r != '':
            self.report({'WARNING'}, "Rest/Pose rotation from different bones; if needed, this entry's rest value will be set to (0,0,0)")
            rest_vals_rot = (0.0,0.0,0.0)
        else:
            rest_vals_rot = tuple(getattr(scene, 'psd_temp_rest', (0.0,0.0,0.0)))

        pose_vals_rot = tuple(getattr(scene, 'psd_temp_pose', (0.0,0.0,0.0)))
        bone_name_rot = getattr(scene, 'psd_temp_pose_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_rot and existing.name == self.entry_name:
                self.report({'ERROR'}, f"Bone {bone_name_rot} already has an entry named '{self.entry_name}'")
                return {'CANCELLED'}

        # 创建条目 (仅旋转)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_rot
        new.rest_rot = rest_vals_rot
        new.pose_rot = pose_vals_rot
        new.is_direct_channel = False
        new.has_rot = True
        # 确保位移字段为空/禁用
        new.rest_loc = (0.0,0.0,0.0)
        new.pose_loc = (0.0,0.0,0.0)
        new.has_loc = False
        new.loc_enabled = False
        new.loc_radius = 0.1

        # 确保缩放字段为空/禁用
        new.rest_sca = (0.0,0.0,0.0)
        new.pose_sca = (0.0,0.0,0.0)
        new.has_sca = False

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 仅清除旋转的临时捕捉数据
        scene.psd_temp_rest = (0.0,0.0,0.0)
        scene.psd_temp_pose = (0.0,0.0,0.0)
        scene.psd_temp_rest_bone = ''
        scene.psd_temp_pose_bone = ''

        # 创建并初始化旋转结果的键
        entry_key_rot = f"{PREFIX_RESULT}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_rot] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"Successfully saved rotation entry '{new.name}' for bone {bone_name_rot} in armature {arm.name}")
        return {'FINISHED'}

class PSDSaveCapturedLocationEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_location"
    bl_label = "Save Captured Location Entry"

    entry_name: bpy.props.StringProperty(name="Entry Name", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        bone_loc_r = getattr(scene, 'psd_temp_loc_rest_bone', '')
        bone_loc_p = getattr(scene, 'psd_temp_loc_bone', '')

        has_loc  = bool(bone_loc_p and bone_loc_p != '<NONE>')
        if not has_loc:
            self.report({'ERROR'}, "Please capture a location first (required)")
            return {'CANCELLED'}

        if bone_loc_r and bone_loc_r != bone_loc_p and bone_loc_r != '':
            self.report({'WARNING'}, "Rest/Pose location from different bones; if needed, this entry's rest value will be set to (0,0,0)")
            rest_vals_loc = (0.0,0.0,0.0)
        else:
            rest_vals_loc = tuple(getattr(scene, 'psd_temp_loc_rest', (0.0,0.0,0.0)))

        pose_vals_loc = tuple(getattr(scene, 'psd_temp_loc', (0.0,0.0,0.0)))
        bone_name_loc = getattr(scene, 'psd_temp_loc_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_loc and existing.name == self.entry_name:
                self.report({'ERROR'}, f"Bone {bone_name_loc} already has an entry named '{self.entry_name}'")
                return {'CANCELLED'}

        # 创建条目 (仅位移)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_loc
        new.is_direct_channel = False
        # 旋转字段为空/禁用
        new.rest_rot = (0.0,0.0,0.0)
        new.pose_rot = (0.0,0.0,0.0)
        new.has_rot = False
        # 确保缩放字段为空/禁用
        new.rest_sca = (0.0,0.0,0.0)
        new.pose_sca = (0.0,0.0,0.0)
        new.has_sca = False
        # 设置位移字段
        new.rest_loc = rest_vals_loc
        new.pose_loc = pose_vals_loc
        new.has_loc = True
        # 保留默认衰减设置 (用户后续可编辑)
        new.loc_enabled = getattr(new, 'loc_enabled', False)
        new.loc_radius = getattr(new, 'loc_radius', 0.1)

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 仅清除位移的临时捕捉数据
        scene.psd_temp_loc_rest = (0.0,0.0,0.0)
        scene.psd_temp_loc = (0.0,0.0,0.0)
        scene.psd_temp_loc_rest_bone = ''
        scene.psd_temp_loc_bone = ''

        # 创建并初始化位移结果的键
        entry_key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_loc] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"Successfully saved location entry '{new.name}' for bone {bone_name_loc} in armature {arm.name}")
        return {'FINISHED'}

class PSDSaveCapturedScaleEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_scale"
    bl_label = "Save Captured Scale Entry"

    entry_name: bpy.props.StringProperty(name="Entry Name", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        bone_sca_r = getattr(scene, 'psd_temp_sca_rest_bone', '')
        bone_sca_p = getattr(scene, 'psd_temp_sca_bone', '')

        has_sca = bool(bone_sca_p and bone_sca_p != '<NONE>')
        if not has_sca:
            self.report({'ERROR'}, "Please capture a scale first (required)")
            return {'CANCELLED'}

        if bone_sca_r and bone_sca_r != bone_sca_p and bone_sca_r != '':
            self.report({'WARNING'}, "Rest/Pose scale from different bones; if needed, this entry's rest value will be set to (1,1,1)")
            rest_vals_sca = (1.0,1.0,1.0)
        else:
            rest_vals_sca = tuple(getattr(scene, 'psd_temp_sca_rest', (1.0,1.0,1.0)))

        pose_vals_sca = tuple(getattr(scene, 'psd_temp_sca', (1.0,1.0,1.0)))
        bone_name_sca = getattr(scene, 'psd_temp_sca_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_sca and existing.name == self.entry_name:
                self.report({'ERROR'}, f"Bone {bone_name_sca} already has an entry named '{self.entry_name}'")
                return {'CANCELLED'}

        # 创建条目 (仅缩放)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_sca
        new.is_direct_channel = False
        # 旋转和位置字段为空/禁用
        new.rest_rot = (0.0,0.0,0.0)
        new.pose_rot = (0.0,0.0,0.0)
        new.has_rot = False
        new.rest_loc = (0.0,0.0,0.0)
        new.pose_loc = (0.0,0.0,0.0)
        new.has_loc = False
        # 设置缩放字段（无衰减）
        new.rest_sca = rest_vals_sca
        new.pose_sca = pose_vals_sca
        new.has_sca = True

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 清除临时捕捉数据
        scene.psd_temp_sca_rest = (1.0,1.0,1.0)
        scene.psd_temp_sca = (1.0,1.0,1.0)
        scene.psd_temp_sca_rest_bone = ''
        scene.psd_temp_sca_bone = ''

        # 创建并初始化缩放结果的键
        entry_key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_sca] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"Successfully saved scale entry '{new.name}' for bone {bone_name_sca} in armature {arm.name}")
        return {'FINISHED'}
# 添加空键
class PSDAddSavedEntry(bpy.types.Operator):
    bl_idname = "psd.add_saved_entry"
    bl_label = "Add Blank Entry"

    entry_name: bpy.props.StringProperty(name="Entry Name", default="default")
    

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}

        selected_bone = _get_selected_pair_bone(context)
        
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the filter list.")
            return {'CANCELLED'}
        # 检查同名冲突
        
        for existing in arm.psd_saved_poses:
            if existing.name == self.entry_name and existing.bone_name == selected_bone:
                self.report({'ERROR'}, f"Entry name '{self.entry_name}' already exists!")
                return {'CANCELLED'}
            
        # 创建条目
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = selected_bone if selected_bone != '<NONE>' else ""
        new.is_direct_channel = False
        # 旋转字段为空/禁用
        new.rest_rot = (0.0,0.0,0.0)
        new.pose_rot = (0.0,0.0,0.0)
        new.has_rot = False
        # 确保缩放字段为空/禁用
        new.rest_sca = (0.0,0.0,0.0)
        new.pose_sca = (0.0,0.0,0.0)
        new.has_sca = False
        # 设置位移字段
        new.rest_loc = (0.0,0.0,0.0)
        new.pose_loc = (0.0,0.0,0.0)
        new.has_loc = False
        # 保留默认衰减设置 (用户后续可编辑)
        new.loc_enabled = getattr(new, 'loc_enabled', False)
        new.loc_radius = getattr(new, 'loc_radius', 0.1)

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        self.report({'INFO'}, f"Successfully added entry: {self.entry_name}")
        return {'FINISHED'}

class PSDRemoveSavedEntry(bpy.types.Operator):
    bl_idname = "psd.remove_saved_entry"
    bl_label = "Remove Saved Entry"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        idx = arm.psd_saved_pose_index
        if idx < 0 or idx >= len(arm.psd_saved_poses):
            self.report({'ERROR'}, "No saved pose selected")
            return {'CANCELLED'}

        try:
            # 1. 获取要删除的条目数据
            entry = arm.psd_saved_poses[idx]
            # 使用与 core.py 相同的 safe_name 逻辑构建属性 Key
            safe_bn = _safe_name(entry.bone_name)
            safe_en = _safe_name(entry.name)
            
            keys_to_delete = [
                f"{PREFIX_RESULT}{safe_bn}_{safe_en}",
                f"{PREFIX_RESULT_LOC}{safe_bn}_{safe_en}",
                f"{PREFIX_RESULT_SCA}{safe_bn}_{safe_en}"
            ]

            # 2. 获取数据源：Armature Data 和 关联的 Empty
            arm_db = arm.data  # 直接访问当前骨架的数据块
            empty_obj = None
            
            # 从骨架数据中查找缓存的 Empty 名称或引用
            empty_name = arm_db.get(_PSD_CACHE_OBJ_PROP, None)
            if empty_name:
                empty_obj = bpy.data.objects.get(empty_name)

            # 3. 级联删除自定义属性
            for key in keys_to_delete:
                # 从 Armature 数据块中删除 (如果存在)
                if key in arm_db:
                    del arm_db[key]
                
                # 从 Empty 对象中删除 (如果存在)
                if empty_obj and key in empty_obj:
                    del empty_obj[key]

            # 4. 从 UI 列表集合中移除
            old_name = entry.name
            arm.psd_saved_poses.remove(idx)

            # 5. 更新索引，确保列表显示不会越界
            new_len = len(arm.psd_saved_poses)
            if new_len == 0:
                arm.psd_saved_pose_index = -1
            else:
                # 如果删的是最后一个，索引减一；否则保持不变
                if arm.psd_saved_pose_index >= new_len:
                    arm.psd_saved_pose_index = new_len - 1

            self.report({'INFO'}, f"Successfully removed entry '{old_name}' and its associated properties")

        except IndexError:
            self.report({'ERROR'}, "Failed to delete: index out of bounds")
        except Exception as e:
            self.report({'ERROR'}, f"An unexpected error occurred while deleting the entry: {str(e)}")

        return {'FINISHED'}

class PSD_OT_AddTrigger(bpy.types.Operator):
    bl_idname = "psd.add_trigger"
    bl_label = "Add Trigger"
    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        pb = None
        if context.mode.startswith('POSE'):
            pb = context.active_pose_bone
        # 新条目
        t = arm.psd_triggers.add()
        t.name = "Trigger"
        if pb:
            t.bone_name = pb.name
        else:
            t.bone_name = ""
        arm.psd_trigger_index = len(arm.psd_triggers) - 1
        return {'FINISHED'}

class PSD_OT_RemoveTrigger(bpy.types.Operator):
    bl_idname = "psd.remove_trigger"
    bl_label = "Remove Trigger"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            return {'CANCELLED'}
            
        idx = arm.psd_trigger_index
        if idx < 0 or idx >= len(arm.psd_triggers):
            return {'CANCELLED'}

        # 获取常量前缀和安全名称工具（确保已在文件头部导入）
        from .utils import _safe_name, PREFIX_RESULT_LOC, _PSD_CACHE_OBJ_PROP

        try:
            # 1. 获取要删除的条目
            trig = arm.psd_triggers[idx]
            old_name = trig.name
            
            # 2. 构建属性 Key (需与 compute_triggers 中的命名规则完全一致)
            # 注意：如果你的触发器结果 key 带有后缀 "_w"，请确保此处匹配
            key_base = f"{PREFIX_RESULT_LOC}{_safe_name(trig.target_bone)}_to_{_safe_name(trig.bone_name)}_{_safe_name(trig.name)}_w"

            
            # 3. 获取数据源
            arm_db = arm.data
            empty_obj = None
            
            # 从骨架数据中查找关联的 Empty
            empty_name = arm_db.get(_PSD_CACHE_OBJ_PROP, None)
            if empty_name:
                empty_obj = bpy.data.objects.get(empty_name)

            # 4. 级联删除自定义属性
            # 从 Armature Data 删除
            if key_base in arm_db:
                del arm_db[key_base]
                print(f"PSD Debug: Removed from Armature {key_base}")
            
            # 从 Empty 对象删除
            if empty_obj and key_base in empty_obj:
                del empty_obj[key_base]
                print(f"PSD Debug: Removed from Empty {key_base}")

            # 5. 从集合中移除条目
            arm.psd_triggers.remove(idx)
            
            # 6. 更新 UI 列表索引
            new_len = len(arm.psd_triggers)
            if new_len == 0:
                arm.psd_trigger_index = -1
            else:
                # 保持选中合理的索引位置
                if arm.psd_trigger_index >= new_len:
                    arm.psd_trigger_index = new_len - 1
            
            self.report({'INFO'}, f"Successfully removed trigger '{old_name}'")

        except Exception as e:
            self.report({'ERROR'}, f"An unexpected error occurred while removing the trigger: {str(e)}")
            return {'CANCELLED'}

        return {'FINISHED'}

# class PSD_OT_RemoveTrigger(bpy.types.Operator):
#     bl_idname = "psd.remove_trigger"
#     bl_label = "Remove Trigger"
#     def execute(self, context):
#         arm = context.object
#         idx = arm.psd_trigger_index
#         if not arm or idx < 0 or idx >= len(arm.psd_triggers):
#             return {'CANCELLED'}
#         #UI更新
#         try:
#             key = arm.psd_triggers[idx]
#             key_base = f"{PREFIX_RESULT_LOC}{_safe_name(key.target_bone)}_{_safe_name(key.name)}_w"
#             arm_db = bpy.data.armatures.get(arm.data.name)
#             if arm_db:
#                 if key_base in arm_db:
#                     print("success del " + key_base)
#                     del arm_db[key_base]
#             arm.psd_triggers.remove(idx)
#             arm.psd_trigger_index = min(max(0, idx-1), len(arm.psd_triggers)-1)
#         except Exception as e:
#             self.report({'ERROR'}, f"移除条目时出错: {e}")
#             return {'CANCELLED'}
#         return {'FINISHED'}

class PSD_OT_SelectTriggerBone(bpy.types.Operator):
    bl_idname = "psd.select_trigger_bone"
    bl_label = "Select Trigger/Target Bone"
    mode: bpy.props.EnumProperty(items=[('TRIGGER','Trigger','Set trigger bone'), ('TARGET','Target','Set target bone')], default='TRIGGER')
    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            return {'CANCELLED'}
        pb = context.active_pose_bone if context.mode.startswith('POSE') else None
        idx = arm.psd_trigger_index
        if idx < 0 or idx >= len(arm.psd_triggers):
            self.report({'ERROR'}, "Please select a trigger entry first")
            return {'CANCELLED'}
        if not pb:
            self.report({'ERROR'}, "Please select a bone head in Pose mode")
            return {'CANCELLED'}
        if self.mode == 'TRIGGER':
            arm.psd_triggers[idx].bone_name = pb.name
        else:
            arm.psd_triggers[idx].target_bone = pb.name
        return {'FINISHED'}

class PSDRecordChannelX(bpy.types.Operator):
    bl_idname = "psd.record_channel_x"
    bl_label = "Record X Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        name = "record_X"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"Bone {selected_bone} already has an entry '{name}'")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'X'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to record: {ex}")
            return {'CANCELLED'}

class PSDRecordChannelY(bpy.types.Operator):
    bl_idname = "psd.record_channel_y"
    bl_label = "Record Y Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        name = "record_Y"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"Bone {selected_bone} already has an entry '{name}'")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'Y'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to record: {ex}")
            return {'CANCELLED'}

class PSDRecordChannelZ(bpy.types.Operator):
    bl_idname = "psd.record_channel_z"
    bl_label = "Record Z Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "No bone selected in the bone filter. Please add/select a bone in the bone filter list.")
            return {'CANCELLED'}
        name = "record_Z"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"Bone {selected_bone} already has an entry '{name}'")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'Z'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"Failed to record: {ex}")
            return {'CANCELLED'}

# Cache Empty 对应操作器

class PSD_OT_register_cache_empty_ui(bpy.types.Operator):
    bl_idname = "psd.register_cache_empty_ui"
    bl_label = "PSD: Register Cache Empty (UI)"
    bl_description = "PSD: Register Cache Empty (UI)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        empty = getattr(context.scene, "psd_cache_empty", None)
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please activate an Armature object first")
            return {'CANCELLED'}
        if not empty or empty.type != 'EMPTY':
            self.report({'ERROR'}, "Please select an Empty object as the cache object in the panel")
            return {'CANCELLED'}
        ok = psd_register_cache_empty(arm, empty, verbose=True)
        if not ok:
            self.report({'ERROR'}, "Failed to register cache empty (check console)")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Successfully registered Empty '{empty.name}' with armature '{arm.name}'")
        return {'FINISHED'}

class PSD_OT_unregister_cache_empty_ui(bpy.types.Operator):
    bl_idname = "psd.unregister_cache_empty_ui"
    bl_label = "PSD: Unregister Cache Empty (UI)"
    bl_description = "PSD: Unregister Cache Empty (UI)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please activate an Armature object first")
            return {'CANCELLED'}
        ok = psd_unregister_cache_empty(arm, verbose=True)
        if not ok:
            self.report({'ERROR'}, "Failed to unregister cache empty (check console)")
            return {'CANCELLED'}
        self.report({'INFO'}, "Successfully unregistered cache empty")
        return {'FINISHED'}

# 清除 Cache 相关操作器
class PSD_OT_invalidate_cache(bpy.types.Operator):
    bl_idname = "psd.invalidate_bone_cache"
    bl_label = "Invalidate PSD Bone Cache"

    def execute(self, context):
        obj = context.object
        if obj and obj.type == 'ARMATURE':
            arm_key = _arm_key_for_obj(obj)
            psd_invalidate_bone_cache(arm_name=arm_key)
            psd_clear_emply_results_cache(obj)
            self.report({'INFO'}, f"Cleared PSD cache for {obj.name}")
        else:
            psd_invalidate_bone_cache()
            psd_clear_emply_results_cache()
            self.report({'INFO'}, "Cleared all PSD caches")
        return {'FINISHED'}
    
# Shape Driver 对应操作器
class PSDAddShapeDriverFile(bpy.types.Operator, ImportHelper):
    bl_idname = "psd.add_shape_driver_file"
    bl_label = "Add Shape Driver JSON"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        item = arm.psd_shape_driver_files.add()
        item.filepath = self.filepath
        arm.psd_shape_driver_files_index = len(arm.psd_shape_driver_files) - 1
        self.report({'INFO'}, f"Successfully added Shape Driver file: {os.path.basename(self.filepath)}")
        return {'FINISHED'}

class PSDRemoveShapeDriverFile(bpy.types.Operator):
    bl_idname = "psd.remove_shape_driver_file"
    bl_label = "Remove Selected Shape Driver JSON"

    def execute(self, context):
        arm = context.object
        idx = arm.psd_shape_driver_files_index
        if 0 <= idx < len(arm.psd_shape_driver_files):
            arm.psd_shape_driver_files.remove(idx)
            arm.psd_shape_driver_files_index = max(0, idx - 1)
            self.report({'INFO'}, "Successfully removed selected file")
        return {'FINISHED'}

class PSDReloadShapeDrivers(bpy.types.Operator):
    bl_idname = "psd.reload_shape_drivers"
    bl_label = "Reload Shape Drivers"

    def execute(self, context):
        from .core import reload_shape_drivers  # 延迟导入避免循环
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        count = reload_shape_drivers(arm)
        self.report({'INFO'}, f"Successfully reloaded Shape Drivers: {count} items")
        return {'FINISHED'}

# Pose Driver 对应操作器（几乎相同）
class PSDAddPoseDriverFile(bpy.types.Operator, ImportHelper):
    bl_idname = "psd.add_pose_driver_file"
    bl_label = "Add Pose Driver JSON"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        item = arm.psd_pose_driver_files.add()
        item.filepath = self.filepath
        arm.psd_pose_driver_files_index = len(arm.psd_pose_driver_files) - 1
        self.report({'INFO'}, f"Successfully added Pose Driver file: {os.path.basename(self.filepath)}")
        return {'FINISHED'}

class PSDRemovePoseDriverFile(bpy.types.Operator):
    bl_idname = "psd.remove_pose_driver_file"
    bl_label = "Remove Selected Pose Driver JSON"

    def execute(self, context):
        arm = context.object
        idx = arm.psd_pose_driver_files_index
        if 0 <= idx < len(arm.psd_pose_driver_files):
            arm.psd_pose_driver_files.remove(idx)
            arm.psd_pose_driver_files_index = max(0, idx - 1)
            self.report({'INFO'}, "Successfully removed selected file")
        return {'FINISHED'}

class PSDReloadPoseDrivers(bpy.types.Operator):
    bl_idname = "psd.reload_pose_drivers"
    bl_label = "Reload Pose Drivers"

    def execute(self, context):
        from .core import reload_pose_drivers  # 延迟导入
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Please select an armature object")
            return {'CANCELLED'}
        count = reload_pose_drivers(arm)
        self.report({'INFO'}, f"Successfully reloaded Pose Drivers: {count} items")
        return {'FINISHED'}