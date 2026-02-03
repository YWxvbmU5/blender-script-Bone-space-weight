import bpy
import gpu
import math
from mathutils import Vector, Matrix, Euler
from gpu_extras.batch import batch_for_shader
from .core import _psd_results_cache, _psd_perf_stats

from .utils import _get_selected_pair_bone, _get_selected_armature_name, psd_get_registered_empty, get_weight_cache, update_direct_channel  # 导入辅助
from .operators import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA
from .core import (
    _psd_perf_stats,
    _shape_expressions_cache,
    _pose_drivers_cache,
    _arm_key_for_obj
)
# =========================================================================
#  Existing UI Classes
# =========================================================================

class PSD_UL_BonePair(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        arm = context.object
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            if arm and hasattr(arm, "data"):
                row.prop_search(item, "bone_name", arm.data, "bones", text="")
            else:
                row.prop(item, "bone_name", text="")
            count = 0
            try:
                if arm and hasattr(arm, "psd_saved_poses"):
                    count = sum(1 for e in arm.psd_saved_poses if e.bone_name == item.bone_name)
            except Exception:
                count = 0
            row.label(text=f"{count}", icon='DOT')
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="")

class PSD_UL_SavedPose(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            txt = item.name
            if getattr(item, 'is_direct_channel', False):
                txt += f" [Direct {item.channel_axis}]"
            tags = []
            if getattr(item, 'has_rot', False):
                tags.append("R")
            if getattr(item, 'has_loc', False):
                tags.append("L")
            if getattr(item, 'has_sca', False):
                tags.append("S")
            if tags:
                txt += f" [{' '.join(tags)}]"
            layout.label(text=txt)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="", icon='BONE_DATA')

    def filter_items(self, context, data, propname):
        items = getattr(data, propname)
        ln = len(items)

        # 默认：全部隐藏（位标志数组长度必须为 ln）
        filtered = [0] * ln
        new_order = list(range(ln))  # 默认新顺序为原始顺序

        # 获取骨骼过滤器里选中的骨骼
        selected_bone = '<NONE>'
        arm = context.object
        if arm and arm.type == 'ARMATURE':
            try:
                idx = int(getattr(arm, 'psd_bone_pairs_index', 0))
                if 0 <= idx < len(arm.psd_bone_pairs):
                    selected_bone = arm.psd_bone_pairs[idx].bone_name or '<NONE>'
            except Exception:
                selected_bone = '<NONE>'

        if selected_bone == '<NONE>':
            # 显示所有，保留原始顺序
            filtered = [self.bitflag_filter_item] * ln
            new_order = list(range(ln))
        else:
            # 只显示匹配 bone_name 的条目，把匹配项排在前面，其余项按原顺序跟在后面
            matching = []
            for i, item in enumerate(items):
                if item.bone_name == selected_bone:
                    filtered[i] = self.bitflag_filter_item
                    matching.append(i)
                else:
                    filtered[i] = 0
            # new_order 必须是长度为 ln 的排列：先 matching，再剩下的
            remaining = [i for i in range(ln) if i not in matching]
            new_order = matching + remaining

        return filtered, new_order


class PSD_UL_BoneTrigger(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        # item 是 PSDBoneTrigger
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            row.prop(item, "enabled", text="")
            row.label(text=item.name)
            row.label(text=f"[T:{item.target_bone} R:{item.radius:.2f}]")
        else:
            layout.label(text=item.name)


class PSD_UL_ShapeDriverFile(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.prop(item, "filepath", text="", emboss=False)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="")

class PSD_UL_PoseDriverFile(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.prop(item, "filepath", text="", emboss=False)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="")



class PSDPanel(bpy.types.Panel):
    bl_label = "骨骼空间姿态权重(BSW)"
    bl_idname = "VIEW3D_PT_psd_corrector_armature"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'BSW'

    @classmethod
    def poll(cls, context):
        obj = context.object
        return (obj is not None) and (obj.type == 'ARMATURE')


    def draw(self, context):
        layout = self.layout
        scene = context.scene
        arm = context.object
        selected_ARMATURE = _get_selected_armature_name(context)
        # 骨骼过滤器（collapsible）
        box = layout.box()
        row = box.row(align=True)
        icon = 'TRIA_DOWN' if arm.show_psd_filter_settings else 'TRIA_RIGHT'
        row.prop(arm, "show_psd_filter_settings", text="", icon=icon, emboss=False)
        row.label(text="骨骼过滤器", icon='BONE_DATA')

        if arm.show_psd_filter_settings:
            box.label(text=f"已选骨骼对象: {selected_ARMATURE}", icon='ARMATURE_DATA')
            row = box.row(align=True)
            row.template_list("PSD_UL_BonePair", "psd_bone_pairs", arm, "psd_bone_pairs", arm, "psd_bone_pairs_index", rows=3)
            col = row.column(align=True)
            col.operator('psd.add_bone_pair', icon='ADD', text='')
            col.operator('psd.remove_bone_pair', icon='REMOVE', text='')
            col.separator()
            col.operator('psd.move_bone_pair', icon='TRIA_UP', text='').direction = 'UP'
            col.operator('psd.move_bone_pair', icon='TRIA_DOWN', text='').direction = 'DOWN'

            # 配置导出/导入
            box = layout.box()
            box.label(text="配置管理", icon='FILE')
            row = box.row(align=True)
            row.operator('psd.export_config', icon='EXPORT', text='导出')
            row.operator('psd.import_config', icon='IMPORT', text='导入')

        # ==== 新增：BSW 输出模式设置 ====
        main_box = layout.box()
        header = main_box.row(align=True)

        # 1. 展开/折叠切换器
        icon = 'TRIA_DOWN' if arm.show_psd_settings else 'TRIA_RIGHT'
        header.prop(arm, "show_psd_settings", text="", icon=icon, emboss=False)
        header.label(text="BSW 输出模式设置", icon='DRIVER')

        # 只有展开时才显示后续内容
        if arm.show_psd_settings:
            # 模式选择
            row = main_box.row()
            row.prop(arm, "psd_output_mode", text="当前模式")
            
            # 根据模式显示不同的子内容
            sub_box = main_box.box()

            if arm.psd_output_mode == 'STORE_TO_EMPTY':
                sub_box.label(text="存储到 Empty (应用 Shape/Pose Drivers)", icon='EMPTY_AXIS')
                
                row = sub_box.row(align=True)
                row.prop(context.scene, "psd_cache_empty", text="")
                
                col = row.column(align=True)
                col.operator("psd.register_cache_empty_ui", icon='CHECKMARK', text="注册")
                col.operator("psd.unregister_cache_empty_ui", icon='X', text="") # 缩小取消按钮

                # 显示注册状态
                registered_name = "<未设置>"
                try:
                    arm_db = bpy.data.armatures.get(arm.data.name)
                    if arm_db:
                        registered_name = arm_db.get("_psd_cache_obj", "") or "<未设置>"
                except: pass
                
                sub_box.label(text=f"当前注册: {registered_name}", icon='INFO')

            elif arm.psd_output_mode == 'DRIVERS_MODS':
                drive_box = sub_box
                drive_box.label(text="驱动器接管模式 (高性能)", icon='DRIVER')
                
                cache_empty = psd_get_registered_empty(arm)
                
                # 初始化状态变量
                is_muted = False
                has_drivers = False
                
                if cache_empty and cache_empty.animation_data:
                    drivers = cache_empty.animation_data.drivers
                    if len(drivers) > 0:
                        has_drivers = True
                        is_muted = drivers[0].mute # 以第一个驱动器作为状态判定参考

                # --- 状态看板 ---
                if not has_drivers:
                    drive_box.label(text="未绑定驱动器，请点击全部绑定", icon='INFO')
                else:
                    status_text = "状态: 已禁用 (静音)" if is_muted else "状态: 运行中 (计算已接管)"
                    drive_box.label(text=status_text, icon='CHECKMARK' if not is_muted else 'CANCEL')

                # --- 动态功能按键区 ---
                row = drive_box.row(align=True)
                
                if not is_muted and has_drivers:
                    # 如果当前在运行，显示“禁用”按钮
                    row.operator('psd.mute_drivers', text='禁用驱动', icon='RESTRICT_VIEW_ON')
                else:
                    # 如果当前已禁用，显示“启动”按钮
                    row.operator('psd.unmute_drivers', text='启动驱动', icon='RESTRICT_VIEW_OFF')
                
                row.operator('psd.refresh_drivers', text='修复刷新', icon='FILE_REFRESH')

                # --- 绑定与管理 ---
                row = drive_box.row(align=True)
                row.operator('psd.batch_bind_all', text='全部绑定', icon='ASSET_MANAGER')
                row.operator('psd.bind_driver_to_cache', text='绑定选中', icon='ADD')

                # --- 第三行：清理操作 ---
                row = drive_box.row(align=True)
                row.operator('psd.remove_driver_from_cache', text='删除选中', icon='REMOVE')
                
                # --- 第四行：危险区域 ---
                drive_box.separator()
                col = drive_box.column()
                col.alert = True
                col.operator('psd.clear_all_cache_drivers', text='清空所有驱动器', icon='TRASH')

            elif arm.psd_output_mode == 'APPLY_DRIVERS_JSON':
                sub_box.label(text="直接应用到 Shape Keys / 骨骼约束", icon='SHAPEKEY_DATA')
                
                # 统计信息
                arm_key = _arm_key_for_obj(arm)
                shape_count = len(_shape_expressions_cache.get(arm_key, {}))
                pose_drivers = _pose_drivers_cache.get(arm_key, {})
                pose_count = sum(len(prop_dict) for constraint_dict in pose_drivers.values() for prop_dict in constraint_dict.values()) if pose_drivers else 0
                
                stats = sub_box.column(align=True)
                stats.label(text=f"• 已加载 Shape Driver: {shape_count}", icon='DOT')
                stats.label(text=f"• 已加载 Pose Driver: {pose_count}", icon='DOT')

                # ---- JSON 文件管理 (列表) ----
                # Shape Driver 列表
                s_box = sub_box.box()
                s_box.label(text="Shape Driver JSON", icon='SHAPEKEY_DATA')
                row = s_box.row(align=True)
                row.template_list("PSD_UL_ShapeDriverFile", "", arm, "psd_shape_driver_files", arm, "psd_shape_driver_files_index", rows=3)
                
                col = row.column(align=True)
                col.operator("psd.add_shape_driver_file", icon='ADD', text="")
                col.operator("psd.remove_shape_driver_file", icon='REMOVE', text="")
                col.separator()
                col.operator("psd.reload_shape_drivers", icon='FILE_REFRESH', text="")

                # Pose Driver 列表
                p_box = sub_box.box()
                p_box.label(text="Pose Driver JSON", icon='CONSTRAINT_BONE')
                row = p_box.row(align=True)
                row.template_list("PSD_UL_PoseDriverFile", "", arm, "psd_pose_driver_files", arm, "psd_pose_driver_files_index", rows=3)
                
                col = row.column(align=True)
                col.operator("psd.add_pose_driver_file", icon='ADD', text="")
                col.operator("psd.remove_pose_driver_file", icon='REMOVE', text="")
                col.separator()
                col.operator("psd.reload_pose_drivers", icon='FILE_REFRESH', text="")


        # 显示已选骨骼
        selected_bone = _get_selected_pair_bone(context)
        # layout.label(text=f"已选骨骼: {selected_bone}", icon='INFO')

        main_box = layout.box()
        header = main_box.row(align=True)

        # 1. 展开/折叠切换器
        icon = 'TRIA_DOWN' if arm.show_psd_manage_settings else 'TRIA_RIGHT'
        header.prop(arm, "show_psd_manage_settings", text="", icon=icon, emboss=False)
        header.label(text="BSW 条目设置", icon='PREFERENCES')
        if arm.show_psd_manage_settings:
            # 捕捉按钮分组
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_captures", icon='ZOOM_IN' if scene.psd_show_captures else 'ZOOM_OUT', text="捕捉姿态")
            if scene.psd_show_captures:
                box.label(text="捕捉姿态", icon='CAMERA_DATA')
                # 显示已选骨骼
                box.label(text=f"已选骨骼: {selected_bone}", icon='INFO')
                row = box.row(align=True)
                row.operator('psd.capture_rest', icon='CON_ROTLIMIT', text='静止旋转')
                row.operator('psd.capture_rotation', icon='CON_ROTLIKE', text='姿态旋转')
                row = box.row(align=True)
                row.operator('psd.capture_loc_rest', icon='CON_LOCLIMIT', text='静止位置')
                row.operator('psd.capture_location', icon='CON_LOCLIKE', text='姿态位置')
                row = box.row(align=True)
                row.operator('psd.capture_sca_rest', icon='CON_SIZELIMIT', text='静止缩放')
                row.operator('psd.capture_scale', icon='CON_SIZELIKE', text='姿态缩放')

            inner_box = box.box()
            # 临时捕捉数据（collapsible）
            if scene.psd_show_captures:
                if scene.psd_temp_rest_bone:
                    r = scene.psd_temp_rest
                    inner_box.label(text=f"旋转 (静止): {scene.psd_temp_rest_bone} (X={r[0]:.2f}°, Y={r[1]:.2f}°, Z={r[2]:.2f}°)")
                if scene.psd_temp_pose_bone:
                    p = scene.psd_temp_pose
                    inner_box.label(text=f"旋转 (姿态): {scene.psd_temp_pose_bone} (X={p[0]:.2f}°, Y={p[1]:.2f}°, Z={p[2]:.2f}°)")
                else:
                    inner_box.label(text='无旋转捕捉')
                if scene.psd_temp_loc_rest_bone:
                    lr = scene.psd_temp_loc_rest
                    inner_box.label(text=f"位置 (静止): {scene.psd_temp_loc_rest_bone} (X={lr[0]:.4f}, Y={lr[1]:.4f}, Z={lr[2]:.4f})")
                if scene.psd_temp_loc_bone:
                    l = scene.psd_temp_loc
                    inner_box.label(text=f"位置 (姿态): {scene.psd_temp_loc_bone} (X={l[0]:.4f}, Y={l[1]:.4f}, Z={l[2]:.4f})")
                else:
                    inner_box.label(text='无位置捕捉')
                if scene.psd_temp_sca_rest_bone:
                    sr = scene.psd_temp_sca_rest
                    inner_box.label(text=f"缩放 (静止): {scene.psd_temp_sca_rest_bone} (X={sr[0]:.4f}, Y={sr[1]:.4f}, Z={sr[2]:.4f})")
                if scene.psd_temp_sca_bone:
                    s = scene.psd_temp_sca
                    inner_box.label(text=f"缩放 (姿态): {scene.psd_temp_sca_bone} (X={s[0]:.4f}, Y={s[1]:.4f}, Z={s[2]:.4f})")
                else:
                    inner_box.label(text='无缩放捕捉')

            # 保存按钮分组
            if scene.psd_show_captures:
                box.label(text="保存条目", icon='FILE_TICK')
                row = box.row(align=True)
                row.operator('psd.save_captured_rotation', text='旋转', icon='CON_ROTLIKE')
                row.operator('psd.save_captured_location', text='位置', icon='CON_LOCLIKE')
                row.operator('psd.save_captured_scale', text='缩放', icon='CON_SIZELIKE')
                row = box.row(align=True)
                row.operator('psd.record_channel_x', text="Record X", icon='AXIS_FRONT')
                row.operator('psd.record_channel_y', text="Record Y", icon='AXIS_SIDE')
                row.operator('psd.record_channel_z', text="Record Z", icon='AXIS_TOP')

            # 已保存姿态（collapsible）
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_saved_poses", icon='ZOOM_IN' if scene.psd_show_saved_poses else 'ZOOM_OUT', text="已保存姿态")
            if scene.psd_show_saved_poses:
                bone_name = selected_bone if selected_bone != '<NONE>' else '无'
                subbox = box.box()
                subbox.label(text=f'骨骼 {bone_name} 的姿态条目:', icon='PRESET')
                subrow = subbox.row()
                subrow.template_list('PSD_UL_SavedPose', 'psd_saved_poses', arm, 'psd_saved_poses', arm, 'psd_saved_pose_index')
                col = subrow.column(align=True)
                col.operator('psd.add_saved_entry', icon='ADD', text='')
                col.operator('psd.remove_saved_entry', icon='REMOVE', text='')

                # 选中条目详情
                #检查索引有效性
                if arm.psd_saved_poses and arm.psd_saved_pose_index >= 0 and arm.psd_saved_pose_index < len(arm.psd_saved_poses):
                    e = arm.psd_saved_poses[arm.psd_saved_pose_index]
                    if e.bone_name == selected_bone or selected_bone == '<NONE>':
                        subbox.prop(e, 'name', text="名称")
                        subbox.label(text=f"骨骼: {e.bone_name}")
                        subbox.operator('psd.apply_save_pose', text='应用保存姿态', icon='POSE_HLT')

                        row_main = subbox.row(align=True)
                        icon_status_r = 'TRIA_RIGHT'
                        text_status_r = "旋转通道配置"

                        if e.use_rot_settings: 
                            icon_status_r = 'TRIA_DOWN'
                            text_status_r = "旋转通道配置"
                            row_main.alert = False
                        else:
                            # 在收起状态下，根据模式显示不同的指示器
                            if e.is_direct_channel:
                                text_status_r = "旋转通道配置(直接旋转模式)"
                                icon_status_r = 'FORWARD'
                                row_main.alert = True
                            elif e.has_rot:
                                text_status_r = "旋转通道配置(默认旋转模式)"
                                icon_status_r = 'CHECKMARK'
                                row_main.alert = True
                            
                        row_main.prop(e, "use_rot_settings", 
                                    text=text_status_r, 
                                    icon=icon_status_r, 
                                    toggle=True)
                        # 旋转详情
                        if e.use_rot_settings:
                            rot_container = subbox.box()
                            
                            # 在容器最上方放置模式切换
                            row_mode = rot_container.row(align=True)
                            row_mode.prop(e, "is_direct_channel", text="直接通道模式", icon='FORWARD')
                            
                            # 分隔线，区分模式选择和具体参数
                            rot_container.separator()

                            if e.is_direct_channel:
                                
                                inner_box = rot_container.box()
                                col = inner_box.column(align=True)
                                
                                
                                col.label(text="直接映射模式配置", icon='CON_TRANSFORM')
                                col.separator() 

                                
                                param_col = inner_box.column() 
                                param_col.prop(e, "channel_axis", text="驱动轴")
                                param_col.separator()
                                param_col.prop(e, 'record_rot_channel_mode', text="算法")
                                
                                
                                inner_box.separator()
                                
                                # 锁定提示区
                                row_lock = inner_box.row()
                                row_lock.enabled = False
                                row_lock.prop(e, 'has_rot', text='已由直接模式接管', icon='LOCKED')

                                
                                rot_container.separator()

                            else:
                                # 第一行：功能总控
                                row = rot_container.row()
                                row.prop(e, 'has_rot', text='参与计算') # 即使这里勾掉，UI也不会消失，只是逻辑上不参与
                                row.prop(e, 'cone_enabled', text='锥形衰减')
                                row.prop(e, "is_editing", text="", icon='DECORATE_UNLOCKED' if e.is_editing else 'DECORATE_LOCKED', toggle=True)
                            
                                # 第二行：数值修改区域
                                col = rot_container.column(align=True)
                                col.enabled = e.has_rot and e.is_editing  # 如果 has_rot 为 False，数值变灰但依然可见，防止UI跳动
                                # --- 静止旋转行 ---
                                row_rest = col.row(align=True)
                                row_rest.prop(e, "rest_rot", text="静止旋转")
                                if e.is_editing:
                                    # 传入 target_type='REST'
                                    op_rest = row_rest.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                    op_rest.target_type, op_rest.channel = 'REST', 'ROT'

                                # --- 姿态旋转行 ---
                                row_pose = col.row(align=True)
                                row_pose.prop(e, "pose_rot", text="姿态旋转")
                                if e.is_editing:
                                    # 传入 target_type='POSE'
                                    op_pose = row_pose.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                    op_pose.target_type, op_pose.channel = 'POSE', 'ROT'
                                if not e.is_editing and e.has_rot:
                                    rot_container.label(text="点击右上方小锁以修改数值", icon='INFO')
                                # 第三行：锥形衰减细节
                                if e.cone_enabled:
                                    cone_box = rot_container.column(align=True)
                                    row_cone = cone_box.row(align=True)
                                    row_cone.prop(e, 'show_cone', text='显示衰减立体角(debug)', icon='CONE')
                                    row_cone.prop(e, 'cone_angle', text='角度')
                                    cone_box.separator()
                                    cone_box.prop(e, 'cone_axis', text='轴向')

                        row_loc = subbox.row(align=True)
                        icon_status_l = 'TRIA_RIGHT'
                        text_status_l = "位置通道配置"

                        if e.use_loc_settings: 
                            icon_status_l = 'TRIA_DOWN'
                            text_status_l = "位置通道配置"
                            row_loc.alert = False
                        else:
                            if e.has_loc:
                                text_status_l = "位置通道配置(已启动)"
                                icon_status_l = 'CHECKMARK'
                                row_loc.alert = True
                            
                        row_loc.prop(e, "use_loc_settings", text=text_status_l, 
                                    icon=icon_status_l, toggle=True)

                        if e.use_loc_settings:
                            loc_box = subbox.box()
                            
                            # 顶部控制行
                            row_ctrl = loc_box.row(align=True)
                            row_ctrl.prop(e, 'has_loc', text='参与计算')
                            row_ctrl.prop(e, 'loc_enabled', text='轴向衰减')
                            row_ctrl.prop(e, "is_editing_loc", text="", icon='DECORATE_UNLOCKED' if e.is_editing_loc else 'DECORATE_LOCKED', toggle=True)

                            # 数值编辑区
                            col = loc_box.column(align=True)
                            col.enabled = e.has_loc and e.is_editing_loc
                            
                            # 静止位置 + 吸附
                            r1 = col.row(align=True)
                            r1.prop(e, "rest_loc", text="静止位置")
                            if e.is_editing_loc:
                                op = r1.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'REST', 'LOC'

                            # 姿态位置 + 吸附
                            r2 = col.row(align=True)
                            r2.prop(e, "pose_loc", text="姿态位置")
                            if e.is_editing_loc:
                                op = r2.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'POSE', 'LOC'

                            # 衰减球体设置
                            if e.loc_enabled:
                                sphere_box = loc_box.column(align=True)
                                row_s = sphere_box.row(align=True)
                                row_s.prop(e, 'show_sphere', text='显示衰减球体(debug)', icon='SPHERE')
                                row_s.prop(e, 'loc_radius', text='半径')

                    # subbox.separator()
                    # 缩放详情
                    # if getattr(e, 'has_sca', False):
                        # --- 缩放通道配置 ---
                        row_sca = subbox.row(align=True)
                        icon_status_s = 'TRIA_RIGHT'
                        text_status_s = "缩放通道配置"

                        if e.use_loc_settings: 
                            icon_status_s = 'TRIA_DOWN'
                            text_status_s = "缩放通道配置"
                            row_sca.alert = False
                        else:
                            if e.has_sca:
                                text_status_s = "缩放通道配置(已启动)"
                                icon_status_s = 'CHECKMARK'
                                row_sca.alert = True
                        row_sca.prop(e, "use_sca_settings", text=text_status_s, 
                                    icon=icon_status_s, toggle=True)

                        if e.use_sca_settings:
                            sca_box = subbox.box()
                            
                            # 顶部控制行
                            row_ctrl = sca_box.row(align=True)
                            row_ctrl.prop(e, 'has_sca', text='参与计算')
                            row_ctrl.prop(e, "is_editing_sca", text="", icon='DECORATE_UNLOCKED' if e.is_editing_sca else 'DECORATE_LOCKED', toggle=True)

                            # 数值编辑区
                            col = sca_box.column(align=True)
                            col.enabled = e.has_sca and e.is_editing_sca
                            
                            # 静止缩放 + 吸附
                            r1 = col.row(align=True)
                            r1.prop(e, "rest_sca", text="静止缩放")
                            if e.is_editing_sca:
                                op = r1.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'REST', 'SCA'

                            # 姿态缩放 + 吸附
                            r2 = col.row(align=True)
                            r2.prop(e, "pose_sca", text="姿态缩放")
                            if e.is_editing_sca:
                                op = r2.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'POSE', 'SCA'


            # 骨骼触发器（collapsible）
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_triggers", icon='ZOOM_IN' if scene.psd_show_triggers else 'ZOOM_OUT', text="骨骼触发器")
            if scene.psd_show_triggers:
                subrow = box.row()
                subrow.template_list("PSD_UL_BoneTrigger", "psd_triggers", arm, "psd_triggers", arm, "psd_trigger_index", rows=4)
                col = subrow.column(align=True)
                col.operator("psd.add_trigger", icon='ADD', text="")
                col.operator("psd.remove_trigger", icon='REMOVE', text="")
                col.separator()
                col.operator("psd.select_trigger_bone", icon='EYEDROPPER', text="").mode = 'TRIGGER'
                col.operator("psd.select_trigger_bone", icon='RESTRICT_SELECT_OFF', text="").mode = 'TARGET'

                # 选中触发器详情
                idx = arm.psd_trigger_index if arm.psd_trigger_index >= 0 and arm.psd_trigger_index < len(arm.psd_triggers) else -1
                if idx != -1:
                    trig = arm.psd_triggers[idx]
                    box.prop(trig, "enabled")
                    box.prop(trig, "name")
                    box.label(text=f"触发骨骼: {trig.bone_name}")
                    box.prop(trig, "target_bone", text="目标骨骼")
                    box.prop(trig, "show_sphere")
                    box.prop(trig, "radius")
                    box.prop(trig, "falloff")
                    roww = box.row()
                    roww.label(text=f"实时权重: {trig.last_weight:.3f}")


        # BSW 结果显示（collapsible with box）
        box = layout.box()
        header = box.row(align=True)
        
        icon_res = 'TRIA_DOWN' if scene.psd_show_results else 'TRIA_RIGHT'
        header.prop(scene, "psd_show_results", text="", icon=icon_res, emboss=False)
        header.label(text="BSW 缓存结果 (Debug)", icon='GRAPH')
        
        if scene.psd_show_results:
            subbox = box.box()
            # 搜索与排序工具栏
            row_tools = subbox.row(align=True)
            row_tools.prop(scene, "psd_results_search", text="", icon='VIEWZOOM')
            row_tools.prop(scene, "psd_results_sort_reverse", text="", icon='SORT_DESC', toggle=True)
            row_tools.prop(scene, "psd_results_limit", text="上限")

            results = []
            
            # 1. 获取当前骨架的 Key (通常是 arm.name 或 _arm_key_for_obj)
            arm_key = _arm_key_for_obj(arm)
            # 从内存字典直接获取该骨架的所有结果数据
            arm_results = _psd_results_cache.get(arm_key, {})
            
            # 获取性能统计数据
            perf_entries = _psd_perf_stats.get(arm.name, {}).get("entries", {}) if scene.psd_perf_enabled else {}

            # 2. 收集数据
            for k, v in arm_results.items():
                # 计算显示名称
                if k.startswith(PREFIX_RESULT): display_name = k[len(PREFIX_RESULT):]+ " (R)"
                elif k.startswith(PREFIX_RESULT_LOC): display_name = k[len(PREFIX_RESULT_LOC):] + " (L)"
                elif k.startswith(PREFIX_RESULT_SCA): display_name = k[len(PREFIX_RESULT_SCA):] + " (S)"
                else: display_name = k
                
                results.append({
                    "name": display_name,
                    "val": v,
                    "key": k,
                    "avg_ms": perf_entries.get(k, {}).get("avg_ms", 0.0)
                })

            # 3. 过滤
            query = (scene.psd_results_search or "").strip().lower()
            if query:
                results = [r for r in results if query in r["name"].lower() or query in r["key"].lower()]
            
            # 4. 排序
            results.sort(key=lambda r: r["name"].lower(), reverse=bool(scene.psd_results_sort_reverse))

            # --- 渲染 ---
            res_col = subbox.column(align=True)
            limit = int(scene.psd_results_limit)
            
            if not results:
                res_col.label(text="等待数据或无匹配项...", icon='INFO')
            else:
                for i, r in enumerate(results):
                    if i >= limit: break
                    
                    row = res_col.row(align=True)
                    # 使用 split 确保对齐
                    split = row.split(factor=0.65)
                    split.label(text=r["name"])
                    
                    val_row = split.row()
                    val_row.alignment = 'RIGHT'
                    
                    # 性能数据显示
                    perf_txt = f" [{r['avg_ms']:.2f}ms]" if scene.psd_perf_enabled else ""
                    val_row.label(text=f"{r['val']:.4f}{perf_txt}")

                if len(results) > limit:
                    subbox.label(text=f"还有 {len(results)-limit} 条未显示", icon='VIEWZOOM')
        else:
            box.label(text="点击启动debug", icon='INFO')


        # 运行模式 & 配置 (collapsible)
        box = layout.box()
        row = box.row(align=True)
        icon_toggle = 'TRIA_DOWN' if arm.show_psd_run_settings else 'TRIA_RIGHT'
        row.prop(arm, "show_psd_run_settings", text="", icon=icon_toggle, emboss=False)
        row.label(text="运行模式 & 配置", icon='SETTINGS') # 这里改成了齿轮
        
        if arm.show_psd_run_settings:
            col = box.column(align=True)
            col.prop(scene, "psd_mode")
            col.prop(scene, "psd_perf_enabled")
            if scene.psd_mode in ('AUTO', 'FORCE_TIMER'):
                box.prop(scene, "psd_idle_hz", text="空闲 Hz", slider=True)
            # --- 性能监控显示区 ---
            if scene.psd_perf_enabled:
                perf_box = box.box() 
                perf_box.label(text="性能分析 (实时)", icon='WORDWRAP_ON')
                
                # 1. 检查全局字典是否存在
                from . import core
                stats_dict = getattr(core, "_psd_perf_stats", None)
                
                if stats_dict is None:
                    perf_box.label(text="错误: 找不到统计字典", icon='ERROR')
                else:
                    # 2. 获取数据
                    g_stats = stats_dict.get("__global__", {})
                    global_total = g_stats.get("last_total_ms", 0.0)
                    
                    # 尝试用物体名和骨架数据名同时查找
                    arm_stats = stats_dict.get(arm.name) or stats_dict.get(arm.data.name)
                    
                    if arm_stats or global_total > 0:
                        col = perf_box.column(align=True)
                        col.label(text=f"总耗时: {global_total:.3f} ms", icon='TIME')
                        
                        if arm_stats:
                            arm_total = arm_stats.get('last_arm_ms', 0.0)
                            col.label(text=f"骨架 [{arm.name}]: {arm_total:.3f} ms")
                            
                            entries = arm_stats.get("entries", {})
                            if entries:
                                col.separator()
                                sorted_entries = sorted(entries.items(), key=lambda x: x[1].get("avg_ms", 0.0), reverse=True)[:10]
                                for key, estats in sorted_entries:
                                    avg = estats.get("avg_ms", 0.0)
                                    # if avg < 0.0001: continue
                                    split = col.split(factor=0.7)
                                    split.label(text=key.split(":")[-1]) # 简化 Key 显示
                                    split.label(text=f"{avg:.3f}ms")
                        else:
                            perf_box.label(text="有全局数据，但无当前骨架数据", icon='INFO')
                            perf_box.label(text=f"查找名: {arm.name}", icon='DOT')
                    else:
                        perf_box.label(text="字典为空，等待计算触发...", icon='INFO')
                        if stats_dict:
                            perf_box.label(text=f"已存 Key: {list(stats_dict.keys())[:3]}")

            # 启动/停止按钮
            row = layout.row(align=True)
            if scene.psd_running:
                row.operator('object.psd_stop', icon='CANCEL', text='停止')
            else:
                row.operator('object.psd_start', icon='PLAY', text='启动')

            layout.label(text='(姿态数据保存在骨架上；结果写入骨架数据块)', icon='INFO')
            layout.operator("psd.invalidate_bone_cache", icon='FILE_REFRESH', text='清缓存')
