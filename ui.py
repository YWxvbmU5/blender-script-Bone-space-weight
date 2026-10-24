import bpy
import gpu
import math
from mathutils import Vector, Matrix, Euler
from gpu_extras.batch import batch_for_shader
from .core import _psd_results_cache, _psd_perf_stats

from .utils import _get_selected_pair_bone, _get_selected_armature_name, psd_get_registered_empty, get_weight_cache, update_direct_channel  # 导入辅助
from .operators import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA, get_mirror_name
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
        
        # 主行
        col_main = layout.column(align=True)
        row = col_main.row(align=True)
        
        icon_tri = 'TRIA_DOWN' if item.show_mirror_details else 'DOT'
        row.label(text="", icon=icon_tri)
        
        # 骨骼搜索
        row.prop_search(item, "bone_name", arm.data, "bones", text="")

        # 如果展开，拉出详情
        if item.show_mirror_details:
            
            sub_box = col_main.box()
            mirror_name = get_mirror_name(item.bone_name)
            
            # 左右状态横向排列
            split = sub_box.split(factor=0.5)
            
            # 左侧显示
            l_col = split.column()
            l_is_me = ".L" in item.bone_name.upper()
            l_name = item.bone_name if l_is_me else mirror_name
            l_col.label(text=f"L: {l_name}", icon='CHECKMARK' if (l_name and l_name in arm.data.bones) else 'CANCEL')
            
            # 右侧显示
            r_col = split.column()
            r_is_me = ".R" in item.bone_name.upper()
            r_name = item.bone_name if r_is_me else mirror_name
            r_col.label(text=f"R: {r_name}", icon='CHECKMARK' if (r_name and r_name in arm.data.bones) else 'CANCEL')
# class PSD_UL_BonePair(bpy.types.UIList):
#     def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
#         arm = context.object
#         if self.layout_type in {'DEFAULT', 'COMPACT'}:
#             row = layout.row(align=True)
#             if arm and hasattr(arm, "data"):
#                 row.prop_search(item, "bone_name", arm.data, "bones", text="")
#             else:
#                 row.prop(item, "bone_name", text="")
#             count = 0
#             try:
#                 if arm and hasattr(arm, "psd_saved_poses"):
#                     count = sum(1 for e in arm.psd_saved_poses if e.bone_name == item.bone_name)
#             except Exception:
#                 count = 0
#             row.label(text=f"{count}", icon='DOT')
#         elif self.layout_type == 'GRID':
#             layout.alignment = 'CENTER'
#             layout.label(text="")

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
    bl_label = "Bone Space Pose Weight (BPW)"
    bl_idname = "VIEW3D_PT_psd_corrector_armature"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'BPW'

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
        row.label(text="Bone Filter", icon='BONE_DATA')

        if arm.show_psd_filter_settings:
            # 标题行增加一键开关
            head = box.row()
            head.label(text=f"Bone Filter ({len(arm.psd_bone_pairs)})", icon='FILTER')
            
            # 全局开关按钮
            icon_toggle = 'FULLSCREEN_ENTER' if not arm.psd_filter_show_all_details else 'FULLSCREEN_EXIT'
            text_toggle = bpy.app.translations.pgettext("Expand All") if not arm.psd_filter_show_all_details else bpy.app.translations.pgettext("Collapse All")
            head.operator("psd.toggle_all_filter_details", text=text_toggle, icon=icon_toggle)

            row = box.row(align=True)
            # 列表部分
            row.template_list("PSD_UL_BonePair", "psd_bone_pairs", arm, "psd_bone_pairs", arm, "psd_bone_pairs_index", rows=5)
            
            # 右侧操作列
            col = row.column(align=True)
            col.operator('psd.add_bone_pair', icon='ADD', text='')
            col.operator('psd.remove_bone_pair', icon='REMOVE', text='')
            
            # 镜像操作组
            col.separator()
            col.operator("psd.mirror_bone_filter", icon='MOD_MIRROR', text="")
            col.operator("psd.sync_mirror_data", icon='PASTEDOWN', text="")

            col.separator()
            col.operator('psd.move_bone_pair', icon='TRIA_UP', text='').direction = 'UP'
            col.operator('psd.move_bone_pair', icon='TRIA_DOWN', text='').direction = 'DOWN'

            # 配置导出/导入
            box = layout.box()
            box.label(text=bpy.app.translations.pgettext("Configuration Management"), icon='FILE')
            row = box.row(align=True)
            row.operator('psd.export_config', icon='EXPORT', text=bpy.app.translations.pgettext("Export"))
            row.operator('psd.import_config', icon='IMPORT', text=bpy.app.translations.pgettext("Import"))

        # ==== 新增：BPW 输出模式设置 ====
        main_box = layout.box()
        header = main_box.row(align=True)

        # 1. 展开/折叠切换器
        icon = 'TRIA_DOWN' if arm.show_psd_settings else 'TRIA_RIGHT'
        header.prop(arm, "show_psd_settings", text="", icon=icon, emboss=False)
        header.label(text="BPW Output Mode Settings", icon='DRIVER')

        # 只有展开时才显示后续内容
        if arm.show_psd_settings:
            # 模式选择
            row = main_box.row()
            row.prop(arm, "psd_output_mode", text="Current Mode")
            
            # 根据模式显示不同的子内容
            sub_box = main_box.box()

            if arm.psd_output_mode == 'STORE_TO_EMPTY':
                sub_box.label(text="Store to Empty (Apply Shape/Pose Drivers)", icon='EMPTY_AXIS')
                
                row = sub_box.row(align=True)
                row.prop(context.scene, "psd_cache_empty", text="")
                
                col = row.column(align=True)
                col.operator("psd.register_cache_empty_ui", icon='CHECKMARK', text="Register")
                col.operator("psd.unregister_cache_empty_ui", icon='X', text="") # 缩小取消按钮

                # 显示注册状态
                registered_name = "<Not Set>"
                try:
                    arm_db = bpy.data.armatures.get(arm.data.name)
                    if arm_db:
                        registered_name = arm_db.get("_psd_cache_obj", "") or "<Not Set>"
                except: pass
                
                sub_box.label(text=f"Current Registered: {registered_name}", icon='INFO')

            elif arm.psd_output_mode == 'DRIVERS_MODS':
                drive_box = sub_box
                drive_box.label(text="Driver Takeover", icon='DRIVER')
                
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
                    drive_box.label(text="No Drivers Bound, Click Bind All", icon='INFO')
                else:
                    status_text = "Status: Disabled (Muted)" if is_muted else "Status: Running (Calculation Taken Over)"
                    drive_box.label(text=status_text, icon='CHECKMARK' if not is_muted else 'CANCEL')

                # --- 动态功能按键区 ---
                row = drive_box.row(align=True)
                
                if not is_muted and has_drivers:
                    # 如果当前在运行，显示“禁用”按钮
                    row.operator('psd.mute_drivers', text=bpy.app.translations.pgettext("Disable Drivers"), icon='RESTRICT_VIEW_ON')
                else:
                    # 如果当前已禁用，显示"启动"按钮
                    row.operator('psd.unmute_drivers', text=bpy.app.translations.pgettext("Enable Drivers"), icon='RESTRICT_VIEW_OFF')
                
                row.operator('psd.refresh_drivers', text=bpy.app.translations.pgettext("Refresh Fix"), icon='FILE_REFRESH')

                # --- 绑定与管理 ---
                row = drive_box.row(align=True)
                row.operator('psd.batch_bind_all', text=bpy.app.translations.pgettext("Bind All"), icon='ASSET_MANAGER')
                row.operator('psd.bind_driver_to_cache', text=bpy.app.translations.pgettext("Bind Selected"), icon='ADD')


                # --- 第三行：清理操作 ---
                row = drive_box.row(align=True)
                row.operator('psd.remove_driver_from_cache', text=bpy.app.translations.pgettext("Delete Selected"), icon='REMOVE')
                
                # --- 第四行：危险区域 ---
                # drive_box.separator()
                col = drive_box.column()
                col.alert = True
                col.operator('psd.clear_all_cache_drivers', text=bpy.app.translations.pgettext("Clear All Drivers"), icon='TRASH')

                # --- 第五行：数据路径与检索 (可折叠 + 性能限制) ---
                # drive_box.separator()
                
                # # 1. 标题切换行
                # row = drive_box.row(align=True)
                # show_icon = 'TRIA_DOWN' if arm.psd_show_path_helper else 'TRIA_RIGHT'
                # row.prop(arm, "psd_show_path_helper", text="PSD Attribute Helper", icon=show_icon, emboss=False)
                
                # if arm.psd_show_path_helper:
                #     search_section = drive_box.box()
                    
                #     # --- 顶部控制栏 ---
                #     ctrl_row = search_section.row(align=True)
                #     ctrl_row.prop(arm, "psd_search_filter", text="", icon='VIEWZOOM', placeholder="Search Props...")
                    
                #     sort_icon = 'SORTALPHA' if arm.psd_sort_order == 'ALPHA' else 'SORTTIME'
                #     ctrl_row.prop(arm, "psd_sort_order", text="", icon=sort_icon, toggle=True)

                #     row = search_section.row(align=True)
                #     row.prop(scene, "psd_results_limit", text="Display Limit")
                    
                #     search_section.separator()

                #     # 获取关联的空物体
                #     cache_empty = psd_get_registered_empty(arm)
                #     if cache_empty:
                #         col = search_section.column(align=True)
                #         search_query = arm.psd_search_filter.lower()
                        
                #         # 过滤并准备数据列表
                #         items = [item for item in cache_empty.items() if item[0].startswith("psd_")]
                #         if search_query:
                #             items = [item for item in items if search_query in item[0].lower()]
                        
                #         # 排序
                #         if arm.psd_sort_order == 'ALPHA':
                #             items = sorted(items)

                #         # --- 核心限制逻辑 ---
                #         limit = int(scene.psd_results_limit) # 引用你现有的限制属性
                #         count = 0
                        
                #         for key, value in items:
                #             # 达到限制则跳出循环
                #             if count >= limit: 
                #                 break
                            
                #             count += 1
                            
                #             # 绘制行
                #             split = col.split(factor=0.85, align=True)
                #             path_text = f'["{key}"]'
                #             path_text_value = f"{float(value):.3f}"

                #             left_row = split.row(align=True)
                #             left_row.alignment = 'LEFT' # 强制左对齐
                            
                #             # 左侧：点击复制文字
                #             left_op = left_row.operator("psd.copy_path", text=path_text, icon='ADD', emboss=False)
                #             left_op.path_to_copy = path_text

                #             # 右侧
                #             right_row = split.row(align=True)
                #             right_row.alignment = 'RIGHT' # 强制右对齐

                #             right_row.label(text=path_text_value) # 显示值
                            

                #         # --- 底部提示：告知用户还有更多内容 ---
                #         if len(items) > limit:
                #             more_count = len(items) - limit
                #             row = search_section.row()
                #             row.alignment = 'CENTER'
                #             row.enabled = False # 变灰显示，仅作为提示
                #             row.label(text=f"... {more_count} more items hidden (check Limit) ...", icon='VIEWZOOM')
                        
                #         if not items:
                #             search_section.label(text="No matches found.", icon='INFO')
                #     else:
                #         search_section.label(text="No Registered Empty Found", icon='ERROR')


            elif arm.psd_output_mode == 'APPLY_DRIVERS_JSON':
                sub_box.label(text="Apply Directly to Shape Keys / Bone Constraints", icon='SHAPEKEY_DATA')
                
                # 统计信息
                arm_key = _arm_key_for_obj(arm)
                shape_count = len(_shape_expressions_cache.get(arm_key, {}))
                pose_drivers = _pose_drivers_cache.get(arm_key, {})
                pose_count = sum(len(prop_dict) for constraint_dict in pose_drivers.values() for prop_dict in constraint_dict.values()) if pose_drivers else 0
                
                stats = sub_box.column(align=True)
                stats.label(text=f"• Loaded Shape Drivers: {shape_count}", icon='DOT')
                stats.label(text=f"• Loaded Pose Drivers: {pose_count}", icon='DOT')

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
        header.label(text="BPW Entry Settings", icon='PREFERENCES')
        if arm.show_psd_manage_settings:
            # 捕捉按钮分组
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_captures", icon='ZOOM_IN' if scene.psd_show_captures else 'ZOOM_OUT', text="Capture Pose")
            if scene.psd_show_captures:
                box.label(text="Capture Pose", icon='CAMERA_DATA')
                # 显示已选骨骼
                box.label(text=f"Selected Bone: {selected_bone}", icon='INFO')
                row = box.row(align=True)
                row.operator('psd.capture_rest', icon='CON_ROTLIMIT', text=bpy.app.translations.pgettext("Rest Rotation"))
                row.operator('psd.capture_rotation', icon='CON_ROTLIKE', text=bpy.app.translations.pgettext("Pose Rotation"))
                row = box.row(align=True)
                row.operator('psd.capture_loc_rest', icon='CON_LOCLIMIT', text=bpy.app.translations.pgettext("Rest Location"))
                row.operator('psd.capture_location', icon='CON_LOCLIKE', text=bpy.app.translations.pgettext("Pose Location"))
                row = box.row(align=True)
                row.operator('psd.capture_sca_rest', icon='CON_SIZELIMIT', text=bpy.app.translations.pgettext("Rest Scale"))
                row.operator('psd.capture_scale', icon='CON_SIZELIKE', text=bpy.app.translations.pgettext("Pose Scale"))

                inner_box = box.box()
                # 临时捕捉数据（collapsible）
                if scene.psd_show_captures:
                    if scene.psd_temp_rest_bone:
                        r = scene.psd_temp_rest
                        inner_box.label(text=f"Rotation (Rest): {scene.psd_temp_rest_bone} (X={r[0]:.2f}°, Y={r[1]:.2f}°, Z={r[2]:.2f}°)")
                    if scene.psd_temp_pose_bone:
                        p = scene.psd_temp_pose
                        inner_box.label(text=f"Rotation (Pose): {scene.psd_temp_pose_bone} (X={p[0]:.2f}°, Y={p[1]:.2f}°, Z={p[2]:.2f}°)")
                    else:
                        inner_box.label(text="No Rotation Captured")
                    if scene.psd_temp_loc_rest_bone:
                        lr = scene.psd_temp_loc_rest
                        inner_box.label(text=f"Location (Rest): {scene.psd_temp_loc_rest_bone} (X={lr[0]:.4f}, Y={lr[1]:.4f}, Z={lr[2]:.4f})")
                    if scene.psd_temp_loc_bone:
                        l = scene.psd_temp_loc
                        inner_box.label(text=f"Location (Pose): {scene.psd_temp_loc_bone} (X={l[0]:.4f}, Y={l[1]:.4f}, Z={l[2]:.4f})")
                    else:
                        inner_box.label(text="No Location Captured")
                    if scene.psd_temp_sca_rest_bone:
                        sr = scene.psd_temp_sca_rest
                        inner_box.label(text=f"Scale (Rest): {scene.psd_temp_sca_rest_bone} (X={sr[0]:.4f}, Y={sr[1]:.4f}, Z={sr[2]:.4f})")
                    if scene.psd_temp_sca_bone:
                        s = scene.psd_temp_sca
                        inner_box.label(text=f"Scale (Pose): {scene.psd_temp_sca_bone} (X={s[0]:.4f}, Y={s[1]:.4f}, Z={s[2]:.4f})")
                    else:
                        inner_box.label(text="No Scale Captured")

            # 保存按钮分组
            if scene.psd_show_captures:
                box.label(text="Save Entry", icon='FILE_TICK')
                row = box.row(align=True)
                row.operator('psd.save_captured_rotation', text="Rotation", icon='CON_ROTLIKE')
                row.operator('psd.save_captured_location', text="Location", icon='CON_LOCLIKE')
                row.operator('psd.save_captured_scale', text="Scale", icon='CON_SIZELIKE')
                row = box.row(align=True)
                row.operator('psd.record_channel_x', text="Record X", icon='AXIS_FRONT')
                row.operator('psd.record_channel_y', text="Record Y", icon='AXIS_SIDE')
                row.operator('psd.record_channel_z', text="Record Z", icon='AXIS_TOP')

            # 已保存姿态（collapsible）
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_saved_poses", icon='ZOOM_IN' if scene.psd_show_saved_poses else 'ZOOM_OUT', text="Saved Poses")
            if scene.psd_show_saved_poses:
                bone_name = selected_bone if selected_bone != '<NONE>' else 'None'
                subbox = box.box()
                subbox.label(text=f'Pose Entries for Bone {bone_name}:', icon='PRESET')
                subrow = subbox.row()
                subrow.template_list('PSD_UL_SavedPose', 'psd_saved_poses', arm, 'psd_saved_poses', arm, 'psd_saved_pose_index')
                col = subrow.column(align=True)
                col.operator('psd.add_saved_entry', icon='ADD', text='')
                #删除过滤：只有当选中条目有效且属于当前选中骨骼时才允许删除
                # --- ui.py 修正后的逻辑 ---

                idx = arm.psd_saved_pose_index
                poses = arm.psd_saved_poses
                selected_bone = context.active_pose_bone.name if context.active_pose_bone else '<NONE>'

                # 1. 初始化变量，确保它们一定是布尔值
                is_valid_idx = bool(0 <= idx < len(poses))
                is_matching_bone = False

                # 2. 只有索引有效时才去拿 entry 算骨骼匹配
                if is_valid_idx:
                    entry = poses[idx]
                    # 强制转换为 bool，防止 Blender 返回 IDProperty 对象
                    is_matching_bone = bool(entry.bone_name == selected_bone or selected_bone == '<NONE>')

                # 3. 赋值给 UI 布局
                del_col = col.column(align=True)

                # 再次强制布尔合并，消除任何返回 bpy_prop_collection_idprop 的可能
                del_col.enabled = bool(is_valid_idx and is_matching_bone)

                del_col.operator('psd.remove_saved_entry', icon='REMOVE', text='')
                
                col.separator(factor=3.0)
                

                col.operator("psd.entries_to_shapekey", text="", icon='SHAPEKEY_DATA')
                col.operator("psd.shape_key_driver", text="", icon='DRIVER')
                #=============== RBF更新按钮 ===============
                col.operator("psd.update_rbf_solver", text="", icon='MESH_ICOSPHERE')


                if is_valid_idx:
                    # 只有当前选中了有效的条目，才显示或者允许切换
                    entry = poses[idx]
                    
                    # 区分开关状态，让按钮在启用时像“按下去”一样高亮
                    # 如果开启了，图标变成实心球或者高亮图标，没开启时是线框
                    ray_icon = 'FIXED_SIZE' if context.scene.show_ray_verts else 'PROP_OFF'
                    
                    col.separator(factor=1.0)
                    # toggle=True 会让它在 true 的时候变成蓝色高亮，交互感极强！
                    col.prop(context.scene, "show_ray_verts", text="", icon=ray_icon, toggle=True)

                # 选中条目详情
                #检查索引有效性
                if arm.psd_saved_poses and arm.psd_saved_pose_index >= 0 and arm.psd_saved_pose_index < len(arm.psd_saved_poses):
                    e = arm.psd_saved_poses[arm.psd_saved_pose_index]
                    if e.bone_name == selected_bone or selected_bone == '<NONE>':
                        subbox.prop(e, 'name', text="Name")
                        subbox.label(text=f"Bone: {e.bone_name}")
                        subbox.operator('psd.apply_save_pose', text=bpy.app.translations.pgettext("Apply Saved Pose"), icon='POSE_HLT')

                        row_main = subbox.row(align=True)
                        icon_status_r = 'TRIA_RIGHT'
                        text_status_r = "Rotation Channel Config"

                        if e.use_rot_settings: 
                            icon_status_r = 'TRIA_DOWN'
                            text_status_r = "Rotation Channel Config"
                            row_main.alert = False
                        else:
                            # 在收起状态下，根据模式显示不同的指示器
                            if e.is_direct_channel:
                                text_status_r = "Rotation Channel Config (Direct Rotation Mode)"
                                icon_status_r = 'FORWARD'
                                row_main.alert = True
                            elif e.has_rot:
                                text_status_r = "Rotation Channel Config (Default Rotation Mode)"
                                icon_status_r = 'CHECKMARK'
                                row_main.alert = True
                            
                        row_main.prop(e, "use_rot_settings", 
                                    text="Rotation Channel Config" if text_status_r == "Rotation Channel Config" else text_status_r, 
                                    icon=icon_status_r, 
                                    toggle=True)
                        # 旋转详情
                        if e.use_rot_settings:
                            rot_container = subbox.box()
                            
                            # 在容器最上方放置模式切换
                            row_mode = rot_container.row(align=True)
                            row_mode.prop(e, "is_direct_channel", text="Direct Channel Mode", icon='FORWARD')
                            
                            # 分隔线，区分模式选择和具体参数
                            rot_container.separator()

                            if e.is_direct_channel:
                                
                                inner_box = rot_container.box()
                                col = inner_box.column(align=True)
                                
                                
                                col.label(text="Direct Mapping Mode Config", icon='CON_TRANSFORM')
                                col.separator() 

                                
                                param_col = inner_box.column() 
                                param_col.prop(e, "channel_axis", text="Drive Axis")
                                param_col.separator()
                                param_col.prop(e, 'record_rot_channel_mode', text="Algorithm")
                                
                                
                                inner_box.separator()
                                
                                # 锁定提示区
                                row_lock = inner_box.row()
                                row_lock.enabled = False
                                row_lock.prop(e, 'has_rot', text='Taken Over by Direct Mode', icon='LOCKED')

                                
                                rot_container.separator()

                            else:
                                # 第一行：功能总控
                                row = rot_container.row()
                                row.prop(e, 'has_rot', text='Participate in Calculation') # 即使这里勾掉，UI也不会消失，只是逻辑上不参与
                                row.prop(e, 'cone_enabled', text='Cone Attenuation')
                                row.prop(e, 'use_rbf', text='Use RBF Mode', icon='MESH_ICOSPHERE') # RBF 模式切换
                                row.prop(e, "is_editing", text="", icon='DECORATE_UNLOCKED' if e.is_editing else 'DECORATE_LOCKED', toggle=True)

                            
                                # 第二行：数值修改区域
                                col = rot_container.column(align=True)
                                col.enabled = e.has_rot and e.is_editing  # 如果 has_rot 为 False，数值变灰但依然可见，防止UI跳动
                                if getattr(e, 'use_rbf', False):
                                    rbf_box = col.box()
                                    rbf_box.label(text="RBF Pose Sample Settings:", icon='FORCE_VORTEX')
                                    
                                    # 姿态旋转行 (RBF 只需要一个目标姿态)
                                    row_pose = rbf_box.row(align=True)
                                    row_pose.prop(e, "pose_rot", text="Target Pose")
                                    if e.is_editing:
                                        op_pose = row_pose.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                        op_pose.target_type, op_pose.channel = 'POSE', 'ROT'
                                    
                                    param_col = rbf_box.column() 
                                    param_col.prop(e, 'record_rot_channel_mode', text="Algorithm")

                                    # RBF 专用参数：平滑度 (Epsilon)
                                    row_para = rbf_box.row(align=True)
                                    row_para.prop(e, "rbf_epsilon", text="Sharpness / Falloff")
                                    
                                    rbf_box.label(text="* RBF automatically mixes all samples for this bone.", icon='INFO')
                                else:
                                    # --- 静止旋转行 ---
                                    row_rest = col.row(align=True)
                                    row_rest.prop(e, "rest_rot", text="Rest Rotation")
                                    if e.is_editing:
                                        # 传入 target_type='REST'
                                        op_rest = row_rest.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                        op_rest.target_type, op_rest.channel = 'REST', 'ROT'

                                    # --- 姿态旋转行 ---
                                    row_pose = col.row(align=True)
                                    row_pose.prop(e, "pose_rot", text="Pose Rotation")
                                    if e.is_editing:
                                        # 传入 target_type='POSE'
                                        op_pose = row_pose.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                        op_pose.target_type, op_pose.channel = 'POSE', 'ROT'
                                    if not e.is_editing and e.has_rot:
                                        rot_container.label(text="Click the small lock in top right to edit values", icon='INFO')
                                    # 第三行：锥形衰减细节
                                    
                                    if e.cone_enabled:
                                        cone_box = rot_container.column(align=True)
                                        row_cone = cone_box.row(align=True)
                                        row_cone.prop(e, 'show_cone', text='Show Attenuation Cone (debug)', icon='CONE')
                                        row_cone.prop(e, 'cone_angle', text='Angle')
                                        cone_box.separator()
                                        
                                        # 轴向与缩放混合行
                                        row_axis_scale = cone_box.row(align=True)
                                        row_axis_scale.prop(e, 'cone_axis', text='Axis')

                                        cone_box.separator()
                                        
                                        # 创建缩放调节区
                                        row_scales = cone_box.row(align=True)
                                        
                                        # X 轴缩放：如果 Axis 不是 X，则允许调节
                                        sub = row_scales.column(align=True)
                                        sub.enabled = (e.cone_axis != 'X')
                                        sub.prop(e, 'cone_scale_x', text='S-X')
                                        
                                        # Y 轴缩放：如果 Axis 不是 Y，则允许调节
                                        sub = row_scales.column(align=True)
                                        sub.enabled = (e.cone_axis != 'Y')
                                        sub.prop(e, 'cone_scale_y', text='S-Y')
                                        
                                        # Z 轴缩放：如果 Axis 不是 Z，则允许调节
                                        sub = row_scales.column(align=True)
                                        sub.enabled = (e.cone_axis != 'Z')
                                        sub.prop(e, 'cone_scale_z', text='S-Z')
                                    else:
                                        col.separator()
                                        inner_box = col.box()
                                        # 模式二：投影映射 + Swing/Twist 算法配置
                                        inner_box.label(text="Projection Settings:", icon='FORCE_DRAG')
                                        param_col = inner_box.column(align=True)
                                        # 只有在 Swing/Twist 模式下才需要选择轴向，NONE 模式下轴向由 Pose 差值决定
                                        param_col.prop(e, 'record_rot_channel_mode', text="Algorithm")

                                        param_col.separator()

                                        param_col.prop(e, 'use_triangle_weights', text="Use Triangle Weights")
                                        
                                        # if e.record_rot_channel_mode != 'NONE':
                                        #     # 当使用了分解算法，需要让用户知道哪个是 Twist 轴
                                        #     param_col.prop(e, "cone_axis", text="Twist Axis") 
                                        #     param_col.label(text="* Swing/Twist will use the selected axis as Twist.", icon='INFO')


                        row_loc = subbox.row(align=True)
                        icon_status_l = 'TRIA_RIGHT'
                        text_status_l = "Location Channel Config"

                        if e.use_loc_settings: 
                            icon_status_l = 'TRIA_DOWN'
                            text_status_l = "Location Channel Config"
                            row_loc.alert = False
                        else:
                            if e.has_loc:
                                text_status_l = "Location Channel Config (Enabled)"
                                icon_status_l = 'CHECKMARK'
                                row_loc.alert = True
                            
                        row_loc.prop(e, "use_loc_settings", text=text_status_l, 
                                    icon=icon_status_l, toggle=True)

                        if e.use_loc_settings:
                            loc_box = subbox.box()
                            
                            # 顶部控制行
                            row_ctrl = loc_box.row(align=True)
                            row_ctrl.prop(e, 'has_loc', text='Participate in Calculation')
                            row_ctrl.prop(e, 'loc_enabled', text='Axial Attenuation')
                            row_ctrl.prop(e, "is_editing_loc", text="", icon='DECORATE_UNLOCKED' if e.is_editing_loc else 'DECORATE_LOCKED', toggle=True)

                            # 数值编辑区
                            col = loc_box.column(align=True)
                            col.enabled = e.has_loc and e.is_editing_loc
                            
                            # 静止位置 + 吸附
                            r1 = col.row(align=True)
                            r1.prop(e, "rest_loc", text="Rest Location")
                            if e.is_editing_loc:
                                op = r1.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'REST', 'LOC'

                            # 姿态位置 + 吸附
                            r2 = col.row(align=True)
                            r2.prop(e, "pose_loc", text="Pose Location")
                            if e.is_editing_loc:
                                op = r2.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'POSE', 'LOC'

                            # 衰减球体设置
                            if e.loc_enabled:
                                sphere_box = loc_box.column(align=True)
                                row_s = sphere_box.row(align=True)
                                row_s.prop(e, 'show_sphere', text='Show Attenuation Sphere (debug)', icon='SPHERE')
                                row_s.prop(e, 'loc_radius', text='Radius')

                    # subbox.separator()
                    # 缩放详情
                    # if getattr(e, 'has_sca', False):
                        # --- 缩放通道配置 ---
                        row_sca = subbox.row(align=True)
                        icon_status_s = 'TRIA_RIGHT'
                        text_status_s = "Scale Channel Config"

                        if e.use_loc_settings: 
                            icon_status_s = 'TRIA_DOWN'
                            text_status_s = "Scale Channel Config"
                            row_sca.alert = False
                        else:
                            if e.has_sca:
                                text_status_s = "Scale Channel Config (Enabled)"
                                icon_status_s = 'CHECKMARK'
                                row_sca.alert = True
                        row_sca.prop(e, "use_sca_settings", text=text_status_s, 
                                    icon=icon_status_s, toggle=True)

                        if e.use_sca_settings:
                            sca_box = subbox.box()
                            
                            # 顶部控制行
                            row_ctrl = sca_box.row(align=True)
                            row_ctrl.prop(e, 'has_sca', text='Participate in Calculation')
                            row_ctrl.prop(e, "is_editing_sca", text="", icon='DECORATE_UNLOCKED' if e.is_editing_sca else 'DECORATE_LOCKED', toggle=True)

                            # 数值编辑区
                            col = sca_box.column(align=True)
                            col.enabled = e.has_sca and e.is_editing_sca
                            
                            # 静止缩放 + 吸附
                            r1 = col.row(align=True)
                            r1.prop(e, "rest_sca", text="Rest Scale")
                            if e.is_editing_sca:
                                op = r1.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'REST', 'SCA'

                            # 姿态缩放 + 吸附
                            r2 = col.row(align=True)
                            r2.prop(e, "pose_sca", text="Pose Scale")
                            if e.is_editing_sca:
                                op = r2.operator("psd.capture_pose", text="", icon='EYEDROPPER')
                                op.target_type, op.channel = 'POSE', 'SCA'


            # 骨骼触发器（collapsible）
            box = layout.box()
            row = box.row(align=True)
            row.prop(scene, "psd_show_triggers", icon='ZOOM_IN' if scene.psd_show_triggers else 'ZOOM_OUT', text="Bone Triggers")
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
                    
                    # 顶部操作行：添加与删除
                    op_row = box.row(align=True)
                    op_row.operator("psd.bind_trigger_driver", icon='ADD', text="Bind Driver")
                    op_row.operator("psd.remove_trigger_driver", icon='REMOVE', text="Remove")
                    
                    box.separator()
                    
                    # 属性编辑区
                    box.prop(trig, "enabled")
                    box.prop(trig, "name")
                    box.label(text=f"Trigger Bone: {trig.bone_name}", icon='BONE_DATA')
                    box.prop(trig, "target_bone", text="Target Bone")
                    
                    split = box.split(factor=0.5)
                    split.prop(trig, "radius")
                    split.prop(trig, "falloff", text="")
                    
                    box.prop(trig, "show_sphere", toggle=True)
                    
                    # 状态显示
                    row_stat = box.row()
                    row_stat.label(text=f"Live Weight: {trig.last_weight:.3f}", icon='FORWARD')


        # BPW 结果显示（collapsible with box）
        box = layout.box()
        header = box.row(align=True)
        
        icon_res = 'TRIA_DOWN' if scene.psd_show_results else 'TRIA_RIGHT'
        header.prop(scene, "psd_show_results", text="", icon=icon_res, emboss=False)
        header.label(text="BPW Cache Results (Debug)", icon='GRAPH')
        
        if scene.psd_show_results:
            subbox = box.box()
            # 搜索与排序工具栏
            row_tools = subbox.row(align=True)
            row_tools.prop(scene, "psd_results_search", text="", icon='VIEWZOOM')
            row_tools.prop(scene, "psd_results_sort_reverse", text="", icon='SORT_DESC', toggle=True)
            row_tools.prop(scene, "psd_results_limit", text="Limit")

            results = []
            
            # 1. 获取当前骨架的 Key (通常是 arm.name 或 _arm_key_for_obj)
            arm_key = _arm_key_for_obj(arm)
            # 从内存字典直接获取该骨架的所有结果数据
            arm_results = _psd_results_cache.get(arm_key, {})
            
            # 获取性能统计数据
            perf_entries = _psd_perf_stats.get(arm.name, {}).get("entries", {}) if scene.psd_perf_enabled else {}

            cache_empty = psd_get_registered_empty(arm)

            # 2. 收集数据
            results = []
            
            # 所有的 Key（来自内存缓存的 key 集合）
            all_keys = set(arm_results.keys())
            
            # 如果是驱动器模式，内存里可能没存，我们还要看看 Empty 身上有哪些 psd_ 开头的属性
            if cache_empty:
                for prop_name in cache_empty.keys():
                    if prop_name.startswith(PREFIX_RESULT) or \
                       prop_name.startswith(PREFIX_RESULT_LOC) or \
                       prop_name.startswith(PREFIX_RESULT_SCA):
                        all_keys.add(prop_name)

            for k in all_keys:
                # 优先级：如果 Empty 身上有这个值，优先取 Empty 的（因为那是驱动器正在用的真值）
                # 如果没有，再取内存里的
                if cache_empty and k in cache_empty:
                    v = cache_empty[k]
                else:
                    v = arm_results.get(k, 0.0)

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
                res_col.label(text="Waiting for Data or No Matching Items...", icon='INFO')
            else:
                for i, r in enumerate(results):
                    if i >= limit: break
                    
                    row = res_col.row(align=True)
                    # 调整 split 比例，给右侧数值和图标留出足够空间
                    split = row.split(factor=0.85) 
                    
                    # --- 左侧：名称 + 点击复制功能 ---
                    left_row = split.row(align=True)
                    left_row.alignment = 'LEFT' # 强制左对齐
                    
                    # 获取原始 key (例如 psd_arm_L)
                    raw_key = r["key"] 
                    path_text = f'["{raw_key}"]'
                    
                    # 组合显示：点击名称直接复制路径
                    # 使用 emboss=False 让它看起来像普通文字但可点击
                    copy_op = left_row.operator("psd.copy_path", text=r["name"], emboss=False)
                    copy_op.path_to_copy = path_text
                    
                    # --- 右侧：数值 + 性能 + 复制图标 ---
                    val_row = split.row(align=True)
                    val_row.alignment = 'RIGHT'
                    
                    # 格式化数值 (两位小数) 和 性能耗时
                    perf_txt = f" [{r['avg_ms']:.2f}ms]" if scene.psd_perf_enabled else ""
                    # 修正显示：val 使用 :.3f 保持整齐
                    val_display = f"{r['val']:.3f}{perf_txt}"
                    val_row.label(text=val_display)
                    

                if len(results) > limit:
                    subbox.label(text=f"{len(results)-limit} more items not shown", icon='VIEWZOOM')
        else:
            box.label(text="Click to Enable Debug", icon='INFO')


        # 运行模式 & 配置 (collapsible)
        box = layout.box()
        row = box.row(align=True)
        icon_toggle = 'TRIA_DOWN' if arm.show_psd_run_settings else 'TRIA_RIGHT'
        row.prop(arm, "show_psd_run_settings", text="", icon=icon_toggle, emboss=False)
        row.label(text="Run Mode & Settings", icon='SETTINGS') # 这里改成了齿轮
        
        if arm.show_psd_run_settings:
            col = box.column(align=True)
            col.prop(scene, "psd_mode")
            col.prop(scene, "psd_perf_enabled")
            if scene.psd_mode in ('AUTO', 'FORCE_TIMER'):
                box.prop(scene, "psd_idle_hz", text="Idle Hz", slider=True)
            # --- 性能监控显示区 ---
            if scene.psd_perf_enabled:
                perf_box = box.box() 
                perf_box.label(text="Performance Analysis (Real-time)", icon='WORDWRAP_ON')
                
                # 1. 检查全局字典是否存在
                from . import core
                stats_dict = getattr(core, "_psd_perf_stats", None)
                
                if stats_dict is None:
                    perf_box.label(text="Error: Statistics Dictionary Not Found", icon='ERROR')
                else:
                    # 2. 获取数据
                    g_stats = stats_dict.get("__global__", {})
                    global_total = g_stats.get("last_total_ms", 0.0)
                    
                    # 尝试用物体名和骨架数据名同时查找
                    arm_stats = stats_dict.get(arm.name) or stats_dict.get(arm.data.name)
                    
                    if arm_stats or global_total > 0:
                        col = perf_box.column(align=True)
                        col.label(text=f"Total Time: {global_total:.3f} ms", icon='TIME')
                        
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
                            perf_box.label(text="Global Data Exists, But No Current Armature Data", icon='INFO')
                            perf_box.label(text=f"Search Name: {arm.name}", icon='DOT')
                    else:
                        perf_box.label(text="Dictionary Is Empty, Waiting for Calculation Trigger...", icon='INFO')
                        if stats_dict:
                            perf_box.label(text=f"Stored Keys: {list(stats_dict.keys())[:3]}")

            # 启动/停止按钮
            row = layout.row(align=True)
            if scene.psd_running:
                row.operator('object.psd_stop', icon='CANCEL', text=bpy.app.translations.pgettext("Stop"))
            else:
                row.operator('object.psd_start', icon='PLAY', text=bpy.app.translations.pgettext("Start"))

            layout.label(text='(Pose Data Saved on Armature; Results Written to Armature Data Block)', icon='INFO')
            layout.operator("psd.invalidate_bone_cache", icon='FILE_REFRESH', text=bpy.app.translations.pgettext("Clear Cache"))
