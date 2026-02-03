import bpy
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA
from .utils import _safe_name,psd_get_registered_empty


def setup_rotation_driver(driver, arm, entry):
    """配置旋转驱动器：支持标准 PSD 锥体和 Direct Channel (Swing/Twist)"""
    bone_name = entry.bone_name
    
    # --- 情况 A: Direct Channel (直接通道模式) ---
    if getattr(entry, 'is_direct_channel', False):
        var = driver.variables.new()
        var.name = "rot_val"
        var.type = 'TRANSFORMS'
        
        target = var.targets[0]
        target.id = arm
        target.bone_target = bone_name
        target.transform_space = 'LOCAL_SPACE'
        
        
        mode = getattr(entry, 'record_rot_channel_mode', 'NONE')
        ch_axis = getattr(entry, 'channel_axis', 'X')
        
        # 根据是否是 Swing/Twist 分解选择旋转模式
        if "SWING" in mode:
            
            target.rotation_mode = 'SWING_TWIST_X' if '_X_' in mode else \
                                 ('SWING_TWIST_Y' if '_Y_' in mode else 'SWING_TWIST_Z')
            # 根据通道选择具体分量
            target.transform_type = f'ROT_{ch_axis}'
        else:
            # 普通欧拉旋转
            target.rotation_mode = 'XYZ'
            target.transform_type = f'ROT_{ch_axis}'
            
        # 表达式非常简单，直接输出变量值 (弧度)
        driver.expression = "rot_val"

    # --- 情况 B: 标准 PSD 旋转  ---
    else:
        for ax in ['X', 'Y', 'Z']:
            var = driver.variables.new()
            var.name = f"cr{ax.lower()}"
            var.type = 'TRANSFORMS'
            target = var.targets[0]
            target.id = arm
            target.bone_target = bone_name
            target.rotation_mode = 'XYZ'
            target.transform_type = f'ROT_{ax}'
            target.transform_space = 'LOCAL_SPACE'

        p, r = entry.pose_rot, entry.rest_rot
        angle = getattr(entry, 'cone_angle', 90.0)
        axis_idx = getattr(entry, 'axis_idx', 1)
        mode_val = 0 if getattr(entry, 'cone_enabled', False) else 1

        driver.expression = (
            f"psd_rot(crx, cry, crz, "
            f"{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}, "
            f"{r[0]:.3f}, {r[1]:.3f}, {r[2]:.3f}, "
            f"{angle:.1f}, {axis_idx}, {mode_val})"
        )

def setup_location_driver(driver, arm, entry):
    """配置位置驱动器 (psd_loc)"""
    # 添加变量 clx, cly, clz
    for ax in ['X', 'Y', 'Z']:
        var = driver.variables.new()
        var.name = f"cl{ax.lower()}"
        var.type = 'TRANSFORMS'
        target = var.targets[0]
        target.id = arm
        target.bone_target = entry.bone_name
        target.transform_type = f'LOC_{ax}'
        target.transform_space = 'LOCAL_SPACE'


    p = entry.pose_loc
    r = entry.rest_loc
    radius = getattr(entry, 'loc_radius', 0.1)
    

    # 假设：True 为球形衰减 (mode 2)，False 为投影映射 (mode 0)

    is_loc_active = getattr(entry, 'loc_enabled', False)
    mode_val = 2 if is_loc_active else 0 

    # 拼接表达式
    driver.expression = (
        f"psd_loc(clx, cly, clz, "
        f"{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}, "
        f"{r[0]:.3f}, {r[1]:.3f}, {r[2]:.3f}, "
        f"{radius:.2f}, {mode_val})"
    )

def setup_scale_driver(driver, arm, entry):
    """配置缩放驱动器 (psd_sca)"""
    for ax in ['X', 'Y', 'Z']:
        var = driver.variables.new()
        var.name = f"cs{ax.lower()}"
        var.type = 'TRANSFORMS'
        target = var.targets[0]
        target.id = arm
        target.bone_target = entry.bone_name
        target.transform_type = f'SCALE_{ax}'
        target.transform_space = 'LOCAL_SPACE'

    p, r = entry.pose_sca, entry.rest_sca
    driver.expression = (
        f"psd_sca(csx, csy, csz, "
        f"{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}, "
        f"{r[0]:.6f}, {r[1]:.6f}, {r[2]:.6f}, "
        f"{0})"
    )




class PSD_OT_BindDriverToCache(bpy.types.Operator):
    """将 PSD 算法赋予到注册的 Empty 缓存对象的自定义属性上"""
    bl_idname = "psd.bind_driver_to_cache"
    bl_label = "绑定驱动器到缓存 Empty"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        if not cache_empty:
            self.report({'ERROR'}, "未找到注册的 Empty 缓存！")
            return {'CANCELLED'}

        idx = arm.psd_saved_pose_index
        if idx < 0 or idx >= len(arm.psd_saved_poses):
            return {'CANCELLED'}
        
        entry = arm.psd_saved_poses[idx]
        bone_name = entry.bone_name
        safe_bone = _safe_name(bone_name)
        safe_name = _safe_name(entry.name)
        
        def apply_channel_driver(prefix, has_flag, setup_func):
            if has_flag:
                entry_key = f"{prefix}{safe_bone}_{safe_name}"
                
                
                if entry_key not in cache_empty:
                    cache_empty[entry_key] = 0.0
                
                # 添加/获取驱动器
                fcurve = cache_empty.driver_add(f'["{entry_key}"]')
                driver = fcurve.driver
                driver.type = 'SCRIPTED'
                
                # 清理旧变量
                while driver.variables:
                    driver.variables.remove(driver.variables[0])
                
                # 执行特定的绑定逻辑 (setup_rotation_driver 等)
                setup_func(driver, arm, entry)
                return entry_key
            return None

        # --- 开始依次执行三个通道的检查
        results = []
        
        # 检查旋转
        if entry.has_rot or getattr(entry, 'is_direct_channel', False):
            # 统一使用 PREFIX_RESULT
            res_rot = apply_channel_driver(PREFIX_RESULT, True, setup_rotation_driver)
            if res_rot: results.append(res_rot)

        # 检查位置
        res_loc = apply_channel_driver(PREFIX_RESULT_LOC, entry.has_loc, setup_location_driver)
        if res_loc: results.append(res_loc)

        # 检查缩放
        res_sca = apply_channel_driver(PREFIX_RESULT_SCA, entry.has_sca, setup_scale_driver)
        if res_sca: results.append(res_sca)

        # 最后统一报告结果
        if results:
            self.report({'INFO'}, f"✅ 已激活通道: {', '.join(results)}")
        else:
            self.report({'WARNING'}, "该条目未启用任何通道 (ROT/LOC/SCA)")

        return {'FINISHED'}

                # #
                # if entry.has_rot:
                #     setup_rotation_driver(driver, arm, entry)
                # elif entry.has_loc:
                #     setup_location_driver(driver, arm, entry)
                # elif entry.has_sca:
                #     setup_scale_driver(driver, arm, entry)
                    

class PSD_OT_BatchBindAll(bpy.types.Operator):
    """一键为当前骨架的所有条目生成/更新 PSD 驱动器"""
    bl_idname = "psd.batch_bind_all"
    bl_label = "一键绑定所有条目"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        if not cache_empty:
            self.report({'ERROR'}, "未找到注册的 Empty 缓存！")
            return {'CANCELLED'}

        if len(arm.psd_saved_poses) == 0:
            self.report({'WARNING'}, "没有保存的条目可供绑定")
            return {'FINISHED'}

        success_entries = 0
        total_channels = 0

        # 遍历所有保存的条目 ---
        for entry in arm.psd_saved_poses:
            bone_name = entry.bone_name
            safe_bone = _safe_name(bone_name)
            safe_name = _safe_name(entry.name)

            has_any_rot = entry.has_rot or getattr(entry, 'is_direct_channel', False)

            # 定义要处理的通道
            channels_to_process = [
                (PREFIX_RESULT,     has_any_rot, setup_rotation_driver),
                (PREFIX_RESULT_LOC, entry.has_loc, setup_location_driver),
                (PREFIX_RESULT_SCA, entry.has_sca, setup_scale_driver)
            ]

            entry_had_action = False
            for prefix, has_flag, setup_func in channels_to_process:
                if has_flag:
                    entry_key = f"{prefix}{safe_bone}_{safe_name}"
                    
                    # 准备属性
                    if entry_key not in cache_empty:
                        cache_empty[entry_key] = 0.0
                    
                    # 添加/覆盖驱动器
                    fcurve = cache_empty.driver_add(f'["{entry_key}"]')
                    driver = fcurve.driver
                    driver.type = 'SCRIPTED'
                    
                    # 清理旧变量
                    while driver.variables:
                        driver.variables.remove(driver.variables[0])
                    
                    # 执行对应的 setup 函数 (需确保这些函数在当前作用域可用)
                    setup_func(driver, arm, entry)
                    
                    total_channels += 1
                    entry_had_action = True
            
            if entry_had_action:
                success_entries += 1

        self.report({'INFO'}, f"😏 批量绑定完成！共处理 {success_entries} 个条目，激活 {total_channels} 个通道")
        return {'FINISHED'}


class PSD_OT_RemoveDriverFromCache(bpy.types.Operator):
    """移除当前条目在 Empty 缓存上对应的所有驱动器"""
    bl_idname = "psd.remove_driver_from_cache"
    bl_label = "移除条目驱动器"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        if not cache_empty:
            self.report({'ERROR'}, "未找到缓存 Empty，无需清理。")
            return {'CANCELLED'}

        idx = arm.psd_saved_pose_index
        if idx < 0 or idx >= len(arm.psd_saved_poses):
            return {'CANCELLED'}
        
        entry = arm.psd_saved_poses[idx]
        bone_name = entry.bone_name
        safe_bone = _safe_name(bone_name)
        safe_name = _safe_name(entry.name)

        # 定义可能存在的通道前缀
        prefixes = [PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA]
        removed_count = 0

        for prefix in prefixes:
            entry_key = f"{prefix}{safe_bone}_{safe_name}"
            
            # 在 Blender 中，driver_remove 会根据 data_path 移除驱动
            data_path = f'["{entry_key}"]'
            success = cache_empty.driver_remove(data_path)
            
            if success:
                removed_count += 1
            

        if removed_count > 0:
            self.report({'INFO'}, f"✅ 已成功从缓存中移除 {removed_count} 个驱动器")
        else:
            self.report({'WARNING'}, "未找到可移除的驱动器")

        return {'FINISHED'}
    

class PSD_OT_ClearAllCacheDrivers(bpy.types.Operator):
    """清空缓存 Empty 上所有的 PSD 相关驱动器 (慎用!)"""
    bl_idname = "psd.clear_all_cache_drivers"
    bl_label = "清空所有缓存驱动"

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        if not cache_empty or not cache_empty.animation_data:
            return {'FINISHED'}

        # 遍历所有驱动器，找出前缀匹配的
        drivers = cache_empty.animation_data.drivers
        for d in list(drivers):
            dp = d.data_path
            # 检查这个驱动器的路径是否包含 PSD 前缀
            if any(p in dp for p in [PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA]):
                cache_empty.driver_remove(dp)
        
        self.report({'INFO'}, "☠️ 所有缓存驱动器已被核平!")
        return {'FINISHED'}
    

#刷新
class PSD_OT_RefreshDrivers(bpy.types.Operator):
    """底层强制刷新：通过重新赋值驱动器表达式来修复红字无效问题"""
    bl_idname = "psd.refresh_drivers"
    bl_label = "底层强制刷新"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        if not cache_empty or not cache_empty.animation_data:
            self.report({'WARNING'}, "没有找到需要刷新的驱动器。")
            return {'CANCELLED'}

        # 核心逻辑：循环“抖动”表达式
        for fcurve in cache_empty.animation_data.drivers:
            driver = fcurve.driver
            # 备份原有的表达式
            expr_backup = driver.expression
            
            # 先清空表达式（触发 Blender 内部重置）
            driver.expression = "" 
            
            # 重新填回原表达式（强制 Blender 重新编译 Python 命名空间）
            driver.expression = expr_backup
            
            # 强制标记为有效并启用
            fcurve.mute = False
            driver.use_self = driver.use_self # 这种无意义的赋值有时能触发更新

        #最终大招：通知场景依赖图刷新
        context.view_layer.depsgraph.update()
        
        self.report({'INFO'}, "🌚 驱动器已强制叫醒！")
        return {'FINISHED'}
    
class PSD_OT_MuteDrivers(bpy.types.Operator):
    """禁用所有 PSD 驱动器"""
    bl_idname = "psd.mute_drivers"
    bl_label = "禁用驱动器"

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        # --- 在这里添加你的判定 ---
        if not cache_empty:
            self.report({'ERROR'}, "判定失败：未找到缓存 Empty")
            return {'CANCELLED'}
            
        if not cache_empty.animation_data:
            return {'CANCELLED'}

        for fcurve in cache_empty.animation_data.drivers:
            # 也可以在这里加判定，比如只禁用特定前缀的驱动器
            fcurve.mute = True
            
        self.report({'INFO'}, "🚫 PSD 驱动器已禁用")
        return {'FINISHED'}

class PSD_OT_UnmuteDrivers(bpy.types.Operator):
    """启动所有 PSD 驱动器"""
    bl_idname = "psd.unmute_drivers"
    bl_label = "启动驱动器"

    def execute(self, context):
        arm = context.object
        cache_empty = psd_get_registered_empty(arm)
        
        # --- 在这里添加你的判定 ---
        # 比如：判定当前是否处于 DRIVERS_MODS 模式，否则不让启动
        if getattr(arm, "psd_output_mode", "") != 'DRIVERS_MODS':
            self.report({'WARNING'}, "判定失败：非驱动器接管模式下无法启动")
            # return {'CANCELLED'} 

        if not cache_empty or not cache_empty.animation_data:
            return {'CANCELLED'}

        for fcurve in cache_empty.animation_data.drivers:
            fcurve.mute = False
            
        # 启动后强制刷新一次，防止红字
        bpy.ops.psd.refresh_drivers()
        
        self.report({'INFO'}, "✅ PSD 驱动器已启动并刷新")
        return {'FINISHED'}