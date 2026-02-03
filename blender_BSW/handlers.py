import bpy
import bpy.app.handlers as handlers
from .core import _psd_compute_all, last_compute_time  # 导入核心
from .utils import _is_animation_playing
from .GUI_draw import draw_psd_visualizers
from .GUI_draw import register_draw_handler, unregister_draw_handler
# 全局处理器标志
_psd_timer_registered = False
_msgbus_subscribed = False
_msgbus_owner = object()



@handlers.persistent
def load_post_handler(dummy):
    """每次打开文件后，强制重新注册驱动器函数"""
    try:
        from .driver_math import register_drivers
        register_drivers()
        print("✅ PSD 驱动器已在文件加载后自动修复注册")
    except Exception as e:
        print(f"❌ 自动修复注册失败: {e}")


@handlers.persistent
def psd_selection_handler(scene, depsgraph):
    context = bpy.context
    # 在回调中使用 view_layer 获取活动对象最稳
    obj = context.view_layer.objects.active
    
    if not obj or obj.type != 'ARMATURE' or obj.mode != 'POSE':
        return

    # 获取当前选中的所有骨骼数量
    selected_pose_bones = context.selected_pose_bones
    
    # 如果选中了多个骨骼（多选/框选），则不更新 UI 列表索引，直接退出
    # 这样可以保留 Blender 原生的多选行为，不被索引切换干扰
    if len(selected_pose_bones) != 1:
        return

    # 只要运行到这里，说明当前只选中了一个骨骼
    active_bone = obj.data.bones.active
    if not active_bone:
        return
    
    active_name = active_bone.name

    # 寻找并匹配 psd_bone_pairs
    found_idx = -1
    for i, pair in enumerate(obj.psd_bone_pairs):
        if pair.bone_name == active_name:
            found_idx = i
            break
    
    # 只有当索引真的改变了才写入，减少 UI 刷新负担
    if found_idx != -1 and obj.psd_bone_pairs_index != found_idx:
        # 修改索引
        obj.psd_bone_pairs_index = found_idx
        
        # 强制属性面板刷新（使用 tag_redraw 比较轻量）
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == 'PROPERTIES':
                    area.tag_redraw()

#
def _get_scene_for_timer():
    if bpy.context and getattr(bpy.context, "scene", None):
        return bpy.context.scene
    if bpy.data.scenes:
        return bpy.data.scenes[0]
    return None

def _psd_timer_func():
    global _psd_timer_registered
    _psd_timer_registered = True
    try:
        sc = _get_scene_for_timer()
        if not sc or not getattr(sc, 'psd_running', False):
            _psd_timer_registered = False
            return None
        if getattr(sc, 'psd_mode', 'AUTO') == 'FORCE_PLAY':
            _psd_timer_registered = False
            return None
        if getattr(sc, 'psd_mode', 'AUTO') == 'AUTO' and _is_animation_playing():
            _psd_timer_registered = False
            return None
        try:
            _psd_compute_all(depsgraph=None)
        except Exception as e:
            print("BSW计时器计算错误:", e)
        hz = int(getattr(sc, 'psd_idle_hz', 10) or 10)
        if hz < 1:
            hz = 1
        return 1.0 / float(hz)
    except Exception as e:
        _psd_timer_registered = False
        print("BSW计时器异常:", e)
        return None

def _on_play_changed():
    global _psd_timer_registered
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        if not sc or not getattr(sc, 'psd_running', False):
            return
        mode = getattr(sc, 'psd_mode', 'AUTO')
        if mode == 'FORCE_PLAY':
            _psd_timer_registered = False
            return
        if mode == 'FORCE_TIMER':
            if not _psd_timer_registered:
                try:
                    bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                    _psd_timer_registered = True
                except Exception:
                    pass
            return
        if _is_animation_playing():
            _psd_timer_registered = False
        else:
            if not _psd_timer_registered:
                try:
                    bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                    _psd_timer_registered = True
                except Exception:
                    pass
    except Exception as e:
        print("BSW消息总线播放状态变化处理器异常:", e)

def _subscribe_msgbus_for_play_change():
    global _msgbus_subscribed
    if _msgbus_subscribed:
        return
    try:
        bpy.msgbus.subscribe_rna(
            key=(bpy.types.Screen, "is_animation_playing"),
            owner=_msgbus_owner,
            notify=_on_play_changed,
        )
        _msgbus_subscribed = True
    except Exception as e:
        print("BSW消息总线订阅失败:", e)
        _msgbus_subscribed = False

def _unsubscribe_msgbus():
    global _msgbus_subscribed
    try:
        bpy.msgbus.clear_by_owner(_msgbus_owner)
    except Exception:
        pass
    _msgbus_subscribed = False
#

def _handler_name_matches(h, target_name):
    try:
        if getattr(h, "__name__", None) == target_name:
            return True
        if getattr(getattr(h, "__func__", None), "__name__", None) == target_name:
            return True
        return target_name in repr(h)
    except Exception:
        return False

def _remove_handlers_with_name(list_ref, target_name):
    removed = False
    for h in list(list_ref):
        if _handler_name_matches(h, target_name):
            try:
                list_ref.remove(h)
                removed = True
            except Exception:
                pass
    return removed

@handlers.persistent
def psd_frame_handler(scene):
    try:
        if scene and getattr(scene, 'psd_mode', 'AUTO') == 'FORCE_TIMER':
            return
        if _is_animation_playing() or (scene and getattr(scene, 'psd_mode', 'FORCE_PLAY')):
            deps = bpy.context.evaluated_depsgraph_get()
            _psd_compute_all(depsgraph=deps)
        else:
            _psd_compute_all(depsgraph=None)
    except Exception as e:
        print("BSW帧变化处理器错误:", e)


@handlers.persistent
def psd_depsgraph_handler(scene,depsgraph):
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        if sc and getattr(sc, 'psd_mode', 'AUTO') == 'FORCE_TIMER':
            return
        if not _is_animation_playing() and (not sc or getattr(sc, 'psd_mode', 'AUTO') != 'FORCE_PLAY'):
            return
        _psd_compute_all(depsgraph=depsgraph)
    except Exception as e:
        print("BSW depsgraph处理器错误:", e)


class PSDStartOperator(bpy.types.Operator):
    bl_idname = "object.psd_start"
    bl_label = "启动BSW"

    def execute(self, context):
        global _msgbus_subscribed, _psd_timer_registered
        sc = context.scene
        if not sc.psd_running:
            try:
                _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
            except Exception:
                pass
            try:
                _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
            except Exception:
                pass

            try:
                if not any(_handler_name_matches(h, psd_depsgraph_handler.__name__) for h in handlers.depsgraph_update_post):
                    handlers.depsgraph_update_post.append(psd_depsgraph_handler)
            except Exception:
                pass
            try:
                if not any(_handler_name_matches(h, psd_frame_handler.__name__) for h in handlers.frame_change_post):
                    handlers.frame_change_post.append(psd_frame_handler)
            except Exception:
                pass

            sc.psd_running = True

            mode = getattr(sc, 'psd_mode', 'AUTO')

            if mode == 'AUTO':
                _subscribe_msgbus_for_play_change()
                try:
                    _on_play_changed()
                except Exception:
                    pass
            elif mode == 'FORCE_TIMER':
                if not _psd_timer_registered:
                    try:
                        bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                        _psd_timer_registered = True
                    except Exception:
                        pass
            elif mode == 'FORCE_PLAY':
                _psd_timer_registered = False

            self.report({'INFO'}, "BSW已启动 (处理器已激活)")
        else:
            self.report({'INFO'}, "BSW已在运行")
        return {'FINISHED'}


class PSDStopOperator(bpy.types.Operator):
    bl_idname = "object.psd_stop"
    bl_label = "停止BSW"

    def execute(self, context):
        global _psd_timer_registered, _msgbus_subscribed
        try:
            if psd_depsgraph_handler in handlers.depsgraph_update_post:
                handlers.depsgraph_update_post.remove(psd_depsgraph_handler)
        except Exception:
            pass
        try:
            _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
        except Exception:
            pass
        try:
            if psd_frame_handler in handlers.frame_change_post:
                handlers.frame_change_post.remove(psd_frame_handler)
        except Exception:
            pass
        try:
            _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
        except Exception:
            pass

        try:
            context.scene.psd_running = False
        except Exception:
            pass

        _psd_timer_registered = False
        _unsubscribe_msgbus()

        self.report({'INFO'}, "BSW已停止 (处理器已移除)")
        return {'FINISHED'}


def register_handlers():


    if load_post_handler not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(load_post_handler)


    if not any(_handler_name_matches(h, psd_selection_handler.__name__) for h in handlers.depsgraph_update_post):
        bpy.app.handlers.depsgraph_update_post.append(psd_selection_handler)
    # 添加处理器
    bpy.app.handlers.depsgraph_update_post.append(psd_depsgraph_handler)
    bpy.app.handlers.frame_change_post.append(psd_frame_handler)
    # 计时器注册（如果有）

    

    if not hasattr(bpy.types.SpaceView3D, "_psd_draw_handler"):
        bpy.types.SpaceView3D._psd_draw_handler = bpy.types.SpaceView3D.draw_handler_add(
            draw_psd_visualizers, (), 'WINDOW', 'POST_VIEW'
        )

def unregister_handlers():


    _remove_handlers_with_name(handlers.depsgraph_update_post, psd_selection_handler.__name__)

    handler = getattr(bpy.types.SpaceView3D, "_psd_draw_handler", None)
    if handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(handler, 'WINDOW')
        del bpy.types.SpaceView3D._psd_draw_handler

    if load_post_handler in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(load_post_handler)


    try:
        _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
    except Exception:
        pass
    try:
        _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
    except Exception:
        pass
