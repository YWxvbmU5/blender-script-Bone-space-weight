import json
import os
import bpy
import time
import math
from mathutils import Vector, Euler, Quaternion # 导入依赖
from .utils import _safe_name, _sync_cache_from_empty, psd_get_registered_empty, _get_rest_matrix, _capture_bone_local_rotation_deg, _capture_bone_local_translation_effective, _is_animation_playing, psd_set_result_cache_only, _arm_key_for_obj, psd_flush_mem_to_registered_empty, psd_set_result_to_registered_empty
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA, _psd_results_cache, _PSD_CACHE_OBJ_PROP
from .json_shape_driver import ShapeDriver
from .json_pose_driver import PoseDriver
from .math_utils import (
    compute_direct_channel_weight, compute_rotation_weight,
    compute_location_weight, compute_scale_weight, compute_triggers,
    compute_rbf_group_weights
)

# 全局变量
last_compute_time = 0.0
_psd_perf_stats = {}
_psd_bone_state_cache = {}

#计算缓存==================
POST_PROCESS_EXPRESSIONS = {}
POSE_DRIVERS = {}

#============================

_shape_expressions_cache = {}   # {arm_key: compiled_dict}
_pose_drivers_cache = {}        # {arm_key: compiled_dict}

def _get_arm_scene(arm_obj):
    """安全获取 arm 所在的 Scene（优先 users_scene）"""
    if arm_obj.users_scene:
        return arm_obj.users_scene[0]
    # fallback（极少情况）
    return bpy.context.scene if bpy.context.scene else list(bpy.data.scenes)[0]

def load_shape_drivers(arm_obj):
    """从 arm_obj 的文件列表加载并编译所有 Shape Driver"""
    if not arm_obj or arm_obj.type != 'ARMATURE':
        return {}
    compiled_data = {}
    loaded_files = 0
    for item in arm_obj.psd_shape_driver_files:
        path = bpy.path.abspath(item.filepath)
        if not os.path.isfile(path):
            print(f"[PSD] Shape Driver 文件不存在: {path}")
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
            # 原编译逻辑（保持不变，仅移到这里）
            for post_key, info in raw_data.items():
                mesh_name = info.get("Mesh_name")
                expr = info.get("expression", "")
                variables = info.get("variables", [])
                if not expr or not variables:
                    continue
                try:
                    compiled = compile(expr, f"<PSD Math: {post_key}>", "eval")
                except Exception as e:
                    print(f"[PSD Math] 表达式编译失败 {post_key}: {e}")
                    continue
                dep_keys = []
                for var in variables:
                    try:
                        var_key = json.loads(var["data_path"])[0]
                        dep_keys.append(var_key)
                    except:
                        pass
                compiled_data[post_key] = {
                    "Mesh_name": mesh_name,
                    "compiled": compiled,
                    "variables": variables,
                    "dep_keys": tuple(sorted(dep_keys))
                }
            loaded_files += 1
        except Exception as e:
            print(f"[PSD] 加载 Shape Driver 失败 {path}: {e}")
    print(f"[PSD Math] 从 {loaded_files} 个文件加载&编译 {len(compiled_data)} 条 Shape Driver")
    return compiled_data

def load_pose_drivers(arm_obj):
    """从 arm_obj 的文件列表加载并编译所有 Pose Driver"""
    if not arm_obj or arm_obj.type != 'ARMATURE':
        return {}
    compiled_data = {}
    loaded_files = 0
    for item in arm_obj.psd_pose_driver_files:
        path = bpy.path.abspath(item.filepath)
        if not os.path.isfile(path):
            print(f"[PSD] Pose Driver 文件不存在: {path}")
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
            # 原编译逻辑（保持不变）
            for bone_name, constraints in raw_data.items():
                bone_dict = compiled_data.setdefault(bone_name, {})
                for constraint_name, prop_dict in constraints.items():
                    constraint_info = bone_dict.setdefault(constraint_name, {})
                    for prop_name, prop_info in prop_dict.items():
                        armature_name = prop_info.get("Armature_name")
                        expr = prop_info.get("expression", "")
                        variables = prop_info.get("variables", [])
                        if not armature_name or not expr or not variables:
                            continue
                        try:
                            compiled = compile(expr, f"<Pose Driver: {bone_name}.{constraint_name}.{prop_name}>", "eval")
                        except Exception as e:
                            print(f"[Pose Driver] 编译失败 {bone_name}.{constraint_name}.{prop_name}: {e}")
                            continue
                        dep_keys = []
                        for var in variables:
                            try:
                                var_key = json.loads(var["data_path"])[0]
                                dep_keys.append(var_key)
                            except:
                                pass
                        constraint_info[prop_name] = {
                            "Armature_name": armature_name,
                            "compiled": compiled,
                            "variables": variables,
                            "dep_keys": tuple(sorted(dep_keys))
                        }
            loaded_files += 1
        except Exception as e:
            print(f"[PSD] 加载 Pose Driver 失败 {path}: {e}")
    total_props = sum(len(c) for b in compiled_data.values() for c in b.values())
    print(f"[Pose Driver] 从 {loaded_files} 个文件加载&编译 {total_props} 条属性驱动")
    return compiled_data

def reload_shape_drivers(arm_obj):
    """强制重新加载并返回条数（供 operator 使用）"""
    arm_key = _arm_key_for_obj(arm_obj)
    new_data = load_shape_drivers(arm_obj)
    _shape_expressions_cache[arm_key] = new_data
    return len(new_data)

def reload_pose_drivers(arm_obj):
    """强制重新加载并返回条数"""
    arm_key = _arm_key_for_obj(arm_obj)
    new_data = load_pose_drivers(arm_obj)
    _pose_drivers_cache[arm_key] = new_data
    total = sum(len(c) for b in new_data.values() for c in b.values())
    return total


def reload_shape_drivers_all():
    total = 0
    for scn in bpy.data.scenes:
        for obj in scn.objects:
            if obj.type == 'ARMATURE':
                arm_key = _arm_key_for_obj(obj)
                new_data = load_shape_drivers(obj)  # 内部已用 _get_arm_scene
                _shape_expressions_cache[arm_key] = new_data
                total += len(new_data)
    return total

def reload_pose_drivers_all():
    total = 0
    for scn in bpy.data.scenes:
        for obj in scn.objects:
            if obj.type == 'ARMATURE':
                arm_key = _arm_key_for_obj(obj)
                new_data = load_pose_drivers(obj)
                _pose_drivers_cache[arm_key] = new_data
                total += len(new_data)
    print(f"[PSD] 全局重新加载 Pose Drivers: {total} 条")
    return total
#============================

# core.py
def psd_get_result_snapshot():
    """返回当前 PSD 计算结果的 dict 快照（线程安全）"""
    return dict(_psd_results_cache)


def init_globals():
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    # global POST_PROCESS_EXPRESSIONS
    # global POSE_DRIVERS

    last_compute_time = 0.0
    _psd_perf_stats = {}
    _psd_bone_state_cache = {}

def psd_invalidate_bone_cache(arm_name=None, bone_name=None):
    """
    清空缓存：
      - psd_invalidate_bone_cache()             -> 清空全部缓存
      - psd_invalidate_bone_cache(arm_name)     -> 清空某个 arm 的缓存
      - psd_invalidate_bone_cache(arm_name, bn) -> 清空某个 arm 下的某个骨骼缓存
    """
    global _psd_bone_state_cache, _psd_results_cache, _psd_perf_stats
    
    print(f"[DEBUG] psd_invalidate_bone_cache called: arm_name={arm_name}, bone_name={bone_name}")
    
    # 1. 全局清理
    if arm_name is None:
        _psd_bone_state_cache.clear()
        _psd_results_cache.clear()
        _psd_perf_stats.clear()
        print("[DEBUG] All caches cleared globally")
        return

    # 2. 清理特定 Armature 的缓存（完全删除该 arm 的所有记录）
    if bone_name is None:
        print(_psd_results_cache)
        _psd_bone_state_cache.pop(arm_name, None)
        _psd_results_cache.pop(arm_name, None)
        _psd_perf_stats.pop(arm_name, None)
        print(f"[DEBUG] All caches cleared for arm: {arm_name}")
        return
    
    # 3. 清理特定骨骼相关缓存
    cleaned = False
    
    # bone state cache
    arm_state_cache = _psd_bone_state_cache.get(arm_name)
    if arm_state_cache and bone_name in arm_state_cache:
        arm_state_cache.pop(bone_name, None)
        cleaned = True
        print(f"[DEBUG] Removed bone state for {bone_name} in {arm_name}")
    
    # results cache
    arm_res_cache = _psd_results_cache.get(arm_name)
    if arm_res_cache:
        # 先删纯 bone_name（兼容）
        if arm_res_cache.pop(bone_name, None) is not None:
            cleaned = True
        
        # 删除所有包含 bone_name 的 key
        keys_to_remove = [k for k in list(arm_res_cache.keys()) if bone_name in k]
        if keys_to_remove:
            for k in keys_to_remove:
                arm_res_cache.pop(k, None)
            cleaned = True
            # print(f"[DEBUG] Removed {len(keys_to_remove)} result keys containing '{bone_name}' in {arm_name}")
        
        # 如果子字典完全空了，删除整个 arm entry（彻底清理）
        if not arm_res_cache:
            _psd_results_cache.pop(arm_name, None)
            print(f"[DEBUG] Sub-cache empty, removed entire results entry for {arm_name}")
    
    # perf stats（如果也按 bone 存的话，可类似处理）
    arm_perf = _psd_perf_stats.get(arm_name)
    if arm_perf and bone_name in arm_perf:
        arm_perf.pop(bone_name, None)
        cleaned = True
    
    print(f"[DEBUG] After invalidate, results cache for {arm_name}: {_psd_results_cache.get(arm_name)}")
    
    if not cleaned:
        print(f"[WARNING] No cache entries found to remove for {bone_name} in {arm_name}")
            
#清除Emply已存储的结果========
def psd_clear_emply_results_cache(arm_obj):
    """清除指定 armature 对应的 Empty 存储的 PSD 结果缓存"""
    if not arm_obj or arm_obj.type != 'ARMATURE':
        return

    # 1. 获取 Empty 对象
    empty_obj = None

    # 直接从 armature data 获取存储的 Empty 名字
    # 假设是用 arm_obj.data[_PSD_CACHE_OBJ_PROP] = empty.name 写入的
    arm_db = arm_obj.data
    empty_name = arm_db.get(_PSD_CACHE_OBJ_PROP, None)

    if empty_name:
        empty_obj = bpy.data.objects.get(empty_name)

    # 2. 尝试从对象本身获取
    if not empty_obj:
        empty_name = arm_obj.get(_PSD_CACHE_OBJ_PROP, None)
        if empty_name:
            empty_obj = bpy.data.objects.get(empty_name)

    # 3. 执行清除逻辑
    if not empty_obj:
        print(f"[PSD] 警告：未找到关联的 Empty 对象，无法清除属性（骨架: {arm_obj.name}）")
        return

    # 4. 获取所有匹配前缀的 Key
    keys_to_remove = [
        key for key in empty_obj.keys() 
        if key.startswith(PREFIX_RESULT) or 
           key.startswith(PREFIX_RESULT_LOC) or 
           key.startswith(PREFIX_RESULT_SCA)
    ]

    # 5. 删除自定义属性
    removed_count = 0
    for key in keys_to_remove:
        try:
            del empty_obj[key]
            removed_count += 1
        except Exception:
            pass
            
    print(f"[PSD] 成功清除 Empty ({empty_obj.name}) 中的 {removed_count} 条缓存结果")

def _psd_make_sample_from_maps(rot_map, loc_map, sca_map, bn, round_ndigits=6):
    """
    从 bone_to_cur_rot / bone_to_cur_loc / bone_to_cur_sca 生成一个可比较的 sample。
    rot_map/loc_map/sca_map 是字典，bn 是骨骼名。
    返回 (loc_tuple, rot_tuple, sca_tuple)，其中每个子项要么是 tuple 要么是 None。
    """
    def _to_tuple(v):
        if v is None:
            return None
        # v 可能是 mathutils.Vector 或 tuple/list
        try:
            return tuple(round(float(x), round_ndigits) for x in v)
        except Exception:
            try:
                return tuple(round(float(x), round_ndigits) for x in tuple(v))
            except Exception:
                return None

    rot_t = _to_tuple(rot_map.get(bn))
    loc_t = _to_tuple(loc_map.get(bn))
    sca_t = _to_tuple(sca_map.get(bn))
    return (loc_t, rot_t, sca_t)

def _psd_samples_equal(s1, s2, eps=1e-5):
    """
    比较两个 sample（三元组）。
    若任一子项为 None 则视为不可比较（返回 False）。
    eps 是每分量允许的最大差值。
    """
    if s1 is None or s2 is None:
        return False
    # s1, s2 都应为 (loc_t, rot_t, sca_t)
    for a, b in zip(s1, s2):
        if a is None or b is None:
            return False
        if len(a) != len(b):
            return False
        for va, vb in zip(a, b):
            if abs(va - vb) > eps:
                return False
    return True


def _fetch_armature_states(arm, depsgraph, bone_filter):
    """
    获取目标骨架在当前评估图下的所有受监控骨骼的 Transform 状态。
    
    Returns:
        tuple: (bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca)
    """
    bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca = {}, {}, {}
    
    # 获取评估后的对象
    source_obj = arm
    if depsgraph is not None:
        try: 
            source_obj = arm.evaluated_get(depsgraph)
        except Exception as e: 
            # 替换单纯的 pass，保留调试余地
            pass 

    for bn in bone_filter:
        try:
            # 捕获旋转
            cur_deg = _capture_bone_local_rotation_deg(arm, bn, depsgraph=depsgraph)
            if cur_deg is not None: 
                bone_to_cur_rot[bn] = Vector(cur_deg)
                
            # 捕获位移和缩放
            pb = source_obj.pose.bones.get(bn)
            if pb:
                cur_loc = _capture_bone_local_translation_effective(arm, bn, depsgraph=depsgraph)
                bone_to_cur_loc[bn] = cur_loc.copy()
                bone_to_cur_sca[bn] = pb.scale.copy()
        except Exception as e:
            # print(f"[BPW DEBUG] Fetch error on bone {bn}: {e}")
            pass
            
    return bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca


def _dispatch_calculations(arm, saved_entries, bone_filter, skip_bones, state_maps, is_driver_mode, perf_enabled, arm_stats):
    """
    将计算任务分发给具体的数学算法，并记录性能。
    """
    bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca = state_maps
    rbf_groups = {}
    normal_entries = []
    
    # 1. 任务分类
    for entry in saved_entries:
        bn = entry.bone_name
        if not bn or not entry.name or bn not in bone_filter: 
            continue
        if getattr(entry, 'use_rbf', False):
            rbf_groups.setdefault(bn, []).append(entry)
        elif not is_driver_mode:
            normal_entries.append(entry)

    # 2. RBF 分组计算
    for bn, entries in rbf_groups.items():
        if bn in skip_bones: 
            continue
        cur_rot = bone_to_cur_rot.get(bn)
        if cur_rot is not None:
            compute_rbf_group_weights(cur_rot, entries, psd_set_result_cache_only, arm, bn, _safe_name, PREFIX_RESULT)

    # 3. 常规条目计算
    for entry in normal_entries:
        bn, en = entry.bone_name, entry.name
        if bn not in bone_filter: continue
        if bn not in bone_to_cur_rot and bn not in bone_to_cur_loc: continue

        # 性能记录起点
        t_entry_start = time.perf_counter() if perf_enabled else None
        
        if bn not in skip_bones:
            try:
                if getattr(entry, 'is_direct_channel', False):
                    compute_direct_channel_weight(entry, bone_to_cur_rot, arm, bn, psd_set_result_cache_only, PREFIX_RESULT, _safe_name)
                if getattr(entry, 'has_rot', False):
                    compute_rotation_weight(entry, bone_to_cur_rot, bn, arm, psd_set_result_cache_only, PREFIX_RESULT, _safe_name)
                if getattr(entry, 'has_loc', False):
                    compute_location_weight(entry, bone_to_cur_loc, bn, arm, psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name)
                if getattr(entry, 'has_sca', False):
                    compute_scale_weight(entry, bone_to_cur_sca, bn, arm, psd_set_result_cache_only, PREFIX_RESULT_SCA, _safe_name)
                if getattr(arm, "psd_triggers", None):
                    compute_triggers(arm, psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name)
            except Exception as e_entry:
                print(f"[BPW ERROR] Error calculating entry {en}: {e_entry}")

        # 性能记录终点
        if perf_enabled and arm_stats:
            dt_ms = (time.perf_counter() - t_entry_start) * 1000.0 if t_entry_start else 0.0
            rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
            ent_stats = arm_stats["entries"].setdefault(rep_key, {"hist": [], "last_ms": 0.0, "avg_ms": 0.0})
            ent_stats["last_ms"] = dt_ms
            ent_stats["hist"].append(dt_ms)
            if len(ent_stats["hist"]) > 10:  # 假设 history_len 默认 10
                ent_stats["hist"].pop(0)
            ent_stats["avg_ms"] = sum(ent_stats["hist"]) / len(ent_stats["hist"])


def _psd_compute_all(arm=None, depsgraph=None, scene=None):
    """
    BPW 核心调度中心。负责节流控制、状态采集、分发计算以及数据回写。
    """
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    current_time = time.time()

    # --- 1. 节流控制 ---
    try:
        sc = bpy.context.scene if (bpy.context and getattr(bpy.context, "scene", None)) else None
        if _is_animation_playing():
            min_interval = 0.0
        else:
            hz = int(getattr(sc, "psd_idle_hz", 10) or 10)
            min_interval = 1.0 / max(1, min(hz, 240))
    except Exception:
        min_interval = 0.05
    if min_interval > 0.0 and (current_time - last_compute_time < min_interval):
        return
    last_compute_time = current_time

    perf_enabled = getattr(bpy.context.scene, 'psd_perf_enabled', False) if bpy.context and bpy.context.scene else False
    t_start_total = time.perf_counter() if perf_enabled else None

    # --- 2. 遍历处理骨架 ---
    for arm in [o for o in bpy.data.objects if o.type == 'ARMATURE']:
        output_mode = getattr(arm, "psd_output_mode", 'STORE_TO_EMPTY')
        is_driver_mode = (output_mode == 'DRIVERS_MODS')
        
        saved_entries = getattr(arm, 'psd_saved_poses', None)
        if not saved_entries: 
            continue

        try:
            arm_key = _arm_key_for_obj(arm)
            arm_stats = _psd_perf_stats.setdefault(arm.name, {"last_total_ms": 0.0, "last_arm_ms": 0.0, "entries": {}}) if perf_enabled else None
            t_start_arm = time.perf_counter() if perf_enabled else None

            # A. 确定过滤名单
            bone_filter = set(p.bone_name for p in arm.psd_bone_pairs if p.bone_name) if getattr(arm, "psd_bone_pairs", None) else set(e.bone_name for e in saved_entries if e.bone_name)

            # B. 采集状态
            state_maps = _fetch_armature_states(arm, depsgraph, bone_filter)
            bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca = state_maps

            # C. 变化检测 (Skip Logic)
            arm_cache = _psd_bone_state_cache.setdefault(arm_key, {})
            skip_bones = set()
            for bn_check in bone_filter:
                sample = _psd_make_sample_from_maps(bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca, bn_check)
                last_sample = arm_cache.get(bn_check)
                if sample is not None and last_sample is not None and _psd_samples_equal(sample, last_sample, eps=1e-5):
                    skip_bones.add(bn_check)
                elif sample is not None:
                    arm_cache[bn_check] = sample
                    
            # 排除触发器骨骼不跳过
            trigger_bones = {getattr(t, "bone_name", "") for t in getattr(arm, "psd_triggers", ())} | {getattr(t, "target_bone", "") for t in getattr(arm, "psd_triggers", ())}
            skip_bones.difference_update(trigger_bones)

            # D. 分发计算
            _dispatch_calculations(arm, saved_entries, bone_filter, skip_bones, state_maps, is_driver_mode, perf_enabled, arm_stats)

            # E. 数据写回
            if not is_driver_mode:
                mem_cache = _psd_results_cache.get(arm_key, {})
                if mem_cache:
                    if output_mode == 'STORE_TO_EMPTY':
                        psd_flush_mem_to_registered_empty(arm)
                    elif output_mode == 'APPLY_DRIVERS_JSON':
                        # JSON 写入逻辑
                        exp = _shape_expressions_cache.get(arm_key) or load_shape_drivers(arm)
                        if exp: ShapeDriver(exp).process(arm_key, mem_cache)
                        drv = _pose_drivers_cache.get(arm_key) or load_pose_drivers(arm)
                        if drv: PoseDriver(drv).process(arm, arm_key, mem_cache)

            if perf_enabled and arm_stats:
                arm_stats["last_arm_ms"] = (time.perf_counter() - t_start_arm) * 1000.0 if t_start_arm else 0.0

        except Exception as e:
            print(f'[BPW ERROR] Failed on armature {getattr(arm, "name", "unknown")}: {e}')

    # --- 3. 总体性能收尾 ---
    # (保留原有的全局性能打印逻辑)
        
        finally:
            # --- [E] 数据写回 (Flush) ---
            # 驱动模式通常不进入这里的存储逻辑，因为计算结果已直接由驱动器或 cache_only 写入
            if is_driver_mode:
                if perf_enabled:
                    arm_stats["last_arm_ms"] = (time.perf_counter() - t_start_arm) * 1000.0 if t_start_arm else 0.0
                continue 

            mem_cache = _psd_results_cache.get(_arm_key_for_obj(arm), {})
            if mem_cache:
                if output_mode == 'STORE_TO_EMPTY':
                    psd_flush_mem_to_registered_empty(arm)
                elif output_mode == 'APPLY_DRIVERS_JSON':
                    arm_key_tmp = _arm_key_for_obj(arm)
                    exp = _shape_expressions_cache.get(arm_key_tmp) or load_shape_drivers(arm)
                    if exp: ShapeDriver(exp).process(arm_key_tmp, mem_cache)
                    drv = _pose_drivers_cache.get(arm_key_tmp) or load_pose_drivers(arm)
                    if drv: PoseDriver(drv).process(arm, arm_key_tmp, mem_cache)

            if perf_enabled and arm_stats:
                arm_stats["last_arm_ms"] = (time.perf_counter() - t_start_arm) * 1000.0 if t_start_arm else 0.0

    # total perf
    if perf_enabled:
        try:
            t_end_total = time.perf_counter()
            total_ms = max(0.0, (t_end_total - t_start_total) * 1000.0) if t_start_total else 0.0
            _psd_perf_stats.setdefault("__global__", {})["last_total_ms"] = total_ms
        except Exception:
            pass

        # === 打印性能统计到控制台（调试用）===
        print("\n=== PSD Performance Stats ===")
        global_stats = _psd_perf_stats.get("__global__", {})
        print(f"Global total: {global_stats.get('last_total_ms', 0.0):.3f} ms")

        for arm_name, stats in _psd_perf_stats.items():
            if arm_name == "__global__":
                continue
            print(f"\nArmature: {arm_name}")
            print(f"  Last arm total: {stats.get('last_arm_ms', 0.0):.3f} ms")
            
            entries = stats.get("entries", {})
            if entries:
                print(f"  Per-entry avg (top 10 slowest):")
                # 按 avg_ms 降序排序，取前10
                sorted_entries = sorted(
                    entries.items(),
                    key=lambda x: x[1].get("avg_ms", 0.0),
                    reverse=True
                )[:10]
                for key, estats in sorted_entries:
                    avg = estats.get("avg_ms", 0.0)
                    last = estats.get("last_ms", 0.0)
                    if avg > 0.01:  # 只显示有意义的
                        print(f"    {key}: avg {avg:.3f} ms | last {last:.3f} ms")
            else:
                print("  No entry stats (all skipped or zero)")
        print("=============================\n")
        # ======================================
