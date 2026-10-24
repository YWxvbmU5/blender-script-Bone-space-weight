import bpy
import bpy.app.handlers as handlers
from .handlers import _unsubscribe_msgbus, _remove_handlers_with_name, psd_depsgraph_handler, psd_frame_handler
from .utils import update_direct_channel


class PSDSavedPose(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Entry Name", default="default")
    bone_name: bpy.props.StringProperty(name="Bone")
    #RBF
    use_rbf: bpy.props.BoolProperty(
        name="Use RBF",
        description="Add this pose to the RBF sample pool. All poses with RBF enabled under the same bone will participate in unified weight distribution",
        default=False
    )
    # RBF 专用：平滑度系数（可选，也可以全局统一设置）
    rbf_epsilon: bpy.props.FloatProperty(
        name="RBF Sharpness",
        description="Controls the effective range of this pose; a higher value results in faster attenuation (and sharper edges).",
        default=1.0,
        min=0.01
    )


    # 旋转通道
    record_rot_channel_mode: bpy.props.EnumProperty(
    name="Rotating Channel Mode",
    description="Compute weight using swing+twist decomposition (active when mode is not 'NONE' and cone attenuation is disabled)",
    items=[
        ('NONE', "None", "No channel mode used") ,
        ('record_rot_SWING_X_TWIST', "Swing and X Twist", "X axis as twist"),
        ('record_rot_SWING_Y_TWIST', "Swing and Y Twist", "Y axis as twist"),
        ('record_rot_SWING_Z_TWIST', "Swing and Z Twist", "Z axis as twist"),
    ],
    default='NONE'
    )
    rest_rot: bpy.props.FloatVectorProperty(name="Rest Rotation (deg)", size=3, default=(0.0,0.0,0.0))
    pose_rot: bpy.props.FloatVectorProperty(name="Pose Rotation (deg)", size=3, default=(0.0,0.0,0.0))
    has_rot: bpy.props.BoolProperty(name="Include Rotation", default=False)
    use_rot_settings: bpy.props.BoolProperty(
        name="Enable Rotation Settings",
        description="Expand and allow editing detailed rotation settings",
        default=False
    )
    is_editing: bpy.props.BoolProperty(
        name="Edit Mode",
        description="Unlock input fields for manual rotation editing",
        default=False
    )
    use_triangle_weights: bpy.props.BoolProperty(
        name="Use Triangle Weights",
        description="Use triangle-based weighting for rotation calculations",
        default=False
    )

    # 旋转的锥形衰减 (现有行为)
    cone_enabled: bpy.props.BoolProperty(name="Cone Attenuation", default=False)
    show_cone: bpy.props.BoolProperty(name="Show Cone", default=False)
    cone_angle: bpy.props.FloatProperty(name="Cone Angle (deg)", default=60.0, min=0.0, max=180.0)
    cone_axis: bpy.props.EnumProperty(
        name="Cone Axis",
        description="Local axis used as the cone central direction",
        items=[('X','X',''), ('Y','Y',''), ('Z','Z','')],
        default='Z'
    )
    # 新增：圆锥非均匀缩放属性
    cone_scale_x: bpy.props.FloatProperty(
        name="Scale X",
        description="Extrude/Stretch Cone Sensing Range Along Local X-Axis",
        default=1.0,
        min=0.01,
        soft_max=10.0
    )
    cone_scale_y: bpy.props.FloatProperty(
        name="Scale Y",
        description="Extrude/Stretch Cone Sensing Range Along Local Y-Axis",
        default=1.0,
        min=0.01,
        soft_max=10.0
    )
    cone_scale_z: bpy.props.FloatProperty(
        name="Scale Z",
        description="Extrude/Stretch Cone Sensing Range Along Local Z-Axis",
        default=1.0,
        min=0.01,
        soft_max=10.0
    )

    # 位移通道
    use_loc_settings: bpy.props.BoolProperty(name="Expand Location Settings", default=False)
    is_editing_loc: bpy.props.BoolProperty(name="Edit Location", default=False)
    rest_loc: bpy.props.FloatVectorProperty(name="Rest Location", size=3, default=(0.0,0.0,0.0))
    pose_loc: bpy.props.FloatVectorProperty(name="Pose Location", size=3, default=(0.0,0.0,0.0))
    has_loc: bpy.props.BoolProperty(name="Include Location", default=False)
    show_sphere: bpy.props.BoolProperty(name="Show Attenuation Sphere (debug)", default=False)
    loc_enabled: bpy.props.BoolProperty(name="Enable Location Attenuation", default=False)
    loc_radius: bpy.props.FloatProperty(name="Location Radius", default=0.1, min=0.0, soft_max=10.0)

    # 缩放通道
    use_sca_settings: bpy.props.BoolProperty(name="Expand Scale Settings", default=False)
    is_editing_sca: bpy.props.BoolProperty(name="Edit Scale", default=False)
    rest_sca: bpy.props.FloatVectorProperty(name="Rest Scale", size=3, default=(1.0,1.0,1.0))
    pose_sca: bpy.props.FloatVectorProperty(name="Pose Scale", size=3, default=(1.0,1.0,1.0))
    has_sca: bpy.props.BoolProperty(name="Include Scale", default=False)

    is_direct_channel: bpy.props.BoolProperty(
        name="Direct Channel Mode",
        description="If enabled, directly extract a bone axis rotation value, bypassing complex rotation calculations",
        default=False,
        update=update_direct_channel  # 绑定回调函数
    )
    channel_axis: bpy.props.EnumProperty(
        name="Channel Axis",
        items=[
            ('X', "X Axis", "Use X-axis rotation component"),
            ('Y', "Y Axis", "Use Y-axis rotation component"),
            ('Z', "Z Axis", "Use Z-axis rotation component"),
        ],
        default='X'
    )


# --- [1] 定义系数行结构 ---
class PSDBoneRBFRow(bpy.types.PropertyGroup):
    # 因为 Blender 无法直接存储嵌套 Collection，我们用一个 String 存一行，或者用多个 Float
    # 这里建议用 String 序列化存储浮点数列表，或者定义足够多的 Float 槽位
    values: bpy.props.StringProperty(default="") # 存储如 "1.2,-0.5,0.3"

# --- [2] 定义每个骨骼的 RBF 数据包 ---
class PSDBoneRBFData(bpy.types.PropertyGroup):
    bone_name: bpy.props.StringProperty()
    
    # 存储解出来的系数矩阵 Lambda
    # 每一行对应一个 Entry 的系数向量
    coeffs: bpy.props.CollectionProperty(type=PSDBoneRBFRow)
    
    # 记录计算此矩阵时用到的 Entry 数量，用于校验是否需要重新计算
    sample_count: bpy.props.IntProperty(default=0)


class PSDBonePair(bpy.types.PropertyGroup):
    bone_name: bpy.props.StringProperty(name="Bone", default="")
    show_mirror_pair: bpy.props.BoolProperty(name="Show Mirror Pair", default=False)
    show_mirror_details: bpy.props.BoolProperty(default=False)

class PSDBoneTrigger(bpy.types.PropertyGroup):
    """Single Trigger Entry: Placed within a spherical radius surrounding the trigger bone's head; generates a weight when the target bone's head enters the range."""
    name: bpy.props.StringProperty(name="Name", default="Trigger")
    bone_name: bpy.props.StringProperty(name="Trigger Bone", default="")     # 触发器骨骼（创建时默认活动骨骼）
    target_bone: bpy.props.StringProperty(name="Target Bone", default="")    # 被检测的目标骨骼
    enabled: bpy.props.BoolProperty(name="Enabled", default=True)
    radius: bpy.props.FloatProperty(name="Radius", default=0.2, min=0.0, description="Trigger radius (world units)")
    show_sphere: bpy.props.BoolProperty(name="Show Attenuation Sphere", default=False)
    # 可选：是否线性/平滑衰减（enum），当前仅用线性
    falloff: bpy.props.EnumProperty(
        name="Falloff",
        items=[
            ('LINEAR', "Linear", "Linear falloff (1 - d / r)"),
            ('SMOOTH', "Smoothstep", "Smoothstep falloff"),
        ],
        default='LINEAR'
    )
    # 运行时结果（只读供 UI 显示），不会被序列化为复杂对象，但会保存为小数
    last_weight: bpy.props.FloatProperty(name="Last Weight", default=0.0)

class PSDShapeDriverFile(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="JSON File",
        subtype='FILE_PATH',
        description="Shape Driver JSON file path"
    )

class PSDPoseDriverFile(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="JSON File",
        subtype='FILE_PATH',
        description="Pose Driver JSON file path"
    )

def update_psd_selection(self, context):
    
    obj = self # 如果定义在 Object 上，self 就是该物体
    
    # 检查索引是否有效
    if obj.psd_bone_pairs_index < 0 or obj.psd_bone_pairs_index >= len(obj.psd_bone_pairs):
        return

    entry = obj.psd_bone_pairs[obj.psd_bone_pairs_index]
#4.3.2 更新：改为直接设置 select 属性（之前版本需要设置骨骼的 select 和 bone.select 两个属性，5.1.0只需要设置 pose bone 的 select 就能正确切换活动骨骼了）
    # 切换选中骨骼逻辑
    if obj.mode == 'POSE':
        for pb in obj.pose.bones:
            pb.select = False
            
        target_pb = obj.pose.bones.get(entry.bone_name)
        if target_pb:
            target_pb.select = True
            obj.data.bones.active = target_pb.bone

def register_props():
    # 注册属性到 bpy.types.Object 和 bpy.types.Scene
    bpy.types.Object.psd_saved_poses = bpy.props.CollectionProperty(type=PSDSavedPose)
    bpy.types.Object.psd_saved_pose_index = bpy.props.IntProperty(default=-1)
    bpy.types.Object.psd_bone_pairs = bpy.props.CollectionProperty(type=PSDBonePair)
    bpy.types.Object.psd_bone_pairs_index = bpy.props.IntProperty(
        name="Bone Pair Index",
        default=0,
        update=update_psd_selection
        )
    bpy.types.Object.psd_triggers = bpy.props.CollectionProperty(type=PSDBoneTrigger)
    bpy.types.Object.psd_trigger_index = bpy.props.IntProperty(default=-1)
    
    bpy.types.Object.psd_filter_show_all_details = bpy.props.BoolProperty(
    name="Expand All Details",
    default=False,
    description="Toggle expand/collapse for all bone mirror details"
)


    # 场景属性（临时存储等）
    bpy.types.Scene.show_ray_verts = bpy.props.BoolProperty(
        name="Show Ray Verts (debug)", 
        default=False
    )
    # 场景临时存储 (旋转)
    bpy.types.Scene.psd_temp_rest = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_pose = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_pose_bone = bpy.props.StringProperty(default='')

    # 场景临时存储 (位移)
    bpy.types.Scene.psd_temp_loc_rest = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_loc = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_loc_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_loc_bone = bpy.props.StringProperty(default='')

    #
    bpy.types.Scene.psd_temp_sca_rest = bpy.props.FloatVectorProperty(size=3, default=(1.0,1.0,1.0))
    bpy.types.Scene.psd_temp_sca = bpy.props.FloatVectorProperty(size=3, default=(1.0,1.0,1.0))
    bpy.types.Scene.psd_temp_sca_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_sca_bone = bpy.props.StringProperty(default='')

    #UI
    bpy.types.Scene.psd_show_captures = bpy.props.BoolProperty(name="Show Captured Data", default=False)
    bpy.types.Scene.psd_show_triggers = bpy.props.BoolProperty(name="Show Triggers", default=False)
    bpy.types.Scene.psd_show_saved_poses = bpy.props.BoolProperty(name="Show Saved Poses", default=True)

    bpy.types.Scene.psd_running = bpy.props.BoolProperty(default=False)

    bpy.types.Scene.psd_mode = bpy.props.EnumProperty(
        name="BPW Operating Mode",
        description="AUTO (no practical use, beta): auto-switch based on playback; FORCE_PLAY (playback rate): always treated as playing; FORCE_TIMER (recommended): always use timer sampling",
        items=[
            ('AUTO', "Auto (Detect)", "Auto-switch between play/timer modes based on playback state"),
            ('FORCE_PLAY', "Force Play", "Always treat as play mode (using final depsgraph handler)"),
            ('FORCE_TIMER', "Force Timer", "Always use timer sampling"),
        ],
        default='FORCE_PLAY'
    )

    bpy.types.Scene.psd_idle_hz = bpy.props.IntProperty(
        name="Idle Frequency (Hz)",
        description="Timer update frequency in non-play mode (Hz) (1..240)",
        default=10,
        min=1,
        max=240
    )

    # 性能调试属性
    bpy.types.Scene.psd_perf_enabled = bpy.props.BoolProperty(
        name="Enable Performance Debug",
        description="Show runtime performance metrics (debug)",
        default=False
    )
    bpy.types.Scene.psd_perf_history_len = bpy.props.IntProperty(
        name="Performance History",
        description="Recent sample count used to calculate avg delay per result",
        default=10,
        min=1,
        max=200
    )
    
    bpy.types.Scene.psd_show_results = bpy.props.BoolProperty(
        name="Show BPW Results",
        description="Display all BPW results stored in armature.data (may impact UI performance)",
        default=False
    )

        # 搜索 / 排序 / 显示上限（用于 BPW 结果面板）
    bpy.types.Scene.psd_results_search = bpy.props.StringProperty(
        name="Search BPW Results",
        description="Search BPW results by key or short name (case-insensitive)",
        default=""
    )
    bpy.types.Scene.psd_results_sort_by = bpy.props.EnumProperty(
        name="Sort By",
        description="Sort BPW results",
        items=[
            ('NAME', "Name", "Sort by name (short name)"),
        ],
        default='NAME'
    )
    bpy.types.Scene.psd_results_sort_reverse = bpy.props.BoolProperty(
        name="Reverse",
        description="Reverse order (on -> descending)",
        default=False
    )
    bpy.types.Scene.psd_results_limit = bpy.props.IntProperty(
        name="Display Limit",
        description="Max number of results to display at once to avoid too much UI",
        default=200,
        min=1,
        max=5000
    )

    bpy.types.Object.psd_output_mode = bpy.props.EnumProperty(
        name="BPW Output Mode",
        description="Choose how to handle BPW calculation results",
        items=[
            ('STORE_TO_EMPTY', "Store to Empty", "Store BPW results to the registered Empty (and apply Drivers simultaneously)"),
            ('DRIVERS_MODS', "Blender Drivers Only", "Core calculations are taken over by Blender drivers, except for bone triggers"),
            ('APPLY_DRIVERS_JSON', "Apply Drivers Only (beta)", "Apply results to the model based on JSON Drivers only (does not store raw results). Requires a JSON file"),
        ],
        default='STORE_TO_EMPTY'
    )

# === 新增：全局（Scene）Shape Driver 和 Pose Driver JSON 文件列表 ===
    bpy.types.Object.psd_shape_driver_files = bpy.props.CollectionProperty(type=PSDShapeDriverFile)
    bpy.types.Object.psd_shape_driver_files_index = bpy.props.IntProperty(default=-1)

    bpy.types.Object.psd_pose_driver_files = bpy.props.CollectionProperty(type=PSDPoseDriverFile)
    bpy.types.Object.psd_pose_driver_files_index = bpy.props.IntProperty(default=-1)

    bpy.types.Object.show_psd_settings = bpy.props.BoolProperty(
        name="Show BPW Output Settings",
        description="Expand or collapse detailed BPW output settings",
        default=False
    )

    bpy.types.Object.show_psd_filter_settings = bpy.props.BoolProperty(name="Show Filters", default=False)
    bpy.types.Object.show_psd_run_settings = bpy.props.BoolProperty(name="Show Run Settings", default=False)
    bpy.types.Object.show_psd_manage_settings = bpy.props.BoolProperty(name="Show Management Settings", default=False)

    #RBF Solver 数据存储
    bpy.types.Object.psd_rbf_storage = bpy.props.CollectionProperty(type=PSDBoneRBFData)

def unregister_props():
    # 删除属性（反向操作）
    try:
        _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
    except Exception:
        pass
    try:
        _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
    except Exception:
        pass

    global _psd_timer_registered, _msgbus_subscribed
    _psd_timer_registered = False
    _unsubscribe_msgbus()


    for p in ('psd_temp_rest','psd_temp_pose','psd_temp_rest_bone','psd_temp_pose_bone',
              'psd_temp_loc_rest','psd_temp_loc','psd_temp_loc_rest_bone','psd_temp_loc_bone',
              'psd_running','psd_mode','psd_idle_hz','psd_perf_enabled','psd_perf_history_len','psd_show_results','psd_results_search','psd_results_sort_by','psd_results_sort_reverse','psd_results_limit',
              'psd_temp_sca_rest','psd_temp_sca','psd_temp_sca_rest_bone','psd_temp_sca_bone','psd_show_captures','psd_show_triggers','psd_show_saved_poses','PSD_OT_invalidate_cache',
              "psd_output_mode","psd_shape_driver_files","psd_shape_driver_files_index",
              "psd_pose_driver_files","psd_pose_driver_files_index","show_psd_settings",
              "show_psd_filter_settings","show_psd_run_settings","show_psd_manage_settings","psd_filter_show_all_details",
              "PSDBoneRBFRow","PSDBoneRBFData","show_ray_verts",
              ):
        try:
            delattr(bpy.types.Scene, p)
        except Exception:
            pass

    try:
        delattr(bpy.types.Object, 'psd_saved_poses')
        delattr(bpy.types.Object, 'psd_saved_pose_index')
    except Exception:
        pass
    try:
        delattr(bpy.types.Object, 'psd_bone_pairs')
        delattr(bpy.types.Object, 'psd_bone_pairs_index')
    except Exception:
        pass