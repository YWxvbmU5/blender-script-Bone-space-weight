bl_info = {
    "name": "骨骼空间姿态权重(BSW)",
    "author": "s0lus",
    "version": (2, 8),
    "blender": (4, 3, 2),
    "location": "3D视图 > 侧边栏 > BSW",
    "description": "骨骼空间姿态权重(Bone Space Weight)捕捉骨骼的静止/姿态旋转和位置，并将其另存为逐骨架的已保存姿态。BSW算法为每个已保存的姿态（旋转+可选位移）计算单个浮点值，并将其写入空物体数据或者根据JSON应用驱动。",
    "category": "动画",
}

import bpy
import os
from . import props
from . import utils
from . import math_utils
from . import core
from . import operators
from . import ui
from . import handlers

classes = (
    props.PSDSavedPose,
    props.PSDBonePair,
    props.PSDBoneTrigger,
    props.PSDShapeDriverFile,
    props.PSDPoseDriverFile,
    ui.PSD_UL_BonePair,
    ui.PSD_UL_SavedPose,
    ui.PSD_UL_BoneTrigger,
    ui.PSDPanel,
    ui.PSD_UL_ShapeDriverFile,
    ui.PSD_UL_PoseDriverFile,
    operators.PSDAddShapeDriverFile,
    operators.PSDRemoveShapeDriverFile,
    operators.PSDAddPoseDriverFile,
    operators.PSDRemovePoseDriverFile, 
    operators.PSDReloadShapeDrivers,
    operators.PSDReloadPoseDrivers,
    operators.PSD_OT_AddBonePair,
    operators.PSD_OT_RemoveBonePair,
    operators.PSD_OT_MoveBonePair,
    operators.PSDExportConfig,
    operators.PSDImportConfig,
    operators.PSDCaptureRest,
    operators.PSDCaptureRotation,
    operators.PSDCaptureLocationRest,
    operators.PSDCaptureLocation,
    operators.PSDCaptureScaleRest,
    operators.PSDCaptureScale,
    operators.PSDSaveCapturedRotationEntry,
    operators.PSDSaveCapturedLocationEntry,
    operators.PSDSaveCapturedScaleEntry,
    operators.PSDRemoveSavedEntry,
    operators.PSDAddSavedEntry,
    operators.PSD_OT_AddTrigger,
    operators.PSD_OT_RemoveTrigger,
    operators.PSD_OT_SelectTriggerBone,
    operators.PSDRecordChannelX,
    operators.PSDRecordChannelY,
    operators.PSDRecordChannelZ,
    operators.PSD_OT_invalidate_cache,
    operators.PSD_OT_register_cache_empty_ui,
    operators.PSD_OT_unregister_cache_empty_ui,
    operators.PSDApplySavePose,
    operators.PSDCapturePose,
    handlers.PSDStartOperator,
    handlers.PSDStopOperator,
)

def register():

    bpy.types.Scene.psd_cache_empty = bpy.props.PointerProperty(
        name="PSD Cache Empty",
        description="选择用于 PSD 缓存的 Empty（必须是 Empty 类型）",
        type=bpy.types.Object
    )

    for cls in classes:
        bpy.utils.register_class(cls)


    # 注册属性（从 props.py 调用）
    props.register_props()
    
    # 注册处理器（从 handlers.py 调用）
    handlers.register_handlers()
    
    # 其他初始化（如核心变量，如果需要）
    core.init_globals()  # 如果 core.py 有初始化函数

def unregister():

    del bpy.types.Scene.psd_cache_empty
    # 注销处理器（从 handlers.py 调用）
    handlers.unregister_handlers()
    
    # 注销属性（从 props.py 调用）
    props.unregister_props()
    
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()