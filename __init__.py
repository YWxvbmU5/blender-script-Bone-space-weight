bl_info = {
    "name": "Bone Pose Weight",
    "author": "s0lus",
    "version": (2, 8),
    "blender": (4, 3, 2),
    "location": "3D View > Sidebar > BPW",
    "description": "Bone Pose Weight captures the rest/pose rotation and location of bones and saves them as per-skeleton saved poses. The BPW algorithm calculates a single floating-point value for each saved pose (rotation + optional translation) and writes it to Empty object data or applies it as a driver based on JSON configuration.",
    "category": "Animation",
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
from . import driver_math
from . import driver_operators
from . import translations

classes = (
    props.PSDSavedPose,
    props.PSDBonePair,
    props.PSDBoneTrigger,
    props.PSDShapeDriverFile,
    props.PSDPoseDriverFile,
    props.PSDBoneRBFRow,
    props.PSDBoneRBFData,
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
    operators.PSD_OT_MirrorBoneFilter,
    operators.PSD_OT_SyncMirrorData,
    operators.PSD_OT_ToggleAllFilterDetails,
    operators.PSD_OT_CopyPath,
    operators.PSD_OT_UpdateRBFSolver,
    operators.PSD_OT_EntriesToShapeKey,
    operators.PSD_OT_ShapeKeyDriver,
    driver_operators.PSD_OT_BindDriverToCache,
    driver_operators.PSD_OT_BatchBindAll,
    driver_operators.PSD_OT_RemoveDriverFromCache,
    driver_operators.PSD_OT_ClearAllCacheDrivers,
    driver_operators.PSD_OT_RefreshDrivers,
    driver_operators.PSD_OT_MuteDrivers,
    driver_operators.PSD_OT_UnmuteDrivers,
    driver_operators.PSD_OT_BindTriggerDriver,
    driver_operators.PSD_OT_RemoveTriggerDriver,
    handlers.PSDStartOperator,
    handlers.PSDStopOperator,
)




def register():



    translations.register()

    bpy.types.Scene.psd_cache_empty = bpy.props.PointerProperty(
        name="PSD Cache Empty",
        description="Select an Empty object for PSD caching (must be of type Empty).",
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

    driver_math.register_drivers(None)  # 注册驱动函数到命名空间
    bpy.app.handlers.load_post.append(driver_math.register_drivers)  # 确保在文件加载后注册驱动函数


def unregister():

    translations.unregister()

    del bpy.types.Scene.psd_cache_empty
    # 注销处理器（从 handlers.py 调用）
    handlers.unregister_handlers()
    
    # 注销属性（从 props.py 调用）
    props.unregister_props()
    
    bpy.app.handlers.load_post.remove(driver_math.register_drivers)  # 移除

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()