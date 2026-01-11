import bpy
import json

if bpy.context.mode != 'POSE':
    print("请切换到Pose模式!")
else:
    armature = bpy.context.active_object
    if armature.type != 'ARMATURE':
        print("激活对象必须是Armature!")
    else:
        selected_bones = {b.name: b for b in bpy.context.selected_pose_bones}
        if not selected_bones:
            print("没有选中骨骼!")
        else:
            json_path = ""  # <--- 改成JSON路径（可以是原文件或修改后的）
            
            with open(json_path, "r", encoding="utf-8") as f:
                drivers_data = json.load(f)
            
            applied_count = 0
            for bone_name, cons in drivers_data.items():
                if bone_name in selected_bones:
                    pose_bone = selected_bones[bone_name]
                    for con_name, props in cons.items():
                        con = pose_bone.constraints.get(con_name)
                        if con:
                            for prop_name, driver_info in props.items():
                                data_path = f'pose.bones["{bone_name}"].constraints["{con_name}"].{prop_name}'
                                # 移除旧驱动器
                                if armature.animation_data:
                                    armature.driver_remove(data_path, -1)  # -1 表示移除所有索引
                                
                                # 添加新驱动器
                                fcurve = con.driver_add(prop_name)
                                driver = fcurve.driver
                                driver.type = driver_info["type"]
                                driver.expression = driver_info["expression"]
                                
                                # 清空变量并重新添加
                                for var_dict in driver_info["variables"]:
                                    var = driver.variables.new()
                                    var.name = var_dict["name"]
                                    var.type = var_dict["type"]
                                    if var.targets and var_dict["target_id"] and var_dict["data_path"]:
                                        target_obj = bpy.data.objects.get(var_dict["target_id"])
                                        if target_obj:
                                            var.targets[0].id = target_obj
                                            var.targets[0].data_path = var_dict["data_path"]
                                        else:
                                            print(f"警告: 未找到目标对象 {var_dict['target_id']}")
                                
                                applied_count += 1
                        else:
                            print(f"未找到约束: {con_name} 在骨骼 {bone_name}")
            
            print(f"导入完成！共应用 {applied_count} 个驱动器。")