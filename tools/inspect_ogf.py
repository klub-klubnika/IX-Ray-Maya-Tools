import sys
import maya.standalone

maya.standalone.initialize(name="Python")
import maya.cmds as cmds

cmds.loadPlugin(r"E:\Programs\Autodesk\Maya2026\bin\plug-ins\ixray_maya_tools_2026.mll")
for path in sys.argv[1:]:
    cmds.file(new=True, force=True)
    cmds.file(path, i=True, type="OGF import")
    joints = sorted(cmds.ls(type="joint") or [])
    meshes = cmds.ls(type="mesh", noIntermediate=True) or []
    print("OGF", path)
    print("JOINT_COUNT", len(joints))
    print("JOINTS", ",".join(joints))
    print("MESH_COUNT", len(meshes))
