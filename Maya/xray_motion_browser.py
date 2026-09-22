import math
import json
import os
import tempfile
from functools import partial

from maya import cmds, OpenMayaUI
from maya.api import OpenMaya as om
try:
    from PySide6 import QtCore, QtWidgets
    from shiboken6 import wrapInstance
except ImportError:
    from PySide2 import QtCore, QtWidgets
    from shiboken2 import wrapInstance

WINDOW = "ixrayMotionWindow"
_filter = None
_target = []
_motions = []
_visible_indices = []
_controls = {}
_busy = False
LIBRARY_ATTR = "ixrayMotionLibrary"
EXTENSIONS = (".skl", ".skls", ".omf")


def _selection():
    roots = set()
    for node in cmds.ls(selection=True, long=True, objectsOnly=True) or []:
        joints = [node] if cmds.nodeType(node) == "joint" else (
            cmds.listRelatives(node, allDescendents=True, type="joint", fullPath=True) or [])
        for joint in joints:
            while True:
                parents = cmds.listRelatives(joint, parent=True, type="joint", fullPath=True) or []
                if not parents:
                    break
                joint = parents[0]
            roots.add(joint)
    if len(roots) != 1:
        return []
    selection = om.MSelectionList()
    selection.add(roots.pop())
    return [om.MObjectHandle(selection.getDependNode(0))]


def _paths(handles):
    if not handles or any(not h.isValid() or not h.isAlive() for h in handles):
        raise RuntimeError("IX-Ray: select the skeleton root or its group first")
    paths = [om.MFnDagNode(h.object()).fullPathName() for h in handles]
    if not any(cmds.nodeType(p) == "joint" or
               cmds.listRelatives(p, allDescendents=True, type="joint") for p in paths):
        raise RuntimeError("IX-Ray: selection does not contain a skeleton")
    return paths


def _option(name, default):
    return cmds.optionVar(query=name) if cmds.optionVar(exists=name) else default


def _unique_display_name(motions, name):
    """Return a list label that does not collide with another registered motion."""
    existing = {str(m.get("display_name", m.get("name", ""))).casefold()
                for m in motions if isinstance(m, dict)}
    if name.casefold() not in existing:
        return name
    suffix = 1
    while (name + "_" + str(suffix)).casefold() in existing:
        suffix += 1
    return name + "_" + str(suffix)


def _display_name(motion):
    return motion.get("display_name", motion["name"])


def _load_paths(paths):
    motions = list(_motions)
    for path in paths:
        path = os.path.normpath(os.path.abspath(path))
        if path.lower().endswith(EXTENSIONS):
            for source_index, name in enumerate(cmds.ixrayMotionList(path) or []):
                motions.append(dict(path=path, name=name, display_name=_unique_display_name(motions, name), source_index=source_index,
                                    scale=1.0, stretch=1.0, start=0.0))
    _motions[:] = motions
    _save_library()
    _fill()


def _save_library():
    root = _paths(_target)[0]
    if not cmds.attributeQuery(LIBRARY_ATTR, node=root, exists=True):
        cmds.addAttr(root, longName=LIBRARY_ATTR, dataType="string")
    cmds.setAttr(root + "." + LIBRARY_ATTR, json.dumps(_motions, ensure_ascii=True), type="string")


def _fill():
    global _visible_indices
    query = cmds.textFieldGrp(_controls["search"], query=True, text=True).strip().casefold() if "search" in _controls else ""
    _visible_indices = [index for index, motion in enumerate(_motions)
                        if not query or query in _display_name(motion).casefold()]
    cmds.textScrollList(_controls["list"], edit=True, removeAll=True)
    for index in _visible_indices:
        motion = _motions[index]
        cmds.textScrollList(_controls["list"], edit=True,
                            append=_display_name(motion))


def _selected_indices():
    return cmds.textScrollList(_controls["list"], query=True, selectIndexedItem=True) or []


def _selected_motion_indices():
    return [_visible_indices[index - 1] for index in _selected_indices()
            if 0 < index <= len(_visible_indices)]


def _filter_changed(*_):
    _fill()


def rename_selected(*_):
    indices = _selected_motion_indices()
    if len(indices) != 1:
        cmds.warning("IX-Ray: select one animation to rename")
        return
    motion = _motions[indices[0]]
    result = cmds.promptDialog(title="Rename Animation", message="Name:",
                               text=_display_name(motion), button=("Rename", "Cancel"),
                               defaultButton="Rename", cancelButton="Cancel", dismissString="Cancel")
    if result != "Rename":
        return
    name = cmds.promptDialog(query=True, text=True).strip()
    if not name:
        cmds.warning("IX-Ray: animation name cannot be empty")
        return
    other = [item for index, item in enumerate(_motions) if index != indices[0]]
    motion["display_name"] = _unique_display_name(other, name)
    _save_library()
    _fill()


def delete_selected(*_):
    indices = _selected_motion_indices()
    if not indices:
        cmds.warning("IX-Ray: select animations to delete")
        return
    for index in sorted(indices, reverse=True):
        del _motions[index]
    _save_library()
    _fill()


def _remember_options(*_):
    indices = _selected_motion_indices()
    if indices:
        for key in ("scale", "stretch", "start"):
            _motions[indices[0]][key] = cmds.floatFieldGrp(_controls[key], query=True, value1=True)
        _save_library()


def _restore():
    _motions.clear()
    if not _target:
        cmds.text(_controls["target"], edit=True, label="")
        cmds.button(_controls["import"], edit=True, enable=False)
        _fill()
        return
    root = _paths(_target)[0]
    cmds.button(_controls["import"], edit=True, enable=True)
    if cmds.attributeQuery(LIBRARY_ATTR, node=root, exists=True):
        try:
            records = json.loads(cmds.getAttr(root + "." + LIBRARY_ATTR) or "[]")
            if not isinstance(records, list) or any(not isinstance(m, dict) or
                    not all(k in m for k in ("path", "name", "scale", "stretch", "start")) for m in records):
                raise ValueError("invalid library")
            for motion in records:
                label = motion.get("display_name", motion["name"])
                if label.casefold() in {str(item.get("display_name", item.get("name", ""))).casefold()
                                        for item in _motions}:
                    label = _unique_display_name(_motions, motion["name"])
                motion["display_name"] = label
                _motions.append(motion)
        except (ValueError, TypeError):
            cmds.warning("IX-Ray: cannot read motion library on " + root)
    cmds.text(_controls["target"], edit=True, label="Skeleton: " + root)
    _fill()


def _selection_changed(*_):
    global _target
    if _busy or not cmds.window(WINDOW, exists=True):
        return
    selected = _selection()
    if (bool(selected) != bool(_target) or (selected and
            (not _target[0].isValid() or not _target[0].isAlive() or
             selected[0].object() != _target[0].object()))):
        cmds.play(state=False)
        _target = selected
        _restore()


def pick_file(*_):
    if not _target:
        return
    paths = cmds.fileDialog2(fileMode=4, caption="Moution Browser",
                            fileFilter="IX-Ray motions (*.skl *.skls *.omf)")
    if paths:
        _load_paths(paths)


def export_selected(*_):
    indices = _selected_motion_indices()
    if not indices:
        cmds.warning("IX-Ray: select animations to export")
        return
    motions = [_motions[index] for index in indices]
    if not _target:
        cmds.warning("IX-Ray: select the target skeleton first")
        return
    for motion in motions:
        name = motion["name"]
        if not name or name in (".", "..") or name[-1] in " ." or any(
                c in '<>:"/\\|?*' or ord(c) < 32 for c in name):
            cmds.warning("IX-Ray: invalid OMF animation name: " + name)
            return
    names = [motion["name"] for motion in motions]
    if len({name.casefold() for name in names}) != len(names):
        cmds.warning("IX-Ray: selected animations have duplicate OMF names")
        return
    export_format = cmds.optionMenu(_controls["export_format"], query=True, value=True)
    if export_format == "SKL (one file per animation)":
        destinations = cmds.fileDialog2(fileMode=3, caption="Re-bake Selected Animations to SKL",
                                        okCaption="Export SKL")
    else:
        destinations = cmds.fileDialog2(fileMode=0, caption="Re-bake Selected Animations to OMF",
                                        okCaption="Export OMF", fileFilter="OMF files (*.omf)")
    if not destinations:
        return
    destination = destinations[0]
    if export_format != "SKL (one file per animation)" and not destination.lower().endswith(".omf"):
        destination += ".omf"
    precision = int(cmds.optionMenu(_controls["export_precision"], query=True, value=True).split()[0])
    target = _paths(_target)[0]
    meshes = _target_meshes(target)
    if not meshes:
        cmds.warning("IX-Ray: target skeleton has no skinned mesh for motion export")
        return
    for motion in motions:
        if not os.path.isfile(motion["path"]):
            cmds.warning("IX-Ray: animation source is missing: " + motion["path"])
            return
    previous = cmds.ls(selection=True, long=True) or []
    saved_range = (cmds.playbackOptions(query=True, minTime=True),
                   cmds.playbackOptions(query=True, maxTime=True), cmds.currentTime(query=True))
    global _busy
    _busy = True
    cmds.play(state=False)
    cmds.undoInfo(openChunk=True, chunkName="IX-Ray Re-bake Motions to OMF")
    saved_merge = cmds.fileInfo("ixrayOmfMerge", query=True) or []
    saved_replace = cmds.fileInfo("ixrayOmfReplace", query=True) or []
    saved_source = cmds.fileInfo("ixrayOmfMergeSource", query=True) or []
    temporary_paths = []
    try:
        if export_format != "SKL (one file per animation)":
            # The exporter writes the first motion normally. Every subsequent
            # pass uses the existing output as the merge source.
            cmds.fileInfo("ixrayOmfMerge", "false")
            cmds.fileInfo("ixrayOmfReplace", "false")
        for motion_index, motion in enumerate(motions):
            values = [float(motion.get(key, default)) for key, default in
                      (("scale", 1.0), ("stretch", 1.0), ("start", 0.0))]
            if not all(math.isfinite(value) for value in values) or min(values[:2]) <= 0:
                raise RuntimeError("IX-Ray: Scale Factor and Time Stretch must be positive")
            cmds.select(target, replace=True)
            end = cmds.ixrayMotionLoad(motion["path"], motion["name"],
                "scale_factor={};time_stretch={};start_frame={};clear_existing_keys=true".format(*values),
                motion.get("source_index", 0))
            cmds.playbackOptions(minTime=values[2], maxTime=end,
                                 animationStartTime=values[2], animationEndTime=end)
            export_path = destination
            if export_format == "SKL (one file per animation)":
                export_path = os.path.join(destination, motion["name"] + ".skl")
                export_type = "SKL export"
                options = ""
            else:
                source_options = cmds.ixrayMotionInfo(motion["path"], motion["name"],
                                                      motion.get("source_index", 0))
                options = "omf_position_precision={};omf_motion_name={};{}".format(
                    precision, motion["name"], source_options)
                export_type = "OMF export"
                if motion_index:
                    fd, export_path = tempfile.mkstemp(prefix="ixray_motion_merge_", suffix=".omf")
                    os.close(fd)
                    os.remove(export_path)
                    temporary_paths.append(export_path)
                    cmds.fileInfo("ixrayOmfMergeSource", destination)
                    options += "omf_merge=true;omf_replace=false;"
                else:
                    options += "omf_merge=false;omf_replace=false;"
            cmds.select(meshes[0], replace=True)
            cmds.file(export_path, force=True, options=options,
                      typ=export_type, exportSelected=True)
    finally:
        for path in temporary_paths:
            if os.path.exists(path):
                os.remove(path)
        cmds.fileInfo("ixrayOmfMerge", saved_merge[0] if saved_merge else "false")
        cmds.fileInfo("ixrayOmfReplace", saved_replace[0] if saved_replace else "false")
        cmds.fileInfo("ixrayOmfMergeSource", saved_source[0] if saved_source else "")
        cmds.playbackOptions(minTime=saved_range[0], maxTime=saved_range[1],
                             animationStartTime=saved_range[0], animationEndTime=saved_range[1])
        cmds.currentTime(saved_range[2])
        try:
            cmds.select(previous, replace=True) if previous else cmds.select(clear=True)
        finally:
            cmds.undoInfo(closeChunk=True)
            _busy = False
    result_name = os.path.basename(destination) if export_format != "SKL (one file per animation)" else "SKL files"
    cmds.inViewMessage(amg="IX-Ray: re-baked {} animation(s) into {}".format(len(motions), result_name),
                       position="topCenter", fade=True)


def _target_meshes(root):
    """Return meshes skinned by the selected skeleton, not unrelated scene meshes."""
    joints = set(cmds.listRelatives(root, allDescendents=True, type="joint", fullPath=True) or [])
    joints.add(root)
    meshes = []
    for cluster in cmds.ls(type="skinCluster") or []:
        influences = set()
        for influence in cmds.skinCluster(cluster, query=True, influence=True) or []:
            influences.update(cmds.ls(influence, long=True) or [influence])
        if not joints.intersection(influences):
            continue
        for geometry in cmds.skinCluster(cluster, query=True, geometry=True) or []:
            shape = (cmds.ls(geometry, long=True) or [geometry])[0]
            parent = cmds.listRelatives(shape, parent=True, fullPath=True) or [shape]
            if parent[0] not in meshes:
                meshes.append(parent[0])
    return meshes


def load_selected(*_, play=False):
    global _busy
    indices = _selected_motion_indices()
    if not indices:
        cmds.warning("IX-Ray: choose an animation")
        return
    if len(indices) != 1:
        cmds.warning("IX-Ray: select one animation for playback; multiple selection is for export")
        return
    target = _paths(_target)
    values = [cmds.floatFieldGrp(_controls[key], query=True, value1=True)
              for key in ("scale", "stretch", "start")]
    clear_existing = cmds.checkBox(_controls["clear_existing"], query=True, value=True)
    if not all(math.isfinite(v) for v in values) or min(values[:2]) <= 0:
        cmds.warning("IX-Ray: Scale Factor and Time Stretch must be positive")
        return
    for key, value in zip(("scale", "stretch", "start"), values):
        cmds.optionVar(floatValue=("ixrayMotion_" + key, value))
    motion = _motions[indices[0]]
    path, name = motion["path"], motion["name"]
    if not os.path.isfile(path):
        cmds.warning("IX-Ray: animation source file is missing: " + path)
        return
    cmds.play(state=False)
    previous = cmds.ls(selection=True, long=True) or []
    _busy = True
    cmds.undoInfo(openChunk=True, chunkName="IX-Ray Import Motion")
    try:
        cmds.select(target, replace=True)
        end = cmds.ixrayMotionLoad(path, name,
                            "scale_factor={};time_stretch={};start_frame={};clear_existing_keys={}".format(
                                *values, "true" if clear_existing else "false"),
                                 motion.get("source_index", 0))
        _remember_options()
    finally:
        try:
            cmds.select(previous, replace=True) if previous else cmds.select(clear=True)
        finally:
            cmds.undoInfo(closeChunk=True)
            _busy = False
    cmds.playbackOptions(minTime=values[2], maxTime=max(values[2], end),
                         animationStartTime=values[2], animationEndTime=max(values[2], end))
    cmds.currentTime(values[2])
    if play:
        cmds.play(forward=True)


def extern_load(paths):
    global _target
    selected = _selection()
    if not selected:
        raise RuntimeError("IX-Ray: select one skeleton to register imported animations")
    root = _paths(selected)[0]
    if not cmds.attributeQuery(LIBRARY_ATTR, node=root, exists=True):
        cmds.addAttr(root, longName=LIBRARY_ATTR, dataType="string")
    record = []
    if cmds.attributeQuery(LIBRARY_ATTR, node=root, exists=True):
        try:
            data = json.loads(cmds.getAttr(root + "." + LIBRARY_ATTR) or "[]")
            if isinstance(data, list):
                record = data
        except (ValueError, TypeError):
            pass
    for path in paths:
        path = os.path.normpath(os.path.abspath(path))
        if not path.lower().endswith(EXTENSIONS):
            continue
        for source_index, name in enumerate(cmds.ixrayMotionList(path) or []):
            record.append(dict(path=path, name=name, display_name=_unique_display_name(record, name), source_index=source_index,
                               scale=1.0, stretch=1.0, start=0.0))
    cmds.setAttr(root + "." + LIBRARY_ATTR, json.dumps(record, ensure_ascii=True), type="string")
    if not cmds.about(batch=True) and cmds.window(WINDOW, exists=True):
        _target = selected
        _restore()


def show(paths=None, target=None):
    global _target
    _target = _selection() if target is None else target
    if cmds.window(WINDOW, exists=True):
        cmds.deleteUI(WINDOW)
    cmds.window(WINDOW, title="Moution Browser", widthHeight=(440, 480))
    cmds.scrollLayout(childResizable=True)
    cmds.columnLayout(adjustableColumn=True, rowSpacing=6)
    _controls["target"] = cmds.text(label="", align="left")
    _controls["import"] = cmds.button(label="Get Animation File...", command=pick_file)
    _controls["search"] = cmds.textFieldGrp(label="Search", textChangedCommand=_filter_changed)
    _controls["list"] = cmds.textScrollList(allowMultiSelection=True, height=260,
                                           deleteKeyCommand=delete_selected,
                                           doubleClickCommand=partial(load_selected, play=True))
    menu = cmds.popupMenu(parent=_controls["list"], button=3)
    cmds.menuItem(parent=menu, label="Rename", command=rename_selected)
    cmds.menuItem(parent=menu, label="Delete", command=delete_selected)
    cmds.button(label="Select All", command=lambda *_: cmds.textScrollList(
        _controls["list"], edit=True, selectIndexedItem=list(range(1, len(_visible_indices) + 1))) if _visible_indices else None)
    _controls["export_precision"] = cmds.optionMenu(label="OMF key precision")
    for precision in ("8 bit", "16 bit", "32 bit"):
        cmds.menuItem(label=precision)
    cmds.optionMenu(_controls["export_precision"], edit=True,
                    value=str(int(_option("ixrayMotion_exportPrecision", 32))) + " bit",
                    changeCommand=lambda value: cmds.optionVar(
                        intValue=("ixrayMotion_exportPrecision", int(value.split()[0]))))
    _controls["export_format"] = cmds.optionMenu(label="Re-bake format")
    cmds.menuItem(label="OMF (one file)")
    cmds.menuItem(label="SKL (one file per animation)")
    cmds.button(label="Re-bake Selected...", command=export_selected)
    for key, label, default in (("scale", "Scale Factor", 1.0),
                                ("stretch", "Time Stretch", 1.0),
                                ("start", "Start Frame", 0.0)):
        _controls[key] = cmds.floatFieldGrp(numberOfFields=1, label=label,
                                          value1=_option("ixrayMotion_" + key, default))
    _controls["clear_existing"] = cmds.checkBox(label="Remove existing keys in import range",
                                                  value=_option("ixrayMotion_clearExisting", True),
                                                  changeCommand=lambda value: cmds.optionVar(intValue=("ixrayMotion_clearExisting", int(value))))
    cmds.button(label="Load Motion", command=load_selected)
    cmds.scriptJob(event=("SelectionChanged", _selection_changed), parent=WINDOW)
    cmds.showWindow(WINDOW)
    _restore()
    if paths and _target:
        _load_paths(paths)


class _ViewportDropFilter(QtCore.QObject):
    def _viewport(self, widget):
        if not isinstance(widget, QtWidgets.QWidget):
            return False
        for panel in cmds.getPanel(type="modelPanel") or []:
            pointer = OpenMayaUI.MQtUtil.findControl(panel)
            if pointer:
                viewport = wrapInstance(int(pointer), QtWidgets.QWidget)
                if widget is viewport or viewport.isAncestorOf(widget):
                    return True
        return False

    def eventFilter(self, widget, event):
        if event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            if cmds.window(WINDOW, exists=True):
                pointer = OpenMayaUI.MQtUtil.findControl(_controls.get("list", ""))
                if pointer:
                    control = wrapInstance(int(pointer), QtWidgets.QWidget)
                    view = control if isinstance(control, QtWidgets.QAbstractItemView) else control.findChild(QtWidgets.QAbstractItemView)
                    if view and widget is view.viewport() and not view.indexAt(event.pos()).isValid():
                        cmds.textScrollList(_controls["list"], edit=True, deselectAll=True)
                        return True
                window_pointer = OpenMayaUI.MQtUtil.findWindow(WINDOW)
                if window_pointer and isinstance(widget, QtWidgets.QWidget):
                    window = wrapInstance(int(window_pointer), QtWidgets.QWidget)
                    if window.isAncestorOf(widget) and type(widget) is QtWidgets.QWidget:
                        cmds.textScrollList(_controls["list"], edit=True, deselectAll=True)
            return False
        if event.type() not in (QtCore.QEvent.DragEnter, QtCore.QEvent.DragMove, QtCore.QEvent.Drop):
            return False
        urls = event.mimeData().urls()
        paths = [u.toLocalFile() for u in urls if u.isLocalFile()]
        if not paths or len(paths) != len(urls) or not all(p.lower().endswith(EXTENSIONS) for p in paths):
            return False
        if not self._viewport(widget):
            return False
        event.acceptProposedAction()
        if event.type() == QtCore.QEvent.Drop:
            target = _selection()
            if target:
                # Defer UI creation until Qt finishes dispatching the drop.
                cmds.evalDeferred(partial(show, paths, target))
        return True


def install_drop():
    global _filter
    if cmds.about(batch=True):
        return
    uninstall_drop()
    app = QtWidgets.QApplication.instance()
    _filter = _ViewportDropFilter(app)
    app.installEventFilter(_filter)


def uninstall_drop():
    global _filter
    if _filter is not None:
        QtWidgets.QApplication.instance().removeEventFilter(_filter)
        _filter.deleteLater()
        _filter = None
    if cmds.window(WINDOW, exists=True):
        cmds.deleteUI(WINDOW)
