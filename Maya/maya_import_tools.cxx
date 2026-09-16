#define NOMINMAX
#include <maya/MTypes.h>
#include <cmath>
#include <io.h>

#if MAYA_API_VERSION >= 20180000 && MAYA_API_VERSION <= 20190200
#	include <MCppCompat.h>
#endif

#include <maya/MAnimControl.h>
#include <maya/MAngle.h>
#include <maya/MFileObject.h>
#include <maya/MDGModifier.h>
#include <maya/MDagPath.h>
#include <maya/MTime.h>
#include <maya/MDagPathArray.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MFloatArray.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnCharacter.h>
#include <maya/MFnClip.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnSet.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MIntArray.h>
#include <maya/MObjectArray.h>
#include <maya/MItDag.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MPlugArray.h>
#include <maya/MPointArray.h>
#include <maya/MProgressWindow.h>
#include <maya/MSelectionList.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MFloatVectorArray.h>
#include "maya_import_tools.h"
#include "maya_bone_collision.h"
#include "xr_object.h"
#include "xr_envelope.h"
#include "xr_file_system.h"

using namespace xray_re;

static MObject create_texture(const std::string& texture, MStatus* return_status = 0);

namespace {

// The order in an X-Ray bone vector is its global OMF bone-ID order.  Maya's
// skinCluster is free to reorder influences, so retain the source order on
// the root joint for a later OMF export.
const char* const k_xray_bone_order_attr = "ixrayBoneOrder";

static MStatus save_xray_bone_order(const xr_bone_vec& bones, const maya_object_map& joints)
{
	if (bones.empty()) return MS::kSuccess;
	const xr_bone* root_bone = 0;
	for (xr_bone_vec_cit it = bones.begin(); it != bones.end(); ++it) {
		if ((*it)->is_root()) { root_bone = *it; break; }
	}
	if (!root_bone) return MS::kFailure;
	const maya_object_map::const_iterator root = joints.find(root_bone->name());
	if (root == joints.end()) return MS::kFailure;

	MStatus status;
	MFnDependencyNode root_fn(root->second, &status);
	if (!status) return status;
	MPlug order_plug = root_fn.findPlug(k_xray_bone_order_attr, true, &status);
	if (!status) {
		MFnTypedAttribute attr_fn;
		MObject attr = attr_fn.create(k_xray_bone_order_attr, "ixbo", MFnData::kString,
			MObject::kNullObj, &status);
		if (!status) return status;
		attr_fn.setHidden(true);
		attr_fn.setStorable(true);
		if (!(status = root_fn.addAttribute(attr))) return status;
		order_plug = root_fn.findPlug(k_xray_bone_order_attr, true, &status);
		if (!status) return status;
	}

	MString order;
	for (xr_bone_vec_cit it = bones.begin(); it != bones.end(); ++it) {
		order += (*it)->name().c_str();
		order += "\n";
	}
	return order_plug.setString(order);
}

} // namespace

maya_import_tools::maya_import_tools(const MString& options)
{
	set_default_options();
	parse_options(options);
}

maya_import_tools::maya_import_tools(const xray_re::xr_object* object, MStatus* return_status, const MString& options)
{
	set_default_options();
	parse_options(options);

	MStatus status = import_object(object);
	if (return_status)
		*return_status = status;
}

maya_import_tools::~maya_import_tools()
{
}

static MString make_maya_name(const std::string& base, const char* old_suffix, const char* suffix = "")
{
	std::string::size_type pos = base.rfind(old_suffix);
	if (pos == std::string::npos)
		return MString(base.c_str()) + suffix;
	else
		return MString(base.c_str(), int(pos & INT_MAX)) + suffix;
}
static void smooth_by_angle(MObject& mesh_obj, double angle_degrees = 60.0)
{
	MFnMesh mesh_fn(mesh_obj);
	int num_polygons = mesh_fn.numPolygons();
	if (num_polygons == 0)
		return;

	MFloatVectorArray poly_normals(num_polygons);
	for (MItMeshPolygon pit(mesh_obj); !pit.isDone(); pit.next())
	{
		MVector n;
		pit.getNormal(n, MSpace::kObject);
		poly_normals.set(MFloatVector(float(n.x), float(n.y), float(n.z)), pit.index());
	}

	double cos_threshold = cos(angle_degrees * 3.14159265358979323846 / 180.0);

	MIntArray connected_faces;
	for (MItMeshEdge eit(mesh_obj); !eit.isDone(); eit.next())
	{
		MStatus status;
		eit.getConnectedFaces(connected_faces, &status);
		if (!status || connected_faces.length() != 2)
		{
			eit.setSmoothing(false);
			continue;
		}
		const MFloatVector& n0 = poly_normals[connected_faces[0]];
		const MFloatVector& n1 = poly_normals[connected_faces[1]];
		double dot = double(n0.x)*n1.x + double(n0.y)*n1.y + double(n0.z)*n1.z;
		eit.setSmoothing(dot >= cos_threshold);
	}
}

MStatus maya_import_tools::import_object(const xr_object* object)
{
	MStatus status = MS::kSuccess;
	const xr_bone_vec& bones = object->bones();

	for (xr_bone_vec_cit it = bones.begin(), end = bones.end(); it != end; ++it)
	{
		if ((*it)->is_root())
		{
			status = import_bone(*it, MObject::kNullObj);
			break;
		}
	}
	if (!status)
		return status;
	if (!(status = save_xray_bone_order(bones, m_joints)))
		return status;

	maya_object_map shared_textures;
	for (xr_surface_vec_cit it = object->surfaces().begin(),
			end = object->surfaces().end(); it != end; ++it)
	{
		xr_surface* surface = *it;
		MObject& texture_obj = shared_textures[surface->texture()];
		if (texture_obj.isNull())
		{
			texture_obj = create_texture(surface->texture(), &status);
			if (!status)
				break;
		}
		if (!(status = import_surface(surface, texture_obj)))
			break;
	}
	if (!status)
		return status;
	shared_textures.clear();

	MObjectArray created_transforms;

	for (xr_mesh_vec_cit it = object->meshes().begin(),
			end = object->meshes().end(); it != end; ++it)
	{
		MObject out_transform;
		if (!(status = import_mesh(*it, bones, &out_transform)))
			break;
		if (!out_transform.isNull())
			created_transforms.append(out_transform);
	}
	if (!status)
		return status;

	if (created_transforms.length() == 1 && !m_group_name.empty())
	{
		MFnDagNode dag_fn(created_transforms[0]);
		dag_fn.setName(MString(m_group_name.c_str()));
	}
	else if (created_transforms.length() > 1 && !m_group_name.empty())
	{
		MFnTransform group_fn;
		MObject group_obj = group_fn.create(MObject::kNullObj, &status);
		if (status)
		{
			std::string safe_name = m_group_name;
			if (!safe_name.empty() && std::isdigit((unsigned char)safe_name[0]))
				safe_name = "_" + safe_name;
			group_fn.setName(MString(safe_name.c_str()));
			for (unsigned i = 0, n = created_transforms.length(); i != n; ++i)
				group_fn.addChild(created_transforms[i]);
		}
	}

	if (!bones.empty())
	{
		for (xr_skl_motion_vec_cit it = object->motions().begin(),
				end = object->motions().end(); it != end; ++it)
		{
			status = import_selected_motion(*it);
			if (!status) break;
		}
	}

	return status;
}

static MPlug get_free_plug(MFn::Type filter, const char* list_name)
{
	MItDependencyNodes it(filter);
	if (!it.isDone())
	{
		MFnDependencyNode dep_fn(it.thisNode());
		MStatus status;
		MPlug list_plug = dep_fn.findPlug(list_name, &status);
		if (status)
		{
			for (unsigned i = 0;; ++i)
			{
				MPlug plug = list_plug.elementByLogicalIndex(i, &status);
				CHECK_MSTATUS(status);
				if (!plug.isConnected())
					return plug;
			}
		}
	}
	return MPlug();
}

static std::string resolve_addon_texture(const std::string& name)
{
	xr_file_system& fs = xr_file_system::instance();
	const char* addons_root = fs.resolve_path("$ixr_addons$");
	if (addons_root == 0)
		return std::string();
	_finddata_t find_data;
	intptr_t find_handle = _findfirst((std::string(addons_root) + "*").c_str(), &find_data);
	if (find_handle == intptr_t(-1))
		return std::string();
	std::string found;
	while (true)
	{
		if (find_data.attrib & _A_SUBDIR)
		{
			std::string candidate = std::string(addons_root) + find_data.name + "\\textures\\" + name;
			if (xr_file_system::file_exist(candidate))
			{
				found = candidate;
				break;
			}
		}
		if (_findnext(find_handle, &find_data) != 0)
			break;
	}
	_findclose(find_handle);
	return found;
}

static MObject create_texture(const std::string& texture, MStatus* return_status)
{
	std::string path(texture);
	std::string probe_path = path + ".dds";
	xr_file_system& fs = xr_file_system::instance();
	if (fs.file_exist(PA_GAME_TEXTURES, probe_path.c_str()))
	{
		std::string full_path;
		fs.resolve_path(PA_GAME_TEXTURES, probe_path, full_path);
		path = full_path;
	}
	else
	{
		std::string addon_path = resolve_addon_texture(probe_path);
		path = addon_path.empty() ? probe_path : addon_path;
	}
	for (std::string::size_type i = 0; (i = path.find('\\', i)) != std::string::npos; ++i)
		path[i] = '/';

	std::string::size_type i = texture.rfind('\\');
	MString maya_name(i != std::string::npos ? texture.c_str() + i + 1 : texture.c_str());

	MStatus status;
	MFnDependencyNode texture_fn;
	MObject texture_obj = texture_fn.create("file", maya_name += "_F", &status);
	if (!status)
	{
		if (return_status)
			*return_status = status;
		return MObject::kNullObj;
	}
	texture_fn.findPlug("ftn").setValue(path.c_str());

	MDGModifier dg_modifier;
	dg_modifier.connect(texture_fn.findPlug("msg"), get_free_plug(MFn::kTextureList, "textures"));
	dg_modifier.doIt();

	MFnDependencyNode p2dt_fn;
	p2dt_fn.create("place2dTexture", maya_name += "_P2DT", &status);
	if (!status)
	{
		if (return_status)
			*return_status = status;
		return MObject::kNullObj;
	}
	dg_modifier.connect(p2dt_fn.findPlug("c"), texture_fn.findPlug("c"));
	dg_modifier.connect(p2dt_fn.findPlug("tf"), texture_fn.findPlug("tf"));
	dg_modifier.connect(p2dt_fn.findPlug("rf"), texture_fn.findPlug("rf"));
	dg_modifier.connect(p2dt_fn.findPlug("mu"), texture_fn.findPlug("mu"));
	dg_modifier.connect(p2dt_fn.findPlug("mv"), texture_fn.findPlug("mv"));
	dg_modifier.connect(p2dt_fn.findPlug("s"), texture_fn.findPlug("s"));
	dg_modifier.connect(p2dt_fn.findPlug("wu"), texture_fn.findPlug("wu"));
	dg_modifier.connect(p2dt_fn.findPlug("wv"), texture_fn.findPlug("wv"));
	dg_modifier.connect(p2dt_fn.findPlug("re"), texture_fn.findPlug("re"));
	dg_modifier.connect(p2dt_fn.findPlug("of"), texture_fn.findPlug("of"));
	dg_modifier.connect(p2dt_fn.findPlug("r"), texture_fn.findPlug("ro"));
	dg_modifier.connect(p2dt_fn.findPlug("n"), texture_fn.findPlug("n"));
	dg_modifier.connect(p2dt_fn.findPlug("vt1"), texture_fn.findPlug("vt1"));
	dg_modifier.connect(p2dt_fn.findPlug("vt2"), texture_fn.findPlug("vt2"));
	dg_modifier.connect(p2dt_fn.findPlug("vt3"), texture_fn.findPlug("vt3"));
	dg_modifier.connect(p2dt_fn.findPlug("vc1"), texture_fn.findPlug("vc1"));
	dg_modifier.connect(p2dt_fn.findPlug("o"), texture_fn.findPlug("uv"));
	dg_modifier.connect(p2dt_fn.findPlug("ofs"), texture_fn.findPlug("fs"));
	dg_modifier.doIt();

	if (return_status)
		*return_status = MS::kSuccess;
	return texture_obj;
}

static void set_xraymtl_attr(MFnDependencyNode& dep_fn, const char* name, const std::string& value)
{
	MStatus status;
	MPlug plug = dep_fn.findPlug(name, &status);
	if (status)
	{
		MFnEnumAttribute attr_fn(plug.attribute(), &status);
		if (status)
		{
			std::string temp(value);
			for (std::string::size_type i = 0; (i = temp.find('\\', i)) != std::string::npos; ++i)
				temp[i] = '/';
			short index = attr_fn.fieldIndex(temp.c_str(), &status);
			if (!status)
			{
				msg("xray_re: can't set attribute %s to %s", name, value.c_str());
				MGlobal::displayError(MString("xray_re: can't set attribute ") +
						name + " to " + value.c_str());
				return;
			}
			plug.setShort(index);
		}
	}
}

MStatus maya_import_tools::import_surface(const xr_surface* surface, MObject& texture_obj)
{
	MStatus status;
	MFnDependencyNode shader_fn;
	MObject shader_obj = shader_fn.create("XRayMtl",
			make_maya_name(surface->name(), "_S", "_M"), &status);
	if (!status)
	{
		msg("xray_re: can't create shader");
		MGlobal::displayError("xray_re: can't create shader");
		return status;
	}

	set_xraymtl_attr(shader_fn, "xrayGameMaterial", surface->gamemtl());
	set_xraymtl_attr(shader_fn, "xrayEngineShader", surface->eshader());
	set_xraymtl_attr(shader_fn, "xrayCompilerShader", surface->cshader());
	shader_fn.findPlug("xrayDoubleSide").setBool(surface->two_sided());

	MDGModifier dg_modifier;
	dg_modifier.connect(shader_fn.findPlug("msg"), get_free_plug(MFn::kShaderList, "shaders"));
	dg_modifier.doIt();

	MFnSet set_fn;
	MObject set_obj = set_fn.create(MSelectionList(), MFnSet::kRenderableOnly, &status);
	if (!status)
	{
		msg("xray_re: can't create shading group");
		MGlobal::displayError("xray_re: can't create shading group");
		return status;
	}
	set_fn.setName(make_maya_name(surface->name(), "_S", "_SG"));

	MPlug ss_plug = set_fn.findPlug("ss", &status);
	MPlugArray connected_plugs;
	if (ss_plug.connectedTo(connected_plugs, true, false))
	{
		for (unsigned i = connected_plugs.length(); i != 0;)
			dg_modifier.disconnect(connected_plugs[--i], ss_plug);
		dg_modifier.doIt();
	}
	dg_modifier.connect(shader_fn.findPlug("oc"), ss_plug);
	dg_modifier.doIt();

	MFnDependencyNode texture_fn(texture_obj);
	dg_modifier.connect(texture_fn.findPlug("oc"), shader_fn.findPlug("c"));
	dg_modifier.doIt();

	auto Plug = set_fn.findPlug("msg");
	MItDependencyGraph dg_it(Plug, MFn::kMaterialInfo,
			MItDependencyGraph::kDownstream, MItDependencyGraph::kDepthFirst,
			MItDependencyGraph::kNodeLevel);
	if (!dg_it.isDone())
	{
		MFnDependencyNode info_fn(dg_it.thisNode());
		status = dg_modifier.connect(texture_fn.findPlug("msg"),
				info_fn.findPlug("t").elementByLogicalIndex(0));
		CHECK_MSTATUS(status);
		dg_modifier.doIt();
	}

	m_sets[surface->name()] = set_obj;

	return status;
}

MStatus maya_import_tools::import_bone(const xr_bone* bone, MObject& parent_obj)
{
	MStatus status;

	MFnIkJoint joint_fn;
	MObject joint_obj = joint_fn.create(parent_obj, &status);
	if (!status)
	{
		msg("xray_re: can't create joint %s", bone->name().c_str());
		MGlobal::displayError(MString("xray_re: can't create joint ") + bone->name().c_str());
		return status;
	}
	joint_fn.setName(bone->name().c_str());

	const fvector3& t = bone->bind_offset();
	double x = MDistance(t.x, MDistance::kMeters).asCentimeters();
	double y = MDistance(t.y, MDistance::kMeters).asCentimeters();
	double z = MDistance(t.z, MDistance::kMeters).asCentimeters();
	joint_fn.setTranslation(MVector(x, y, -z), MSpace::kTransform);

	const fvector3& r = bone->bind_rotate();
	joint_fn.setRotation(MEulerRotation(-r.x, -r.y, r.z, MEulerRotation::kZXY));
	if (!(status = import_bone_collision(joint_obj, *bone)))
		return status;

	m_joints.insert(maya_object_pair(bone->name(), joint_obj));

	for (xr_bone_vec_cit it = bone->children().begin(), end = bone->children().end();
			it != end; ++it)
	{
		if (!(status = import_bone(*it, joint_obj)))
			break;
	}
	return status;
}

static inline void append_uvs(const std::vector<fvector2>& uvs, MFloatArray& u_values, MFloatArray& v_values)
{
	unsigned offset = u_values.length(), size = unsigned(uvs.size() & UINT_MAX) + offset;
	u_values.setLength(size);
	v_values.setLength(size);
	for (std::vector<fvector2>::const_iterator it = uvs.begin(), end = uvs.end(); it != end; ++it)
	{
		u_values.set(it->x, offset);
		v_values.set(1.f - it->y, offset);
		++offset;
	}
}

MStatus maya_import_tools::import_mesh(const xr_mesh* mesh, const xr_bone_vec& bones, MObject* out_transform)
{
	MStatus status;

	MFnTransform transform_fn;
	MObject transform_obj = transform_fn.create(MObject::kNullObj, &status);
	if (!status)
	{
		msg("xray_re: can't create mesh %s transform", mesh->name().c_str());
		MGlobal::displayError(MString("xray_re: can't create mesh ") +
				mesh->name().c_str() + " transform");
		return status;
	}
	transform_fn.setName(make_maya_name(mesh->name(), "Shape"));
	if (out_transform)
		*out_transform = transform_obj;

	const std::vector<fvector3>& points = mesh->points();
	MPointArray vertices;
	vertices.setLength(unsigned(points.size() & UINT_MAX));
	uint_fast32_t vert_idx = 0;
	for (std::vector<fvector3>::const_iterator it = points.begin(), end = points.end();
			it != end; ++it)
	{
		double x = MDistance(it->x, MDistance::kMeters).asCentimeters();
		double y = MDistance(it->y, MDistance::kMeters).asCentimeters();
		double z = MDistance(it->z, MDistance::kMeters).asCentimeters();
		vertices.set(unsigned(vert_idx++), x, y, -z);
	}

	const xr_vmap_vec& vmaps = mesh->vmaps();

	MFloatArray u_values, v_values;
	unsigned* uv_offsets = new unsigned[vmaps.size()];
	for (xr_vmap_vec_cit it = vmaps.begin(), end = vmaps.end(); it != end; ++it)
	{
		if ((*it)->type() == xr_vmap::VMT_UV)
		{
			const xr_uv_vmap* vmap = static_cast<const xr_uv_vmap*>(*it);
			uv_offsets[it - vmaps.begin()] = u_values.length();
			append_uvs(vmap->uvs(), u_values, v_values);
		}
	}

	const lw_face_vec& faces = mesh->faces();
	MIntArray poly_connects, uv_ids;
	poly_connects.setLength(unsigned(faces.size() & UINT_MAX)*3);
	uv_ids.setLength(poly_connects.length());
	const lw_vmref_vec& vmrefs = mesh->vmrefs();
	vert_idx = 0;
	for (lw_face_vec_cit it = faces.begin(), end = faces.end(); it != end; ++it)
	{
		for (uint_fast32_t i = 3; i != 0;)
		{
			poly_connects.set(it->v[--i], unsigned(vert_idx));
			const lw_vmref& vmref = vmrefs[it->ref[i]];
			for (lw_vmref::const_iterator it1 = vmref.begin(), end1 = vmref.end(); it1 != end1; ++it1)
			{
				if (vmaps[it1->vmap]->type() == xr_vmap::VMT_UV)
					uv_ids.set(it1->offset + uv_offsets[it1->vmap], unsigned(vert_idx));
			}
			++vert_idx;
		}
	}
	delete[] uv_offsets;

	MIntArray poly_counts(unsigned(faces.size() & UINT_MAX), 3);

	MFnMesh mesh_fn;
	MObject mesh_obj = mesh_fn.create(vertices.length(), poly_counts.length(),
			vertices, poly_counts, poly_connects, u_values, v_values,
			transform_obj, &status);
	if (!status)
	{
		msg("xray_re: can't create mesh %s", mesh->name().c_str());
		MGlobal::displayError(MString("xray_re: can't create mesh ") + mesh->name().c_str());
		return status;
	}
	mesh_fn.setName(make_maya_name(mesh->name(), "Shape", "Shape"));

	if (mesh_fn.numPolygons() != faces.size())
	{
		msg("xray_re: mesh polygon count was changed");
		MGlobal::displayError("xray_re: mesh polygon count was changed");
	}

	status = mesh_fn.assignUVs(poly_counts, uv_ids, 0);
	CHECK_MSTATUS(status);

	bool want_normals = (m_smoothing_mode == "soc" || m_smoothing_mode == "cscop") ? false : true;

	if (want_normals && !mesh->cnorm().empty())
	{
		const fvector3_vec& cn = mesh->cnorm();
		unsigned num_corners = unsigned(cn.size() & UINT_MAX);
		MVectorArray normals(num_corners);
		MIntArray face_list(num_corners);
		MIntArray vertex_list(num_corners);
		unsigned corner = 0;
		for (lw_face_vec_cit it = faces.begin(), end = faces.end(); it != end; ++it)
		{
			int face_idx = int(it - faces.begin());
			for (uint_fast32_t i = 0; i != 3; ++i, ++corner)
			{
				const fvector3& n = cn[corner];
				normals.set(MVector(n.x, n.y, -n.z), corner);
				face_list.set(face_idx, corner);
				vertex_list.set(int(it->v[i]), corner);
			}
		}
		status = mesh_fn.setFaceVertexNormals(normals, face_list, vertex_list);
		CHECK_MSTATUS(status);
	}
	else if (!want_normals && !mesh->sgroups().empty())
	{
		MIntArray connected;
		const std::vector<uint32_t>& sgroups = mesh->sgroups();
		if (m_smoothing_mode == "soc")
		{
			if (mesh->flags() & EMF_3DSMAX)
			{
				for (MItMeshEdge it(mesh_obj); !it.isDone(); it.next())
				{
					it.getConnectedFaces(connected, &status);
					it.setSmoothing(status && connected.length() == 2 &&
							(sgroups[connected[0]] & sgroups[connected[1]]) != 0);
				}
			}
			else
			{
				for (MItMeshEdge it(mesh_obj); !it.isDone(); it.next())
				{
					it.getConnectedFaces(connected, &status);
					it.setSmoothing(status && connected.length() == 2 &&
							sgroups[connected[0]] != EMESH_NO_SG &&
							sgroups[connected[0]] == sgroups[connected[1]]);
				}
			}
		}
		else
		{
			for (MItMeshPolygon it(mesh_obj); !it.isDone(); it.next())
			{
				status = it.getEdges(connected);
				if (!status)
					return MS::kFailure;

				mesh_fn.setEdgeSmoothing(connected[0], !(sgroups[it.index()] & 0x2));
				mesh_fn.setEdgeSmoothing(connected[1], !(sgroups[it.index()] & 0x1));
				mesh_fn.setEdgeSmoothing(connected[2], !(sgroups[it.index()] & 0x4));
			}
		}
	}
	else
	{
		smooth_by_angle(mesh_obj);
	}

	MDagPath mesh_path;
	if (!(status = mesh_fn.getPath(mesh_path)))
		return MS::kFailure;

	MDGModifier dg_modifier;
	for (xr_surfmap_vec_cit it = mesh->surfmaps().begin(), end = mesh->surfmaps().end();
			it != end; ++it)
	{
		const xr_surfmap* smap = *it;
		MObject& set_obj = m_sets[smap->surface->name()];
		if (set_obj.isNull())
		{
			msg("xray_re: null shading group object");
			MGlobal::displayError("xray_re: null shading group object");
			return MS::kFailure;
		}

		MIntArray shaded_faces;
		shaded_faces.setLength(unsigned(smap->faces.size() & UINT_MAX));
		std::vector<uint32_t>::const_reverse_iterator it1 = smap->faces.rbegin();
		for (unsigned i = shaded_faces.length(); i != 0; ++it1)
			shaded_faces.set(*it1, --i);

		MFnSingleIndexedComponent component_fn;
		MObject component_obj = component_fn.create(MFn::kMeshPolygonComponent, &status);
		if (!status || !(status = component_fn.addElements(shaded_faces)))
			return status;

		MSelectionList members;
		if (!(status = members.add(mesh_path, component_obj)))
			return status;

		MFnSet set_fn(set_obj);
		if (!(status = set_fn.addMembers(members)))
			return status;
	}

	if (mesh_fn.numVertices() != points.size())
	{
		msg("xray_re: mesh vertex count was changed");
		MGlobal::displayError("xray_re: mesh vertex count was changed");
		return MS::kFailure;
	}

	if (!bones.empty())
	{
		MString command("skinCluster -mi 2 -tsb ");
		for (maya_object_map_it it = m_joints.begin(), end = m_joints.end(); it != end; ++it)
		{
			MFnIkJoint joint_fn(it->second, &status);
			CHECK_MSTATUS(status);
			command += joint_fn.partialPathName();
			command += " ";
		}
		command += mesh_fn.partialPathName();

		MStringArray result;
		status = MGlobal::executeCommand(command, result);
		if (!status)
		{
			msg("xray_re: can't create skin cluster");
			MGlobal::displayError("xray_re: can't create skin cluster");
			return status;
		}

		MSelectionList selection_list;
		status = selection_list.add(result[0]);
		CHECK_MSTATUS(status);
		MObject skin_obj;
		selection_list.getDependNode(0, skin_obj);
		MFnSkinCluster skin_fn(skin_obj, &status);
		if (!status)
			return status;

		MFnSingleIndexedComponent component_fn;
		MObject component_obj = component_fn.create(MFn::kMeshVertComponent, &status);
		if (!status)
			return status;
		status = mesh_fn.setObject(mesh_path);
		if (!status)
			return status;
		MIntArray vertex_indices;
		vertex_indices.setLength(mesh_fn.numVertices());
		for (unsigned i = vertex_indices.length(); i != 0;)
		{
			--i;
			vertex_indices[i] = i;
		}
		status = component_fn.addElements(vertex_indices);
		CHECK_MSTATUS(status);

		unsigned num_bones = unsigned(bones.size() & UINT_MAX), bone_idx = 0;
		MIntArray influence_indices(num_bones);
		MDoubleArray vertex_weights(vertex_indices.length()*num_bones, 0);
		for (xr_bone_vec_cit it = bones.begin(), end = bones.end(); it != end; ++it, ++bone_idx)
		{
			const xr_bone* bone = *it;

			maya_object_map_it joint_it = m_joints.find(bone->name());
			xr_assert(joint_it != m_joints.end());
			MFnIkJoint joint_fn(joint_it->second, &status);
			CHECK_MSTATUS(status);
			MDagPath joint_path;
			status = joint_fn.getPath(joint_path);
			CHECK_MSTATUS(status);
			unsigned influence_idx = skin_fn.indexForInfluenceObject(joint_path, &status);
			CHECK_MSTATUS(status);
			influence_indices[bone_idx] = influence_idx;

			const xr_weight_vmap* vmap = 0;
			for (xr_vmap_vec_cit it1 = vmaps.begin(), end1 = vmaps.end(); it1 != end1; ++it1)
			{
				if ((*it1)->name() == bone->name())
				{
					vmap = static_cast<const xr_weight_vmap*>(*it1);
					break;
				}
			}
			if (vmap)
			{
				xr_assert(vmap->type() == xr_vmap::VMT_WEIGHT);
				std::vector<uint32_t>::const_iterator v = vmap->vertices().begin();
				for (std::vector<float>::const_iterator w = vmap->weights().begin(),
						w_end = vmap->weights().end(); w != w_end; ++v, ++w)
				{
					vertex_weights[*v * num_bones + bone_idx] = *w;
				}
			}
		}
		status = skin_fn.setWeights(mesh_path, component_obj,
				influence_indices, vertex_weights, false);
		CHECK_MSTATUS(status);
	}
	return MS::kSuccess;
}

MObject maya_import_tools::lookup_character(MStatus* return_status)
{
	if (return_status)
		*return_status = MS::kSuccess;
	return MObject::kNullObj;
}

MObject maya_import_tools::create_character(MStatus* return_status)
{
	if (return_status)
		*return_status = MS::kSuccess;
	return MObject::kNullObj;
}

void maya_import_tools::reset_animation_state() const
{
	MTime::setUIUnit(MTime::kNTSCFrame);
	MAnimControl::setMinTime(MTime(0, MTime::kSeconds));
	MAnimControl::setAnimationStartTime(MTime(0, MTime::kSeconds));
}

static MFnAnimCurve::InfinityType maya_infinity(uint8_t behaviour)
{
	switch (behaviour)
	{
		case xr_envelope::BEH_RESET:
			return MFnAnimCurve::kConstant;
		default:
		case xr_envelope::BEH_CONSTANT:
			return MFnAnimCurve::kConstant;
		case xr_envelope::BEH_REPEAT:
			return MFnAnimCurve::kCycle;
		case xr_envelope::BEH_OSCILLATE:
			return MFnAnimCurve::kOscillate;
		case xr_envelope::BEH_OFFSET:
			return MFnAnimCurve::kConstant;
		case xr_envelope::BEH_LINEAR:
			return MFnAnimCurve::kLinear;
	}
}

static MStatus write_motion_keys(const maya_object_map& joints, const xr_skl_motion* motion,
		double scale_factor, double time_stretch, double start_frame, double* end_frame)
{
	if (!std::isfinite(scale_factor) || scale_factor <= 0 ||
		!std::isfinite(time_stretch) || time_stretch <= 0 ||
		!std::isfinite(start_frame) || !std::isfinite(motion->fps()) ||
		motion->fps() <= 0 || motion->frame_end() <= motion->frame_start())
	{
		MGlobal::displayError("IX-Ray: invalid motion timing or import options");
		return MS::kFailure;
	}
	const char* attrs[] = { "tx", "ty", "tz", "rx", "ry", "rz" };
	// Validate only destinations that exist in the selected Maya skeleton.
	// OMFs can contain helper bones that are absent in a lighter target rig.
	unsigned skipped_bones = 0;
	for (const auto* bone : motion->bone_motions())
	{
		auto found = joints.find(bone->name());
		if (found == joints.end())
		{
			++skipped_bones;
			continue;
		}
		MFnDependencyNode node(found->second);
		for (const char* attr : attrs)
		{
			MPlug plug = node.findPlug(attr, true);
			MPlugArray sources;
			plug.connectedTo(sources, true, false);
			if (plug.isLocked() || (sources.length() && !sources[0].node().hasFn(MFn::kAnimCurve)))
			{
				MGlobal::displayError(MString("IX-Ray: locked or driven bone channel: ") + plug.name());
				return MS::kFailure;
			}
		}
	}
	if (skipped_bones)
		MGlobal::displayWarning(MString("IX-Ray: skipped ") + int(skipped_bones)
			+ " OMF bone(s) that are absent in the selected skeleton.");
	const double frame_step = MTime(time_stretch / motion->fps(), MTime::kSeconds).as(MTime::uiUnit());
	const double last = start_frame + (motion->frame_end() - motion->frame_start() - 1) * frame_step;
	if (end_frame) *end_frame = last;
	for (const auto* bone : motion->bone_motions())
	{
		auto found = joints.find(bone->name());
		if (found == joints.end()) continue;
		MString path = MFnDagNode(found->second).fullPathName();
		MString command("cutKey -clear -time \"");
		command += start_frame;
		command += ":";
		command += last;
		command += "\" -at tx -at ty -at tz -at rx -at ry -at rz \"";
		command += path;
		command += "\";";
		for (int32_t frame = motion->frame_start(); frame < motion->frame_end(); ++frame)
		{
			fvector3 offset, rotation;
			bone->evaluate(float(frame / double(motion->fps())), offset, rotation);
			MEulerRotation euler(-rotation.x, -rotation.y, rotation.z, MEulerRotation::kZXY);
			MFnDependencyNode node(found->second);
			euler.reorderIt(static_cast<MEulerRotation::RotationOrder>(node.findPlug("rotateOrder", true).asShort()));
			double values[] = {
				MDistance(offset.x * scale_factor, MDistance::kMeters).as(MDistance::uiUnit()),
				MDistance(offset.y * scale_factor, MDistance::kMeters).as(MDistance::uiUnit()),
				MDistance(-offset.z * scale_factor, MDistance::kMeters).as(MDistance::uiUnit()),
				MAngle(euler.x).as(MAngle::uiUnit()), MAngle(euler.y).as(MAngle::uiUnit()), MAngle(euler.z).as(MAngle::uiUnit()) };
			for (unsigned axis = 0; axis < 6; ++axis)
			{
				command += "setKeyframe -itt linear -ott linear -time ";
				command += start_frame + (frame - motion->frame_start()) * frame_step;
				command += " -value ";
				command += values[axis];
				command += " -at "; command += attrs[axis];
				command += " \""; command += path; command += "\";";
			}
		}
		MStatus status = MGlobal::executeCommand(command, false, true);
		if (!status) return status;
	}
	return MS::kSuccess;
}

MStatus maya_import_tools::import_selected_motion(const xr_skl_motion* motion, double* end_frame)
{
	MSelectionList selected;
	MGlobal::getActiveSelectionList(selected);
	if (selected.isEmpty())
	{
		if (!m_joints.empty())
			return write_motion_keys(m_joints, motion, m_scale_factor, m_time_stretch, m_start_frame, end_frame);
		MGlobal::displayError("IX-Ray: no skeleton selected");
		return MS::kFailure;
	}
	m_joints.clear();
	for (unsigned i = 0; i < selected.length(); ++i)
	{
		MDagPath root;
		if (!selected.getDagPath(i, root)) continue;
		MItDag it;
		if (!it.reset(root, MItDag::kDepthFirst, MFn::kJoint)) continue;
		for (; !it.isDone(); it.next())
		{
			MObject joint = it.currentItem();
			std::string name = MFnDagNode(joint).name().asChar();
			const auto colon = name.rfind(':');
			if (colon != std::string::npos) name.erase(0, colon + 1);
			auto found = m_joints.find(name);
			if (found != m_joints.end() && found->second != joint)
			{
				MGlobal::displayError("IX-Ray: duplicate bone names; select one skeleton");
				return MS::kFailure;
			}
			m_joints[name] = joint;
		}
	}
	return write_motion_keys(m_joints, motion, m_scale_factor, m_time_stretch, m_start_frame, end_frame);
}

void maya_import_tools::set_default_options(void)
{
	m_target_sdk = xray_re::SDK_VER_DEFAULT;
	m_smoothing_mode = "normals";
	m_group_name.clear();
	m_scale_factor = 1.0;
	m_time_stretch = 1.0;
	m_start_frame = 0.0;
}

MStatus maya_import_tools::parse_options(const MString& options)
{
	MStatus status;
	MStringArray params;

	if (!(status = options.split(';', params)))
		return status;

	for (size_t i = 0; i < params.length(); i++)
	{
		MStringArray key_value;
		if (!(status = params[i].split('=', key_value)))
			return status;

		if (key_value.length() < 2)
			continue;

		if (key_value[0] == "sdk_ver")
		{
			xray_re::sdk_version ver = xray_re::sdk_version_from_string(key_value[1].asChar());
			m_target_sdk = (ver == xray_re::SDK_VER_UNKNOWN ? xray_re::SDK_VER_0_6 : ver);
		}
		else if (key_value[0] == "smoothing_mode")
		{
			m_smoothing_mode = key_value[1].asChar();
		}
		else if (key_value[0] == "group_name")
		{
			m_group_name = key_value[1].asChar();
		}
		else if (key_value[0] == "scale_factor")
		{
			m_scale_factor = key_value[1].asDouble();
		}
		else if (key_value[0] == "time_stretch")
		{
			m_time_stretch = key_value[1].asDouble();
		}
		else if (key_value[0] == "start_frame")
		{
			m_start_frame = key_value[1].asDouble();
		}
	}

	return MS::kSuccess;
}
