#define NOMINMAX
#include <algorithm>
#include <cmath>
#include <maya/MTypes.h>
#if MAYA_API_VERSION >= 20180000 && MAYA_API_VERSION <= 20190200
#include <MCppCompat.h>
#endif
#include <maya/MAnimControl.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSet.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MFnCharacter.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnClip.h>
#include <maya/MGlobal.h>
#include <maya/MIntArray.h>
#include <maya/MItDag.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPointArray.h>
#include <maya/MSelectionList.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>
#include <cctype>
#include <map>
#include <sstream>
#include "maya_export_tools.h"
#include "maya_bone_collision.h"
#include "xr_object.h"
#include "xr_skl_motion.h"
#include "xr_envelope.h"
#include "xr_utils.h"
#include "xr_obj_motion.h"
#include "xr_ogf_format.h"
#include "xr_writer.h"
#include "xr_sdk_version.h"

using namespace xray_re;

namespace {

struct omf_quaternion_key { int16_t x, y, z, w; };
struct omf_quaternion_key_32 { float x, y, z, w; };
struct omf_translation_key_8 { int8_t x, y, z; };
struct omf_translation_key_16 { int16_t x, y, z; };
struct omf_translation_key_32 { float x, y, z; };
struct omf_sample {
	fvector3 translation;
	MQuaternion rotation;
};

const char* const k_xray_bone_order_attr = "ixrayBoneOrder";

// Return the original global bone order saved by the X-Ray object importer.
// The attribute is deliberately read from ancestors because only the root
// joint owns it, while a skinCluster reports every influence separately.
static bool get_imported_bone_order(const MDagPathArray& joints,
	const std::vector<std::string>& bone_names, std::vector<unsigned>& bone_order)
{
	MString stored_order;
	bool found = false;
	for (unsigned i = 0; i != joints.length() && !found; ++i) {
		MObject current = joints[i].node();
		while (!current.isNull()) {
			MStatus status;
			MFnDependencyNode node_fn(current, &status);
			if (!status) break;
			MPlug plug = node_fn.findPlug(k_xray_bone_order_attr, true, &status);
			if (status && plug.getValue(stored_order) == MS::kSuccess) {
				found = true;
				break;
			}
			MFnDagNode dag_fn(current, &status);
			if (!status || dag_fn.parentCount() == 0) break;
			current = dag_fn.parent(0);
		}
	}
	if (!found || stored_order.length() == 0) return false;

	std::map<std::string, unsigned> joint_indices;
	for (unsigned i = 0; i != bone_names.size(); ++i)
		if (!joint_indices.insert(std::make_pair(bone_names[i], i)).second) return false;

	bone_order.clear();
	std::istringstream stream(stored_order.asChar());
	std::string name;
	while (std::getline(stream, name)) {
		if (!name.empty()) {
			const std::map<std::string, unsigned>::const_iterator it = joint_indices.find(name);
			if (it == joint_indices.end()) return false;
			bone_order.push_back(it->second);
		}
	}
	return bone_order.size() == joints.length();
}

static int16_t quantize_rotation(double value)
{
	return static_cast<int16_t>(std::max(-32767.0, std::min(32767.0, value * 32767.0)));
}

static bool same_translation(const std::vector<omf_sample>& samples)
{
	for (size_t i = 1; i < samples.size(); ++i) {
		const fvector3& a = samples[0].translation;
		const fvector3& b = samples[i].translation;
		if (std::fabs(a.x - b.x) > 1e-6f || std::fabs(a.y - b.y) > 1e-6f || std::fabs(a.z - b.z) > 1e-6f)
			return false;
	}
	return true;
}

static bool same_rotation(const std::vector<omf_sample>& samples)
{
	for (size_t i = 1; i < samples.size(); ++i) {
		const MQuaternion& a = samples[0].rotation;
		const MQuaternion& b = samples[i].rotation;
		if (std::fabs(a.x - b.x) > 1e-7 || std::fabs(a.y - b.y) > 1e-7 ||
			std::fabs(a.z - b.z) > 1e-7 || std::fabs(a.w - b.w) > 1e-7)
			return false;
	}
	return true;
}

static void write_omf_bone_keys(xr_writer& writer, const std::vector<omf_sample>& samples, unsigned precision)
{
	const bool translation_present = !same_translation(samples);
	const bool rotation_present = !same_rotation(samples);
	uint8_t flags = (translation_present ? KPF_T_PRESENT : 0) |
		(rotation_present ? 0 : KPF_R_ABSENT);
	if (precision == 16) flags |= KPF_T_HQ;
	if (precision == 32) flags |= 0x08; // XrayExportTool's flTKeyFFT_Bit.
	writer.w_u8(flags);

	auto write_rotation = [&writer, precision](const MQuaternion& q) {
		if (precision == 32) {
			writer.w(omf_quaternion_key_32 { float(q.x), float(q.y), float(q.z), float(q.w) });
		} else {
			writer.w(omf_quaternion_key { quantize_rotation(q.x), quantize_rotation(q.y),
				quantize_rotation(q.z), quantize_rotation(q.w) });
		}
	};
	if (rotation_present) {
		writer.w_u32(0); // CRC is advisory; game readers do not validate it.
		for (const omf_sample& sample: samples) write_rotation(sample.rotation);
	} else {
		write_rotation(samples[0].rotation);
	}

	if (!translation_present) {
		writer.w_fvector3(samples[0].translation);
		return;
	}

	writer.w_u32(0);
	if (precision == 32) {
		for (const omf_sample& sample: samples)
			writer.w(omf_translation_key_32 { sample.translation.x, sample.translation.y, sample.translation.z });
		return;
	}

	fvector3 minimum = samples[0].translation;
	fvector3 maximum = minimum;
	for (const omf_sample& sample: samples) {
		minimum.x = std::min(minimum.x, sample.translation.x); minimum.y = std::min(minimum.y, sample.translation.y); minimum.z = std::min(minimum.z, sample.translation.z);
		maximum.x = std::max(maximum.x, sample.translation.x); maximum.y = std::max(maximum.y, sample.translation.y); maximum.z = std::max(maximum.z, sample.translation.z);
	}
	fvector3 center; center.add(minimum, maximum); center.mul(0.5f);
	fvector3 extent; extent.sub(maximum, minimum); extent.mul(0.5f);
	const float scale = precision == 16 ? 32767.f : 127.f;
	auto quantize = [scale](float value, float c, float e) {
		if (std::fabs(e) <= 1e-20f) return 0;
		return std::max(-static_cast<int>(scale), std::min(static_cast<int>(scale), static_cast<int>(value * scale / e)));
	};
	for (const omf_sample& sample: samples) {
		if (precision == 16) writer.w(omf_translation_key_16 {
			static_cast<int16_t>(quantize(sample.translation.x - center.x, center.x, extent.x)),
			static_cast<int16_t>(quantize(sample.translation.y - center.y, center.y, extent.y)),
			static_cast<int16_t>(quantize(sample.translation.z - center.z, center.z, extent.z)) });
		else writer.w(omf_translation_key_8 {
			static_cast<int8_t>(quantize(sample.translation.x - center.x, center.x, extent.x)),
			static_cast<int8_t>(quantize(sample.translation.y - center.y, center.y, extent.y)),
			static_cast<int8_t>(quantize(sample.translation.z - center.z, center.z, extent.z)) });
	}
	extent.mul(1.f / scale);
	writer.w_fvector3(extent);
	writer.w_fvector3(center);
}

} // namespace

std::string getRealName(MFnDependencyNode& n)
{
	std::string name = n.absoluteName().asChar();
	const auto delimPos = name.find_last_of(':');
	if (delimPos == std::string::npos)
		return name;

	return name.substr(delimPos + 1, std::string::npos);
}

static std::string texture_path_from_file(const MString& file_path)
{
	std::string path = file_path.asChar();
	std::replace(path.begin(), path.end(), '/', '\\');
	const size_t slash = path.find_last_of('\\');
	const size_t dot = path.find_last_of('.');
	if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) path.resize(dot);

	std::string lowercase = path;
	std::transform(lowercase.begin(), lowercase.end(), lowercase.begin(), [](unsigned char c) {
		return char(std::tolower(c));
	});
	const std::string marker = "\\textures\\";
	const size_t root = lowercase.rfind(marker);
	if (root != std::string::npos) return path.substr(root + marker.size());
	const std::string relative_root = "textures\\";
	if (lowercase.rfind(relative_root, 0) == 0) return path.substr(relative_root.size());
	return slash == std::string::npos ? path : path.substr(slash + 1);
}

static std::string normalize_omf_refs(std::string refs)
{
	std::string result;
	result.reserve(refs.size());
	bool slash = false;
	for (char c: refs) {
		if (c == '/' || c == '\\') {
			if (!slash) result += '\\';
			slash = true;
		} else {
			result += c;
			slash = false;
		}
	}
	return result;
}

maya_export_tools::maya_export_tools(const MString& options)
{
	set_default_options();
	m_options_status = parse_options(options);
}

static MStatus extract_bones(MFnSkinCluster& skin_fn, xr_bone_vec& bones)
{
	MStatus status;

	MDagPathArray joints;
	skin_fn.influenceObjects(joints, &status);
	unsigned num_joints = joints.length();
	if (num_joints == 0)
	{
		msg("xray_re: can't find any influence object");
		MGlobal::displayError("xray_re: can't find any influence object");
		return MS::kInvalidParameter;
	}
	else if (num_joints > MAX_BONES)
	{
		msg("xray_re: Warning! The number of bones is greater than the default maximum value (%u of %u possible)", num_joints, MAX_BONES);
		MGlobal::displayError(MString("xray_re: Warning! The number of bones is greater than the default maximum value") +
			"(" + num_joints + " of " + MAX_BONES + " possible)");
	}

	bones.resize(num_joints);

	MString command("dagPose -r -g -bp ");
	for (unsigned i = num_joints; i != 0;)
	{
		MFnIkJoint joint_fn(joints[--i], &status);
		if (!status)
		{
			msg("xray_re: can't handle non-joint node %s",
				joints[i].partialPathName().asChar());
			MGlobal::displayError(MString("xray_re: can't handle non-joint node ") +
				joints[i].partialPathName().asChar());
			return status;
		}
		command += joint_fn.partialPathName();
		command += " ";

		xr_bone* bone = new xr_bone;
		bones[i] = bone;
		auto name = getRealName(joint_fn);
		bone->vmap_name() = bone->name() = name;
		if (!(status = export_bone_collision(joint_fn.object(), *bone)))
			return status;

		unsigned num_parents = joint_fn.parentCount();
		if (num_parents > 1)
		{
			msg("xray_re: can't handle multi-parented joint %s", name.c_str());
			MGlobal::displayError(MString("xray_re: can't handle multi-parented joint ") + name.c_str());
			return MS::kInvalidParameter;
		}
		else if (num_parents == 1)
		{
			MObject parent_obj = joint_fn.parent(0);
			if (parent_obj.hasFn(MFn::kJoint) &&
					joint_fn.setObject(parent_obj))
			{
				bone->parent_name() = getRealName(joint_fn);
			}
		}
	}

	xr_bone* root = 0;
	for (xr_bone_vec_it it = bones.begin(), end = bones.end(); it != end; ++it)
	{
		xr_bone* bone = *it;
		xr_assert(bone);
		if (bone->parent_name().empty())
		{
			if (root == 0)
			{
				root = bone;
				continue;
			}
			else
			{
				msg("xray_re: can't handle multiple root joints in skeleton");
				MGlobal::displayError("xray_re: can't handle multiple root joints in skeleton");
				return MS::kInvalidParameter;
			}
		}
		xr_bone* parent = find_by_name(bones, bone->parent_name());
		if (parent == 0)
		{
			msg("xray_re: can't find parent bone %s", bone->parent_name().c_str());
			MGlobal::displayError(MString("xray_re: can't find parent bone ") +
				bone->parent_name().c_str());
			return MS::kFailure;
		}
		parent->children().push_back(bone);
	}
	if (root == 0)
	{
		msg("xray_re: can't find root joint");
		MGlobal::displayError("xray_re: can't find root joint");
		return MS::kInvalidParameter;
	}

	if (!(status = MGlobal::executeCommand(command)))
	{
		msg("xray_re: can't set skeleton to bind pose");
		MGlobal::displayError("xray_re: can't set skeleton to bind pose");
		return MS::kFailure;
	}

	for (unsigned i = num_joints; i != 0;)
	{
		MFnIkJoint joint_fn(joints[--i]);
		xr_bone* bone = bones[i];

		MTransformationMatrix mat = joint_fn.transformationMatrix(&status);
		CHECK_MSTATUS(status);

		MVector t = mat.getTranslation(MSpace::kTransform, &status);
		CHECK_MSTATUS(status);
		bone->bind_offset().set(float(MDistance(t.x, MDistance::kCentimeters).asMeters()),
								float(MDistance(t.y, MDistance::kCentimeters).asMeters()),
								float(MDistance(-t.z, MDistance::kCentimeters).asMeters()));

		MEulerRotation r = mat.eulerRotation();
		r.reorderIt(MEulerRotation::kZXY);
		bone->bind_rotate().set(float(-r.x), float(-r.y), float(r.z));
	}

	return status;
}

static MStatus extract_points(MFnMesh& mesh_fn, std::vector<fvector3>& points, fbox& aabb)
{
	MStatus status;

	int num_points = mesh_fn.numVertices();
	if (num_points < 4)
	{
		msg("xray_re: can't export mesh %s with less than four vertices",
			mesh_fn.name().asChar());
		MGlobal::displayError(MString("xray_re: can't export mesh ") +
			mesh_fn.name().asChar() + " with less than four vertices");
		return MS::kInvalidParameter;
	}
	points.reserve(size_t(num_points & INT_MAX));
	aabb.invalidate();
	MPoint p0;
	fvector3 p;
	for (int i = 0; i != num_points; ++i)
	{
		mesh_fn.getPoint(i, p0);
		p0.cartesianize();
		aabb.extend(p.set(float(MDistance(p0.x, MDistance::kCentimeters).asMeters()),
						  float(MDistance(p0.y, MDistance::kCentimeters).asMeters()),
						  float(MDistance(-p0.z, MDistance::kCentimeters).asMeters())));
		points.push_back(p);
	}
	return status;
}

static MStatus extract_faces(MFnMesh& mesh_fn, lw_face_vec& faces, fvector3_vec* normals)
{
	MStatus status;

	MItMeshPolygon it(mesh_fn.object());
	if (it.isLamina())
	{
		msg("xray_re: lamina faces found in mesh %s", mesh_fn.name().asChar());
		MGlobal::displayWarning(MString("xray_re: lamina faces found in mesh ") +
			mesh_fn.name().asChar());
	}
	int num_polys = mesh_fn.numPolygons(&status);
	if (num_polys < 2)
	{
		msg("xray_re: can't export mesh %s with less than two faces",
			mesh_fn.name().asChar());
		MGlobal::displayError(MString("xray_re: can't export mesh ") +
			mesh_fn.name().asChar() + " with less than two faces");
		return MS::kInvalidParameter;
	}
	faces.reserve(size_t(num_polys & INT_MAX));
	MIntArray verts;
	for (int i = 0; i != num_polys; ++i)
	{
		status = mesh_fn.getPolygonVertices(i, verts);
		if (!status || verts.length() != 3)
		{
			msg("xray_re: can't handle polygons with 4 or more sides for mesh %s",
				mesh_fn.name().asChar());
			MGlobal::displayError(MString("xray_re: can't handle polygons with 4 or more sides for mesh ") +
				mesh_fn.name().asChar());
			return MS::kInvalidParameter;
		}
		lw_face face(verts[2], verts[1], verts[0]);
		faces.push_back(face);

		if (normals)
		{
			MFloatVectorArray NormalArray;
			mesh_fn.getFaceVertexNormals(i, NormalArray);
			if (NormalArray.length() != 3)
			{
				msg("can't export mesh %s with face with more than 3 normals",
					mesh_fn.name().asChar());
				return MS::kInvalidParameter;
			}

			for (int a = 0; a < 3; a++)
			{
				auto& Normal = NormalArray[2 - a];

				normals->push_back({ (float)Normal.x, (float)Normal.y, -(float)Normal.z, });
			}
		}


	}
	return status;
}

static MStatus extract_uvs(MFnMesh& mesh_fn, lw_face_vec& faces,
		lw_vmref_vec& vmrefs, xr_vmap_vec& vmaps)
{
	MStatus status;

	xr_uv_vmap* uv_vmap = 0;
	xr_face_uv_vmap* face_uv_vmap = 0;

	auto MayaObject = mesh_fn.object();

	for (MItMeshVertex it(MayaObject); !it.isDone(); it.next())
	{
		uint32_t vert_idx = uint32_t(it.index() & INT_MAX);

		fvector2 uv0;
		if (!it.getUV(uv0.xy))
		{
			msg("xray_re: can't extract shared UVs for vert %" PRIu32 " on mesh %s",
				vert_idx, mesh_fn.name().asChar());
			MGlobal::displayError(MString("xray_re: can't extract shared UVs for vert ") +
				vert_idx + " on mesh " + mesh_fn.name().asChar());
			return MS::kInvalidParameter;
		}
		uv0.v = 1.f - uv0.v;

		if (uv_vmap == 0)
		{
			uv_vmap = new xr_uv_vmap("Texture");
			uv_vmap->reserve(size_t(mesh_fn.numVertices() & INT_MAX));
			vmaps.push_back(uv_vmap);
		}
		lw_vmref vmref0;
		vmref0.push_back(lw_vmref_entry(0, uv_vmap->add_uv(uv0, vert_idx)));

		uint32_t vmref0_idx = uint32_t(vmrefs.size() & UINT32_MAX);
		vmrefs.push_back(vmref0);

		MIntArray adjacents;
		it.getConnectedFaces(adjacents);
		for (unsigned i = adjacents.length(); i != 0;)
		{
			uint32_t face_idx = uint32_t(adjacents[--i] & INT_MAX), vmref_idx;

			fvector2 uv;
			if (!it.getUV(face_idx, uv.xy))
			{
				msg("xray_re: can't extract UVs for vert %" PRIu32 " face %" PRIu32, vert_idx, face_idx);
				MGlobal::displayWarning(MString("xray_re: can't extract UVs for vert ") +
					vert_idx + " face " + face_idx);
				uv = uv0;
			}
			uv.v = 1.f - uv.v;

			lw_face& face = faces[face_idx];
			if (uv == uv0)
			{
				vmref_idx = vmref0_idx;
			}
			else
			{
				if (face_uv_vmap == 0)
				{
					face_uv_vmap = new xr_face_uv_vmap("Texture");
					vmaps.push_back(face_uv_vmap);
				}
				lw_vmref vmref;
				vmref.push_back(lw_vmref_entry(1, face_uv_vmap->add_uv(uv, vert_idx, face_idx)));
				vmref_idx = uint32_t(vmrefs.size() & UINT32_MAX);
				vmrefs.push_back(vmref);
			}
			for (uint_fast32_t j = 3; j != 0;)
			{
				if (face.v[--j] == vert_idx)
				{
					face.ref[j] = vmref_idx;
					vmref_idx = UINT32_MAX;
					break;
				}
			}
			xr_assert(vmref_idx == UINT32_MAX);
		}
	}
	return status;
}

static MStatus extract_weights(MFnMesh& mesh_fn, MFnSkinCluster& skin_fn,
		lw_face_vec& faces, lw_vmref_vec& vmrefs, xr_vmap_vec& vmaps, unsigned influence_limit)
{
	MStatus status;

	MDagPathArray joints;
	skin_fn.influenceObjects(joints, &status);
	CHECK_MSTATUS(status);

	std::vector<std::vector<lw_vmref_entry>> weight_vmrefs(size_t(mesh_fn.numVertices() & INT_MAX));
	for (unsigned joint_idx = joints.length(); joint_idx != 0;)
	{
		MDoubleArray weights;
		MSelectionList affected;
		status = skin_fn.getPointsAffectedByInfluence(joints[--joint_idx], affected, weights);
		CHECK_MSTATUS(status);
		if (affected.isEmpty())
			continue;

		MFnIkJoint joint_fn(joints[joint_idx], &status);
		CHECK_MSTATUS(status);

		msg("xray_re: joint=%s", getRealName(joint_fn).c_str());
		MGlobal::displayInfo(MString("xray_re: joint=") + joint_fn.name().asChar());

		xr_weight_vmap* weight_vmap = new xr_weight_vmap(getRealName(joint_fn));
		weight_vmap->reserve(weights.length());
		uint32_t vmap_idx = uint32_t(vmaps.size() & UINT32_MAX);
		vmaps.push_back(weight_vmap);

		msg("         num_affected=%u, num_weights=%u", affected.length(), weights.length());
		MGlobal::displayInfo(MString("         num_affected=") + affected.length() +
			" ," + " num_weights=" + weights.length());

		for (unsigned i = affected.length(), k = weights.length(); i != 0;)
		{
			MDagPath dag_path;
			MObject component_obj;
			status = affected.getDagPath(--i, dag_path, component_obj);
			CHECK_MSTATUS(status);
			MFnSingleIndexedComponent component_fn(component_obj, &status);
			CHECK_MSTATUS(status);
			for (int j = component_fn.elementCount() - 1; j >= 0; --j)
			{
				int vert_idx = component_fn.element(j, &status);
				CHECK_MSTATUS(status);
				float weight = float(weights[--k]);
				if (!std::isfinite(weight) || weight < 0) {
					MGlobal::displayError("IX-Ray: invalid skin weight.");
					return MS::kFailure;
				}
				if (weight == 0) continue;
				uint32_t weight_idx = weight_vmap->add_weight(weight, uint32_t(vert_idx & INT_MAX));
				weight_vmrefs[vert_idx].push_back(lw_vmref_entry(vmap_idx, weight_idx));
			}
		}
	}

	if (influence_limit) {
		for (auto& refs: weight_vmrefs) {
			std::stable_sort(refs.begin(), refs.end(), [&vmaps](const lw_vmref_entry& a, const lw_vmref_entry& b) {
				return static_cast<xr_weight_vmap*>(vmaps[a.vmap])->weights()[a.offset] >
					static_cast<xr_weight_vmap*>(vmaps[b.vmap])->weights()[b.offset];
			});
			if (refs.size() > influence_limit) refs.resize(influence_limit);
		}
	}
	for (lw_face_vec_it it = faces.begin(), end = faces.end(); it != end; ++it)
	{
		for (uint_fast32_t i = 3; i != 0;)
		{
			lw_vmref& vmref = vmrefs[it->ref[--i]];
			if (vmref.size() > 1)
				continue;
			for (const auto& ref: weight_vmrefs[it->v[i]]) {
				if (vmref.full()) {
					MGlobal::displayError("IX-Ray: too many bone influences for .object (maximum 5 plus UV).");
					return MS::kFailure;
				}
				vmref.push_back(ref);
			}
		}
	}

	return status;
}

static void get_xraymtl_attr(MFnDependencyNode& dep_fn, const char* name, std::string& value)
{
	MStatus status;
	MPlug plug = dep_fn.findPlug(name, &status);
	if (status)
	{
		MFnEnumAttribute attr_fn(plug.attribute(&status), &status);
		if (status)
		{
			MString temp = attr_fn.fieldName(plug.asShort(), &status);
			if (status)
			{
				value = temp.asChar();
				for (std::string::size_type i = 0; (i = value.find('/', i)) != std::string::npos; ++i)
					value[i] = '\\';
				return;
			}
		}
	}
	msg("xray_re: can't get attribute %s", name);
	MGlobal::displayWarning(MString("xray_re: can't get attribute ") + name);
}

xr_surface* maya_export_tools::create_surface(const char* surf_name, MFnSet& set_fn)
{
	xr_surface* surface = new xr_surface(m_skeletal);
	surface->name() = surf_name;

	MStatus status;
	MPlugArray connected_plugs;
	set_fn.findPlug("ss").connectedTo(connected_plugs, true, false, &status);
	MObject shader_obj;
	const char* base_color_attr = "color";

	for (unsigned i = connected_plugs.length(); i != 0;)
	{
		MObject obj = connected_plugs[--i].node();
		MFnDependencyNode dep_fn(obj);
		if (dep_fn.typeName() == "XRayMtl")
		{
			shader_obj = obj;
			get_xraymtl_attr(dep_fn, "xrayGameMaterial", surface->gamemtl());
			get_xraymtl_attr(dep_fn, "xrayEngineShader", surface->eshader());
			get_xraymtl_attr(dep_fn, "xrayCompilerShader", surface->cshader());
			MPlug plug = dep_fn.findPlug("xrayDoubleSide", &status);
			if (status && plug.asBool())
				surface->set_two_sided();

			if (m_skeletal)
			{
				if (surface->eshader() == "default")
				{
					surface->eshader() = "models\\model";

					msg("xray_re: surface '%s' uses 'default' shader, reset to 'models\\model'", surf_name);
					MGlobal::displayWarning(MString("xray_re: surface '") + surf_name + "' uses 'default' shader, reset to 'models\\model'");
				}

				if (surface->gamemtl() == "default")
				{
					surface->gamemtl() = "default_object";

					msg("xray_re: surface '%s' uses 'default' game-material, reset to 'default_object'", surf_name);
					MGlobal::displayWarning(MString("xray_re: surface '") + surf_name + "' uses 'default' game-material, reset to 'default_object'");
				}
			}


			break;
		}
		else if (obj.hasFn(MFn::kLambert) || obj.hasFn(MFn::kPhong) || obj.hasFn(MFn::kBlinn))
		{
			if (shader_obj.isNull())
			{
				shader_obj = obj;
				base_color_attr = "color";
			}
		}
		else if (obj.hasFn(MFn::kStandardSurface))
		{
			if (shader_obj.isNull())
			{
				shader_obj = obj;
				base_color_attr = "baseColor";
			}
		}

	}
	if (shader_obj.isNull())
	{
		msg("xray_re: can't find shader node for surface %s", surf_name);
		MGlobal::displayError(MString("xray_re: can't find shader node for surface ") + surf_name);
		return surface;
	}

	MFnDependencyNode shader_fn(shader_obj);
	shader_fn.findPlug(base_color_attr).connectedTo(connected_plugs, true, false);
	if (connected_plugs.length() == 0)
	{
		msg("xray_re: can't find texture node connected to the color attribute on %s", surf_name);
		MGlobal::displayWarning(MString("xray_re: can't find texture node connected to the color attribute on ") + surf_name);
		MString command("hyperShade -objects ");
		command += surf_name;
		MGlobal::executeCommand(command, true, false);
	}

	for (unsigned i = connected_plugs.length(); i != 0;)
	{
		MFnDependencyNode dep_fn(connected_plugs[--i].node());
		if (dep_fn.typeName() == "file")
		{
			MString file_path = dep_fn.findPlug("ftn").asString();
			if (file_path.numChars() != 0)
			{
				surface->texture() = texture_path_from_file(file_path);
			}
			else
			{
				msg("xray_re: texture node connected to the color attribute on the %s, but path to the texture isn't specified", surf_name);
				MGlobal::displayWarning(MString("xray_re: texture node connected to the color attribute ")
					+ surf_name + (", but path to the texture isn't specified"));
			}
			break;
		}
	}
	return surface;
}

MStatus maya_export_tools::extract_surfaces(MFnMesh& mesh_fn, xr_surfmap_vec& surfmaps)
{
	MObjectArray shading_groups;
	MIntArray faces;
	MStatus status = mesh_fn.getConnectedShaders(0, shading_groups, faces);
	if (!status || shading_groups.length() == 0)
	{
		msg("xray_re: can't get connected shaders for mesh %s", mesh_fn.name().asChar());
		MGlobal::displayError(MString("xray_re: can't get connected shaders for mesh ") +
			mesh_fn.fullPathName().asChar());
		return MS::kInvalidParameter;
	}
	surfmaps.resize(shading_groups.length());
	for (unsigned i = 0; i < faces.length(); ++i) {
		if (faces[i] < 0 || unsigned(faces[i]) >= shading_groups.length()) {
			MGlobal::displayError(MString("IX-Ray: a face has no material: ") + mesh_fn.name());
			return MS::kInvalidParameter;
		}
	}
	for (unsigned i = faces.length(); i != 0;)
	{
		xr_surfmap* smap = surfmaps[faces[--i]];
		if (smap == 0)
		{
			MFnSet set_fn(shading_groups[faces[i]], &status);
			CHECK_MSTATUS(status);
			auto surf_name = getRealName(set_fn);
			xr_surface*& surface = m_shared_surfaces[surf_name];
			if (surface == 0)
			{
				smap = new xr_surfmap(create_surface(surf_name.c_str(), set_fn));
				surface = smap->surface;
			}
			else
			{
				smap = new xr_surfmap(surface);
			}
			surfmaps[faces[i]] = smap;
		}
		smap->faces.push_back(i);
	}
	return status;
}

struct temp_edge
{
	temp_edge();
	uint32_t	faces[2];
};

inline temp_edge::temp_edge() { faces[0] = UINT32_MAX; faces[1] = UINT32_MAX; }

struct temp_face
{
	uint32_t	edges[3];
};

static MStatus extract_smoothing_groups_soc(MFnMesh& mesh_fn, std::vector<uint32_t>& sgroups)
{
	MStatus status = MS::kSuccess;

	MIntArray connected;

	temp_edge* temp_edges = new temp_edge[unsigned(mesh_fn.numEdges() & INT_MAX)];
	auto MayaObject = mesh_fn.object();

	for (MItMeshEdge it(MayaObject); !it.isDone(); it.next())
	{
		if (it.isSmooth())
		{
			it.getConnectedFaces(connected, &status);
			CHECK_MSTATUS(status);
			unsigned n = connected.length();
			if (n <= 2)
			{
				temp_edge& edge = temp_edges[it.index()];
				while (n)
				{
					--n;
					edge.faces[n] = uint32_t(connected[n] & INT_MAX);
				}
			}
		}
	}

	unsigned num_faces = unsigned(mesh_fn.numPolygons() & INT_MAX);
	temp_face* temp_faces = new temp_face[num_faces];
	for (MItMeshPolygon it(MayaObject); !it.isDone(); it.next())
	{
		status = it.getEdges(connected);
		CHECK_MSTATUS(status);
		unsigned n = connected.length();
		if (n != 3)
		{
			msg("xray_re: can't build smoothing groups");
			MGlobal::displayError(MString("xray_re: can't build smoothing groups"));
			delete[] temp_edges;
			delete[] temp_faces;
			return MS::kInvalidParameter;
		}
		temp_face& face = temp_faces[it.index(&status)];
		CHECK_MSTATUS(status);
		while (n)
		{
			--n;
			face.edges[n] = uint32_t(connected[n] & INT_MAX);
		}
	}

	sgroups.assign(num_faces, EMESH_NO_SG);
	std::vector<uint32_t> adjacents;
	adjacents.reserve(512);
	uint32_t sgroup = 0;
	for (uint_fast32_t base_idx = num_faces; base_idx != 0;)
	{
		if (sgroups[--base_idx] != EMESH_NO_SG)
			continue;
		bool new_sgroup = false;
		for (uint_fast32_t face_idx = base_idx;;)
		{
			const temp_face& face = temp_faces[face_idx];
			for (uint_fast32_t i = 3; i != 0;)
			{
				temp_edge& edge = temp_edges[face.edges[--i]];
				if (edge.faces[0] == UINT32_MAX || edge.faces[1] == UINT32_MAX)
					continue;
				if (!new_sgroup)
				{
					new_sgroup = true;
					sgroups[face_idx] = sgroup;
				}
				uint32_t adj_face_idx = edge.faces[0] == face_idx ?
						edge.faces[1] : edge.faces[0];
				if (sgroups[adj_face_idx] == EMESH_NO_SG)
				{
					adjacents.push_back(adj_face_idx);
					sgroups[adj_face_idx] = sgroup;
				}
			}
			if (adjacents.empty())
				break;
			face_idx = adjacents.back();
			adjacents.pop_back();
		}
		if (new_sgroup)
			++sgroup;
	}

	delete[] temp_edges;
	delete[] temp_faces;

	return status;
}

static MStatus extract_smoothing_groups_cs(MFnMesh& mesh_fn, std::vector<uint32_t>& sgroups)
{
	MStatus status = MS::kSuccess;
	MIntArray connected;

	std::vector<bool> smooth_edge(mesh_fn.numEdges());
	auto MayaObject = mesh_fn.object();
	for (MItMeshEdge it(MayaObject); !it.isDone(); it.next())
	{
		smooth_edge[it.index()] = it.isSmooth();
	}

	int num_faces = mesh_fn.numPolygons();
	sgroups.assign(num_faces, 0);

	for (MItMeshPolygon it(MayaObject); !it.isDone(); it.next())
	{
		status = it.getEdges(connected);
		CHECK_MSTATUS(status);
		unsigned n = connected.length();
		if (n != 3)
		{
			msg("xray_re: can't build smoothing groups");
			MGlobal::displayError(MString("xray_re: can't build smoothing groups"));
			return MS::kInvalidParameter;
		}
		uint32_t smooth = 0;
		for (int i = 0; i < 3; i++)
		{
			int edge_index = connected[i];
			if (smooth_edge[edge_index])
				continue;
			int shift = (4 - i) % 3;
			smooth |= 1 << shift;
		}
		sgroups[it.index()] = smooth;
	}

	return status;
}

void maya_export_tools::commit_surfaces(xr_surface_vec& surfaces)
{
	surfaces.reserve(m_shared_surfaces.size());
	for (xr_surface_map_it it = m_shared_surfaces.begin(),
			end = m_shared_surfaces.end(); it != end; ++it)
	{
		surfaces.push_back(it->second);
	}
}

xr_object* maya_export_tools::create_object(MObjectArray& mesh_objs)
{
	MStatus status;

	xr_object* object = new xr_object;
	object->flags() = EOF_STATIC;
	object->meshes().reserve(mesh_objs.length());

	for (unsigned i = mesh_objs.length(); i != 0;)
	{
		MFnMesh mesh_fn(mesh_objs[--i]);

		xr_mesh* mesh = new xr_mesh;
		object->meshes().push_back(mesh);
		auto pVNormals = m_vnormals ? &mesh->vnorm() : nullptr;

		mesh->name() = getRealName(mesh_fn);

		if (!(status = extract_points(mesh_fn, mesh->points(), mesh->bbox())))
			goto fail;

		if (!(status = extract_faces(mesh_fn, mesh->faces(), pVNormals)))
			goto fail;

		if (!(status = extract_uvs(mesh_fn, mesh->faces(), mesh->vmrefs(), mesh->vmaps())))
			goto fail;

		if (!(status = extract_surfaces(mesh_fn, mesh->surfmaps())))
			goto fail;

		if (m_target_sdk <= xray_re::SDK_VER_0_4)
		{
			if (!(status = extract_smoothing_groups_soc(mesh_fn, mesh->sgroups())))
				goto fail;
		}
		else
		{
			if (!(status = extract_smoothing_groups_cs(mesh_fn, mesh->sgroups())))
				goto fail;
		}

	}

	commit_surfaces(object->surfaces());

	return object;

fail:
	delete object;
	return 0;
}

xr_object* maya_export_tools::create_skl_object(MObject& mesh_obj, MObject& skin_obj, unsigned influence_limit)
{
	MStatus status;

	xr_object* object = new xr_object;
	object->flags() = EOF_DYNAMIC;

	MFnMesh mesh_fn(mesh_obj);

	xr_mesh* mesh = new xr_mesh;
	auto pVNormals = m_vnormals ? &mesh->vnorm() : nullptr;
	object->meshes().push_back(mesh);
	mesh->name() = getRealName(mesh_fn);

	MFnSkinCluster skin_fn(skin_obj);
	if (!(status = extract_bones(skin_fn, object->bones())))
		goto fail;

	if (!(status = extract_points(mesh_fn, mesh->points(), mesh->bbox())))
		goto fail;

	if (!(status = extract_faces(mesh_fn, mesh->faces(), pVNormals)))
		goto fail;

	if (!(status = extract_uvs(mesh_fn, mesh->faces(), mesh->vmrefs(), mesh->vmaps())))
		goto fail;

	if (!(status = extract_weights(mesh_fn, skin_fn, mesh->faces(), mesh->vmrefs(), mesh->vmaps(), influence_limit)))
		goto fail;

	if (!(status = extract_surfaces(mesh_fn, mesh->surfmaps())))
		goto fail;

	if (m_target_sdk <= xray_re::SDK_VER_0_4)
	{
		if (!(status = extract_smoothing_groups_soc(mesh_fn, mesh->sgroups())))
			goto fail;
	}
	else
	{
		if (!(status = extract_smoothing_groups_cs(mesh_fn, mesh->sgroups())))
			goto fail;
	}

	object->partitions().push_back(new xr_partition(object->bones()));

	commit_surfaces(object->surfaces());

	return object;

fail:
	delete object;
	return 0;
}

static void collect_meshes(MObjectArray& mesh_objs, MObject& root_obj = MObject::kNullObj)
{
	MItDag dag_it;
	if (!root_obj.isNull() && !dag_it.reset(root_obj))
		return;

	MStatus status;
	for (MDagPath dag_path; !dag_it.isDone(); dag_it.next())
	{
		status = dag_it.getPath(dag_path);
		if (!status)
			continue;
		MFnDagNode dag_fn(dag_path);
		if (is_bone_collision_helper(dag_fn.object()))
		{
			dag_it.prune();
			continue;
		}
		if (dag_fn.isIntermediateObject())
			continue;
		if (!dag_path.hasFn(MFn::kTransform))
			continue;
		for (unsigned i = dag_fn.childCount(); i != 0;)
		{
			MObject obj = dag_fn.child(--i);
			if (obj.hasFn(MFn::kMesh))
			{
				MFnMesh mesh_fn(obj);
				if (!mesh_fn.isIntermediateObject())
					mesh_objs.append(obj);
			}
		}
	}
}

static void collect_meshes(MObjectArray& mesh_objs, bool selection_only)
{
	if (selection_only)
	{
		MSelectionList selection;
		MStatus status = MGlobal::getActiveSelectionList(selection);
		if (!status || selection.isEmpty())
			return;
		MDagPath dag_path;
		for (MItSelectionList it(selection); !it.isDone(); it.next())
		{
			if (!it.getDagPath(dag_path))
				continue;
			MObject root_obj = dag_path.node(&status);
			if (status)
				collect_meshes(mesh_objs, root_obj);
		}
	}
	else
	{
		collect_meshes(mesh_objs);
	}
}

MStatus maya_export_tools::export_object(const char* path, bool selection_only)
{
	MObjectArray mesh_objs;
	collect_meshes(mesh_objs, selection_only);
	if (mesh_objs.length() == 0)
	{
		msg("xray_re: can't find any mesh to export");
		MGlobal::displayError("xray_re: can't find any mesh to export");
		return MS::kFailure;
	}

	m_skeletal = false;

	MStatus status = MS::kFailure;
	if (xr_object* object = create_object(mesh_objs))
	{
		if (object->save_object(path, m_compressed ? compress_options::compress : compress_options::none))
			status = MS::kSuccess;
		delete object;
	}

	return status;
}

static MStatus find_mesh_and_skin(MObject* mesh_obj, MObject* skin_obj, bool selection_only)
{
	MObjectArray mesh_objs;
	collect_meshes(mesh_objs, selection_only);
	switch (mesh_objs.length())
	{
	case 0:
		msg("xray_re: can't find any mesh to export");
		MGlobal::displayError("xray_re: can't find any mesh to export");
		return MS::kFailure;

	case 1:
		break;

	default:
		msg("xray_re: can't handle multiple meshes in skeletal object");
		MGlobal::displayError("xray_re: can't handle multiple meshes in skeletal object");
		return MS::kFailure;
	}

	MObjectArray skin_objs;
	for (MItDependencyNodes dep_it(MFn::kSkinClusterFilter); !dep_it.isDone(); dep_it.next())
	{
		MObject skin_obj = dep_it.thisNode();
		MFnSkinCluster skin_fn(skin_obj);
		MObjectArray affected;
		skin_fn.getOutputGeometry(affected);
		msg("xray_re: skin cluster %s", skin_fn.name().asChar());
		MGlobal::displayInfo(MString("xray_re: skin cluster ") + skin_fn.name().asChar());
		for (unsigned i = affected.length(); i != 0;)
		{
			if (affected[--i] == mesh_objs[0])
				skin_objs.append(skin_obj);
		}
	}
	switch (skin_objs.length())
	{
	case 0:
		msg("xray_re: can't find skin cluster for mesh");
		MGlobal::displayError("xray_re: can't find skin cluster for mesh");
		return MS::kFailure;

	case 1:
		break;

	default:
		msg("xray_re: can't handle multiple skin clusters in skeletal object");
		MGlobal::displayError("xray_re: can't handle multiple skin clusters in skeletal object");
		return MS::kFailure;
	}

	if (mesh_obj)
		*mesh_obj = mesh_objs[0];
	if (skin_obj)
		*skin_obj = skin_objs[0];
	return MS::kSuccess;
}

MStatus maya_export_tools::export_skl_object(const char* path, bool selection_only)
{
	MObject mesh_obj, skin_obj;
	MStatus status = find_mesh_and_skin(&mesh_obj, &skin_obj, selection_only);
	if (!status)
		return status;

	m_skeletal = true;

	status = MS::kFailure;
	if (xr_object* object = create_skl_object(mesh_obj, skin_obj))
	{
		if (object->save_object(path, m_compressed ? compress_options::compress : compress_options::none))
			status = MS::kSuccess;
		delete object;
	}
	return status;
}

MStatus maya_export_tools::export_ogf(const char* path, bool selection_only)
{
	if (!m_options_status) {
		MGlobal::displayError("IX-Ray: invalid OGF options.");
		return m_options_status;
	}
	MObject mesh_obj, skin_obj;
	MStatus status = find_mesh_and_skin(&mesh_obj, &skin_obj, selection_only);
	if (!status) return status;

	// The editable-object extractor works in mesh-local bind space.
	MDagPath mesh_path;
	MDagPath::getAPathTo(mesh_obj, mesh_path);
	if (!mesh_path.inclusiveMatrix().isEquivalent(MMatrix::identity, 1e-6)) {
		MGlobal::displayError("IX-Ray: freeze mesh transforms before OGF export (including parent groups).");
		return MS::kFailure;
	}
	struct restore_pose {
		std::vector<std::pair<MObject, MTransformationMatrix>> joints;
		~restore_pose() { for (const auto& joint: joints) { MFnTransform fn(joint.first); fn.set(joint.second); } }
	} pose;
	for (MItDag it(MItDag::kDepthFirst, MFn::kJoint); !it.isDone(); it.next()) {
		MFnTransform fn(it.currentItem());
		pose.joints.emplace_back(fn.object(), fn.transformation());
	}
	m_skeletal = true;
	m_vnormals = m_ogf_smoothing == ogf_smoothing::normals;
	m_target_sdk = m_ogf_smoothing == ogf_smoothing::soc ? SDK_VER_0_4 : SDK_VER_0_6;
	status = MS::kFailure;
	if (xr_object* object = create_skl_object(mesh_obj, skin_obj, m_ogf_influences)) {
		std::string error;
		if (save_skeletal_ogf(*object, path, m_ogf_smoothing, m_ogf_influences, m_ogf_motion_refs, error)) {
			status = MS::kSuccess;
			MGlobal::displayInfo(MString("IX-Ray: exported skeletal OGF: ") + path);
		} else MGlobal::displayError(MString("IX-Ray: ") + error.c_str());
		delete object;
	}
	return status;
}

MStatus maya_export_tools::export_skl(const char* path, bool selection_only)
{
	set_default_options();

	if (MTime::uiUnit() != MTime::kNTSCFrame)
	{
		msg("xray_re: exporting motion with custom FPS");
		MGlobal::displayInfo("xray_re: exporting motion with custom FPS");
	}

	MObject skin_obj;
	MStatus status = find_mesh_and_skin(0, &skin_obj, selection_only);
	if (!status)
		return status;

	MFnSkinCluster skin_fn(skin_obj);
	MDagPathArray joints;
	skin_fn.influenceObjects(joints, &status);

	const unsigned num_joints = joints.length();
	if (num_joints == 0)
	{
		msg("xray_re: can't find any influence object");
		MGlobal::displayError("xray_re: can't find any influence object");
		return MS::kFailure;
	}

	xr_bone_motion_vec bmotions(num_joints);

	for (unsigned i = num_joints; i != 0;)
	{
		MFnIkJoint joint_fn(joints[--i], &status);
		if (!status)
		{
			msg("xray_re: can't handle non-joint node %s", joints[i].partialPathName().asChar());
			MGlobal::displayWarning(MString("xray_re: can't handle non-joint node ") + joints[i].partialPathName().asChar());
			return status;
		}

		xr_bone_motion* bmotion = new xr_bone_motion(getRealName(joint_fn).c_str());
		bmotion->create_envelopes();
		bmotions[i] = bmotion;
	}

	MTime saved_time(MAnimControl::currentTime());

	const MTime::Unit ui_unit = MTime::uiUnit();
	const int32_t frame_start = int32_t(MAnimControl::minTime().as(ui_unit));
	const int32_t frame_end = int32_t(MAnimControl::maxTime().as(ui_unit));

	const int32_t frame_count = frame_end - frame_start + 1;

	msg("xray_re: animation range=%d-%d (%d frames)", frame_start, frame_end, frame_count);

	MGlobal::displayInfo(MString("xray_re: animation range=") + frame_start + "-" + frame_end);

	MTime tm;
	tm.setUnit(ui_unit);

	for (int32_t frame = frame_start; frame <= frame_end; ++frame)
	{
		tm.setValue(frame);
		MGlobal::viewFrame(tm);

		const float displacedTime = float(tm.as(MTime::kSeconds));

		for (unsigned i = num_joints; i != 0;)
		{
			MFnIkJoint joint_fn(joints[--i]);
			xr_envelope* const* envelopes = bmotions[i]->envelopes();

			MTransformationMatrix mat = joint_fn.transformationMatrix(&status);
			CHECK_MSTATUS(status);

			MVector t = mat.getTranslation(MSpace::kTransform, &status);
			CHECK_MSTATUS(status);

			envelopes[0]->insert_key(displacedTime, float(MDistance(t.x, MDistance::kCentimeters).asMeters()));
			envelopes[1]->insert_key(displacedTime, float(MDistance(t.y, MDistance::kCentimeters).asMeters()));
			envelopes[2]->insert_key(displacedTime, float(MDistance(-t.z, MDistance::kCentimeters).asMeters()));

			MEulerRotation r = mat.eulerRotation();
			r.reorderIt(MEulerRotation::kZXY);

			envelopes[4]->insert_key(displacedTime, float(-r.x));
			envelopes[3]->insert_key(displacedTime, float(-r.y));
			envelopes[5]->insert_key(displacedTime, float(r.z));
		}
	}

	xr_skl_motion* smotion = new xr_skl_motion;

	char name[_MAX_FNAME];
	_splitpath_s(path, NULL, 0, NULL, 0, name, sizeof(name), NULL, 0);

	smotion->name() = name;

	const float fps = float(1.0 / MTime(1, ui_unit).as(MTime::kSeconds));
	smotion->fps() = fps;

	smotion->set_frame_range(0, frame_count - 1);
	smotion->bone_motions().swap(bmotions);
	status = smotion->save_skl(path) ? MS::kSuccess : MS::kFailure;
	delete smotion;

	MAnimControl::setCurrentTime(saved_time);

	return status;
}

MStatus maya_export_tools::export_omf(const char* path, bool selection_only)
{
	if (!m_options_status) {
		MGlobal::displayError("IX-Ray: invalid OMF options.");
		return m_options_status;
	}
	MObject skin_obj;
	MStatus status = find_mesh_and_skin(0, &skin_obj, selection_only);
	if (!status) return status;

	MFnSkinCluster skin_fn(skin_obj, &status);
	if (!status) return status;
	MDagPathArray joints;
	skin_fn.influenceObjects(joints, &status);
	if (!status || joints.length() == 0) {
		MGlobal::displayError("IX-Ray: OMF export requires a skinCluster with joints.");
		return MS::kFailure;
	}
	if (joints.length() > UINT16_MAX) {
		MGlobal::displayError("IX-Ray: too many joints for OMF export.");
		return MS::kFailure;
	}

	std::vector<std::string> bone_names(joints.length());
	for (unsigned i = 0; i != joints.length(); ++i) {
		MFnIkJoint joint(joints[i], &status);
		if (!status) {
			MGlobal::displayError("IX-Ray: OMF exporter found a non-joint influence.");
			return status;
		}
		bone_names[i] = getRealName(joint);
	}
	std::vector<unsigned> bone_order(joints.length());
	if (!get_imported_bone_order(joints, bone_names, bone_order)) {
		// A manually created skeleton has no source global IDs.  The OMF remains
		// self-consistent, but cannot safely be merged with an unrelated OMF.
		for (unsigned i = 0; i != joints.length(); ++i) bone_order[i] = i;
		MGlobal::displayWarning("IX-Ray: source bone order was not found; OMF uses skinCluster influence order and may not merge correctly. Re-import the source .object/.ogf before exporting.");
	}

	const MTime saved_time(MAnimControl::currentTime());
	const MTime::Unit unit = MTime::uiUnit();
	const int32_t frame_start = int32_t(MAnimControl::minTime().as(unit));
	const int32_t frame_end = int32_t(MAnimControl::maxTime().as(unit));
	if (frame_end < frame_start) return MS::kInvalidParameter;
	const uint32_t frame_count = uint32_t(frame_end - frame_start + 1);
	std::vector<std::vector<omf_sample>> samples(joints.length());
	for (auto& bone_samples: samples) bone_samples.reserve(frame_count);

	MTime time; time.setUnit(unit);
	for (int32_t frame = frame_start; frame <= frame_end; ++frame) {
		time.setValue(frame);
		MGlobal::viewFrame(time);
		for (unsigned i = 0; i != joints.length(); ++i) {
			MFnIkJoint joint(joints[i], &status);
			if (!status) { MAnimControl::setCurrentTime(saved_time); return status; }
			MTransformationMatrix matrix = joint.transformationMatrix(&status);
			if (!status) { MAnimControl::setCurrentTime(saved_time); return status; }
			MVector translation = matrix.getTranslation(MSpace::kTransform, &status);
			if (!status) { MAnimControl::setCurrentTime(saved_time); return status; }
			// This is the inverse of the ZXY conversion used by the OMF importer.
			// Keep it in Euler space: Maya jointOrient is represented through this
			// rotation order rather than as a plain reflected world quaternion.
			MEulerRotation rotation = matrix.eulerRotation();
			rotation.reorderIt(MEulerRotation::kZXY);
			MEulerRotation xray_rotation(-rotation.x, -rotation.y, rotation.z, MEulerRotation::kZXY);
			MQuaternion xray_quaternion = xray_rotation.asQuaternion();
			// Maya's local joint transform is the inverse of the OMF local rotation.
			// The required OMF key is therefore the quaternion conjugate.
			xray_quaternion.x = -xray_quaternion.x;
			xray_quaternion.y = -xray_quaternion.y;
			xray_quaternion.z = -xray_quaternion.z;
			samples[i].push_back({ fvector3().set(
				float(MDistance(translation.x, MDistance::kCentimeters).asMeters()),
				float(MDistance(translation.y, MDistance::kCentimeters).asMeters()),
				float(MDistance(-translation.z, MDistance::kCentimeters).asMeters())),
				xray_quaternion });
		}
	}
	MAnimControl::setCurrentTime(saved_time);

	char motion_name[_MAX_FNAME];
	_splitpath_s(path, NULL, 0, NULL, 0, motion_name, sizeof(motion_name), NULL, 0);
	const std::string exported_motion_name = m_omf_motion_name.empty() ? motion_name : m_omf_motion_name;
	xr_memory_writer writer;
	writer.open_chunk(OGF4_S_MOTIONS);
	writer.open_chunk(0); writer.w_u32(1); writer.close_chunk();
	writer.open_chunk(1);
	writer.w_sz(exported_motion_name);
	writer.w_u32(frame_count);
	for (unsigned index: bone_order) write_omf_bone_keys(writer, samples[index], m_omf_position_precision);
	writer.close_chunk();
	writer.close_chunk();

	writer.open_chunk(OGF4_S_SMPARAMS);
	writer.w_u16(OGF4_S_SMPARAMS_VERSION_4);
	writer.w_u16(1);
	writer.w_sz("default");
	writer.w_size_u16(bone_names.size());
	for (size_t i = 0; i != bone_order.size(); ++i) {
		writer.w_sz(bone_names[bone_order[i]]);
		writer.w_u32(uint32_t(i));
	}
	writer.w_u16(1);
	writer.w_sz(exported_motion_name);
	writer.w_u32(m_omf_stop_at_end ? 0x2u : 0u);
	writer.w_u16(ALL_PARTITIONS);
	writer.w_u16(0);
	writer.w_float(m_omf_speed); writer.w_float(1.f); writer.w_float(m_omf_accrue); writer.w_float(m_omf_falloff);
	const uint32_t marks_count = m_omf_has_motion_marks ? uint32_t(m_omf_marks.size()) : 0;
	writer.w_u32(marks_count);
	for (uint32_t mark_index = 0; mark_index < marks_count; ++mark_index) {
		const omf_mark& mark = m_omf_marks[mark_index];
		writer.w_s(mark.name);
		writer.w_u32(uint32_t(mark.intervals.size()));
		for (const auto& interval: mark.intervals) {
			writer.w_float(interval.first);
			writer.w_float(float(frame_count) / 30.0f);
		}
	}
	writer.close_chunk();

	if (!writer.save_to(path)) return MS::kFailure;
	MGlobal::displayInfo(MString("IX-Ray: exported OMF: ") + path);
	return MS::kSuccess;
}

MStatus maya_export_tools::export_anm(const char *path, bool selection_only)
{
	MSelectionList s_list;
	MGlobal::getActiveSelectionList(s_list);

	if(s_list.length() != 1 || !selection_only)
	{
		MGlobal::displayInfo("xray_re: select one object to export");
		return MS::kFailure;
	}

	MStatus status;
	MDagPath dp;

	status = s_list.getDagPath(0, dp);
	if(status != MS::kSuccess)
		return MS::kFailure;

	MFnTransform transform(dp, &status);

	if(status != MS::kSuccess)
		return MS::kFailure;

	xr_obj_motion anm;
	anm.create_envelopes();
	xr_envelope* const* envelopes = anm.envelopes();

	MTime saved_time(MAnimControl::currentTime());

	int32_t frame_start = int32_t(MAnimControl::minTime().as(MTime::uiUnit()));
	int32_t frame_end = int32_t(MAnimControl::maxTime().as(MTime::uiUnit()));
	msg("xray_re: animation range=%d-%d", frame_start, frame_end);
	MGlobal::displayInfo(MString("xray_re: animation range=") + frame_start + "-" + frame_end);

	MTime	tmNew;
	tmNew.setUnit(MTime::uiUnit());

	for (int32_t frame = frame_start; frame <= frame_end; ++frame)
	{
		tmNew.setValue(frame);
		MGlobal::viewFrame(tmNew);

		float displacedTime = (float)tmNew.as(MTime::kSeconds);

		MVector t = transform.getTranslation(MSpace::kTransform, &status);
		CHECK_MSTATUS(status);
		envelopes[0]->insert_key(displacedTime, float(MDistance(t.x, MDistance::kCentimeters).asMeters()));
		envelopes[1]->insert_key(displacedTime, float(MDistance(t.y, MDistance::kCentimeters).asMeters()));
		envelopes[2]->insert_key(displacedTime, float(MDistance(-t.z, MDistance::kCentimeters).asMeters()));

		MEulerRotation r;
		status = transform.getRotation(r);
		CHECK_MSTATUS(status);
		r.reorderIt(MEulerRotation::kZXY);
		envelopes[4]->insert_key(displacedTime, float(-r.x));
		envelopes[3]->insert_key(displacedTime, float(-r.y));
		envelopes[5]->insert_key(displacedTime, float(r.z));
	}

	MAnimControl::setCurrentTime(saved_time);

	char name[_MAX_FNAME];
	_splitpath_s(path, NULL, 0, NULL, 0, name, sizeof(name), NULL, 0);

	anm.name() = name;
	anm.fps() = (float)(double(1) / MTime(1, MTime::uiUnit()).as(MTime::kSeconds));
	anm.set_frame_range(frame_start, frame_end);

	return anm.save_anm(path) ? MS::kSuccess : MS::kFailure;
}

void maya_export_tools::set_default_options(void)
{
	m_target_sdk = xray_re::SDK_VER_DEFAULT;
	m_compressed = false;
	m_vnormals = false;
	m_ogf_smoothing = ogf_smoothing::normals;
	m_ogf_influences = 4;
	m_ogf_motion_refs.clear();
	m_omf_position_precision = 32;
	m_omf_motion_name.clear();
	m_omf_speed = 1.f;
	m_omf_accrue = 2.f;
	m_omf_falloff = 2.f;
	m_omf_stop_at_end = false;
	m_omf_has_motion_marks = false;
	m_omf_marks.clear();
}

MStatus maya_export_tools::parse_options(const MString& options)
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
			m_target_sdk = (ver == xray_re::SDK_VER_UNKNOWN ? xray_re::SDK_VER_DEFAULT : ver);
		}
		else if (key_value[0] == "compressed")
		{
			m_compressed = (key_value[1] == "true");
		}
		else if (key_value[0] == "vnormals")
			m_vnormals = (key_value[1] == "true");
		else if (key_value[0] == "ogf_smoothing") {
			if (key_value[1] == "normals") m_ogf_smoothing = ogf_smoothing::normals;
			else if (key_value[1] == "soc") m_ogf_smoothing = ogf_smoothing::soc;
			else if (key_value[1] == "cscop") m_ogf_smoothing = ogf_smoothing::cscop;
			else return MS::kInvalidParameter;
		}
		else if (key_value[0] == "ogf_influences") {
			if (key_value[1] != "2" && key_value[1] != "4") return MS::kInvalidParameter;
			m_ogf_influences = key_value[1].asInt();
		}
		else if (key_value[0] == "ogf_motion_refs") m_ogf_motion_refs = normalize_omf_refs(key_value[1].asChar());
		else if (key_value[0] == "omf_position_precision") {
			if (key_value[1] != "8" && key_value[1] != "16" && key_value[1] != "32") return MS::kInvalidParameter;
			m_omf_position_precision = key_value[1].asInt();
		}
		else if (key_value[0] == "omf_motion_name") m_omf_motion_name = key_value[1].asChar();
		else if (key_value[0] == "omf_speed") m_omf_speed = key_value[1].asFloat();
		else if (key_value[0] == "omf_accrue") m_omf_accrue = key_value[1].asFloat();
		else if (key_value[0] == "omf_falloff") m_omf_falloff = key_value[1].asFloat();
		else if (key_value[0] == "omf_stop_at_end") m_omf_stop_at_end = key_value[1] == "true";
		else if (key_value[0] == "omf_has_motion_marks") m_omf_has_motion_marks = key_value[1] == "true";
		else if (key_value[0] == "omf_marks") {
			m_omf_marks.clear();
			std::istringstream marks(key_value[1].asChar());
			std::string group;
			while (std::getline(marks, group, ',')) {
				if (group.empty()) continue;
				omf_mark mark;
				const size_t separator = group.find('/');
				const size_t legacy_separator = group.find(':');
				const size_t name_separator = separator != std::string::npos ? separator : legacy_separator;
				if (name_separator == 0) return MS::kInvalidParameter;
				if (name_separator == std::string::npos) {
					mark.name = group;
					m_omf_marks.push_back(std::move(mark));
					continue;
				}
				mark.name = group.substr(0, name_separator);
				std::istringstream intervals(group.substr(name_separator + 1));
				std::string interval;
				while (std::getline(intervals, interval, '|')) {
					std::replace(interval.begin(), interval.end(), ':', ' ');
					std::istringstream values(interval);
					float start, end;
					if (!(values >> start >> end) || end < start || !std::isfinite(start) || !std::isfinite(end))
						return MS::kInvalidParameter;
					mark.intervals.emplace_back(start, end);
				}
				m_omf_marks.push_back(std::move(mark));
			}
		}
	}

	return MS::kSuccess;
}
