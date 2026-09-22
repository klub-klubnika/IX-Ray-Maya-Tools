#include <maya/MGlobal.h>
#include <maya/MStringArray.h>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MItDag.h>
#include <maya/MSelectionList.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include "maya_motions_browser.h"
#include "maya_import_tools.h"
#include "xr_object.h"
#include "xr_ogf_v4.h"
#include "xr_skl_motion.h"

MStatus record_imported_motion_file(const MString& path)
{
	// Encode UTF-8 bytes rather than interpolating Windows paths into Python literals.
	MString command("import xray_motion_browser; xray_motion_browser.extern_load([bytes.fromhex('");
	const char* hex = "0123456789abcdef";
	for (const unsigned char* p = reinterpret_cast<const unsigned char*>(path.asUTF8()); *p; ++p)
	{
		char pair[] = { hex[*p >> 4], hex[*p & 15], 0 };
		command += pair;
	}
	command += "').decode('utf-8')])";
	return MGlobal::executePythonCommand(command);
}

using namespace xray_re;

namespace {

static MString extract_extension(const MString& path)
{
	const int dot = path.rindex('.');
	return dot >= 0 ? path.substring(dot + 1, path.numChars() - 1).toLowerCase() : MString();
}

static MStatus cant_open(const MString& path)
{
	MGlobal::displayError(MString("xray_re: can't open ") + path);
	return MS::kFailure;
}

class ixrayMotionList: public MPxCommand
{
public:
	virtual MStatus	doIt(const MArgList& args);

	static void*	creator();
	static MSyntax	syntax_creator();
};

void* ixrayMotionList::creator() { return new ixrayMotionList; }

MSyntax ixrayMotionList::syntax_creator()
{
	MSyntax syntax;
	syntax.addArg(MSyntax::kString);
	return syntax;
}

MStatus ixrayMotionList::doIt(const MArgList& args)
{
	if (args.length() < 1)
		return MS::kFailure;

	MString path;
	args.get(0, path);

	MStringArray names;
	const MString extension = extract_extension(path);

	if (extension == "skls")
	{
		xr_object* object = new xr_object;
		if (!object->load_skls(path.asChar()))
		{
			delete object;
			return cant_open(path);
		}
		const xr_skl_motion_vec& motions = object->motions();
		for (xr_skl_motion_vec_cit it = motions.begin(), end = motions.end(); it != end; ++it)
			names.append((*it)->name().c_str());
		delete object;
	}
	else if (extension == "omf")
	{
		xr_ogf_v4* omf = new xr_ogf_v4;
		if (!omf->load_omf(path.asChar()))
		{
			delete omf;
			return cant_open(path);
		}
		const xr_skl_motion_vec& motions = omf->motions();
		for (xr_skl_motion_vec_cit it = motions.begin(), end = motions.end(); it != end; ++it)
			names.append((*it)->name().c_str());
		delete omf;
	}
	else if (extension == "skl")
	{
		xr_skl_motion* smotion = new xr_skl_motion;
		if (!smotion->load_skl(path.asChar()))
		{
			delete smotion;
			return cant_open(path);
		}
		names.append(smotion->name().c_str());
		delete smotion;
	}
	else
	{
		MGlobal::displayError(MString("xray_re: unsupported motion file extension ") + extension);
		return MS::kFailure;
	}

	setResult(names);
	return MS::kSuccess;
}

class ixrayMotionExport: public MPxCommand
{
public:
	MStatus doIt(const MArgList& args) override
	{
		if (args.length() != 3) return MS::kFailure;
		MString path, name, destination;
		args.get(0, path);
		args.get(1, name);
		args.get(2, destination);
		xr_object object;
		xr_ogf_v4 omf;
		xr_skl_motion single;
		const xr_skl_motion_vec* motions = nullptr;
		const xr_skl_motion* chosen = nullptr;
		const MString extension = extract_extension(path);
		if (extension == "skls")
		{
			if (!object.load_skls(path.asChar())) return cant_open(path);
			motions = &object.motions();
		}
		else if (extension == "omf")
		{
			if (!omf.load_omf(path.asChar())) return cant_open(path);
			motions = &omf.motions();
		}
		else if (extension == "skl")
		{
			if (!single.load_skl(path.asChar())) return cant_open(path);
			if (single.name() == name.asChar()) chosen = &single;
		}
		else
		{
			MGlobal::displayError("IX-Ray: unsupported motion file type");
			return MS::kFailure;
		}
		if (motions)
			for (const auto* motion : *motions)
				if (motion->name() == name.asChar())
				{
					chosen = motion;
					break;
				}
		if (!chosen)
		{
			MGlobal::displayError(MString("IX-Ray: animation not found: ") + name);
			return MS::kFailure;
		}
		if (!chosen->save_skl(destination.asChar()))
		{
			MGlobal::displayError(MString("IX-Ray: cannot write animation: ") + destination);
			return MS::kFailure;
		}
		setResult(destination);
		return MS::kSuccess;
	}
};

// Returns OMF settings that are stored alongside a source animation.  The
// browser uses this before re-baking so metadata is not replaced by defaults.
class ixrayMotionInfo: public MPxCommand
{
public:
	MStatus doIt(const MArgList& args) override
	{
		if (args.length() != 3) return MS::kFailure;
		MString path, name;
		int source_index = 0;
		args.get(0, path); args.get(1, name); args.get(2, source_index);
		if (source_index < 0) return MS::kInvalidParameter;

		xr_object object;
		xr_ogf_v4 omf;
		xr_skl_motion single;
		const xr_skl_motion_vec* motions = nullptr;
		const MString extension = extract_extension(path);
		if (extension == "skls") {
			if (!object.load_skls(path.asChar())) return cant_open(path);
			motions = &object.motions();
		} else if (extension == "omf") {
			if (!omf.load_omf(path.asChar())) return cant_open(path);
			motions = &omf.motions();
		} else if (extension == "skl") {
			if (!single.load_skl(path.asChar())) return cant_open(path);
			if (source_index == 0 && single.name() == name.asChar()) motions = nullptr;
			else return MS::kFailure;
		} else {
			MGlobal::displayError("IX-Ray: unsupported motion file type");
			return MS::kFailure;
		}

		const xr_skl_motion* chosen = extension == "skl" ? &single : nullptr;
		if (motions) {
			for (unsigned index = 0; index != motions->size(); ++index) {
				const xr_skl_motion* motion = (*motions)[index];
				if (index == unsigned(source_index) && motion->name() == name.asChar()) {
					chosen = motion;
					break;
				}
			}
		}
		if (!chosen) {
			MGlobal::displayError(MString("IX-Ray: animation not found: ") + name);
			return MS::kFailure;
		}
		MString result("omf_speed="); result += chosen->speed();
		result += ";omf_accrue="; result += chosen->accrue();
		result += ";omf_falloff="; result += chosen->falloff();
		result += ";omf_flags="; result += int(chosen->flags());
		result += ";omf_stop_at_end=";
		result += (chosen->flags() & xr_skl_motion::SMF_STOP_AT_END) ? "true" : "false";
		result += ";";
		setResult(result);
		return MS::kSuccess;
	}
};

class ixrayMotionLoad: public MPxCommand
{
public:
	virtual MStatus	doIt(const MArgList& args);

	static void*	creator();
	static MSyntax	syntax_creator();
};

void* ixrayMotionLoad::creator() { return new ixrayMotionLoad; }

MSyntax ixrayMotionLoad::syntax_creator()
{
	MSyntax syntax;
	syntax.addArg(MSyntax::kString);
	syntax.addArg(MSyntax::kString);
	syntax.addArg(MSyntax::kString);
	syntax.addArg(MSyntax::kLong);
	return syntax;
}

MStatus ixrayMotionLoad::doIt(const MArgList& args)
{
	if (args.length() < 1)
		return MS::kFailure;

	MString path, motion_name, options;
	int source_index = 0;
	args.get(0, path);
	if (args.length() > 1)
		args.get(1, motion_name);
	if (args.length() > 2)
		args.get(2, options);
	if (args.length() > 3)
		args.get(3, source_index);
	if (source_index < 0) return MS::kInvalidParameter;

	maya_import_tools imp_tools(options);
	double end_frame = 0;
	MStatus status = MS::kFailure;
	const MString extension = extract_extension(path);

	if (extension == "skls")
	{
		xr_object* object = new xr_object;
		if (!object->load_skls(path.asChar()))
		{
			delete object;
			return cant_open(path);
		}

		std::vector<xr_skl_motion*> chosen;
		const xr_skl_motion_vec& motions = object->motions();
		unsigned index = 0;
		for (xr_skl_motion_vec_cit it = motions.begin(), end = motions.end(); it != end; ++it, ++index)
		{
			xr_skl_motion* motion = *it;
			if (motion_name.length() == 0 || (motion->name() == motion_name.asChar() && index == unsigned(source_index)))
				chosen.push_back(motion);
		}
		if (chosen.empty())
		{
			delete object;
			MGlobal::displayError(MString("xray_re: no matching motion ") + motion_name);
			return MS::kFailure;
		}

		for (auto* motion : chosen)
		{
			status = imp_tools.import_selected_motion(motion, &end_frame);
			if (!status) break;
		}
		delete object;
	}
	else if (extension == "omf")
	{
		xr_ogf_v4* omf = new xr_ogf_v4;
		if (!omf->load_omf(path.asChar()))
		{
			delete omf;
			return cant_open(path);
		}
		unsigned index = 0;
		for (auto* motion : omf->motions())
		{
			if (motion->name() != motion_name.asChar()) { ++index; continue; }
			if (index++ != unsigned(source_index)) continue;
			status = imp_tools.import_selected_motion(motion, &end_frame);
			break;
		}
		delete omf;
	}
	else if (extension == "skl")
	{
		xr_skl_motion* smotion = new xr_skl_motion;
		if (!smotion->load_skl(path.asChar()))
		{
			delete smotion;
			return cant_open(path);
		}
		status = imp_tools.import_selected_motion(smotion, &end_frame);
		delete smotion;
	}
	else
	{
		MGlobal::displayError(MString("xray_re: unsupported motion file extension ") + extension);
		return MS::kFailure;
	}

	if (status) setResult(end_frame);
	return status;
}

}

void* motion_list_command_creator() { return ixrayMotionList::creator(); }

MSyntax motion_list_syntax_creator() { return ixrayMotionList::syntax_creator(); }

void* motion_load_command_creator() { return ixrayMotionLoad::creator(); }

MSyntax motion_load_syntax_creator() { return ixrayMotionLoad::syntax_creator(); }

void* motion_export_command_creator() { return new ixrayMotionExport; }

MSyntax motion_export_syntax_creator() { return ixrayMotionLoad::syntax_creator(); }

void* motion_info_command_creator() { return new ixrayMotionInfo; }

MSyntax motion_info_syntax_creator() { return ixrayMotionLoad::syntax_creator(); }
