#define NOMINMAX
#include "ogf_export.h"
#include "xr_object.h"
#include "xr_ogf.h"
#include "xr_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace xray_re;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
bool near(float a, float b) { return std::abs(a-b) < 1e-5f; }

std::unique_ptr<xr_object> fixture()
{
    std::unique_ptr<xr_object> object(new xr_object);
    for (int i = 0; i < 4; ++i) {
        auto* b = new xr_bone;
        b->name() = b->vmap_name() = "bone" + std::to_string(i);
        if (i) b->parent_name() = "bone0";
        b->bind_offset().set(float(i), 0, 0);
        object->bones().push_back(b);
    }
    auto* mesh = new xr_mesh; object->meshes().push_back(mesh);
    mesh->points() = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    mesh->faces() = {lw_face(0,1,2), lw_face(1,0,3)};
    mesh->sgroups() = {0,0};
    mesh->vnorm().assign(6, {1,0,0});
    auto* uv = new xr_uv_vmap("Texture"); mesh->vmaps().push_back(uv);
    for (int i = 0; i < 4; ++i) mesh->vmaps().push_back(new xr_weight_vmap("bone" + std::to_string(i)));
    for (auto& face: mesh->faces())
        for (unsigned c = 0; c < 3; ++c) {
            lw_vmref refs;
            fvector2 tc; tc.set(c == 1 ? 1.f : 0.f, c == 2 ? 1.f : 0.f);
            refs.push_back(lw_vmref_entry(0, uv->add_uv(tc, face.v[c])));
            for (unsigned i = 0; i < 4; ++i) {
                auto* vm = static_cast<xr_weight_vmap*>(mesh->vmaps()[i+1]);
                refs.push_back(lw_vmref_entry(i+1, vm->add_weight(.1f*(i+1), face.v[c])));
            }
            face.ref[c] = uint32_t(mesh->vmrefs().size()); mesh->vmrefs().push_back(refs);
        }
    auto* surface = new xr_surface(true); surface->texture() = "test/texture";
    object->surfaces().push_back(surface);
    auto* sm = new xr_surfmap(surface); sm->faces = {0,1}; mesh->surfmaps().push_back(sm);
    return object;
}

std::unique_ptr<xr_ogf> compile(xr_object& object, const std::string& path, ogf_smoothing mode, unsigned limit = 4)
{
    std::string error;
    const bool saved = save_skeletal_ogf(object, path.c_str(), mode, limit, "", error);
    check(saved, error.c_str());
    std::unique_ptr<xr_ogf> result(xr_ogf::load_ogf(path));
    check(result.get() && result->skeletal() && !result->animated(), "Skeleton round-trip failed");
    check(result->bones().size() == 4 && result->children().size() >= 1, "Missing skeleton chunks");
    return result;
}

int main(int argc, char** argv)
{
    try {
        check(argc == 2, "Pass an output OGF path in a scratch directory");
        const std::string path = argv[1];
        auto object = fixture();
        auto maya = compile(*object, path, ogf_smoothing::normals);
        const auto& vb = maya->children()[0]->vb();
        check(near(vb.n(0).x, 1) && near(vb.n(0).z, 0), "Custom normals changed");
        check(vb.w(0).size() == 4, "4W influences lost");
        for (const auto& weight: vb.w(0))
            check(near(weight.weight, .1f * (weight.bone + 1)), "4W weights changed");
        check(maya->children()[0]->texture() == "test/texture", "Material lost");
        auto soc = compile(*object, path, ogf_smoothing::soc);
        check(near(soc->children()[0]->vb().n(0).y, std::sqrt(.5f)), "SoC smooth group not averaged");
        auto cs = compile(*object, path, ogf_smoothing::cscop);
        check(near(cs->children()[0]->vb().n(0).z, std::sqrt(.5f)), "CS smooth fan not averaged");
        object->meshes()[0]->sgroups() = {1,1};
        auto hard = compile(*object, path, ogf_smoothing::cscop, 2);
        check(near(hard->children()[0]->vb().n(0).z, 1), "Hard edge lost");
        object->meshes()[0]->sgroups() = {EMESH_NO_SG, EMESH_NO_SG};
        auto flat = compile(*object, path, ogf_smoothing::soc);
        check(near(flat->children()[0]->vb().n(0).z, 1), "SoC hard face lost");

        // Verify binary 2W layout independently from the full OGF reader.
        compile(*object, path, ogf_smoothing::normals, 2);
        std::ifstream file(path, std::ios::binary);
        std::vector<char> bytes((std::istreambuf_iterator<char>(file)), {});
        file.close();
        xr_reader root(bytes.data(), bytes.size());
        std::unique_ptr<xr_reader> children(root.open_chunk(OGF4_CHILDREN));
        std::unique_ptr<xr_reader> child(children->open_chunk(0));
        std::unique_ptr<xr_reader> vertices(child->open_chunk(OGF4_VERTICES));
        check(vertices->r_u32() == OGF4_VERTEXFORMAT_FVF_2L, "Wrong 2W vertex format");
        vertices->r_u32();
        check(vertices->r_u16() == 3 && vertices->r_u16() == 2, "Strongest influences not selected");
        vertices->advance(48);
        check(near(vertices->r_float(), 3.f/7.f), "2W weight order or normalization wrong");

        object->surfaces()[0]->set_two_sided();
        auto two_sided = compile(*object, path, ogf_smoothing::normals);
        check(two_sided->children()[0]->ib().size() == 12, "Back faces missing");
        object->surfaces()[0]->flags() = 0;
        auto* second = new xr_surface(true); second->texture() = "test/second";
        object->surfaces().push_back(second);
        auto* sm = new xr_surfmap(second); sm->faces = {1};
        object->meshes()[0]->surfmaps()[0]->faces = {0};
        object->meshes()[0]->surfmaps().push_back(sm);
        auto materials = compile(*object, path, ogf_smoothing::normals);
        check(materials->children().size() == 2, "Material split lost");

        auto large = fixture();
        auto* mesh = large->meshes()[0];
        mesh->points().clear(); mesh->faces().clear(); mesh->surfmaps()[0]->faces.clear();
        for (uint32_t i = 0; i < 21846; ++i) {
            float x = float(i * 2);
            mesh->points().push_back({x,0,0}); mesh->points().push_back({x+1,0,0}); mesh->points().push_back({x,1,0});
            lw_face face(i*3,i*3+1,i*3+2); face.ref[0] = 0; face.ref[1] = 1; face.ref[2] = 2;
            mesh->faces().push_back(face); mesh->surfmaps()[0]->faces.push_back(i);
        }
        mesh->vnorm().assign(mesh->faces().size()*3, {0,0,1});
        auto split = compile(*large, path, ogf_smoothing::normals);
        check(split->children().size() == 2, "16-bit vertex limit not split");
        size_t indices = 0;
        for (const auto* child_ogf: split->children()) {
            check(child_ogf->vb().size() <= 65535, "Vertex index overflow"); indices += child_ogf->ib().size();
        }
        check(indices == 21846 * 3, "Triangles lost at split boundary");

        auto rigid = fixture();
        for (auto& refs: rigid->meshes()[0]->vmrefs()) {
            const lw_vmref_entry uv = refs[0], bone = refs[1];
            refs.clear(); refs.push_back(uv); refs.push_back(bone);
        }
        compile(*rigid, path, ogf_smoothing::normals);
        std::ifstream rigid_file(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(rigid_file), {}); rigid_file.close();
        xr_reader rigid_root(bytes.data(), bytes.size());
        std::unique_ptr<xr_reader> rigid_children(rigid_root.open_chunk(OGF4_CHILDREN));
        std::unique_ptr<xr_reader> rigid_child(rigid_children->open_chunk(0));
        std::unique_ptr<xr_reader> rigid_vertices(rigid_child->open_chunk(OGF4_VERTICES));
        check(rigid_vertices->r_u32() == OGF4_VERTEXFORMAT_FVF_1L_CS, "Rigid 4W part was not compacted to 1L");

        std::string error;
        check(save_skeletal_ogf(*object, path.c_str(), ogf_smoothing::normals, 4, "weapons/test", error), "OMF-reference export failed");
        std::ifstream refs_file(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(refs_file), {}); refs_file.close();
        xr_reader refs_reader(bytes.data(), bytes.size());
        check(refs_reader.find_chunk(OGF4_S_MOTION_REFS_1) != 0, "Missing CS/CoP OMF references");
        check(refs_reader.r_u32() == 1 && std::string(refs_reader.skip_sz()) == "weapons/test", "OMF reference changed");
        refs_reader.find_chunk(OGF_HEADER); refs_reader.r_u8();
        check(refs_reader.r_u8() == MT4_SKELETON_ANIM, "Wrong animated model type");
        object->bones()[1]->parent_name() = "bone1";
        check(!save_skeletal_ogf(*object, path.c_str(), ogf_smoothing::normals, 4, "", error), "Accepted cyclic bones");
        object->bones()[1]->parent_name() = "bone0";
        object->meshes()[0]->vnorm()[0].set();
        check(save_skeletal_ogf(*object, path.c_str(), ogf_smoothing::normals, 4, "", error), "Did not rebuild zero normals");
        object->meshes()[0]->faces()[0] = lw_face(0, 0, 1);
        auto without_degenerate = compile(*object, path, ogf_smoothing::normals);
        check(without_degenerate->children()[0]->ib().size() == 3, "Did not omit degenerate triangle");
        std::cout << "OGF export regression tests passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
