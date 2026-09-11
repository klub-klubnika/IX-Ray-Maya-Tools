#define NOMINMAX
#include "ogf_export.h"
#include "xr_object.h"
#include "xr_ogf_format.h"
#include "xr_writer.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

using namespace xray_re;

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

bool finite(const fvector3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

fvector3 unit(fvector3 v)
{
    require(finite(v) && v.square_magnitude() > 1e-20f, "Invalid or zero normal / degenerate triangle.");
    return v.normalize();
}

struct corner_sets {
    std::vector<size_t> parent;
    explicit corner_sets(size_t size): parent(size) { for (size_t i = 0; i < size; ++i) parent[i] = i; }
    size_t root(size_t i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
    void join(size_t a, size_t b) { parent[root(a)] = root(b); }
};

std::vector<fvector3> normals(const xr_mesh& mesh, ogf_smoothing mode)
{
    const auto& faces = mesh.faces();
    if (mode == ogf_smoothing::normals) {
        // Maya's existing extractor writes three normals per triangle into vnorm.
        require(mesh.vnorm().size() == faces.size() * 3, "Missing Maya face-vertex normals.");
        auto result = mesh.vnorm();
        for (auto& n: result) n = unit(n);
        return result;
    }
    require(mesh.sgroups().size() == faces.size(), "Missing smoothing groups.");
    corner_sets sets(faces.size() * 3);
    if (mode == ogf_smoothing::soc) {
        // SDK Maya-style groups: equal IDs smooth, 0xffffffff is a hard face.
        std::map<std::pair<uint32_t, uint32_t>, size_t> groups;
        for (size_t f = 0; f < faces.size(); ++f) {
            if (mesh.sgroups()[f] == EMESH_NO_SG) continue;
            for (size_t c = 0; c < 3; ++c) {
                auto inserted = groups.emplace(std::make_pair(faces[f].v[c], mesh.sgroups()[f]), f * 3 + c);
                if (!inserted.second) sets.join(inserted.first->second, f * 3 + c);
            }
        }
    } else {
        // CS/CoP stores hard-edge bits. Walk smooth fans separately at each vertex.
        std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> edges;
        for (size_t f = 0; f < faces.size(); ++f)
            for (size_t c = 0; c < 3; ++c)
                edges[std::minmax(faces[f].v[c], faces[f].v[(c + 1) % 3])].push_back(f * 3 + c);
        for (const auto& edge: edges) {
            if (edge.second.size() != 2) continue;
            size_t a = edge.second[0], b = edge.second[1];
            if ((mesh.sgroups()[a / 3] & (1u << (a % 3))) ||
                (mesh.sgroups()[b / 3] & (1u << (b % 3)))) continue;
            for (size_t i = 0; i < 2; ++i)
                for (size_t j = 0; j < 2; ++j) {
                    size_t ac = (a % 3 + i) % 3, bc = (b % 3 + j) % 3;
                    if (faces[a / 3].v[ac] == faces[b / 3].v[bc])
                        sets.join(a / 3 * 3 + ac, b / 3 * 3 + bc);
                }
        }
    }
    std::vector<fvector3> sums(faces.size() * 3), result(faces.size() * 3);
    for (auto& n: sums) n.set();
    for (size_t f = 0; f < faces.size(); ++f) {
        const auto& face = faces[f];
        fvector3 n;
        n.calc_normal_non_normalized(mesh.points().at(face.v[0]), mesh.points().at(face.v[1]), mesh.points().at(face.v[2]));
        n = unit(n);
        for (size_t c = 0; c < 3; ++c) sums[sets.root(f * 3 + c)].add(n);
    }
    for (size_t i = 0; i < result.size(); ++i) result[i] = unit(sums[sets.root(i)]);
    return result;
}

struct vertex {
    fvector3 p, n, t, b;
    fvector2 uv;
    std::array<uint16_t, 4> bones{};
    std::array<float, 4> weights{};
};
// Position, normal, UV, weights and tangent handedness all delimit seams.
using vertex_key = std::pair<std::array<float, 13>, std::array<uint16_t, 4>>;
vertex_key key(const vertex& v, float sign)
{
    return {{v.p.x, v.p.y, v.p.z, v.n.x, v.n.y, v.n.z, v.uv.x, v.uv.y,
        v.weights[0], v.weights[1], v.weights[2], v.weights[3], sign}, v.bones};
}

struct part {
    const xr_surface* surface = nullptr;
    std::vector<vertex> vertices;
    std::vector<uint16_t> indices;
    std::map<vertex_key, uint16_t> lookup;
    fbox bounds;
    part() { bounds.invalidate(); }
};

void header(xr_writer& w, uint8_t type, const fbox& box)
{
    w.open_chunk(OGF_HEADER);
    w.w_u8(OGF4_VERSION); w.w_u8(type); w.w_u16(0);
    w.w_fvector3(box.min); w.w_fvector3(box.max);
    fvector3 center, radius;
    box.center(center); radius.sub(box.max, center);
    w.w_fvector3(center); w.w_float(radius.magnitude());
    w.close_chunk();
}

void write_part(xr_writer& w, part& p, unsigned limit)
{
    header(w, MT4_SKELETON_GEOMDEF_ST, p.bounds);
    w.open_chunk(OGF4_TEXTURE);
    w.w_sz(p.surface->texture()); w.w_sz(p.surface->eshader()); w.close_chunk();
    w.open_chunk(OGF4_VERTICES);
    // Legacy 2W works in SoC as well as CS/CoP; 4W is the CS/CoP layout.
    w.w_u32(limit == 2 ? OGF4_VERTEXFORMAT_FVF_2L : OGF4_VERTEXFORMAT_FVF_4L_CS);
    w.w_size_u32(p.vertices.size());
    for (auto& v: p.vertices) {
        fvector3 projected; projected.mul(v.n, v.n.dot_product(v.t));
        v.t.sub(projected);
        if (v.t.square_magnitude() < 1e-20f) {
            fvector3 axis; axis.set(std::abs(v.n.x) < .9f ? 1.f : 0.f, std::abs(v.n.x) < .9f ? 0.f : 1.f, 0.f);
            v.t.cross_product(axis, v.n);
        }
        v.t = unit(v.t);
        fvector3 b; b.cross_product(v.n, v.t);
        if (b.dot_product(v.b) < 0) b.mul(-1.f);
        for (unsigned i = 0; i < limit; ++i) w.w_u16(v.bones[i]);
        w.w_fvector3(v.p); w.w_fvector3(v.n); w.w_fvector3(v.t); w.w_fvector3(b);
        if (limit == 2) w.w_float(v.weights[1]);
        else for (unsigned i = 0; i < 3; ++i) w.w_float(v.weights[i]);
        w.w_fvector2(v.uv);
    }
    w.close_chunk();
    w.open_chunk(OGF4_INDICES); w.w_size_u32(p.indices.size()); w.w_seq(p.indices); w.close_chunk();
}
}

bool save_skeletal_ogf(xr_object& object, const char* path, ogf_smoothing smoothing,
    unsigned influence_limit, const std::string& motion_refs, std::string& error)
{
    try {
        require(influence_limit == 2 || influence_limit == 4, "OGF influence limit must be 2 or 4.");
        auto& bones = object.bones();
        require(!bones.empty() && bones.size() <= MAX_BONES, "Skeleton must contain 1 to 64 bones.");
        std::map<std::string, uint16_t> bone_ids;
        for (size_t i = 0; i < bones.size(); ++i) {
            require(bone_ids.emplace(bones[i]->name(), uint16_t(i)).second, "Duplicate bone names.");
            bones[i]->children().clear();
        }
        size_t roots = 0;
        for (const auto* bone: bones) {
            if (bone->parent_name().empty()) ++roots;
            else require(bone_ids.count(bone->parent_name()) != 0, "Missing parent bone.");
            const xr_bone* ancestor = bone;
            for (size_t depth = 0; !ancestor->parent_name().empty(); ++depth) {
                require(depth < bones.size(), "Cycle in skeleton hierarchy.");
                auto parent = bone_ids.find(ancestor->parent_name());
                require(parent != bone_ids.end(), "Missing parent bone.");
                ancestor = bones[parent->second];
            }
        }
        require(roots == 1, "Skeleton must have exactly one root bone.");
        for (size_t i = 0; i < bones.size(); ++i) bones[i]->setup(uint16_t(i), object);
        object.calculate_bind();
        std::vector<fbox> bone_bounds(bones.size());
        std::vector<bool> bone_used(bones.size(), false);
        for (auto& box: bone_bounds) box.invalidate();
        std::vector<part> parts;
        fbox bounds; bounds.invalidate();
        for (const auto* mesh: object.meshes()) {
            auto ns = normals(*mesh, smoothing);
            for (const auto* sm: mesh->surfmaps()) {
                if (!sm || sm->faces.empty()) continue;
                require(sm->surface && !sm->surface->texture().empty() && !sm->surface->eshader().empty(),
                    "Every material needs an engine shader and a texture.");
                parts.emplace_back(); parts.back().surface = sm->surface;
                for (uint32_t f: sm->faces) {
                    const auto& face = mesh->faces().at(f);
                    vertex tri[3];
                    for (size_t c = 0; c < 3; ++c) {
                        auto& v = tri[c]; v.p = mesh->points().at(face.v[c]); v.n = ns.at(f * 3 + c);
                        require(finite(v.p), "Non-finite vertex position.");
                        bool has_uv = false;
                        std::vector<std::pair<float, uint16_t>> weights;
                        for (const auto& ref: mesh->vmrefs().at(face.ref[c])) {
                            const auto* vm = mesh->vmaps().at(ref.vmap);
                            if (vm->type() == xr_vmap::VMT_UV) {
                                v.uv = static_cast<const xr_uv_vmap*>(vm)->uvs().at(ref.offset); has_uv = true;
                            } else {
                                float weight = static_cast<const xr_weight_vmap*>(vm)->weights().at(ref.offset);
                                require(std::isfinite(weight) && weight >= 0, "Invalid skin weight.");
                                auto bone = bone_ids.find(vm->name());
                                require(bone != bone_ids.end(), "Weight map references a missing bone.");
                                if (weight > 0) weights.emplace_back(weight, bone->second);
                            }
                        }
                        require(has_uv && std::isfinite(v.uv.x) && std::isfinite(v.uv.y), "Missing or invalid UV coordinates.");
                        require(!weights.empty(), "Vertex has no bone weights.");
                        std::sort(weights.begin(), weights.end(), std::greater<std::pair<float, uint16_t>>());
                        if (weights.size() > influence_limit) weights.resize(influence_limit);
                        float total = 0; for (const auto& weight: weights) total += weight.first;
                        v.bones.fill(weights.front().second);
                        for (size_t i = 0; i < weights.size(); ++i) {
                            v.bones[i] = weights[i].second; v.weights[i] = weights[i].first / total;
                            fvector3 local = v.p;
                            // PCore matrices use the same row-vector convention as X-Ray.
                            const auto& m = bones[v.bones[i]]->bind_i_xform();
                            local.set(v.p.x*m._11 + v.p.y*m._21 + v.p.z*m._31 + m._41,
                                v.p.x*m._12 + v.p.y*m._22 + v.p.z*m._32 + m._42,
                                v.p.x*m._13 + v.p.y*m._23 + v.p.z*m._33 + m._43);
                            bone_bounds[v.bones[i]].extend(local); bone_used[v.bones[i]] = true;
                        }
                    }
                    fvector3 e1, e2, t, b, geometric;
                    e1.sub(tri[1].p, tri[0].p); e2.sub(tri[2].p, tri[0].p);
                    geometric.cross_product(e1, e2); unit(geometric);
                    float u1 = tri[1].uv.x-tri[0].uv.x, v1 = tri[1].uv.y-tri[0].uv.y;
                    float u2 = tri[2].uv.x-tri[0].uv.x, v2 = tri[2].uv.y-tri[0].uv.y;
                    float det = u1*v2-u2*v1;
                    if (std::abs(det) > 1e-20f) {
                        t.mul(e1, v2).sub(fvector3().mul(e2, v1)).mul(1.f/det);
                        b.mul(e2, u1).sub(fvector3().mul(e1, u2)).mul(1.f/det);
                    } else { t = e1; b = e2; }
                    unsigned sides = sm->surface->two_sided() ? 2 : 1;
                    for (unsigned side = 0; side < sides; ++side) {
                        if (parts.back().vertices.size() + 3 > 65535) {
                            parts.emplace_back(); parts.back().surface = sm->surface;
                        }
                        auto& p = parts.back();
                        for (size_t c = 0; c < 3; ++c) {
                            vertex v = tri[side ? 2-c : c];
                            if (side) v.n.mul(-1.f);
                            fvector3 cross; cross.cross_product(v.n, t);
                            auto k = key(v, cross.dot_product(b) < 0 ? -1.f : 1.f);
                            auto inserted = p.lookup.emplace(k, uint16_t(p.vertices.size()));
                            if (inserted.second) {
                                v.t = t; v.b = b; p.vertices.push_back(v); p.bounds.extend(v.p); bounds.extend(v.p);
                            } else { auto& existing = p.vertices[inserted.first->second]; existing.t.add(t); existing.b.add(b); }
                            p.indices.push_back(inserted.first->second);
                        }
                    }
                }
            }
        }
        require(!parts.empty(), "No skeletal geometry to export.");
        xr_memory_writer w;
        header(w, motion_refs.empty() ? MT4_SKELETON_RIGID : MT4_SKELETON_ANIM, bounds);
        w.open_chunk(OGF4_CHILDREN);
        for (size_t i = 0; i < parts.size(); ++i) {
            w.open_chunk(uint32_t(i)); write_part(w, parts[i], influence_limit); w.close_chunk();
        }
        w.close_chunk();
        w.open_chunk(OGF4_S_BONE_NAMES); w.w_size_u32(bones.size());
        for (size_t i = 0; i < bones.size(); ++i) {
            w.w_sz(bones[i]->name()); w.w_sz(bones[i]->parent_name());
            fobb box; box.reset();
            if (bone_used[i]) {
                bone_bounds[i].center(box.translate);
                box.halfsize.sub(bone_bounds[i].max, bone_bounds[i].min).mul(.5f);
            }
            w.w(box);
        }
        w.close_chunk();
        w.open_chunk(OGF4_S_IKDATA);
        for (const auto* bone: bones) {
            w.w_u32(OGF4_S_JOINT_IK_DATA_VERSION); w.w_sz(bone->gamemtl());
            w.w(bone->shape()); w.w(bone->joint_ik_data());
            w.w_fvector3(bone->bind_rotate()); w.w_fvector3(bone->bind_offset());
            w.w_float(bone->mass()); w.w_fvector3(bone->center_of_mass());
        }
        w.close_chunk();
        if (!motion_refs.empty()) w.w_chunk(OGF4_S_MOTION_REFS_0, motion_refs);
        require(w.save_to(path), "Cannot write OGF file.");
        return true;
    } catch (const std::exception& e) { error = e.what(); return false; }
}
