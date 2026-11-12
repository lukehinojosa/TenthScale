#include "gpu/BallSimulation.h"

// Split into pieces because MSVC caps a single string literal at about 16 KB
namespace
{
    const char* const SHADER_COMMON = R"WGSL(
// Balls in a box, one kernel per step: clear, integrate, prefix sum over the grid, scatter into cell order,
// resolve ball contacts colour by colour, walls, then write back. Ball i keeps index i throughout.

struct Params
{
    ballCount: u32,
    cellCount: u32,
    blockCount: u32,
    countContacts: u32, // Only the first pass counts, so passes don't count twice
    dims: vec3<i32>,
    cellSize: f32,
    origin: vec3<f32>,
    dt: f32,
    gravity: vec3<f32>,
    damping: f32,
    colorOffset: vec3<i32>, // Which of the 27 cell colours resolveColor works on
    restitution: f32,
    boxHalfSize: f32,
    pad0: f32,
    pad1: f32,
    pad2: f32,
};

struct Stats
{
    ballContacts: atomic<u32>,
    wallContacts: atomic<u32>,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> spheres: array<vec4f>; // xyz center, w radius
@group(0) @binding(2) var<storage, read_write> velocities: array<vec4f>; // xyz velocity, w inverse mass
@group(0) @binding(3) var<storage, read_write> cellOf: array<u32>;
@group(0) @binding(4) var<storage, read_write> rankInCell: array<u32>;
@group(0) @binding(5) var<storage, read_write> cellStart: array<atomic<u32>>; // Counts per cell, then where each cell starts
@group(0) @binding(6) var<storage, read_write> blockSums: array<u32>;
@group(0) @binding(7) var<storage, read_write> sortedIndex: array<u32>;
@group(0) @binding(8) var<storage, read_write> sortedSpheres: array<vec4f>;
@group(0) @binding(9) var<storage, read_write> sortedVelocities: array<vec4f>;
@group(0) @binding(10) var<storage, read_write> stats: Stats;

const WORKGROUP: u32 = 256u;

var<workgroup> scanTemp: array<u32, 256>;
var<workgroup> carry: u32;

// Dispatches spread over x and y past 65535 workgroups, so this counts along x first
fn threadIndex(id: vec3u, groups: vec3u) -> u32
{
    return id.x + id.y * groups.x * WORKGROUP;
}

fn groupIndex(group: vec3u, groups: vec3u) -> u32
{
    return group.x + group.y * groups.x;
}

// Truncate, then clamp into the grid, so touching balls stay in neighbouring cells even outside the box
fn cellCoord(p: vec3f) -> vec3i
{
    return clamp(vec3i((p - params.origin) / params.cellSize), vec3i(0), params.dims - vec3i(1));
}

fn cellIndex(c: vec3i) -> i32
{
    return c.x + params.dims.x * (c.y + params.dims.y * c.z);
}

@compute @workgroup_size(256)
fn clearCells(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let i = threadIndex(id, groups);
    if (i <= params.cellCount)
    {
        atomicStore(&cellStart[i], 0u);
    }
    if (i == 0u)
    {
        atomicStore(&stats.ballContacts, 0u);
        atomicStore(&stats.wallContacts, 0u);
    }
}

// Accelerate, damp, move, then count the ball into its cell and remember its place there
@compute @workgroup_size(256)
fn integrate(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let i = threadIndex(id, groups);
    if (i >= params.ballCount)
    {
        return;
    }
    var v = velocities[i];
    var p = spheres[i];
    var velocity = v.xyz + params.gravity * params.dt;
    if (params.damping != 1.0)
    {
        velocity = velocity * pow(params.damping, params.dt);
    }
    p = vec4f(p.xyz + velocity * params.dt, p.w);
    velocities[i] = vec4f(velocity, v.w);
    spheres[i] = p;

    let cell = u32(cellIndex(cellCoord(p.xyz)));
    cellOf[i] = cell;
    rankInCell[i] = atomicAdd(&cellStart[cell], 1u);
}

// The prefix sum turns cell counts into cell starts: each block of 256 cells, then the block totals,
// then each block's running total added back
@compute @workgroup_size(256)
fn scanBlocks(@builtin(global_invocation_id) id: vec3u, @builtin(local_invocation_id) local: vec3u,
              @builtin(workgroup_id) group: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let i = threadIndex(id, groups);
    let l = local.x;
    var value = 0u;
    if (i <= params.cellCount)
    {
        value = atomicLoad(&cellStart[i]);
    }
    scanTemp[l] = value;
    workgroupBarrier();
    for (var offset = 1u; offset < WORKGROUP; offset = offset << 1u)
    {
        var add = 0u;
        if (l >= offset)
        {
            add = scanTemp[l - offset];
        }
        workgroupBarrier();
        scanTemp[l] = scanTemp[l] + add;
        workgroupBarrier();
    }
    if (i <= params.cellCount)
    {
        atomicStore(&cellStart[i], scanTemp[l] - value);
    }
    let g = groupIndex(group, groups);
    if (l == WORKGROUP - 1u && g < params.blockCount)
    {
        blockSums[g] = scanTemp[l];
    }
}

// One workgroup, through the block totals 256 at a time
@compute @workgroup_size(256)
fn scanBlockSums(@builtin(local_invocation_id) local: vec3u)
{
    let l = local.x;
    if (l == 0u)
    {
        carry = 0u;
    }
    workgroupBarrier();
    for (var base = 0u; base < params.blockCount; base = base + WORKGROUP)
    {
        let index = base + l;
        var value = 0u;
        if (index < params.blockCount)
        {
            value = blockSums[index];
        }
        scanTemp[l] = value;
        workgroupBarrier();
        for (var offset = 1u; offset < WORKGROUP; offset = offset << 1u)
        {
            var add = 0u;
            if (l >= offset)
            {
                add = scanTemp[l - offset];
            }
            workgroupBarrier();
            scanTemp[l] = scanTemp[l] + add;
            workgroupBarrier();
        }
        if (index < params.blockCount)
        {
            blockSums[index] = carry + scanTemp[l] - value;
        }
        workgroupBarrier();
        if (l == WORKGROUP - 1u)
        {
            carry = carry + scanTemp[WORKGROUP - 1u];
        }
        workgroupBarrier();
    }
}

@compute @workgroup_size(256)
fn addBlockOffsets(@builtin(global_invocation_id) id: vec3u, @builtin(workgroup_id) group: vec3u,
                   @builtin(num_workgroups) groups: vec3u)
{
    let i = threadIndex(id, groups);
    if (i <= params.cellCount)
    {
        let start = atomicLoad(&cellStart[i]) + blockSums[groupIndex(group, groups)];
        atomicStore(&cellStart[i], start);
    }
}

// Copies every ball into its cell's slots, so the contact search reads neighbours one after another
@compute @workgroup_size(256)
fn scatter(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let i = threadIndex(id, groups);
    if (i >= params.ballCount)
    {
        return;
    }
    let slot = atomicLoad(&cellStart[cellOf[i]]) + rankInCell[i];
    sortedIndex[slot] = i;
    sortedSpheres[slot] = spheres[i];
    sortedVelocities[slot] = velocities[i];
}
)WGSL";

    const char* const SHADER_CONTACTS = R"WGSL(
// Resolves the contact, if any, between the ball held in pa and va and the ball in slot m. Both change at once,
// so later contacts see the change.
fn resolvePair(pa: ptr<function, vec4f>, va: ptr<function, vec4f>, m: u32) -> u32
{
    var pb = sortedSpheres[m];
    let d = (*pa).xyz - pb.xyz;
    let radiusSum = (*pa).w + pb.w;
    let distanceSquared = dot(d, d);
    if (distanceSquared >= radiusSum * radiusSum || distanceSquared <= 0.0)
    {
        return 0u;
    }
    let dist = sqrt(distanceSquared);
    let normal = d / dist; // Points from b into a
    var vb = sortedVelocities[m];
    let totalInverseMass = (*va).w + vb.w;
    if (totalInverseMass <= 0.0)
    {
        return 1u;
    }

    let separatingVelocity = dot((*va).xyz - vb.xyz, normal);
    if (separatingVelocity < 0.0)
    {
        let impulse = -(1.0 + params.restitution) * separatingVelocity / totalInverseMass;
        *va = vec4f((*va).xyz + normal * impulse * (*va).w, (*va).w);
        vb = vec4f(vb.xyz - normal * impulse * vb.w, vb.w);
        sortedVelocities[m] = vb;
    }

    let push = (radiusSum - dist) / totalInverseMass;
    *pa = vec4f((*pa).xyz + normal * push * (*va).w, (*pa).w);
    pb = vec4f(pb.xyz - normal * push * vb.w, pb.w);
    sortedSpheres[m] = pb;
    return 1u;
}

// Cells come in 27 colours, 3 apart on every axis, and one colour runs at a time. Each thread takes one cell and
// only touches it and its neighbours, which no other thread touches. A pair belongs to its earlier ball.
@compute @workgroup_size(256)
fn resolveColor(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let dims = params.dims;
    let perColor = (dims - params.colorOffset + vec3i(2)) / 3;
    let t = i32(threadIndex(id, groups));
    if (t >= perColor.x * perColor.y * perColor.z)
    {
        return;
    }
    let c = params.colorOffset + 3 * vec3i(t % perColor.x, (t / perColor.x) % perColor.y, t / (perColor.x * perColor.y));
    let cell = cellIndex(c);
    let cellBegin = atomicLoad(&cellStart[cell]);
    let cellEnd = atomicLoad(&cellStart[cell + 1]);
    if (cellBegin == cellEnd)
    {
        return;
    }

    var count = 0u;
    let x0 = max(c.x - 1, 0);
    let x1 = min(c.x + 1, dims.x - 1);
    for (var k = cellBegin; k < cellEnd; k++)
    {
        var pa = sortedSpheres[k];
        var va = sortedVelocities[k];
        // Each row of 3 neighbouring cells along x is one run of balls
        for (var z = max(c.z - 1, 0); z <= min(c.z + 1, dims.z - 1); z++)
        {
            for (var y = max(c.y - 1, 0); y <= min(c.y + 1, dims.y - 1); y++)
            {
                let row = dims.x * (y + dims.y * z);
                let begin = max(atomicLoad(&cellStart[row + x0]), k + 1u);
                let end = atomicLoad(&cellStart[row + x1 + 1]);
                for (var m = begin; m < end; m++)
                {
                    count += resolvePair(&pa, &va, m);
                }
            }
        }
        sortedSpheres[k] = pa;
        sortedVelocities[k] = va;
    }
    if (params.countContacts != 0u && count > 0u)
    {
        atomicAdd(&stats.ballContacts, count);
    }
}

// Each ball against the 6 walls of the box
@compute @workgroup_size(256)
fn walls(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let k = threadIndex(id, groups);
    if (k >= params.ballCount)
    {
        return;
    }
    var p = sortedSpheres[k];
    var v = sortedVelocities[k];
    var count = 0u;
    for (var wall = 0; wall < 6; wall++)
    {
        var normal = vec3f(0.0);
        normal[wall / 2] = select(-1.0, 1.0, wall % 2 == 0);
        let dist = dot(normal, p.xyz) + params.boxHalfSize - p.w;
        if (dist >= 0.0)
        {
            continue;
        }
        count++;
        if (v.w <= 0.0)
        {
            continue;
        }
        let separatingVelocity = dot(v.xyz, normal);
        if (separatingVelocity < 0.0)
        {
            v = vec4f(v.xyz + normal * (-(1.0 + params.restitution) * separatingVelocity), v.w);
        }
        p = vec4f(p.xyz - normal * dist, p.w);
    }
    sortedSpheres[k] = p;
    sortedVelocities[k] = v;
    if (params.countContacts != 0u && count > 0u)
    {
        atomicAdd(&stats.wallContacts, count);
    }
}

// Puts every ball back at its own index
@compute @workgroup_size(256)
fn writeBack(@builtin(global_invocation_id) id: vec3u, @builtin(num_workgroups) groups: vec3u)
{
    let k = threadIndex(id, groups);
    if (k >= params.ballCount)
    {
        return;
    }
    let i = sortedIndex[k];
    spheres[i] = sortedSpheres[k];
    velocities[i] = sortedVelocities[k];
}
)WGSL";
}

namespace gpu
{
    const std::string& getBallShaderSource()
    {
        static const std::string source = std::string(SHADER_COMMON) + SHADER_CONTACTS;
        return source;
    }
}
