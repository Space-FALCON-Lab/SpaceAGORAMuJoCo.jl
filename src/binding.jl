"""
The only file in this package that talks to libmujoco.

It loads MuJoCo 3.11.0 from the package's lazy artifact (official release tarball, checksum pinned in
`Artifacts.toml`; on macOS arm64 an artifact built from the official disk image pinned in
`NativeLibrary.toml`), resolves the C entry points it needs, and exposes raw-pointer field access at byte
offsets taken from the 3.11.0 headers with `offsetof` (`mujoco/mjmodel.h`, `mjdata.h`, `mjoption.h`).
Those offsets are valid for exactly that version; `load!` refuses any other `mj_version()`. Moving to
another MuJoCo release means regenerating `OFFSETS` and nothing else.

Pointers are plain `Ptr{Cvoid}`; ownership lives in `MjModel` and `MjData`, freed by `close` or, failing
that, by their finalizers. Every call below reads the pointer through `_ptr`, which throws once the owner
is closed, so no call reaches MuJoCo with a freed handle.
"""
module Binding

using Libdl
using Artifacts
using LazyArtifacts
import Downloads, Pkg, SHA, TOML   # macOS artifact build only; LazyArtifacts already loads Pkg

const MUJOCO_VERSION = 3_011_000

# offsetof() values from the 3.11.0 headers (mjtNum = double, mjtBool = unsigned char), verified identical
# on Linux x86_64 and macOS arm64.
const OFFSETS = (
    model = (nq = 0, nv = 8, nu = 16, na = 40, nbody = 48, njnt = 88, neq = 472,
        body_rootid = 1816, body_jntnum = 1840, body_jntadr = 1848, body_dofnum = 1856,
        jnt_type = 2104, jnt_qposadr = 2112, jnt_dofadr = 2120,
        body_mass = 1944, body_inertia = 1960, ngeom = 136, geom_bodyid = 2448, opt = 792),
    opt = (timestep = 0, gravity = 56, integrator = 248),
    data = (narena = 0, maxuse_arena = 72,
        warning = 160264, time = 160640, qpos = 160680, qvel = 160688, act = 160696, ctrl = 160728,
        xfrc_applied = 160744, eq_active = 160752, xquat = 160840, xipos = 160856, ximat = 160864,
        ncon = 160560, nefc = 160576, contact = 161424,
        qacc = 160776, qacc_smooth = 161376, qfrc_constraint = 161384,
        efc_J = 161480, efc_R = 161536, efc_aref = 161824, efc_force = 161952,
        efc_vel = 161816, efc_pos = 161488, efc_diagA = 161512),
    contact = (dist = 0, pos = 8, frame = 32, dim = 528, geom = 540, efc_address = 576),
)
const SIZEOF_MJCONTACT = 584
# mjWarningStat is {int lastinfo; int number}; mjtWarning enum values from mujoco/mjtype.h, mjNWARNING = 7.
const WARNING_STRIDE = 8
const WARNING_NUMBER = 4
const WARN_BADQPOS = 3
const WARN_BADQVEL = 4
const WARN_BADQACC = 5
const SIZEOF_MJMODEL = 5672
const SIZEOF_MJDATA = 161976
const SIZEOF_MJOPTION = 304

# enums (mjtIntegrator, mjtObj, mjtJoint, mjtState)
const INT_EULER = Cint(0)
const INT_RK4 = Cint(1)
const INT_IMPLICIT = Cint(2)
const INT_IMPLICITFAST = Cint(3)
const OBJ_BODY = Cint(1)
const JNT_FREE = Cint(0)
const STATE_INTEGRATION = Cint(1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 13 |            # FULLPHYSICS
    1 << 6 | 1 << 7 | 1 << 8 | 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 5)                    # USER + WARMSTART

const FUNCTIONS = (:mj_version, :mj_loadXML, :mj_parseXMLString, :mj_compile, :mjs_getError, :mj_deleteSpec,
    :mj_makeData, :mj_deleteData, :mj_deleteModel, :mj_copyModel, :mj_step1, :mj_step2, :mj_forward,
    :mj_resetData, :mj_stateSize, :mj_getState, :mj_setState, :mj_name2id, :mj_id2name, :mj_objectVelocity, :mj_getTotalmass,
    :mj_contactForce, :mj_isSparse)

struct Api
    handle::Ptr{Cvoid}
    fn::NamedTuple{FUNCTIONS, NTuple{length(FUNCTIONS), Ptr{Cvoid}}}
end

const _API = Ref{Union{Nothing, Api}}(nothing)   # the loaded library, process-wide by nature
const _API_LOCK = ReentrantLock()

include("macos_artifact.jl")   # NativeLibrary.toml lookup and the macOS arm64 artifact built from the official dmg

"""Path of `libmujoco`; throws on platforms without a pinned official asset."""
function library_path()
    if Sys.islinux() && Sys.ARCH === :x86_64
        root = artifact"mujoco_3_11_0_linux_x86_64"
        for dir in (joinpath(root, "lib"), joinpath(root, "mujoco-3.11.0", "lib"))
            isdir(dir) || continue
            for name in readdir(dir)
                startswith(name, "libmujoco.so.") && return joinpath(dir, name)
            end
        end
        error("libmujoco.so.* not found under the artifact at $root")
    elseif Sys.isapple() && Sys.ARCH === :aarch64
        spec = native_platform()
        root = _macos_artifact_root(spec)
        lib = joinpath(root, split(spec["library"], '/')...)
        isfile(lib) || error("$(basename(lib)) not found under the artifact at $root")
        return lib
    else
        _unsupported_platform()
    end
end

function _load()
    handle = Libdl.dlopen(library_path(), Libdl.RTLD_LOCAL | Libdl.RTLD_NOW)
    fn = NamedTuple{FUNCTIONS}(Tuple(Libdl.dlsym(handle, f) for f in FUNCTIONS))
    version = ccall(fn.mj_version, Cint, ())
    version == MUJOCO_VERSION || error("SpaceAGORAMuJoCo needs MuJoCo $MUJOCO_VERSION, loaded $version")
    return Api(handle, fn)
end

function api()::Api
    a = _API[]
    a === nothing || return a
    lock(_API_LOCK) do
        _API[] === nothing && (_API[] = _load())
        return _API[]::Api
    end
end

version() = Int(ccall(api().fn.mj_version, Cint, ()))

"""
Make sure this platform's native library is installed and loads, and return its `mj_version()`. Throws
the unsupported-platform error where `NativeLibrary.toml` has no table. `scripts/setup_env.jl` calls
this at setup on macOS, and on Linux with `--prefetch-native`; otherwise the first native call does the same.
"""
function ensure_native!()
    native_platform()
    return version()
end

# --- native memory owners ---------------------------------------------------------------------------

mutable struct MjModel
    ptr::Ptr{Cvoid}
    function MjModel(ptr::Ptr{Cvoid})
        ptr == C_NULL && error("MuJoCo returned a null model")
        return finalizer(_free!, new(ptr))
    end
end

mutable struct MjData
    ptr::Ptr{Cvoid}
    function MjData(model::MjModel)
        ptr = ccall(api().fn.mj_makeData, Ptr{Cvoid}, (Ptr{Cvoid},), _ptr(model))
        ptr == C_NULL && error("mj_makeData failed")
        return finalizer(_free!, new(ptr))
    end
end

# Shared by `close` and the finalizer: free once, then keep a null pointer so the other one does nothing.
function _free!(m::MjModel)
    m.ptr == C_NULL || ccall(api().fn.mj_deleteModel, Cvoid, (Ptr{Cvoid},), m.ptr)
    m.ptr = C_NULL
    return nothing
end
function _free!(d::MjData)
    d.ptr == C_NULL || ccall(api().fn.mj_deleteData, Cvoid, (Ptr{Cvoid},), d.ptr)
    d.ptr = C_NULL
    return nothing
end

"""Free the native object now instead of at garbage collection. Idempotent; later calls on it throw."""
Base.close(x::Union{MjModel, MjData}) = _free!(x)
Base.isopen(x::Union{MjModel, MjData}) = x.ptr != C_NULL

@noinline _closed(x) = throw(InvalidStateException("the MuJoCo $(x isa MjModel ? "model" : "data") is closed", :closed))
@inline function _ptr(x::Union{MjModel, MjData})
    p = x.ptr
    p == C_NULL && _closed(x)
    return p
end

# --- loading ----------------------------------------------------------------------------------------

function load_xml_file(path::AbstractString)
    err = zeros(UInt8, 1024)
    ptr = GC.@preserve err ccall(api().fn.mj_loadXML, Ptr{Cvoid}, (Cstring, Ptr{Cvoid}, Ptr{UInt8}, Cint), path, C_NULL, err, 1024)
    ptr == C_NULL && error("mj_loadXML failed: ", unsafe_string(pointer(err)))
    return MjModel(ptr)
end

function load_xml_string(xml::AbstractString)
    f = api().fn
    err = zeros(UInt8, 1024)
    spec = GC.@preserve err ccall(f.mj_parseXMLString, Ptr{Cvoid}, (Cstring, Ptr{Cvoid}, Ptr{UInt8}, Cint), xml, C_NULL, err, 1024)
    spec == C_NULL && error("mj_parseXMLString failed: ", unsafe_string(pointer(err)))
    try
        ptr = ccall(f.mj_compile, Ptr{Cvoid}, (Ptr{Cvoid}, Ptr{Cvoid}), spec, C_NULL)
        ptr == C_NULL && error("mj_compile failed: ", unsafe_string(ccall(f.mjs_getError, Cstring, (Ptr{Cvoid},), spec)))
        return MjModel(ptr)
    finally
        ccall(f.mj_deleteSpec, Cvoid, (Ptr{Cvoid},), spec)
    end
end

copy_model(model::MjModel) = MjModel(ccall(api().fn.mj_copyModel, Ptr{Cvoid}, (Ptr{Cvoid}, Ptr{Cvoid}), C_NULL, _ptr(model)))

# --- stepping and state -----------------------------------------------------------------------------

step1!(m::MjModel, d::MjData) = ccall(api().fn.mj_step1, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), _ptr(m), _ptr(d))
step2!(m::MjModel, d::MjData) = ccall(api().fn.mj_step2, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), _ptr(m), _ptr(d))
forward!(m::MjModel, d::MjData) = ccall(api().fn.mj_forward, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), _ptr(m), _ptr(d))
reset_data!(m::MjModel, d::MjData) = ccall(api().fn.mj_resetData, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), _ptr(m), _ptr(d))
state_size(m::MjModel, sig::Integer=STATE_INTEGRATION) = Int(ccall(api().fn.mj_stateSize, Cint, (Ptr{Cvoid}, Cint), _ptr(m), sig))
get_state!(buf::Vector{Float64}, m::MjModel, d::MjData, sig::Integer=STATE_INTEGRATION) =
    (ccall(api().fn.mj_getState, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}, Ptr{Float64}, Cint), _ptr(m), _ptr(d), buf, sig); buf)
set_state!(m::MjModel, d::MjData, buf::Vector{Float64}, sig::Integer=STATE_INTEGRATION) =
    (ccall(api().fn.mj_setState, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}, Ptr{Float64}, Cint), _ptr(m), _ptr(d), buf, sig); nothing)
name2id(m::MjModel, objtype::Integer, name::AbstractString) =
    Int(ccall(api().fn.mj_name2id, Cint, (Ptr{Cvoid}, Cint, Cstring), _ptr(m), objtype, name))
function id2name(m::MjModel, objtype::Integer, id::Integer)
    p = ccall(api().fn.mj_id2name, Cstring, (Ptr{Cvoid}, Cint, Cint), _ptr(m), objtype, id)
    return p == C_NULL ? "" : unsafe_string(p)
end
total_mass(m::MjModel) = ccall(api().fn.mj_getTotalmass, Float64, (Ptr{Cvoid},), _ptr(m))

"""Copy the contact-frame force/torque for a one-based contact index into six entries."""
function contact_force!(res::Vector{Float64}, m::MjModel, d::MjData, index::Integer)
    length(res) == 6 || throw(DimensionMismatch("contact force buffer must have six entries"))
    1 <= index <= ncon(d) || throw(BoundsError(1:ncon(d), index))
    ccall(api().fn.mj_contactForce, Cvoid,
        (Ptr{Cvoid}, Ptr{Cvoid}, Cint, Ptr{Float64}), _ptr(m), _ptr(d), index - 1, res)
    return res
end

"""6D velocity (angular; linear) of a body about its center of mass, in world axes. Needs fresh `cvel`."""
function body_velocity!(res::Vector{Float64}, m::MjModel, d::MjData, body::Integer)
    ccall(api().fn.mj_objectVelocity, Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}, Cint, Cint, Ptr{Float64}, Cint), _ptr(m), _ptr(d), OBJ_BODY, body, res, 0)
    return res
end

# --- field access -----------------------------------------------------------------------------------

@inline _int(m::MjModel, off) = Int(unsafe_load(Ptr{Cint}(_ptr(m) + off)))
@inline _arr(base::Ptr{Cvoid}, off, ::Type{T}, n) where {T} =
    n == 0 ? T[] : unsafe_wrap(Array, unsafe_load(Ptr{Ptr{T}}(base + off)), n; own = false)

nq(m::MjModel) = _int(m, OFFSETS.model.nq)
nv(m::MjModel) = _int(m, OFFSETS.model.nv)
nu(m::MjModel) = _int(m, OFFSETS.model.nu)
na(m::MjModel) = _int(m, OFFSETS.model.na)
nbody(m::MjModel) = _int(m, OFFSETS.model.nbody)
ngeom(m::MjModel) = _int(m, OFFSETS.model.ngeom)
geom_bodyid(m::MjModel) = _arr(_ptr(m), OFFSETS.model.geom_bodyid, Cint, ngeom(m))
njnt(m::MjModel) = _int(m, OFFSETS.model.njnt)
neq(m::MjModel) = _int(m, OFFSETS.model.neq)

body_mass(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_mass, Float64, nbody(m))
body_inertia(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_inertia, Float64, 3 * nbody(m))
body_rootid(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_rootid, Cint, nbody(m))
body_jntnum(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_jntnum, Cint, nbody(m))
body_jntadr(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_jntadr, Cint, nbody(m))
body_dofnum(m::MjModel) = _arr(_ptr(m), OFFSETS.model.body_dofnum, Cint, nbody(m))
jnt_type(m::MjModel) = _arr(_ptr(m), OFFSETS.model.jnt_type, Cint, njnt(m))
jnt_qposadr(m::MjModel) = _arr(_ptr(m), OFFSETS.model.jnt_qposadr, Cint, njnt(m))
jnt_dofadr(m::MjModel) = _arr(_ptr(m), OFFSETS.model.jnt_dofadr, Cint, njnt(m))

timestep(m::MjModel) = unsafe_load(Ptr{Float64}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.timestep))
set_timestep!(m::MjModel, dt::Float64) = unsafe_store!(Ptr{Float64}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.timestep), dt)
integrator(m::MjModel) = Int(unsafe_load(Ptr{Cint}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.integrator)))
set_integrator!(m::MjModel, i::Integer) = unsafe_store!(Ptr{Cint}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.integrator), Cint(i))
gravity(m::MjModel) = ntuple(i -> unsafe_load(Ptr{Float64}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.gravity), i), 3)
set_gravity!(m::MjModel, g) = for i in 1:3
    unsafe_store!(Ptr{Float64}(_ptr(m) + OFFSETS.model.opt + OFFSETS.opt.gravity), Float64(g[i]), i)
end

data_time(d::MjData) = unsafe_load(Ptr{Float64}(_ptr(d) + OFFSETS.data.time))
set_data_time!(d::MjData, t::Float64) = unsafe_store!(Ptr{Float64}(_ptr(d) + OFFSETS.data.time), t)
ncon(d::MjData) = Int(unsafe_load(Ptr{Cint}(_ptr(d) + OFFSETS.data.ncon)))
nefc(d::MjData) = Int(unsafe_load(Ptr{Cint}(_ptr(d) + OFFSETS.data.nefc)))
qacc(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.qacc, Float64, nv(m))
qacc_smooth(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.qacc_smooth, Float64, nv(m))
qfrc_constraint(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.qfrc_constraint, Float64, nv(m))
efc_R(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_R, Float64, nefc(d))
efc_aref(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_aref, Float64, nefc(d))
efc_force(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_force, Float64, nefc(d))
efc_vel(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_vel, Float64, nefc(d))
efc_pos(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_pos, Float64, nefc(d))
efc_diagA(d::MjData) = _arr(_ptr(d), OFFSETS.data.efc_diagA, Float64, nefc(d))

"""Dense constraint Jacobian view, with rows as constraints; refuse sparse models."""
function constraint_jacobian(m::MjModel, d::MjData)
    ccall(api().fn.mj_isSparse, Cint, (Ptr{Cvoid},), _ptr(m)) == 0 ||
        throw(ArgumentError("constraint_jacobian requires a dense native Jacobian"))
    return transpose(reshape(_arr(_ptr(d), OFFSETS.data.efc_J, Float64, nefc(d) * nv(m)), nv(m), nefc(d)))
end

"""Copy contact geometry. Geom IDs and efc address remain native zero-based values.

Frame is row-major: the first three entries are the normal from geom 1 to geom 2.
Copies remain valid across subsequent native queries; no arena pointer escapes.
"""
function contact(d::MjData, index::Integer)
    1 <= index <= ncon(d) || throw(BoundsError(1:ncon(d), index))
    base = unsafe_load(Ptr{Ptr{UInt8}}(_ptr(d) + OFFSETS.data.contact)) + (index-1)*SIZEOF_MJCONTACT
    o = OFFSETS.contact
    floats(off, n) = ntuple(i -> unsafe_load(Ptr{Float64}(base + off), i), n)
    ints(off, n) = ntuple(i -> Int(unsafe_load(Ptr{Cint}(base + off), i)), n)
    return (dist=only(floats(o.dist,1)), pos=floats(o.pos,3), frame=floats(o.frame,9),
        dim=only(ints(o.dim,1)), geom=ints(o.geom,2), efc_address=only(ints(o.efc_address,1)))
end
qpos(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.qpos, Float64, nq(m))
qvel(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.qvel, Float64, nv(m))
ctrl(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.ctrl, Float64, nu(m))
xfrc_applied(m::MjModel, d::MjData) = unsafe_wrap(Array, unsafe_load(Ptr{Ptr{Float64}}(_ptr(d) + OFFSETS.data.xfrc_applied)), (6, nbody(m)); own = false)
xipos(m::MjModel, d::MjData) = unsafe_wrap(Array, unsafe_load(Ptr{Ptr{Float64}}(_ptr(d) + OFFSETS.data.xipos)), (3, nbody(m)); own = false)
xquat(m::MjModel, d::MjData) = unsafe_wrap(Array, unsafe_load(Ptr{Ptr{Float64}}(_ptr(d) + OFFSETS.data.xquat)), (4, nbody(m)); own = false)
"""Times MuJoCo raised warning `w` (a `WARN_*` index) since the counter was last cleared."""
warning_count(d::MjData, w::Integer) = Int(unsafe_load(Ptr{Cint}(_ptr(d) + OFFSETS.data.warning + WARNING_STRIDE * w + WARNING_NUMBER)))
"""Body orientation matrices (body to world), row-major 3x3 per body: a `9 x nbody` array."""
ximat(m::MjModel, d::MjData) = unsafe_wrap(Array, unsafe_load(Ptr{Ptr{Float64}}(_ptr(d) + OFFSETS.data.ximat)), (9, nbody(m)); own = false)
eq_active(m::MjModel, d::MjData) = _arr(_ptr(d), OFFSETS.data.eq_active, UInt8, neq(m))
"""Size of the data's arena (stack plus contact and constraint arrays) in bytes, fixed when the data is made."""
arena_size(d::MjData) = Int(unsafe_load(Ptr{Int64}(_ptr(d) + OFFSETS.data.narena)))
"""High-water mark of the whole arena in bytes since the data was made or last reset."""
arena_peak(d::MjData) = Int(unsafe_load(Ptr{Int64}(_ptr(d) + OFFSETS.data.maxuse_arena)))

end # module Binding
