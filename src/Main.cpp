#include <RED4ext/RED4ext.hpp>
#include <RED4ext/RTTITypes.hpp>
#include <RED4ext/Scripting/Natives/vehicleBaseObject.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysicsData.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysics.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static bool is_enable_original_physics = true;
static RED4ext::Handle<RED4ext::vehicle::BaseObject> g_fly_vehicle;
static uint64_t g_fly_vehicle_entity_id_hash = 0;

static const RED4ext::Sdk* g_sdk = nullptr;
static RED4ext::PluginHandle g_plugin_handle = 0;
static uint32_t g_resolve_attempts = 0;

// Vehicle pointer published by the physics hook; the vehicle stores a serial-less EntityID, so the
// hook matches on the local id only and hands the object to the game thread this way.
static std::atomic<uint64_t> g_fly_vehicle_raw{0};
static std::atomic<float> g_native_mass{0.0f};

// --- Event-driven command state. Lua pushes (action, value) on press/release/axis-change and the DLL
// holds it, so the control loop no longer re-sends the whole list at 20 Hz. Index = Def.ActionList + 1
// (Idle = -1). Shadow mode first: compared against the list SetFlightModel pushes.
static std::atomic<float> g_cmd_val[64];
static std::atomic<uint32_t> g_cmd_seq{0};

// Phase (input): one command edge. id = Def.ActionList value, value = 0/1 or analog magnitude.
void SetNativeCommand(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    int32_t id = 0;
    float value = 0.0f;
    RED4ext::GetParameter(aFrame, &id);
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    // Nothing (0) means "no input": clear the whole held state. Used for dismount/menu/abort resets.
    if (id == 0)
    {
        for (int i = 0; i < 64; ++i)
            g_cmd_val[i].store(0.0f, std::memory_order_relaxed);
        g_cmd_seq.fetch_add(1, std::memory_order_relaxed);
        *aOut = 1.0f;
        return;
    }
    const int idx = static_cast<int>(id) + 1;
    if (idx >= 0 && idx < 64)
    {
        g_cmd_val[idx].store(value, std::memory_order_relaxed);
        g_cmd_seq.fetch_add(1, std::memory_order_relaxed);
        *aOut = 1.0f;
        return;
    }
    *aOut = 0.0f; // out of range: Lua should not send this
}

// Build the command list from the event-pushed held state. Nothing (0) and Idle (-1) are never stored,
// so an empty table yields an empty list - the same no-op the pushed Nothing entry produced.
static int BuildHeldCommands(int* aActions, float* aValues, int aMax)
{
    int n = 0;
    for (int i = 0; i < 64 && n < aMax; ++i)
    {
        const float v = g_cmd_val[i].load(std::memory_order_relaxed);
        if (v == 0.0f)
            continue;
        aActions[n] = i - 1;
        aValues[n] = v;
        ++n;
    }
    return n;
}

// --- Stage 0 (self-supplied attitude): can the physics thread read the entity transform itself?
// Offsets come from RTTI property names, so no relocation hash is involved. Resolution runs on the
// game thread; the physics hook only reads the resolved offset.
static std::atomic<int32_t> g_att_offset{-1};
static std::atomic<int32_t> g_wpos_offset{-1};

static void ResolveEntityTransformOffsets()
{
    if (g_att_offset.load(std::memory_order_relaxed) >= 0)
        return;
    auto rtti = RED4ext::CRTTISystem::Get();
    if (rtti == nullptr)
        return;
    int32_t att = -1;
    int32_t pos = -1;
    int found_classes = 0;
    uint32_t last_props = 0;
    const char* candidates[] = {"worldObject", "worldEntity", "gameObject"};
    for (const char* cname : candidates)
    {
        auto cls = rtti->GetClass(cname);
        if (cls == nullptr)
            continue;
        ++found_classes;
        RED4ext::DynArray<RED4ext::CProperty*> props;
        cls->GetProperties(props);
        last_props = static_cast<uint32_t>(props.size);
        for (uint32_t i = 0; i < props.size; ++i)
        {
            const char* nm = props[i]->name.ToString();
            if (nm == nullptr)
                continue;
            if (att < 0 && std::strcmp(nm, "worldOrientation") == 0)
                att = static_cast<int32_t>(props[i]->valueOffset);
            else if (pos < 0 && std::strcmp(nm, "worldPosition") == 0)
                pos = static_cast<int32_t>(props[i]->valueOffset);
        }
        if (att >= 0)
            break;
    }
    g_att_offset.store(att, std::memory_order_relaxed);
    g_wpos_offset.store(pos, std::memory_order_relaxed);
    if (g_sdk)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "DAV dbg: transform offsets orientation=%d position=%d classes=%d props=%u", att,
                      pos, found_classes, last_props);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
}

// vehicle::BaseObject stores the entity transform directly (see vehicleBaseObject.hpp):
//   physics @0x2C8, physicsData @0x2D0, curveSetData @0x2D8, chassis @0x2E8, unk2F8[16] @0x2F8,
//   chassisType @0x338, worldTransform @0x340 (WorldTransform = Position @0x00, Orientation @0x10),
//   unk360 @0x360 -- the arithmetic closes, so the offsets are not guesses.
// The game writes this same field in vehicleBaseObject sub_280 ("applies worldTransform to ... this->worldTransform").
constexpr int32_t kVehicleWorldTransform = 0x340;
constexpr int32_t kVehicleWorldOrientation = 0x350;

// Use the fixed offsets when RTTI has no such property (it does not: these are C++-only members).
static void UseFixedTransformOffsets()
{
    if (g_att_offset.load(std::memory_order_relaxed) >= 0)
        return;
    g_att_offset.store(kVehicleWorldOrientation, std::memory_order_relaxed);
    g_wpos_offset.store(kVehicleWorldTransform, std::memory_order_relaxed);
    if (g_sdk)
        g_sdk->logger->Info(g_plugin_handle, "DAV dbg: transform offsets from vehicleBaseObject: orientation=0x350 position=0x340");
}

// Stage 0b: RTTI does not expose the entity transform, so locate the orientation field empirically
// by matching the quaternion CET pushes. Runs a few times per session, then stops.
static int ScanForQuaternion(const void* aBase, const float* aWant, int aMaxBytes)
{
    const auto* f = reinterpret_cast<const float*>(aBase);
    const int words = aMaxBytes / 4;
    float best = 0.0f;
    int best_off = -1;
    for (int i = 0; i + 3 < words; ++i)
    {
        const float* c = f + i;
        const float n2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + c[3] * c[3];
        if (!(n2 > 0.9f && n2 < 1.1f)) // unit-norm candidates only
            continue;
        float dot = c[0] * aWant[0] + c[1] * aWant[1] + c[2] * aWant[2] + c[3] * aWant[3];
        if (dot < 0.0f)
            dot = -dot; // q and -q are the same rotation
        if (dot > best)
        {
            best = dot;
            best_off = i * 4;
        }
    }
    return (best > 0.999f) ? best_off : -1;
}

static std::atomic<float> g_native_gravity{1.0f};

// Desired gravity state; the hook applies it every tick because a one-shot write can be dropped.
static std::atomic<bool> g_gravity_enable{true};

// Does this vehicle object belong to the AV we drive? Full id first, then local-only id.
static inline bool IsFlyVehicle(const RED4ext::vehicle::BaseObject* aObject)
{
    if (!aObject)
    {
        return false;
    }
    const uint64_t hash = aObject->entityID.hash;
    if (hash == 0 || g_fly_vehicle_entity_id_hash == 0)
    {
        return false;
    }
    if (hash == g_fly_vehicle_entity_id_hash)
    {
        return true;
    }
    // serial-less id from the vehicle side: compare the local half only.
    return (uint32_t)hash == (uint32_t)g_fly_vehicle_entity_id_hash;
}

// Resolve an entity by id hash through the script VM.
// Arguments are built from FindEntityByID's declared parameters: passing the ScriptGameInstance
// wrapper when it is not a parameter silently types it as the EntityID and yields an empty handle.
static bool FindVehicleByHash(uint64_t aHash, RED4ext::Handle<RED4ext::vehicle::BaseObject>& aOut, const char** aWhy)
{
    auto rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
    {
        *aWhy = "rtti missing";
        return false;
    }
    auto cls = rtti->GetClass("ScriptGameInstance");
    if (!cls)
    {
        *aWhy = "ScriptGameInstance missing";
        return false;
    }
    auto func = cls->GetFunction("FindEntityByID");
    if (!func)
    {
        *aWhy = "FindEntityByID missing";
        return false;
    }

    static bool s_signature_logged = false;
    if (!s_signature_logged && g_sdk)
    {
        s_signature_logged = true;
        char buf[192];
        std::snprintf(buf, sizeof(buf), "DAV dbg: FindEntityByID params=%u ret=%s", func->params.size,
                      (func->returnType && func->returnType->type) ? func->returnType->type->GetName().ToString()
                                                                   : "?");
        g_sdk->logger->Info(g_plugin_handle, buf);
    }

    RED4ext::ScriptGameInstance gameInstance;
    RED4ext::EntityID entity_id;
    entity_id.hash = aHash;

    RED4ext::StackArgs_t args;
    for (uint32_t i = 0; i < func->params.size; ++i)
    {
        auto* prop = func->params[i];
        const RED4ext::CName type_name = (prop && prop->type) ? prop->type->GetName() : RED4ext::CName();
        if (type_name == "ScriptGameInstance")
        {
            args.emplace_back(nullptr, &gameInstance);
        }
        else if (type_name == "EntityID" || type_name == "entEntityID")
        {
            args.emplace_back(nullptr, &entity_id);
        }
        else
        {
            *aWhy = "unexpected signature";
            if (g_sdk)
            {
                char buf[192];
                std::snprintf(buf, sizeof(buf), "DAV dbg: FindEntityByID param %u type=%s", i,
                              (prop && prop->type) ? prop->type->GetName().ToString() : "?");
                g_sdk->logger->Error(g_plugin_handle, buf);
            }
            return false;
        }
    }

    RED4ext::Handle<RED4ext::vehicle::BaseObject> handle;
    const bool ok = RED4ext::ExecuteFunction(cls, func, &handle, args);
    if (!ok || !handle)
    {
        *aWhy = ok ? "empty handle" : "execute failed";
        return false;
    }

    // Callers write physicsData at a fixed offset, so a wrong-typed entity would corrupt memory.
    auto vehicle_cls = rtti->GetClass("vehicleBaseObject");
    auto obj_cls = handle.instance ? handle.instance->GetType() : nullptr;
    if (vehicle_cls != nullptr && obj_cls != nullptr && !obj_cls->IsA(vehicle_cls))
    {
        *aWhy = "resolved entity is not a vehicle";
        return false;
    }

    aOut = handle;
    *aWhy = "ok";
    return true;
}

// Resolve the fly vehicle by entity id; every failure silently kills the whole native API, so it
// is retried lazily from game-thread calls and logged.
static bool TryResolveVehicle(const char* aReason)
{
    if (g_fly_vehicle)
    {
        return true;
    }
    // Adopt the pointer the hook published: the only path that works before FindEntityByID can.
    const uint64_t raw = g_fly_vehicle_raw.load(std::memory_order_acquire);
    if (raw != 0)
    {
        g_fly_vehicle = RED4ext::Handle<RED4ext::vehicle::BaseObject>(
            reinterpret_cast<RED4ext::vehicle::BaseObject*>(raw));
        if (g_sdk)
        {
            g_sdk->logger->Info(g_plugin_handle, "DAV dbg: vehicle adopted from physics hook");
        }
        return true;
    }
    if (g_fly_vehicle_entity_id_hash == 0)
    {
        return false;
    }

    // Called from every game-thread entry point; throttle so a missing entity is not a lookup per call.
    if ((g_resolve_attempts++ & 0x0F) != 0)
    {
        return false;
    }

    const uint64_t full_hash = g_fly_vehicle_entity_id_hash;
    const char* why = "?";
    RED4ext::Handle<RED4ext::vehicle::BaseObject> handle;
    bool found = FindVehicleByHash(full_hash, handle, &why);

    // The vehicle stores its id without the serial; if the registry keys on that, only local resolves.
    const uint64_t local_only = static_cast<uint64_t>(static_cast<uint32_t>(full_hash));
    if (!found && local_only != full_hash)
    {
        found = FindVehicleByHash(local_only, handle, &why);
        if (found && g_sdk)
        {
            g_sdk->logger->Info(g_plugin_handle, "DAV dbg: vehicle found via local-only id (serial dropped)");
        }
    }

    if (!found || !handle)
    {
        if (g_sdk && g_resolve_attempts <= 32)
        {
            char buf[192];
            std::snprintf(buf, sizeof(buf), "DAV dbg: FindEntityByID miss (%s) hash=%llu", why,
                          (unsigned long long)full_hash);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
        return false;
    }

    g_resolve_attempts = 0;
    g_fly_vehicle = handle;
    if (g_sdk)
    {
        char buf[192];
        const char* cls_name =
            (g_fly_vehicle.instance && g_fly_vehicle.instance->GetType())
                ? g_fly_vehicle.instance->GetType()->GetName().ToString()
                : "?";
        std::snprintf(buf, sizeof(buf), "DAV dbg: vehicle resolved by '%s' hash=%llu class=%s physdata=%d mass=%.0f",
                      aReason ? aReason : "?", (unsigned long long)full_hash, cls_name,
                      (g_fly_vehicle.instance && g_fly_vehicle->physicsData) ? 1 : 0,
                      (g_fly_vehicle.instance && g_fly_vehicle->physicsData) ? g_fly_vehicle->physicsData->total_mass
                                                                             : -1.0f);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
    return true;
}

//////////////////////////////////////////////////////////////////////
// In this area, class names and macro names have been changed based on Jackhambert's code.
// https://github.com/jackhumbert/let_there_be_flight/blob/main/src/red4ext/Utils/FlightModule.hpp
////////

#ifdef FLY_TANK_MOD
struct FlyTankModule
#else
struct DAVModule
#endif
{
    virtual void Load(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) {};
    virtual void RegisterTypes() {};
    virtual void PostRegisterTypes() {};
    virtual void Unload(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) {};
};
#ifdef FLY_TANK_MOD
class IFlyTankModuleHook : FlyTankModule
#else
class IDAVModuleHook : DAVModule
#endif
{
public:
    std::string m_name;
    uintptr_t m_address;
    void* m_hook;
    void** m_original;

    virtual void Load(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) override
    {
        while (!aSdk->hooking->Attach(aHandle, RED4EXT_OFFSET_TO_ADDR(this->m_address), this->m_hook, this->m_original))
        {
            aSdk->logger->Trace(aHandle, "try to hooking again");
        }
    };

    virtual void Unload(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) override
    {
        aSdk->hooking->Detach(aHandle, RED4EXT_OFFSET_TO_ADDR(this->m_address));
    };
};
#ifdef FLY_TANK_MOD
class FlyTankModuleFactory
#else
class DAVModuleFactory
#endif
{
    std::vector<std::function<void(const RED4ext::Sdk*, RED4ext::PluginHandle)>> s_loads;
    std::vector<std::function<void(const RED4ext::Sdk*, RED4ext::PluginHandle)>> s_unloads;
    std::vector<std::function<void()>> s_registers;
    std::vector<std::function<void()>> s_postRegisters;
#ifdef FLY_TANK_MOD
    std::vector<IFlyTankModuleHook*> s_hooks;
public:
    static FlyTankModuleFactory& GetInstance()
    {
        static FlyTankModuleFactory s_instance;
#else
    std::vector<IDAVModuleHook*> s_hooks;
public:
    static DAVModuleFactory& GetInstance()
    {
        static DAVModuleFactory s_instance;
#endif
        return s_instance;
    }
    template<class T>
    void registerClass(const std::string& name)
    {
        s_loads.emplace_back([](const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) -> void
                             { (new T())->Load(aSdk, aHandle); });
        s_unloads.emplace_back([](const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle) -> void
                               { (new T())->Unload(aSdk, aHandle); });
        s_registers.emplace_back([]() -> void { (new T())->RegisterTypes(); });
        s_postRegisters.emplace_back([]() -> void { (new T())->PostRegisterTypes(); });
    }
#ifdef FLY_TANK_MOD
    void registerHook(IFlyTankModuleHook* moduleHook)
#else
    void registerHook(IDAVModuleHook* moduleHook)
#endif
    {
        s_hooks.emplace_back(moduleHook);
    }
    void Load(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle)
    {
        for (const auto& load : s_loads)
        {
            load(aSdk, aHandle);
        }
        for (const auto& hook : s_hooks)
        {
            hook->Load(aSdk, aHandle);
        }
    }
    void Unload(const RED4ext::Sdk* aSdk, RED4ext::PluginHandle aHandle)
    {
        for (const auto& hook : s_hooks)
        {
            hook->Unload(aSdk, aHandle);
        }
        for (const auto& unload : s_unloads)
        {
            unload(aSdk, aHandle);
        }
    }
    void RegisterTypes()
    {
        for (const auto& s_register : s_registers)
        {
            s_register();
        }
    }
    void PostRegisterTypes()
    {
        for (const auto& postRegister : s_postRegisters)
        {
            postRegister();
        }
    }
};
#ifdef FLY_TANK_MOD
class FlyTankModuleHookHash : IFlyTankModuleHook
{
public:
    explicit FlyTankModuleHookHash(std::string name, uint32_t hash, void* hook, void** original)
    {
        this->m_name = name;
        this->m_address =
            RED4ext::UniversalRelocBase::Resolve(hash) - reinterpret_cast<uintptr_t>(GetModuleHandle(nullptr));
        this->m_hook = hook;
        this->m_original = original;
        FlyTankModuleFactory::GetInstance().registerHook(this);
    }
};
#else
class DAVModuleHookHash : IDAVModuleHook
{
public:
    explicit DAVModuleHookHash(std::string name, uint32_t hash, void* hook, void** original)
    {
        this->m_name = name;
        this->m_address =
            RED4ext::UniversalRelocBase::Resolve(hash) - reinterpret_cast<uintptr_t>(GetModuleHandle(nullptr));
        this->m_hook = hook;
        this->m_original = original;
        DAVModuleFactory::GetInstance().registerHook(this);
    }
};
#endif

#ifdef FLY_TANK_MOD
#define REGISTER_DAV_HOOK_HASH(retType, hash, func, ...) \
    retType func(__VA_ARGS__); \
    decltype(&func) func##_Original; \
    FlyTankModuleHookHash s_##func##_Hook(#func, hash, reinterpret_cast<void*>(&func), \
                                         reinterpret_cast<void**>(&func##_Original)); \
    retType func(__VA_ARGS__)
#else
#define REGISTER_DAV_HOOK_HASH(retType, hash, func, ...) \
    retType func(__VA_ARGS__); \
    decltype(&func) func##_Original; \
    DAVModuleHookHash s_##func##_Hook(#func, hash, reinterpret_cast<void*>(&func), \
                                      reinterpret_cast<void**>(&func##_Original)); \
    retType func(__VA_ARGS__)
#endif

REGISTER_DAV_HOOK_HASH(void __fastcall, 3303544265, vehiclePhysicsData_ApplyTorqueAtPosition,
                          RED4ext::vehicle::PhysicsData* physicsData, RED4ext::Vector3* offset,
                          RED4ext::Vector3* torque)
{
    // Same id rule as the flight hook: the vehicle stores a serial-less EntityID.
    if (is_enable_original_physics || !IsFlyVehicle(physicsData->vehicle))
    {
        vehiclePhysicsData_ApplyTorqueAtPosition_Original(physicsData, offset, torque);
    }
    else
    {
        return;
    }
}

REGISTER_DAV_HOOK_HASH(void __fastcall, 611586815, ApplyForceAtPosition, RED4ext::vehicle::PhysicsData* physicsData,
                          RED4ext::Vector3* offset, RED4ext::Vector3* force)
{
    if (is_enable_original_physics || !IsFlyVehicle(physicsData->vehicle))
    {
        ApplyForceAtPosition_Original(physicsData, offset, force);
    }
    else
    {
        return;
    }
}

// --- Phase B-1/B-2: vehicle physics fixed-update hook (physics thread, once per tick per vehicle) ---
static std::atomic<uint32_t> g_hook_thread_id{0};
static std::atomic<uint64_t> g_native_tick_count{0};
static std::atomic<uint64_t> g_native_phys_enable{0};
static std::atomic<float> g_native_last_dt{0.0f};

// Flight control state pushed from CET (game thread), read on the physics thread (double buffer).
struct FlightControlState
{
    RED4ext::Vector3 velocity;
    RED4ext::Vector3 angularVelocity;
    float torqueGain;
    int mode; // 0=off (CET drives), 1=add force/torque, 2=write velocity directly, 3=native flight model
};

// --- Phase B-3: native flight model (port of Engine:CalculateAddVelocity/Run for manual flight) ---
// Tunables mirror Engine:New + user_setting_table; pushed from Lua (SetFlightParams).
struct FlightParams
{
    float max_speed_mph;
    float h_air, v_air;
    // AV mode
    float acceleration, vertical_acceleration, left_right_acceleration;
    float roll_change_amount, roll_restore_amount;
    float pitch_change_amount, pitch_restore_amount;
    float yaw_change_amount, rotate_roll_change_amount;
    // Helicopter mode
    float h_roll_change, h_roll_restore, h_pitch_change, h_pitch_restore;
    float h_yaw_change, h_acceleration, h_lift_idle, h_ascend, h_descend;
    // Engine constants
    float rpm_step, rpm_restore, rpm_scale;
    float max_roll, max_pitch, force_restore_angle, restore_boundary_deg;
};
// Pushed model state (game thread writes, physics thread reads) via double buffer.
struct FlightModel
{
    FlightParams p;
    int flight_mode;   // 0=AV, 1=Helicopter
    int has_gravity;   // physicsData->unk1B0 equivalent, pushed by Lua
    float tick_dt;     // Lua control-loop tick period in s
    float dt_scale;    // TimeScale:Scale()
    int ncmd;
    int action[6];
    float value[6];
    // Attitude pushed by Lua (CET's ToEulerAngles), degrees; att_valid = 0 means "use extraction".
    float att[3];
    float att_valid;
    // Entity world quaternion pushed by Lua; primary attitude/axis source, validated by norm.
    float cetq[4];
    // Entity forward/right/up pushed by Lua (v8.W = 1 when valid): what the Lua thrust math uses.
    float ax[9];
    float ax_valid;
};
static FlightModel g_fm_buf[2] = {};
static std::atomic<int> g_fm_idx{0};
// Persistent native integrators (physics thread writes; GetNativeRPM reads).
static std::atomic<float> g_fm_rpm{0.0f};
static float g_fm_heli_lift = 0.0f;

static inline float fm_deg(float r) { return r * (180.0f / 3.14159265358979323846f); }
static inline float fm_rad(float d) { return d * (3.14159265358979323846f / 180.0f); }

// Wrap an angle delta into [-180,180].
static inline float fm_wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

// Port of Utils:CalculateRotationalSpeed (local_yaw fixed at 0); outputs (d_pitch, d_roll, d_yaw).
static void RotSpeed(float lr, float lp, float cur_roll, float cur_pitch, float cur_yaw,
                     float& d_pitch, float& d_roll, float& d_yaw)
{
    float cr = std::cos(fm_rad(cur_roll)), sr = std::sin(fm_rad(cur_roll));
    float cp = std::cos(fm_rad(cur_pitch)), sp = std::sin(fm_rad(cur_pitch));
    float cy = std::cos(fm_rad(cur_yaw)), sy = std::sin(fm_rad(cur_yaw));
    float clr = std::cos(fm_rad(lr)), slr = std::sin(fm_rad(lr));
    float clp = std::cos(fm_rad(lp)), slp = std::sin(fm_rad(lp));
    float R1[3][3] = {
        {cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy},
        {sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy},
        {-sp, cp * sy, cp * cy}};
    float R2[3][3] = {
        {clr * clp, -slr, clr * slp},
        {slr * clp, clr, slr * slp},
        {-slp, 0.0f, clp}};
    float R[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                R[i][j] += R1[i][k] * R2[k][j];
        }
    float nr = fm_deg(std::atan2(R[1][0], R[0][0]));
    float np = fm_deg(std::atan2(-R[2][0], std::sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2])));
    float ny = fm_deg(std::atan2(R[2][1], R[2][2]));
    // The Lua caller binds the FIRST return value (new_pitch - pitch) to d_roll and the second
    // (new_roll - roll) to d_pitch, then does roll += d_roll / pitch += d_pitch. The crossing is in
    // the original and must be reproduced exactly; without it the helicopter's lean axes permute.
    // Also wrapped to [-180,180]: at cur_yaw = 179.96 the raw difference jumped to -359.90 in-game.
    d_roll = fm_wrap180(np - cur_pitch);
    d_pitch = fm_wrap180(nr - cur_roll);
    d_yaw = fm_wrap180(ny - cur_yaw);
}
// Clamp a flight-model target to a sane range.
static inline float fm_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Restore rate with boundary layer (Engine:RestoreRate); dt_scale scales the boundary.
static float RestoreRate(float excess, float full_rate, float boundary_deg, float dt_scale)
{
    float boundary = boundary_deg * dt_scale;
    if (boundary <= 0.0f || excess >= boundary)
        return full_rate;
    if (excess <= 0.0f)
        return 0.0f;
    return full_rate * (excess / boundary);
}
// Native flight model output (physics thread only): per-tick velocity/angular targets.
struct FlightTarget
{
    RED4ext::Vector3 vel;
    RED4ext::Vector3 ang;
};
static FlightTarget g_fm_target = {};

// One physics-tick step of Engine:CalculateAddVelocity + Engine:Run (manual flight only).
// Integrators (RPM, heli lift) scale by dt/BASE_RESOLUTION to match the Lua loop's per-second rate.
static void FmStep(RED4ext::vehicle::PhysicsData* pd, const FlightModel& m, float dt)
{
    const FlightParams& p = m.p;
    const float dt_scale = m.dt_scale;
    const float acc = dt / 0.01f; // BASE_RESOLUTION = 0.01 s
    const bool heli = m.flight_mode == 1;

    // Attitude source, best first: the transform the game maintains on the entity itself (read
    // straight off vehicle::BaseObject on the physics thread; verified identical to what CET
    // pushes), then CET's push, then the physics field. The common path needs no game-thread feed.
    RED4ext::Quaternion q = pd->orientation;
    bool q_from_entity = false;
    const int32_t att_off = g_att_offset.load(std::memory_order_relaxed);
    if (att_off >= 0 && pd->vehicle != nullptr)
    {
        const auto* ent = reinterpret_cast<const RED4ext::Quaternion*>(reinterpret_cast<uintptr_t>(pd->vehicle) +
                                                                       att_off);
        const float n2 = ent->i * ent->i + ent->j * ent->j + ent->k * ent->k + ent->r * ent->r;
        if (std::isfinite(n2) && n2 > 0.9f && n2 < 1.1f)
        {
            q = *ent;
            q_from_entity = true;
        }
    }
    const float cq_norm = std::sqrt(m.cetq[0] * m.cetq[0] + m.cetq[1] * m.cetq[1] + m.cetq[2] * m.cetq[2] +
                                    m.cetq[3] * m.cetq[3]);
    const bool cetq_usable = std::isfinite(cq_norm) && cq_norm > 0.5f && cq_norm < 2.0f;
    if (!q_from_entity && cetq_usable)
    {
        q.i = m.cetq[0];
        q.j = m.cetq[1];
        q.k = m.cetq[2];
        q.r = m.cetq[3];
    }
    if (!q_from_entity && cetq_usable && pd->vehicle != nullptr && g_sdk)
    {
        // The fixed offset failed: report where the matching quaternion actually lives (once).
        const int off = ScanForQuaternion(pd->vehicle, m.cetq, 0x600);
        char buf[192];
        std::snprintf(buf, sizeof(buf), "DAV dbg: att fallback scan vehicle_off=0x%X", off);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
    const float qn = std::sqrt(q.i * q.i + q.j * q.j + q.k * q.k + q.r * q.r);
    if (qn > 0.0f)
    {
        q.i /= qn; q.j /= qn; q.k /= qn; q.r /= qn;
    }
    // Measured against the game's GetForward/GetRight/GetUp across banked and pitched samples: the
    // entity quaternion's local axes are forward = AxisY, right = AxisX, up = AxisZ. X and Y are
    // swapped relative to the usual assumption, which is what made the attitude look sign-flipped.
    float fx = 2.0f * (q.i * q.j - q.r * q.k); // forward = AxisY
    float fy = 1.0f - 2.0f * (q.i * q.i + q.k * q.k);
    float fz = 2.0f * (q.j * q.k + q.r * q.i);
    float rx = 1.0f - 2.0f * (q.j * q.j + q.k * q.k); // right = AxisX
    float ry = 2.0f * (q.i * q.j + q.r * q.k);
    float rz = 2.0f * (q.i * q.k - q.r * q.j);
    float ux = 2.0f * (q.i * q.k + q.r * q.j); // up = AxisZ
    float uy = 2.0f * (q.j * q.k - q.r * q.i);
    float uz = 1.0f - 2.0f * (q.i * q.i + q.j * q.j);
    // Game Euler convention, derived from the same measurement and verified on steep bank/pitch:
    //   roll = atan2(-right_z, hypot(forward_z, up_z)), pitch = atan2(forward_z, up_z),
    //   yaw = heading of the right vector.
    const float game_roll_ext = fm_deg(std::atan2(-rz, std::sqrt(fz * fz + uz * uz)));
    const float game_pitch_ext = fm_deg(std::atan2(fz, uz));
    const float game_yaw_ext = fm_wrap180(fm_deg(std::atan2(ry, rx)));
    // The Lua thrust math used the entity's GetWorldForward/Right/Up; the DLL now derives the same
    // vectors from the quaternion, so a pushed set still wins when one arrives (older CET builds).
    const bool ax_usable = m.ax_valid != 0.0f && std::isfinite(m.ax[0] + m.ax[4] + m.ax[8]);
    if (ax_usable)
    {
        fx = m.ax[0];
        fy = m.ax[1];
        fz = m.ax[2];
        rx = m.ax[3];
        ry = m.ax[4];
        rz = m.ax[5];
        ux = m.ax[6];
        uy = m.ax[7];
        uz = m.ax[8];
    }
    // Attitude from the entity quaternion, converted to the game's ToEulerAngles convention. Measured
    // against CET across many samples: game_roll = -ext_roll, game_pitch = -ext_pitch and
    // game_yaw = ext_yaw - 90. With the conversion the DLL is self-sufficient for attitude.
    const float cur_roll_ext = game_roll_ext;
    const float cur_pitch_ext = game_pitch_ext;
    const float cur_yaw_ext = game_yaw_ext;
    // A pushed attitude still wins when present; a garbage value would turn the restore into a spin.
    const bool att_sane = m.att_valid != 0.0f && std::isfinite(m.att[0]) && std::isfinite(m.att[1]) &&
                          std::isfinite(m.att[2]) && std::abs(m.att[0]) <= 540.0f &&
                          std::abs(m.att[1]) <= 540.0f && std::abs(m.att[2]) <= 540.0f;
    const float cur_roll = att_sane ? m.att[0] : cur_roll_ext;
    const float cur_pitch = att_sane ? m.att[1] : cur_pitch_ext;
    const float cur_yaw = att_sane ? m.att[2] : cur_yaw_ext;

    float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    float rpm = g_fm_rpm.load(std::memory_order_relaxed);
    const float max_rpm = 10.0f * p.rpm_scale;

    // Event-driven input: once Lua has pushed any command edge, the held state replaces the list that
    // SetFlightModel pushes. Before the first edge the pushed list is used unchanged.
    int act_buf[6];
    float val_buf[6];
    const int* use_action = m.action;
    const float* use_value = m.value;
    int use_ncmd = m.ncmd;
    if (g_cmd_seq.load(std::memory_order_relaxed) > 0)
    {
        use_ncmd = BuildHeldCommands(act_buf, val_buf, 6);
        use_action = act_buf;
        use_value = val_buf;
    }

    for (int i = 0; i < use_ncmd; i++)
    {
        const int a = use_action[i];
        const float v = use_value[i];

        // Idle: CalculateAddVelocity resets RPM and returns CalculateIdleMode (pitch restore only).
        if (a == -1)
        {
            rpm = 0.0f;
            float lp_idle = 0.0f;
            // Signs flipped to match the swapped RotSpeed slots: arg1 positive drives the nose down,
            // so a nose-up error has to add to lp_idle (negative feedback, verified offline).
            if (cur_pitch > p.pitch_restore_amount)
                lp_idle -= RestoreRate(cur_pitch - p.pitch_restore_amount, p.pitch_restore_amount,
                                       p.restore_boundary_deg, dt_scale);
            else if (cur_pitch < -p.pitch_restore_amount)
                lp_idle += RestoreRate(-cur_pitch - p.pitch_restore_amount, p.pitch_restore_amount,
                                       p.restore_boundary_deg, dt_scale);
            else
                lp_idle -= cur_pitch;
            float dp, dr, dy;
            RotSpeed(0.0f, lp_idle, cur_roll, cur_pitch, cur_yaw, dp, dr, dy);
            roll += dr * v;
            pitch += dp * v;
            yaw += dy * v;
            continue;
        }

        // RPM integration (per command, matching CalculateAddVelocity; rate per second via acc).
        if ((a == 1 || a == 7 || a == 29 || a == 21) && rpm <= max_rpm)
            rpm += p.rpm_step * acc;
        else if ((a == 2 || a == 8 || a == 22) && rpm >= -max_rpm)
            rpm -= p.rpm_step * acc;
        if (rpm > 0.0f)
            rpm -= p.rpm_restore * acc;
        else if (rpm < 0.0f)
            rpm += p.rpm_restore * acc;

        float cx = 0, cy = 0, cz = 0, croll = 0, cpitch = 0, cyaw = 0;
        float lr = 0, lp = 0;
        if (!heli)
        {
            switch (a)
            {
            case 7: cz += p.vertical_acceleration; break;
            case 8: cz -= p.vertical_acceleration; break;
            case 1:
                cx += p.acceleration * fx; cy += p.acceleration * fy; cz += p.acceleration * fz;
                if (cur_pitch > p.pitch_restore_amount) lp -= p.pitch_restore_amount;
                else if (cur_pitch < -p.pitch_restore_amount) lp += p.pitch_restore_amount;
                else lp -= cur_pitch;
                break;
            case 2:
                cx -= p.acceleration * fx; cy -= p.acceleration * fy; cz -= p.acceleration * fz;
                if (cur_pitch > p.pitch_restore_amount) lp -= p.pitch_restore_amount;
                else if (cur_pitch < -p.pitch_restore_amount) lp += p.pitch_restore_amount;
                else lp -= cur_pitch;
                break;
            case 4: // LeftRotate
                cyaw += p.yaw_change_amount;
                if (cur_roll > p.max_roll) lr = 0;
                else if (cur_roll > 0) lr -= p.rotate_roll_change_amount * ((p.max_roll - cur_roll) / p.max_roll);
                else lr -= p.rotate_roll_change_amount;
                break;
            case 3: // RightRotate
                cyaw -= p.yaw_change_amount;
                if (cur_roll < -p.max_roll) lr = 0;
                else if (cur_roll < 0) lr += p.rotate_roll_change_amount * ((p.max_roll + cur_roll) / p.max_roll);
                else lr += p.rotate_roll_change_amount;
                break;
            case 9: // Right
                cx += p.left_right_acceleration * rx; cy += p.left_right_acceleration * ry;
                if (cur_roll > p.max_roll) lr = 0;
                else if (cur_roll > 0) lr += p.roll_change_amount * ((p.max_roll - cur_roll) / p.max_roll);
                else lr += p.roll_change_amount;
                break;
            case 10: // Left
                cx -= p.left_right_acceleration * rx; cy -= p.left_right_acceleration * ry;
                if (cur_roll < -p.max_roll) lr = 0;
                else if (cur_roll < 0) lr -= p.roll_change_amount * ((p.max_roll + cur_roll) / p.max_roll);
                else lr -= p.roll_change_amount;
                break;
            case 5: // LeanForward (faithful to the Lua: local_pitch -= pitch_change_amount)
                if (cur_pitch < -p.max_pitch) lp = 0;
                else if (cur_pitch < 0) lp -= p.pitch_change_amount * ((p.max_pitch + cur_pitch) / p.max_pitch);
                else lp -= p.pitch_change_amount;
                break;
            case 6: // LeanBackward
                if (cur_pitch > p.max_pitch) lp = 0;
                else if (cur_pitch > 0) lp += p.pitch_change_amount * ((p.max_pitch - cur_pitch) / p.max_pitch);
                else lp += p.pitch_change_amount;
                break;
            case 11: case 0: // LeanReset / Nothing
                if (cur_pitch > p.pitch_restore_amount) lp -= p.pitch_restore_amount;
                else if (cur_pitch < -p.pitch_restore_amount) lp += p.pitch_restore_amount;
                else lp -= cur_pitch;
                break;
            }
        }
        else
        {
            switch (a)
            {
            case 23: // HLeanForward (faithful to the Lua: local_pitch -= h_pitch_change)
                if (cur_pitch < -p.max_pitch) lp = 0;
                else if (cur_pitch < 0) lp -= p.h_pitch_change * ((p.max_pitch + cur_pitch) / p.max_pitch);
                else lp -= p.h_pitch_change;
                break;
            case 24: // HLeanBackward
                if (cur_pitch > p.max_pitch) lp = 0;
                else if (cur_pitch > 0) lp += p.h_pitch_change * ((p.max_pitch - cur_pitch) / p.max_pitch);
                else lp += p.h_pitch_change;
                break;
            case 26: // HLeanLeft
                if (cur_roll < -p.max_roll) lr = 0;
                else if (cur_roll < 0) lr -= p.h_roll_change * ((p.max_roll + cur_roll) / p.max_roll);
                else lr -= p.h_roll_change;
                break;
            case 25: // HLeanRight
                if (cur_roll > p.max_roll) lr = 0;
                else if (cur_roll > 0) lr += p.h_roll_change * ((p.max_roll - cur_roll) / p.max_roll);
                else lr += p.h_roll_change;
                break;
            case 27: cyaw -= p.h_yaw_change; break; // HRightRotate
            case 28: cyaw += p.h_yaw_change; break; // HLeftRotate
            case 29: // HAccelerate
                cx += p.h_acceleration * fx; cy += p.h_acceleration * fy; cz += p.h_acceleration * fz;
                break;
            case 21: // HUp
                cz += p.h_ascend * uz;
                g_fm_heli_lift += p.h_ascend * acc;
                break;
            case 22: // HDown
                cz -= p.h_descend * uz;
                g_fm_heli_lift -= p.h_descend * acc;
                break;
            }
        }
        float dp, dr, dy;
        // Lua argument order (local_roll, local_pitch); RotSpeed reproduces the original's crossed
        // return values, which is what makes the axes come out right.
        RotSpeed(lr, lp, cur_roll, cur_pitch, cur_yaw, dp, dr, dy);
        croll += dr; cpitch += dp; cyaw += dy;
        x += cx * v; y += cy * v; z += cz * v;
        roll += croll * v; pitch += cpitch * v; yaw += cyaw * v;
    }
    g_fm_rpm.store(rpm, std::memory_order_relaxed);

    // Engine:Run: attitude restore with boundary layer.
    const float roll_restore = heli ? p.h_roll_restore : p.roll_restore_amount;
    const float pitch_restore = heli ? p.h_pitch_restore : p.pitch_restore_amount;
    float lr2 = 0, lp2 = 0;
    const bool has_gravity = pd->unk1B0 != 0;
    if (heli || has_gravity)
    {
        // Signs follow the Lua exactly: restore always subtracts from the positive side.
        if (cur_pitch > pitch_restore)
            lp2 -= RestoreRate(cur_pitch - pitch_restore, pitch_restore, p.restore_boundary_deg, dt_scale);
        else if (cur_pitch < -pitch_restore)
            lp2 += RestoreRate(-cur_pitch - pitch_restore, pitch_restore, p.restore_boundary_deg, dt_scale);
    }
    if (cur_roll > roll_restore)
        lr2 -= RestoreRate(cur_roll - roll_restore, roll_restore, p.restore_boundary_deg, dt_scale);
    else if (cur_roll < -roll_restore)
        lr2 += RestoreRate(-cur_roll - roll_restore, roll_restore, p.restore_boundary_deg, dt_scale);
    if (cur_roll > p.max_roll) lr2 -= (cur_roll - p.max_roll) * 0.5f;
    else if (cur_roll < -p.max_roll) lr2 += (-p.max_roll - cur_roll) * 0.5f;
    // Pitch had no over-limit correction while roll did, so past max_pitch only the weak 0.2/tick
    // restore opposed the motion and the helicopter ran all the way to vertical. Symmetric now.
    if (cur_pitch > p.max_pitch) lp2 -= (cur_pitch - p.max_pitch) * 0.5f;
    else if (cur_pitch < -p.max_pitch) lp2 += (-p.max_pitch - cur_pitch) * 0.5f;
    if (cur_roll > p.force_restore_angle || cur_roll < -p.force_restore_angle) lr2 = -cur_roll;
    else if (cur_pitch > p.force_restore_angle || cur_pitch < -p.force_restore_angle) lp2 = -cur_pitch;
    {
        float dp, dr, dy;
        RotSpeed(lr2, lp2, cur_roll, cur_pitch, cur_yaw, dp, dr, dy);
        roll += dr; pitch += dp; yaw += dy;
    }
    if (heli)
    {
        if (g_fm_heli_lift < 0.0f) g_fm_heli_lift = 0.0f;
        x += g_fm_heli_lift * ux;
        y += g_fm_heli_lift * uy;
        g_fm_heli_lift = p.h_lift_idle;
    }

    const float vx = pd->velocity.X, vy = pd->velocity.Y, vz = pd->velocity.Z;
    const float nx = x + vx, ny = y + vy, nz = z + vz;
    const float max_speed = p.max_speed_mph * 0.44704f;
    if (std::sqrt(nx * nx + ny * ny + nz * nz) > max_speed)
    {
        x = 0; y = 0; z = 0;
    }
    x -= p.h_air * vx; y -= p.h_air * vy; z -= p.v_air * vz;
    g_fm_target.vel = {x, y, z};
    // Sanity guard only: the original targets reach ~180 via force_restore, so this must not bind.
    g_fm_target.ang.X = fm_clamp(roll, -500.0f, 500.0f);
    g_fm_target.ang.Y = fm_clamp(pitch, -500.0f, 500.0f);
    g_fm_target.ang.Z = fm_clamp(yaw, -500.0f, 500.0f);
}

static FlightControlState g_fc_buf[2] = {};
static std::atomic<int> g_fc_idx{0};

static void SetFlightControlState(int mode, const RED4ext::Vector3& vel, const RED4ext::Vector3& angvel, float gain)
{
    if (g_sdk)
    {
        static int s_lastLoggedMode = -999;
        static RED4ext::Vector3 s_lastLoggedVel{0, 0, 0};
        const bool velChanged = std::abs(vel.X - s_lastLoggedVel.X) > 0.5f ||
                                std::abs(vel.Y - s_lastLoggedVel.Y) > 0.5f ||
                                std::abs(vel.Z - s_lastLoggedVel.Z) > 0.5f;
        if (mode != s_lastLoggedMode || (mode == 2 && velChanged))
        {
            s_lastLoggedMode = mode;
            s_lastLoggedVel = vel;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "DAV dbg: SetFlightControl mode=%d vel=(%.2f,%.2f,%.2f)",
                          mode, vel.X, vel.Y, vel.Z);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
    }
    int next = 1 - g_fc_idx.load(std::memory_order_relaxed);
    g_fc_buf[next].velocity = vel;
    g_fc_buf[next].angularVelocity = angvel;
    g_fc_buf[next].torqueGain = gain;
    g_fc_buf[next].mode = mode;
    g_fc_idx.store(next, std::memory_order_release);
}

REGISTER_DAV_HOOK_HASH(void __fastcall, 2526549425, VehicleApplyAllResistances,
                          RED4ext::vehicle::WheeledPhysics* physics, float deltaTime)
{
    static std::atomic<uint64_t> s_entry{0};
    if (g_sdk)
    {
        const uint64_t e = s_entry.fetch_add(1, std::memory_order_relaxed);
        if (e < 5)
        {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "DAV dbg: hook entry #%llu hash=%llu expected=%llu",
                          (unsigned long long)e,
                          physics->parent ? (unsigned long long)physics->parent->entityID.hash : 0ULL,
                          (unsigned long long)g_fly_vehicle_entity_id_hash);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
    }
    if (IsFlyVehicle(physics->parent))
    {
        static std::atomic<uint64_t> s_dbg_counter{0};
        const uint64_t dbg_n = s_dbg_counter.fetch_add(1, std::memory_order_relaxed);
        if (dbg_n == 0 && g_sdk)
        {
            // One-shot proof that the physics hook runs for the AV (the whole native path depends on it).
            g_hook_thread_id.store(static_cast<uint32_t>(GetCurrentThreadId()), std::memory_order_relaxed);
            char buf[128];
            std::snprintf(buf, sizeof(buf), "DAV dbg: physics hook live for our vehicle (tid=%lu)",
                          (unsigned long)GetCurrentThreadId());
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
        // Publish the object for the game thread; Handle ref-counting must stay off this thread.
        g_fly_vehicle_raw.store(reinterpret_cast<uint64_t>(physics->parent), std::memory_order_release);
        // Safety net, held every tick the way gravity is: if the game left our body in a non-simulated
        // state, put it back. Never observed firing in testing, kept in case a spawn path needs it.
        if (physics->parent->physicsState != 0)
        {
            physics->parent->ForceEnablePhysics();
            const uint64_t n = g_native_phys_enable.fetch_add(1, std::memory_order_relaxed);
            if (n < 3 && g_sdk)
            {
                char pbuf[128];
                std::snprintf(pbuf, sizeof(pbuf), "DAV dbg: hook re-enabled physics (#%llu)",
                              (unsigned long long)n);
                g_sdk->logger->Info(g_plugin_handle, pbuf);
            }
        }
        // Tick probe read by CET's watchdog: if it stops advancing while we drive, fall back to Lua.
        g_native_tick_count.fetch_add(1, std::memory_order_relaxed);
        g_native_last_dt.store(deltaTime, std::memory_order_relaxed);

        // Not gated on is_enable_original_physics: the Lua path wrote velocity in every state.
        auto physicsData = physics->parent->physicsData;
        if (physicsData != nullptr)
        {
            // Publish what the game-thread API needs (the handle may stay unresolved).
            g_native_mass.store(physicsData->total_mass, std::memory_order_relaxed);

            if (!is_enable_original_physics)
            {
                // DAV owns the physics: keep gravity at the state CET asked for.
                physicsData->unk1B0 = g_gravity_enable.load(std::memory_order_relaxed) ? 1 : 0;
            }
            g_native_gravity.store(physicsData->unk1B0, std::memory_order_relaxed);
            const FlightControlState& fc = g_fc_buf[g_fc_idx.load(std::memory_order_acquire)];
            if (fc.mode == 1)
            {
                // AddForce semantics: force = target_accel * mass, torque = angvel error * gain.
                float mass = physicsData->total_mass;
                physicsData->force.X += fc.velocity.X * mass;
                physicsData->force.Y += fc.velocity.Y * mass;
                physicsData->force.Z += fc.velocity.Z * mass;
                physicsData->torque.X += (fc.angularVelocity.X - physicsData->angularVelocity.X) * fc.torqueGain;
                physicsData->torque.Y += (fc.angularVelocity.Y - physicsData->angularVelocity.Y) * fc.torqueGain;
                physicsData->torque.Z += (fc.angularVelocity.Z - physicsData->angularVelocity.Z) * fc.torqueGain;
            }
            else if (fc.mode == 2)
            {
                // ChangeVelocity semantics: hold velocity at target every tick (old code re-wrote per frame).
                physicsData->velocity = fc.velocity;
                physicsData->angularVelocity = fc.angularVelocity;
            }
            else if (fc.mode == 3)
            {
                // Native flight model: compute the per-tick target from pushed commands, then apply AddForce semantics.
                const FlightModel& fm = g_fm_buf[g_fm_idx.load(std::memory_order_acquire)];
                FmStep(physicsData, fm, deltaTime);
                float mass = physicsData->total_mass;
                physicsData->force.X += g_fm_target.vel.X * mass;
                physicsData->force.Y += g_fm_target.vel.Y * mass;
                physicsData->force.Z += g_fm_target.vel.Z * mass;
                physicsData->torque.X += (g_fm_target.ang.X - physicsData->angularVelocity.X) * fc.torqueGain;
                physicsData->torque.Y += (g_fm_target.ang.Y - physicsData->angularVelocity.Y) * fc.torqueGain;
                physicsData->torque.Z += (g_fm_target.ang.Z - physicsData->angularVelocity.Z) * fc.torqueGain;
            }
        }
    }
    VehicleApplyAllResistances_Original(physics, deltaTime);
}
//////////////////////////////////////////////////////////////////////////////////

#ifdef FLY_TANK_MOD
struct FlyTankSystem : RED4ext::IScriptable
#else
struct FlyAVSystem : RED4ext::IScriptable
#endif
{
    RED4ext::CClass* GetNativeType();
};

#ifdef FLY_TANK_MOD
RED4ext::TTypedClass<FlyTankSystem> cls("FlyTankSystem");
RED4ext::CClass* FlyTankSystem::GetNativeType()
#else
RED4ext::TTypedClass<FlyAVSystem> cls("FlyAVSystem");
RED4ext::CClass* FlyAVSystem::GetNativeType()
#endif
{
    return &cls;
}

void SetVehicle(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    uint64_t entity_id_hash;
    RED4ext::EntityID entity_id;

    RED4ext::GetParameter(aFrame, &entity_id_hash);
    aFrame->code++; // skip ParamEnd

    entity_id.hash = entity_id_hash;
    const bool changed = (g_fly_vehicle_entity_id_hash != entity_id_hash);
    g_fly_vehicle_entity_id_hash = entity_id_hash;
    if (changed)
    {
        // A new vehicle: drop the previous handle so it gets resolved (and re-enabled) fresh.
        g_fly_vehicle = RED4ext::Handle<RED4ext::vehicle::BaseObject>();
        g_fly_vehicle_raw.store(0, std::memory_order_release);
        g_native_mass.store(0.0f, std::memory_order_relaxed);
    }
    is_enable_original_physics = true;
    g_resolve_attempts = 0; // SetVehicle always gets an immediate attempt
    TryResolveVehicle("SetVehicle");
    ResolveEntityTransformOffsets();
    UseFixedTransformOffsets();

    *aOut = 0;

    if (g_fly_vehicle)
    {
        *aOut = 1;
    }
    if (g_sdk)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "DAV dbg: SetVehicle hash=%llu resolved=%d",
                      (unsigned long long)entity_id_hash, g_fly_vehicle ? 1 : 0);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
}

void EnableOriginalPhysics(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);
    bool is_enable;
    RED4ext::GetParameter(aFrame, &is_enable);
    aFrame->code++; // skip ParamEnd

    // Pure flag deciding whether the game's force/torque hooks pass through; must not need a handle.
    is_enable_original_physics = is_enable;
    *aOut = 1;
}

void GetMass(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = 0;

    TryResolveVehicle("GetMass");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->total_mass;
    }
    else
    {
        // Published by the physics hook when the handle could not be resolved.
        *aOut = g_native_mass.load(std::memory_order_relaxed);
    }
}

void HasGravity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = false;

    TryResolveVehicle("HasGravity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->unk1B0 != 0;
    }
    else
    {
        *aOut = g_native_gravity.load(std::memory_order_relaxed) != 0.0f;
    }
}

void EnableGravity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    bool is_enable;

    RED4ext::GetParameter(aFrame, &is_enable);
    aFrame->code++; // skip ParamEnd

    // Remember the desired state unconditionally; the physics hook applies it to the vehicle every
    // tick. Writing it once from here is silently dropped while the vehicle is unresolved.
    g_gravity_enable.store(is_enable, std::memory_order_relaxed);

    *aOut = 0;

    TryResolveVehicle("EnableGravity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        g_fly_vehicle->physicsData->unk1B0 = is_enable;
        *aOut = 1;
    }
}

void AddVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector3 velocity;
    RED4ext::Vector3 angularVelocity;

    RED4ext::GetParameter(aFrame, &velocity);
    RED4ext::GetParameter(aFrame, &angularVelocity);
    aFrame->code++; // skip ParamEnd

    *aOut = 0;

    TryResolveVehicle("AddVelocity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        g_fly_vehicle->physicsData->velocity += velocity;
        g_fly_vehicle->physicsData->angularVelocity += angularVelocity;
        *aOut = 1;
    }
}

void AddForce(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector3 force;
    RED4ext::Vector3 torque;

    RED4ext::GetParameter(aFrame, &force);
    RED4ext::GetParameter(aFrame, &torque);
    aFrame->code++; // skip ParamEnd

    *aOut = 0;

    TryResolveVehicle("AddForce");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        g_fly_vehicle->physicsData->force += force;
        g_fly_vehicle->physicsData->torque += torque;
        *aOut = 1;
    }
}

void ChangeVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector3 velocity;
    RED4ext::Vector3 angularVelocity;
    float switchIndex;

    RED4ext::GetParameter(aFrame, &velocity);
    RED4ext::GetParameter(aFrame, &angularVelocity);
    RED4ext::GetParameter(aFrame, &switchIndex);

    aFrame->code++; // skip ParamEnd

    *aOut = 0;

    TryResolveVehicle("ChangeVelocity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        if (switchIndex == 1)
        {
            g_fly_vehicle->physicsData->velocity = velocity;
        }
        else if (switchIndex == 2)
        {
            g_fly_vehicle->physicsData->angularVelocity = angularVelocity;
        }
        else
        {
            g_fly_vehicle->physicsData->velocity = velocity;
            g_fly_vehicle->physicsData->angularVelocity = angularVelocity;
        }
        *aOut = 1;
    }
}

void ChangeForce(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    RED4ext::Vector3 force;
    RED4ext::Vector3 torque;
    float switchIndex;
    RED4ext::GetParameter(aFrame, &force);
    RED4ext::GetParameter(aFrame, &torque);
    RED4ext::GetParameter(aFrame, &switchIndex);
    aFrame->code++; // skip ParamEnd
    *aOut = 0;
    TryResolveVehicle("ChangeForce");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        if (switchIndex == 1)
        {
            g_fly_vehicle->physicsData->force = force;
        }
        else if (switchIndex == 2)
        {
            g_fly_vehicle->physicsData->torque = torque;
        }
        else
        {
            g_fly_vehicle->physicsData->force = force;
            g_fly_vehicle->physicsData->torque = torque;
        }
        *aOut = 1;
    }
}

void GetVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, RED4ext::Vector3* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector3 velocity;
    velocity.X = 0;
    velocity.Y = 0;
    velocity.Z = 0;

    *aOut = velocity;

    TryResolveVehicle("GetVelocity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->velocity;
    }
}

void GetAngularVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, RED4ext::Vector3* aOut,
                        int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector3 angularVelocity;
    angularVelocity.X = 0;
    angularVelocity.Y = 0;
    angularVelocity.Z = 0;

    *aOut = angularVelocity;

    TryResolveVehicle("GetAngularVelocity");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->angularVelocity;
    }
}

void GetForce(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, RED4ext::Vector3* aOut,
                        int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);
    RED4ext::Vector3 force;
    force.X = 0;
    force.Y = 0;
    force.Z = 0;
    *aOut = force;
    TryResolveVehicle("GetForce");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->force;
    }
}

void GetTorque(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, RED4ext::Vector3* aOut,
                        int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);
    RED4ext::Vector3 torque;
    torque.X = 0;
    torque.Y = 0;
    torque.Z = 0;
    *aOut = torque;
    TryResolveVehicle("GetTorque");
    if (g_fly_vehicle && g_fly_vehicle->physicsData)
    {
        *aOut = g_fly_vehicle->physicsData->torque;
    }
}

void GetPhysicsState(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = -1;

    TryResolveVehicle("GetPhysicsState");
    if (g_fly_vehicle)
    {
        *aOut = g_fly_vehicle->physicsState;
    }
}

void UnsetPhysicsState(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = -1;

    TryResolveVehicle("UnsetPhysicsState");
    if (g_fly_vehicle)
    {
        g_fly_vehicle->ForceEnablePhysics();
        *aOut = 1;
    }
}

void IsOnGround(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = false;

    TryResolveVehicle("IsOnGround");
    if (g_fly_vehicle)
    {
        *aOut = g_fly_vehicle->isOnGround;
    }
}

// Phase B-1 probe: lets CET verify the native physics tick hook actually fires.
void GetNativeTickCount(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = (float)g_native_tick_count.load(std::memory_order_relaxed);
}

// Phase B-2: CET pushes the flight control target; the physics-tick hook applies it every tick.
// mode: 0=off, 1=add force/torque (AddForce semantics), 2=write velocity directly (ChangeVelocity semantics).
void SetFlightControl(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    float mode;
    RED4ext::Vector3 velocity;
    RED4ext::Vector3 angularVelocity;
    float torqueGain;

    RED4ext::GetParameter(aFrame, &mode);
    RED4ext::GetParameter(aFrame, &velocity);
    RED4ext::GetParameter(aFrame, &angularVelocity);
    RED4ext::GetParameter(aFrame, &torqueGain);
    aFrame->code++; // skip ParamEnd

    // Called every frame from CET: doubles as the lazy re-resolution point for the vehicle handle.
    TryResolveVehicle("SetFlightControl");

    SetFlightControlState((int)mode, velocity, angularVelocity, torqueGain);
    *aOut = 1;
}

// Phase B-3: push all flight model tunables (order must match Lua Engine:PushNativeParams).
// Params are packed into two Vector4s: CET's native-call path is safest with few, known-good types.
void SetFlightParams(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector4 a, b, c;
    RED4ext::GetParameter(aFrame, &a);
    RED4ext::GetParameter(aFrame, &b);
    RED4ext::GetParameter(aFrame, &c);
    aFrame->code++; // skip ParamEnd

    const float vals[12] = {a.X, a.Y, a.Z, a.W, b.X, b.Y, b.Z, b.W, c.X, c.Y, c.Z, c.W};

    int cur = g_fm_idx.load(std::memory_order_acquire);
    int next = 1 - cur;
    g_fm_buf[next] = g_fm_buf[cur]; // preserve params set by the other SetFlightParams* calls
    FlightParams& p = g_fm_buf[next].p;
    p.max_speed_mph = vals[0];
    p.h_air = vals[1];
    p.v_air = vals[2];
    p.acceleration = vals[3];
    p.vertical_acceleration = vals[4];
    p.left_right_acceleration = vals[5];
    p.roll_change_amount = vals[6];
    p.roll_restore_amount = vals[7];
    p.pitch_change_amount = vals[8];
    p.pitch_restore_amount = vals[9];
    p.yaw_change_amount = vals[10];
    p.rotate_roll_change_amount = vals[11];
    g_fm_idx.store(next, std::memory_order_release);
    *aOut = 1;
}

// Phase B-3: push helicopter-mode tunables + engine constants (same packing as SetFlightParams).
void SetFlightParams2(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector4 a, b, c;
    RED4ext::GetParameter(aFrame, &a);
    RED4ext::GetParameter(aFrame, &b);
    RED4ext::GetParameter(aFrame, &c);
    aFrame->code++; // skip ParamEnd

    const float vals[12] = {a.X, a.Y, a.Z, a.W, b.X, b.Y, b.Z, b.W, c.X, c.Y, c.Z, c.W};

    int cur = g_fm_idx.load(std::memory_order_acquire);
    int next = 1 - cur;
    g_fm_buf[next] = g_fm_buf[cur]; // preserve params set by the other SetFlightParams* calls
    FlightParams& p = g_fm_buf[next].p;
    p.h_roll_change = vals[0];
    p.h_roll_restore = vals[1];
    p.h_pitch_change = vals[2];
    p.h_pitch_restore = vals[3];
    p.h_yaw_change = vals[4];
    p.h_acceleration = vals[5];
    p.h_lift_idle = vals[6];
    p.h_ascend = vals[7];
    p.h_descend = vals[8];
    p.rpm_step = vals[9];
    p.rpm_restore = vals[10];
    p.rpm_scale = vals[11];
    g_fm_idx.store(next, std::memory_order_release);
    *aOut = 1;
}

// Phase B-3: push attitude-limit constants (same packing as SetFlightParams).
void SetFlightParams3(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector4 a;
    RED4ext::GetParameter(aFrame, &a);
    aFrame->code++; // skip ParamEnd

    int cur = g_fm_idx.load(std::memory_order_acquire);
    int next = 1 - cur;
    g_fm_buf[next] = g_fm_buf[cur]; // preserve params set by the other SetFlightParams* calls
    FlightParams& p = g_fm_buf[next].p;
    p.max_roll = a.X;
    p.max_pitch = a.Y;
    p.force_restore_angle = a.Z;
    p.restore_boundary_deg = a.W;
    g_fm_idx.store(next, std::memory_order_release);
    *aOut = 1;
}

// Phase B-3: push the per-tick command list (game thread; physics hook consumes it).
// Packed into 5 Vector4s (few, known-good param types; see SetFlightParams note).
void SetFlightModel(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector4 v1, v2, v3, v4, v5, v6, v7, v8, v9, v10;
    RED4ext::GetParameter(aFrame, &v1);
    RED4ext::GetParameter(aFrame, &v2);
    RED4ext::GetParameter(aFrame, &v3);
    RED4ext::GetParameter(aFrame, &v4);
    RED4ext::GetParameter(aFrame, &v5);
    RED4ext::GetParameter(aFrame, &v6);
    RED4ext::GetParameter(aFrame, &v7);
    RED4ext::GetParameter(aFrame, &v8);
    RED4ext::GetParameter(aFrame, &v9);
    RED4ext::GetParameter(aFrame, &v10);
    aFrame->code++; // skip ParamEnd

    // v1 = flight_mode, has_gravity, dt_scale, ncmd
    // v2 = reset, action1..3
    // v3 = action4..6, value1
    // v4 = value2..5
    // v5 = value6, -, -, -
    const float acts[6] = {v2.Y, v2.Z, v2.W, v3.X, v3.Y, v3.Z};
    const float vals[6] = {v3.W, v4.X, v4.Y, v4.Z, v4.W, v5.X};

    // Copy params from the active slot so a params-only update is never lost.
    int cur = g_fm_idx.load(std::memory_order_acquire);
    int next = 1 - cur;
    FlightModel& m = g_fm_buf[next];
    m = g_fm_buf[cur];
    m.flight_mode = (int)v1.X;
    m.has_gravity = (int)v1.Y;
    m.dt_scale = v1.Z;
    if (v2.X != 0.0f)
    {
        // Fresh boarding: clear native integrators (RPM, heli lift at hover baseline).
        g_fm_rpm.store(0.0f, std::memory_order_relaxed);
        g_fm_heli_lift = m.p.h_lift_idle;
    }
    int n = (int)v1.W;
    if (n < 0) n = 0;
    if (n > 6) n = 6;
    m.ncmd = n;
    // v6 = attitude (roll, pitch, yaw in CET's Euler convention), W = 1 when valid.
    m.att[0] = v6.X;
    m.att[1] = v6.Y;
    m.att[2] = v6.Z;
    m.att_valid = v6.W;
    // v7 = entity world quaternion (i, j, k, r), W = 1 when valid.
    m.cetq[0] = v7.X;
    m.cetq[1] = v7.Y;
    m.cetq[2] = v7.Z;
    m.cetq[3] = v7.W;
    // v8/v9/v10 = entity forward/right/up; v8.W = 1 when all three are valid.
    m.ax[0] = v8.X;
    m.ax[1] = v8.Y;
    m.ax[2] = v8.Z;
    m.ax[3] = v9.X;
    m.ax[4] = v9.Y;
    m.ax[5] = v9.Z;
    m.ax[6] = v10.X;
    m.ax[7] = v10.Y;
    m.ax[8] = v10.Z;
    m.ax_valid = v8.W;
    for (int i = 0; i < n; i++)
    {
        m.action[i] = (int)acts[i];
        m.value[i] = vals[i];
    }
    g_fm_idx.store(next, std::memory_order_release);
    *aOut = 1;
}

// Phase B-3: read the native RPM integrator for the HUD (same scale as Engine:GetRPMCount).
void GetNativeRPM(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);
    *aOut = g_fm_rpm.load(std::memory_order_relaxed);
}

#ifdef FLY_TANK_MOD
RED4EXT_C_EXPORT void RED4EXT_CALL RegisterFlyTankSystem()
{
    RED4ext::CNamePool::Add("FlyTankSystem");
#else
RED4EXT_C_EXPORT void RED4EXT_CALL RegisterFlyAVSystem()
{
    RED4ext::CNamePool::Add("FlyAVSystem");
#endif

    cls.flags = {.isNative = true};
    RED4ext::CRTTISystem::Get()->RegisterType(&cls);
#ifdef FLY_TANK_MOD
    FlyTankModuleFactory::GetInstance().RegisterTypes();
#else
    DAVModuleFactory::GetInstance().RegisterTypes();
#endif
}

// Phase (input): SetNativeCommand native.
RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetNativeCommand()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetNativeCommand", "SetNativeCommand", &SetNativeCommand,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Int32", "action_id");
    func->AddParam("Float", "value");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetVehicle()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "SetVehicle", "SetVehicle", &SetVehicle, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Uint32", "entity_id_hash");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterEnableOriginalPhysics()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "EnableOriginalPhysics", "EnableOriginalPhysics",
                                                &EnableOriginalPhysics, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Bool", "is_enable");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetMass()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "GetMass", "GetMass", &GetMass, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterHasGravity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "HasGravity", "HasGravity", &HasGravity, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Bool");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterEnableGravity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "EnableGravity", "EnableGravity", &EnableGravity, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Bool", "is_enable");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterAddVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "AddVelocity", "AddVelocity", &AddVelocity,
                                                {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "velocity");
    func->AddParam("Vector3", "angularVelocity");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterAddForce()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "AddForce", "AddForce", &AddForce, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "force");
    func->AddParam("Vector3", "torque");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterChangeVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "ChangeVelocity", "ChangeVelocity",
                                                &ChangeVelocity, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "velocity");
    func->AddParam("Vector3", "angularVelocity");
    func->AddParam("Float", "switchIndex");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterChangeForce()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "ChangeForce", "ChangeForce", &ChangeForce, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "force");
    func->AddParam("Vector3", "torque");
    func->AddParam("Float", "switchIndex");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "GetVelocity", "GetVelocity", &GetVelocity, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Vector3");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetAngularVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "GetAngularVelocity", "GetAngularVelocity", &GetAngularVelocity,
                                                {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Vector3");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetForce()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "GetForce", "GetForce", &GetForce, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Vector3");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetTorque()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "GetTorque", "GetTorque", &GetTorque, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Vector3");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetPhysicsState()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "GetPhysicsState", "GetPhysicsState", &GetPhysicsState,
                                                {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterUnsetPhysicsState()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "UnsetPhysicsState", "UnsetPhysicsState", &UnsetPhysicsState, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterIsOnGround()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "IsOnGround", "IsOnGround", &IsOnGround, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Bool");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetNativeTickCount()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "GetNativeTickCount", "GetNativeTickCount", &GetNativeTickCount, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightControl()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetFlightControl", "SetFlightControl", &SetFlightControl, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Float", "mode");
    func->AddParam("Vector3", "velocity");
    func->AddParam("Vector3", "angularVelocity");
    func->AddParam("Float", "torqueGain");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightParams()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetFlightParams", "SetFlightParams", &SetFlightParams, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector4", "a");
    func->AddParam("Vector4", "b");
    func->AddParam("Vector4", "c");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightParams2()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetFlightParams2", "SetFlightParams2", &SetFlightParams2, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector4", "a");
    func->AddParam("Vector4", "b");
    func->AddParam("Vector4", "c");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightParams3()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetFlightParams3", "SetFlightParams3", &SetFlightParams3, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector4", "a");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightModel()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetFlightModel", "SetFlightModel", &SetFlightModel, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector4", "v1");
    func->AddParam("Vector4", "v2");
    func->AddParam("Vector4", "v3");
    func->AddParam("Vector4", "v4");
    func->AddParam("Vector4", "v5");
    func->AddParam("Vector4", "v6");
    func->AddParam("Vector4", "v7");
    func->AddParam("Vector4", "v8");
    func->AddParam("Vector4", "v9");
    func->AddParam("Vector4", "v10");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetNativeRPM()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "GetNativeRPM", "GetNativeRPM", &GetNativeRPM, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    cls.RegisterFunction(func);
}


RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterDAVHock()
{
#ifdef FLY_TANK_MOD
    FlyTankModuleFactory::GetInstance().PostRegisterTypes();
#else
    DAVModuleFactory::GetInstance().PostRegisterTypes();
#endif
}

RED4EXT_C_EXPORT bool RED4EXT_CALL Main(RED4ext::PluginHandle aHandle, RED4ext::EMainReason aReason,
                                        const RED4ext::Sdk* aSdk)
{
    //RED4EXT_UNUSED_PARAMETER(aHandle);
    //RED4EXT_UNUSED_PARAMETER(aSdk);

    switch (aReason)
    {
    case RED4ext::EMainReason::Load:
    {
        g_sdk = aSdk;
        g_plugin_handle = aHandle;
#ifdef FLY_TANK_MOD
        RED4ext::CRTTISystem::Get()->AddRegisterCallback(RegisterFlyTankSystem);
#else
        RED4ext::CRTTISystem::Get()->AddRegisterCallback(RegisterFlyAVSystem);
#endif
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetVehicle);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetNativeCommand);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterEnableOriginalPhysics);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetMass);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterHasGravity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterEnableGravity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterAddVelocity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterAddForce);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterChangeVelocity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterChangeForce);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetVelocity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetAngularVelocity);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetForce);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetTorque);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetPhysicsState);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterUnsetPhysicsState);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterIsOnGround);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetNativeTickCount);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightControl);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams2);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams3);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightModel);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetNativeRPM);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterDAVHock);
#ifdef FLY_TANK_MOD
        FlyTankModuleFactory::GetInstance().Load(aSdk, aHandle);
#else
        DAVModuleFactory::GetInstance().Load(aSdk, aHandle);
#endif
        break;
    }
    case RED4ext::EMainReason::Unload:
    {
#ifdef FLY_TANK_MOD
        FlyTankModuleFactory::GetInstance().Unload(aSdk, aHandle);
#else
        DAVModuleFactory::GetInstance().Unload(aSdk, aHandle);
#endif
        break;
    }
    }

    return true;
}

RED4EXT_C_EXPORT void RED4EXT_CALL Query(RED4ext::PluginInfo* aInfo)
{
#ifdef FLY_TANK_MOD
    aInfo->name = L"Fly Tank API";
    aInfo->author = L"tidusmd";
    aInfo->version = RED4EXT_SEMVER(1, 2, 3);
    aInfo->runtime = RED4EXT_RUNTIME_INDEPENDENT;
    aInfo->sdk = RED4EXT_SDK_LATEST;
#else
    aInfo->name = L"DAV API";
    aInfo->author = L"tidusmd";
    aInfo->version = RED4EXT_SEMVER(3, 1, 1);
    aInfo->runtime = RED4EXT_RUNTIME_INDEPENDENT;
    aInfo->sdk = RED4EXT_SDK_LATEST;
#endif
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_LATEST;
}
