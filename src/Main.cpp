#include <RED4ext/RED4ext.hpp>
#include <RED4ext/RTTITypes.hpp>
#include <RED4ext/Scripting/Natives/vehicleBaseObject.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysicsData.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysics.hpp>
#include <RED4ext/Scripting/Natives/Generated/Quaternion.hpp>
#include <RED4ext/Scripting/Natives/IGameInstance.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <excpt.h>
#include <string>
#include <vector>

static bool is_enable_original_physics = true;
static RED4ext::Handle<RED4ext::vehicle::BaseObject> g_fly_vehicle;
static uint64_t g_fly_vehicle_entity_id_hash = 0;

// Set when the Running game state is exited (quit to desktop). The frame clock stops touching the
// world from that point on; see DavGameState_OnExit.
static std::atomic<bool> g_shutting_down{false};

static bool SafePeekPtr(void* aAddr, void** aOut);
static bool SafePeekBytes(void* aAddr, void* aDst, size_t aLen);

// Shutdown etiquette: on quit the world teardown frees the vehicle entity while our handle still
// points at it (Lua never runs SetVehicle(0) on exit). Every native getter and the frame clock
// must pass through this before reading the body: a dangling read faults into the SEH filter and
// reports "body gone" instead of taking the process down. Same-thread callers (Lua natives and
// the game-thread OnUpdate) can use this as a check-then-read; the physics thread has its own
// SEH-wrapped access inside the hook.
static bool VehicleBodyAlive()
{
    if (g_fly_vehicle.instance == nullptr)
        return false;
    void* pd = nullptr;
    return SafePeekPtr(&g_fly_vehicle.instance->physicsData, &pd);
}

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

// Flight context bits for the auto mode re-assert in the physics hook: bit0 = player aboard,
// bit1 = autopilot engaged, bit2 = helicopter flight mode (mode 2 velocity hold IS the normal
// manual control there, so the re-assert must never touch it). Lua pushes on situation and
// autopilot transitions.
static std::atomic<int32_t> g_flight_flags{0};

// Any held command non-zero? (the successor to Lua's per-input mode 3 re-assert edge).
static bool HeldCommandActive()
{
    for (int i = 0; i < 64; ++i)
    {
        if (g_cmd_val[i].load(std::memory_order_relaxed) != 0.0f)
            return true;
    }
    return false;
}
static std::atomic<uint32_t> g_cmd_seq{0};

// One command edge. id = Def.ActionList value, value = 0/1 or analog magnitude.
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

// --- Vehicle physics fixed-update hook (physics thread, once per tick per vehicle) ---
static std::atomic<uint32_t> g_hook_thread_id{0};

// Published by the game-thread game state (declared here because the physics hook reads the ground
// reference for the idle hover). See DavGameState_OnUpdate near the plugin entry point.
static std::atomic<uint64_t> g_game_frame_count{0};
static std::atomic<uint32_t> g_game_thread_id{0};
static std::atomic<float> g_ground_z{0.0f};
static std::atomic<float> g_ground_valid{0.0f};
static std::atomic<float> g_av_world_z{0.0f};
static std::atomic<float> g_av_on_ground{0.0f};
static std::atomic<uint64_t> g_native_tick_count{0};
static std::atomic<uint64_t> g_native_phys_enable{0};
static std::atomic<float> g_native_last_dt{0.0f};

//////////////////////////////////////////////////////////////////////////////////
// The ground raycast, moved out of CET.
//
// The query is the game's own - the global script function GetSpatialQueriesSystem() and its
// SyncRaycastByQueryFilter method, which is exactly what Lua was calling. Nothing about the
// signature is hardcoded: parameter roles, the filter's mask offsets and the hit position's offset
// are read out of RTTI on first use. A game update that reorders a parameter disables the raycast
// instead of writing past a buffer, and Lua's own raycast keeps working underneath.
//////////////////////////////////////////////////////////////////////////////////
static std::atomic<float> g_query_offset{2.0f};     // start the ray this far above the vehicle
static std::atomic<float> g_query_distance{100.0f}; // how far down to look
static std::atomic<uint64_t> g_query_mask1{0};
// The bits the default collision_filters produce. This is a floor, not the answer: the groups come
// from a user setting, and which bit a group lands on can move between game versions, so Lua's
// AddGroup result stays the primary source and only fails to arrive if CET never got the chance.
static std::atomic<uint64_t> g_query_mask2{144115188075855872ULL};

struct GroundRayLayout
{
    enum Role : uint8_t
    {
        kFrom = 0,
        kTo,
        kFilter,
        kBool,
        kOut,
        kSkip,
    };

    bool dead = false; // the signature could not be read; retrying will not help
    bool usable = false;
    const char* why = "not probed";

    const RED4ext::CBaseRTTIType* sys_type = nullptr; // gameISpatialQueriesSystem, via IGameInstance::GetSystem
    RED4ext::CBaseFunction* func = nullptr;
    std::vector<uint8_t> roles;
    uint32_t filter_size = 0;
    int32_t mask1_off = -1;
    int32_t mask2_off = -1;
    uint32_t out_size = 0;
    int32_t hit_z_off = -1;
    int32_t param_count = 0;
};
static GroundRayLayout g_ray;
static bool g_ray_reported = false;

static void ReportGroundRay()
{
    if (g_ray_reported || g_sdk == nullptr)
    {
        return;
    }
    g_ray_reported = true;
    if (!g_ray.usable)
    {
        // Every failure that reports is permanent - a missing type or an unreadable signature does not
        // fix itself between frames. The one retryable case (no world loaded yet) returns silently.
        g_ray.dead = true;
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf), "DAV dbg: ground ray %s (%s) params=%d filter=%u m1=%d m2=%d out=%u hit_z=%d",
                  g_ray.usable ? "ready" : "off", g_ray.why, g_ray.param_count, g_ray.filter_size, g_ray.mask1_off,
                  g_ray.mask2_off, g_ray.out_size, g_ray.hit_z_off);
    g_sdk->logger->Info(g_plugin_handle, buf);
}

// Property offset inside a struct type, or -1. Structures are CClass in this SDK, so the same lookup
// serves the collision filter and the hit info.
static int32_t FindPropertyOffset(RED4ext::CBaseRTTIType* aType, const char* aName)
{
    if (aType == nullptr || aType->GetType() != RED4ext::ERTTIType::Class)
    {
        return -1;
    }
    auto* cls = static_cast<RED4ext::CClass*>(aType);
    RED4ext::DynArray<RED4ext::CProperty*> props;
    cls->GetProperties(props);
    for (uint32_t i = 0; i < props.size; ++i)
    {
        auto* p = props[i];
        if (p != nullptr && p->name == aName)
        {
            return static_cast<int32_t>(p->valueOffset);
        }
    }
    return -1;
}

// A wrong layout is an access violation rather than a C++ exception, and the game would take it.
// Kept in its own function with nothing that needs unwinding so the guard is legal.
// Exception code from the most recent SafeExecute (0 = no exception); logged by the sound path
// so "threw" finally says WHAT was thrown (0xC0000005 = access violation, etc.).
static std::atomic<uint32_t> g_last_exception_code{0};

static bool SafeExecute(void* aInstance, RED4ext::CBaseFunction* aFunc, void* aOut, RED4ext::StackArgs_t* aArgs)
{
    __try
    {
        g_last_exception_code.store(0, std::memory_order_relaxed);
        return RED4ext::ExecuteFunction(aInstance, aFunc, aOut, *aArgs);
    }
    __except ((g_last_exception_code.store(static_cast<uint32_t>(GetExceptionCode()), std::memory_order_relaxed),
               EXCEPTION_EXECUTE_HANDLER))
    {
        return false;
    }
}

// Read the query's shape out of RTTI. Game thread only. Retries until a world is loaded, then stops.
static RED4ext::IScriptable* SafeGetSystem(RED4ext::IGameInstance* aGi, const RED4ext::CBaseRTTIType* aType);

static bool ResolveGroundRay()
{
    if (g_ray.usable)
    {
        return true;
    }
    if (g_ray.dead)
    {
        return false;
    }
    auto* rtti = RED4ext::CRTTISystem::Get();
    if (rtti == nullptr)
    {
        g_ray.why = "rtti missing";
        ReportGroundRay();
        return false;
    }
    auto* sq_type = rtti->GetClass("gameISpatialQueriesSystem");
    if (sq_type == nullptr)
    {
        // RTTI fills in during boot, so an early probe can legitimately miss the class.
        // Retryable - unlike the structural failures below this must not mark the ray dead.
        g_ray.why = "gameISpatialQueriesSystem missing";
        return false;
    }
    g_ray.sys_type = sq_type;

    RED4ext::ScriptGameInstance gi;
    RED4ext::IScriptable* sys_inst = (gi.instance != nullptr) ? SafeGetSystem(gi.instance, sq_type) : nullptr;
    if (sys_inst == nullptr)
    {
        // No world yet (boot, or between saves). Not an error - try again next frame.
        return false;
    }

    auto* cls = sys_inst->GetType();
    auto* func = (cls != nullptr) ? cls->GetFunction("SyncRaycastByQueryFilter") : nullptr;
    if (func == nullptr)
    {
        g_ray.why = "SyncRaycastByQueryFilter missing";
        ReportGroundRay();
        return false;
    }
    g_ray.func = func;

    bool have_from = false;
    bool have_to = false;
    for (uint32_t i = 0; i < func->params.size; ++i)
    {
        auto* p = func->params[i];
        auto* t = (p != nullptr) ? p->type : nullptr;
        if (t == nullptr)
        {
            g_ray.why = "param without a type";
            ReportGroundRay();
            return false;
        }
        g_ray.param_count = static_cast<int32_t>(i) + 1;
        uint8_t role = GroundRayLayout::kBool;
        if (p->flags.isReturn)
        {
            role = GroundRayLayout::kSkip; // the return slot is not a stack argument
        }
        else if (p->flags.isOut)
        {
            role = GroundRayLayout::kOut;
            g_ray.out_size = t->GetSize();
            const int32_t pos_off = FindPropertyOffset(t, "position");
            if (pos_off < 0 || static_cast<uint32_t>(pos_off) + 12 > g_ray.out_size)
            {
                g_ray.why = "hit info has no usable position";
                ReportGroundRay();
                return false;
            }
            g_ray.hit_z_off = pos_off + 8; // Vector3 and Vector4 both put z third
        }
        else
        {
            const RED4ext::CName tn = t->GetName();
            if (tn == "Vector4" && !have_from)
            {
                role = GroundRayLayout::kFrom;
                have_from = true;
            }
            else if (tn == "Vector4")
            {
                role = GroundRayLayout::kTo;
                have_to = true;
            }
            else if (tn == "Boolean" || tn == "Bool")
            {
                // staticOnly/dynamicOnly inputs (and any other flag). Lua's own ground query passes
                // false/false - the masks decide - and kBool args land as false, matching it.
                role = GroundRayLayout::kBool;
            }
            else
            {
                // The collision filter, whatever the game calls it this patch.
                const char* s = tn.ToString();
                if (s != nullptr && std::strstr(s, "QueryFilter") != nullptr)
                {
                    role = GroundRayLayout::kFilter;
                    g_ray.filter_size = t->GetSize();
                    g_ray.mask1_off = FindPropertyOffset(t, "mask1");
                    g_ray.mask2_off = FindPropertyOffset(t, "mask2");
                    if (g_ray.filter_size < 16 || (g_ray.mask1_off < 0 && g_ray.mask2_off < 0))
                    {
                        g_ray.why = "cannot find the filter masks";
                        ReportGroundRay();
                        return false;
                    }
                }
                else
                {
                    static char s_why_buf[320];
                    int used = std::snprintf(s_why_buf, sizeof(s_why_buf), "unexpected parameter '%s' [", s ? s : "?");
                    for (uint32_t k = 0; k < func->params.size && used > 0 && used < 260; ++k)
                    {
                        auto* q = func->params.entries[k];
                        auto* qt = (q != nullptr) ? q->type : nullptr;
                        const char* qn = (q != nullptr && q->name.ToString() != nullptr) ? q->name.ToString() : "?";
                        const char* qtn = (qt != nullptr && qt->GetName().ToString() != nullptr) ? qt->GetName().ToString() : "?";
                        used += std::snprintf(s_why_buf + used, sizeof(s_why_buf) - used, "%s%s:%s%s%s", k ? "," : "",
                                              qn, qtn, (q != nullptr && q->flags.isReturn) ? "(ret)" : "",
                                              (q != nullptr && q->flags.isOut) ? "(out)" : "");
                    }
                    if (used > 0)
                        std::snprintf(s_why_buf + used, sizeof(s_why_buf) - used, "]");
                    g_ray.why = s_why_buf;
                    ReportGroundRay();
                    return false;
                }
            }
        }
        g_ray.roles.push_back(role);
    }
    if (!have_from || !have_to || g_ray.out_size == 0)
    {
        g_ray.why = "signature not understood";
        ReportGroundRay();
        return false;
    }
    g_ray.usable = true;
    g_ray.why = "ok";
    ReportGroundRay();
    return true;
}

// One downward query, from (ax, ay, aTop) to aDist below it. Game thread only - it walks the live
// physics scene, which is what CET's own loop used to pay for.
static bool RaycastGroundZ(float ax, float ay, float aTop, float aDist, float* aOutZ)
{
    if (!ResolveGroundRay())
    {
        return false;
    }
    // The system comes from IGameInstance::GetSystem (the proven cpPlayerSystem path); there is
    // no global GetSpatialQueriesSystem function in RTTI - the earlier lookups all missed.
    RED4ext::ScriptGameInstance gi;
    RED4ext::IScriptable* sys_inst = (gi.instance != nullptr && g_ray.sys_type != nullptr)
                                         ? SafeGetSystem(gi.instance, g_ray.sys_type)
                                         : nullptr;
    if (sys_inst == nullptr)
    {
        return false;
    }

    RED4ext::Vector4 from{ax, ay, aTop, 1.0f};
    RED4ext::Vector4 to{ax, ay, aTop - aDist, 1.0f};
    std::vector<uint8_t> filter(g_ray.filter_size, 0);
    const uint64_t m1 = g_query_mask1.load(std::memory_order_relaxed);
    const uint64_t m2 = g_query_mask2.load(std::memory_order_relaxed);
    if (g_ray.mask1_off >= 0)
    {
        std::memcpy(filter.data() + g_ray.mask1_off, &m1, sizeof(m1));
    }
    if (g_ray.mask2_off >= 0)
    {
        std::memcpy(filter.data() + g_ray.mask2_off, &m2, sizeof(m2));
    }
    std::vector<uint8_t> out(g_ray.out_size, 0);
    bool is_false = false;

    RED4ext::StackArgs_t args;
    for (size_t i = 0; i < g_ray.roles.size(); ++i)
    {
        switch (g_ray.roles[i])
        {
        case GroundRayLayout::kFrom:
            args.emplace_back(nullptr, &from);
            break;
        case GroundRayLayout::kTo:
            args.emplace_back(nullptr, &to);
            break;
        case GroundRayLayout::kFilter:
            args.emplace_back(nullptr, filter.data());
            break;
        case GroundRayLayout::kOut:
            args.emplace_back(nullptr, out.data());
            break;
        case GroundRayLayout::kSkip:
            break;
        default:
            args.emplace_back(nullptr, &is_false);
            break;
        }
    }

    bool is_hit = false;
    if (!SafeExecute(sys_inst, g_ray.func, &is_hit, &args) || !is_hit)
    {
        return false;
    }
    float z = 0.0f;
    std::memcpy(&z, out.data() + g_ray.hit_z_off, sizeof(z));
    if (!std::isfinite(z))
    {
        return false;
    }
    *aOutZ = z;
    return true;
}

// Flight control state pushed from CET (game thread), read on the physics thread (double buffer).
struct FlightControlState
{
    RED4ext::Vector3 velocity;
    RED4ext::Vector3 angularVelocity;
    float torqueGain;
    int mode; // 0=off (CET drives), 1=add force/torque, 2=write velocity directly, 3=native flight model
};

// --- Native flight model (port of Engine:CalculateAddVelocity/Run for manual flight) ---
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

// Native idle hover (Lua mode 4). Inside the physics hook the velocity, angular velocity,
// orientation and world position are already in hand, so the hover control law runs here for free
// instead of costing Lua a raycast and several game calls on every tick.
struct IdleParams
{
    float ground_z;           // world Z of the ground under the AV, pushed by Lua (slow refresh)
    float dest_height;        // Engine.minimum_distance_to_ground
    float damping;            // CalculateIdleMode velocity damping (0.2)
    float height_gain;        // CalculateIdleMode height_gain (0.5)
    float pitch_band;         // pitch_restore_amount (also the RestoreRate full rate, as in Lua)
    float roll_band;          // roll_restore_amount
    float restore_boundary;   // restore_boundary_deg
    float dt_scale;
    float max_roll;
    float max_pitch;
    float force_restore_angle;
    float air_h;              // horizontal_air_resistance_const
    float air_v;              // vertical_air_resistance_const
    float idle_gravity;       // is_enable_idle_gravity (0 = no altitude term at all)
    float max_vz;             // safety clamp on the vertical target
    float valid;
};
static IdleParams g_idle_buf[2] = {};
static std::atomic<int> g_idle_idx{0};

// Game-convention Euler angles from a quaternion. Same formulas FmStep verified in game (local axes
// are forward = AxisY, right = AxisX, up = AxisZ), kept separate so FmStep stays untouched.
static void GameEuler(const RED4ext::Quaternion& q, float& roll, float& pitch, float& yaw)
{
    const float fz = 2.0f * (q.j * q.k + q.r * q.i);
    const float rx = 1.0f - 2.0f * (q.j * q.j + q.k * q.k);
    const float ry = 2.0f * (q.i * q.j + q.r * q.k);
    const float rz = 2.0f * (q.i * q.k - q.r * q.j);
    const float uz = 1.0f - 2.0f * (q.i * q.i + q.j * q.j);
    roll = fm_deg(std::atan2(-rz, std::sqrt(fz * fz + uz * uz)));
    pitch = fm_deg(std::atan2(fz, uz));
    yaw = fm_wrap180(fm_deg(std::atan2(ry, rx)));
}

// One physics tick of the unboarded hover: CalculateIdleMode's law, then Engine:Run's own attitude
// restore added on top. Lua really does restore twice on this path (CalculateIdleMode returns world
// rates, Run adds its own d_roll/d_pitch/d_yaw to them) and that stiffness is what the hover feels
// like, so the duplication is reproduced rather than cleaned up.
static void IdleStep(const RED4ext::vehicle::PhysicsData* pd, const IdleParams& ip,
                     RED4ext::Vector3& out_vel, RED4ext::Vector3& out_ang)
{
    const RED4ext::Vector3& v = pd->velocity;

    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (ip.idle_gravity != 0.0f)
    {
        z = -v.Z * ip.damping;
        const float h = pd->worldPosition.Z - ip.ground_z;
        const float dh = ip.dest_height - h;
        if (std::abs(dh) > 0.01f)
            z += dh * ip.height_gain;
        z = fm_clamp(z, -ip.max_vz, ip.max_vz);
    }
    // Engine:Run's air resistance, applied after the idle term exactly as in Lua.
    x -= ip.air_h * v.X;
    y -= ip.air_h * v.Y;
    z -= ip.air_v * v.Z;
    out_vel.X = x;
    out_vel.Y = y;
    out_vel.Z = z;

    RED4ext::Quaternion q = pd->orientation;
    // Same source preference FmStep uses: the transform the game maintains on the entity itself is
    // what CET's GetEulerAngles reported, and the physics field can lag it. Restoring against a stale
    // orientation is a good way to lock the vehicle into a tilted pose.
    const int32_t att_off = g_att_offset.load(std::memory_order_relaxed);
    if (att_off >= 0 && pd->vehicle != nullptr)
    {
        const auto* ent = reinterpret_cast<const RED4ext::Quaternion*>(reinterpret_cast<uintptr_t>(pd->vehicle) +
                                                                       att_off);
        const float n2 = ent->i * ent->i + ent->j * ent->j + ent->k * ent->k + ent->r * ent->r;
        if (std::isfinite(n2) && n2 > 0.9f && n2 < 1.1f)
        {
            q = *ent;
        }
    }
    const float qn = std::sqrt(q.i * q.i + q.j * q.j + q.k * q.k + q.r * q.r);
    if (qn > 0.0f)
    {
        q.i /= qn; q.j /= qn; q.k /= qn; q.r /= qn;
    }
    float roll, pitch, yaw;
    GameEuler(q, roll, pitch, yaw);

    // CalculateIdleMode: pitch only (local roll stays 0); the roll/yaw terms come out of the Euler
    // coupling inside RotSpeed, exactly as they do in Lua.
    float lp0 = 0.0f;
    if (pitch > ip.pitch_band)
        lp0 = -RestoreRate(pitch - ip.pitch_band, ip.pitch_band, ip.restore_boundary, ip.dt_scale);
    else if (pitch < -ip.pitch_band)
        lp0 = RestoreRate(-pitch - ip.pitch_band, ip.pitch_band, ip.restore_boundary, ip.dt_scale);
    else
        lp0 = -pitch;
    float dp0, dr0, dy0;
    RotSpeed(0.0f, lp0, roll, pitch, yaw, dp0, dr0, dy0);

    // Engine:Run's restore (roll and pitch, the max_roll taper, the force-restore hard cut).
    float lr = 0.0f, lp = 0.0f;
    if (pitch > ip.pitch_band)
        lp -= RestoreRate(pitch - ip.pitch_band, ip.pitch_band, ip.restore_boundary, ip.dt_scale);
    else if (pitch < -ip.pitch_band)
        lp += RestoreRate(-pitch - ip.pitch_band, ip.pitch_band, ip.restore_boundary, ip.dt_scale);
    if (roll > ip.roll_band)
        lr -= RestoreRate(roll - ip.roll_band, ip.roll_band, ip.restore_boundary, ip.dt_scale);
    else if (roll < -ip.roll_band)
        lr += RestoreRate(-roll - ip.roll_band, ip.roll_band, ip.restore_boundary, ip.dt_scale);
    if (roll > ip.max_roll)
        lr -= (roll - ip.max_roll) * 0.5f;
    else if (roll < -ip.max_roll)
        lr += (-ip.max_roll - roll) * 0.5f;
    if (roll > ip.force_restore_angle || roll < -ip.force_restore_angle)
        lr = -roll;
    else if (pitch > ip.force_restore_angle || pitch < -ip.force_restore_angle)
        lp = -pitch;
    float dp1, dr1, dy1;
    RotSpeed(lr, lp, roll, pitch, yaw, dp1, dr1, dy1);

    out_ang.X = fm_clamp(dr0 + dr1, -500.0f, 500.0f);
    out_ang.Y = fm_clamp(dp0 + dp1, -500.0f, 500.0f);
    out_ang.Z = fm_clamp(dy0 + dy1, -500.0f, 500.0f);
}

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
            FlightControlState fc = g_fc_buf[g_fc_idx.load(std::memory_order_acquire)];
            // Phase 5 auto re-assert: Lua's per-input mode 3 re-assert (the AV:Operate shim) is
            // gone. With a driver aboard, no autopilot, and any held command non-zero, the flight
            // model must run regardless of the last mode Lua pushed - landing with throttle held
            // means takeoff intent, exactly the old shim's semantics.
            if ((fc.mode == 2 || fc.mode == 4) && HeldCommandActive())
            {
                const int32_t flags = g_flight_flags.load(std::memory_order_relaxed);
                // bit2: helicopter mode drives manual flight through mode 2 velocity targets; the
                // mode 3 re-assert is an airplane-only rescue and would freeze every heli input.
                if ((flags & 1) != 0 && (flags & 2) == 0 && (flags & 4) == 0)
                {
                    fc.mode = 3;
                    fc.velocity = RED4ext::Vector3{0, 0, 0};
                    fc.angularVelocity = RED4ext::Vector3{0, 0, 0};
                    // Persist so the game-thread views (idle flag, heartbeat) match what the hook
                    // actually runs; Lua's next SetFlightControl overwrites this as usual.
                    FlightControlState& w = g_fc_buf[g_fc_idx.load(std::memory_order_acquire)];
                    w.mode = 3;
                    w.velocity = RED4ext::Vector3{0, 0, 0};
                    w.angularVelocity = RED4ext::Vector3{0, 0, 0};
                }
            }
            if ((dbg_n % 240) == 120 && g_sdk)
            {
                // Heartbeat: proves the hook keeps running for our vehicle (dismount/park states were
                // suspected of stopping the physics step, which would silently drop every mode push).
                char hbuf[128];
                std::snprintf(hbuf, sizeof(hbuf), "DAV dbg: hook apply heartbeat #%llu mode=%d vz=%.2f",
                              (unsigned long long)dbg_n, fc.mode, fc.velocity.Z);
                g_sdk->logger->Info(g_plugin_handle, hbuf);
            }
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
            else if (fc.mode == 4)
            {
                // Native idle hover: the regulator runs here, and the ground reference comes from the
                // game-thread state rather than a Lua raycast.
                IdleParams ip = g_idle_buf[g_idle_idx.load(std::memory_order_acquire)];
                if (g_ground_valid.load(std::memory_order_relaxed) != 0.0f)
                {
                    // Contact teaches the ground on the game thread; CET only seeds it.
                    ip.ground_z = g_ground_z.load(std::memory_order_relaxed);
                }
                // Engine:CalculateIdleMode skips its altitude term while the vehicle touches
                // something - that is what lets a landed AV rest instead of being shoved upward. Lua
                // re-checked IsCollision() every tick; the published contact flag is the same signal,
                // and without it a settled AV was pinned against whatever it had landed on.
                if (g_av_on_ground.load(std::memory_order_relaxed) != 0.0f)
                {
                    ip.idle_gravity = 0.0f;
                }
                RED4ext::Vector3 vel = fc.velocity;
                RED4ext::Vector3 ang = fc.angularVelocity;
                // A non-zero Lua target means something else is driving the vehicle (a scripted
                // descent, a command that arrived after the mode push): hold that, do not regulate.
                const bool idle_target = std::abs(fc.velocity.X) < 0.01f && std::abs(fc.velocity.Y) < 0.01f
                                         && std::abs(fc.velocity.Z) < 0.01f
                                         && std::abs(fc.angularVelocity.X) < 0.01f
                                         && std::abs(fc.angularVelocity.Y) < 0.01f
                                         && std::abs(fc.angularVelocity.Z) < 0.01f;
                if (ip.valid != 0.0f && idle_target)
                {
                    IdleStep(physicsData, ip, vel, ang);
                }
                physicsData->velocity = vel;
                physicsData->angularVelocity = ang;
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

// Fallback puppet source removed: cpPlayerSystem::GetPlayer (with a proper 0x10 Handle out) is
// the sole, proven puppet resolution path. The earlier Lua-pushed entity-id fallback existed only
// while the handle-out size bug made DLL-side resolution crash; it is dead code now.

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
        // A fresh vehicle has no ground history. A stale reference from the previous one would send
        // the new hover to the wrong altitude, which is what made a spawn lurch before despawning.
        g_ground_valid.store(0.0f, std::memory_order_relaxed);
        g_ground_z.store(0.0f, std::memory_order_relaxed);
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive() && g_fly_vehicle->physicsData)
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
    if (VehicleBodyAlive())
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
    if (VehicleBodyAlive())
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
    if (VehicleBodyAlive())
    {
        *aOut = g_fly_vehicle->isOnGround;
    }
}

// Probe: lets CET verify the native physics tick hook actually fires.
void GetNativeTickCount(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = (float)g_native_tick_count.load(std::memory_order_relaxed);
}

// CET pushes the flight control target; the physics-tick hook applies it every tick.
// mode: 0=off, 1=add force/torque (AddForce semantics), 2=write velocity directly (ChangeVelocity semantics).
// Push the flight context bits (bit0 = player aboard, bit1 = autopilot) that gate the physics
// hook's auto mode re-assert. The hook must not run the flight model without a driver aboard
// (the mode 4 idle hover lives outside the vehicle), and must not fight the autopilot.
void SetFlightFlags(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    int32_t flags = 0;
    RED4ext::GetParameter(aFrame, &flags);
    aFrame->code++;
    g_flight_flags.store(flags, std::memory_order_relaxed);
    *aOut = 1.0f;
}

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

// Push all flight model tunables (order must match Lua Engine:PushNativeParams).
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

// Push helicopter-mode tunables + engine constants (same packing as SetFlightParams).
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

// Push attitude-limit constants (same packing as SetFlightParams).
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

// Push the idle hover reference + gains (game thread; physics hook consumes it).
// ground_z is the only value that changes often, and "often" is a couple of times a second.
void SetIdleHoverParams(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::Vector4 a, b, c, d;
    RED4ext::GetParameter(aFrame, &a);
    RED4ext::GetParameter(aFrame, &b);
    RED4ext::GetParameter(aFrame, &c);
    RED4ext::GetParameter(aFrame, &d);
    aFrame->code++; // skip ParamEnd

    int next = 1 - g_idle_idx.load(std::memory_order_relaxed);
    IdleParams& ip = g_idle_buf[next];
    ip.ground_z = a.X;
    ip.dest_height = a.Y;
    ip.damping = a.Z;
    ip.height_gain = a.W;
    ip.pitch_band = b.X;
    ip.roll_band = b.Y;
    ip.restore_boundary = b.Z;
    ip.dt_scale = b.W;
    ip.max_roll = c.X;
    ip.max_pitch = c.Y;
    ip.force_restore_angle = c.Z;
    ip.air_h = c.W;
    ip.air_v = d.X;
    ip.idle_gravity = d.Y;
    ip.max_vz = d.Z;
    ip.valid = d.W;
    g_idle_idx.store(next, std::memory_order_release);
    *aOut = 1;
}

// Push the per-tick command list (game thread; physics hook consumes it).
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

// Read the native RPM integrator for the HUD (same scale as Engine:GetRPMCount).
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

// SetNativeCommand native: one command edge.
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

// Defined further down with the threshold monitor code.
void SetThresholds(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4);
void SetLandingVFX(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4);

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetThresholds()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetThresholds", "SetThresholds", &SetThresholds,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Float", "height_thr");
    func->AddParam("Float", "dist_thr");
    func->AddParam("Float", "entry_radius");
    func->AddParam("Float", "keepalive");
    func->AddParam("Int32", "mask");
    func->AddParam("Vector4", "entry_off");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetLandingVFX()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetLandingVFX", "SetLandingVFX", &SetLandingVFX,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Float", "enabled");
    func->AddParam("Vector4", "offset");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetFlightFlags()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetFlightFlags", "SetFlightFlags", &SetFlightFlags,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Int32", "flags");
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

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetIdleHoverParams()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetIdleHoverParams", "SetIdleHoverParams", &SetIdleHoverParams, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector4", "a");
    func->AddParam("Vector4", "b");
    func->AddParam("Vector4", "c");
    func->AddParam("Vector4", "d");
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

//////////////////////////////////////////////////////////////////////////////////
// Thruster swing + sound layer edges (game thread, driven from DavGameState_OnUpdate).
//
// Both were Lua loop work: MoveThruster integrated the swing angle per control tick, ControlSound
// latched the accel/thruster layers and stopped them after a grace of idle ticks. The held command
// state (g_cmd_val) already carries everything they consumed, so the DLL runs both from its own
// per-frame cadence. Lua pushes the setup: thruster gains and component names once per spawn, the
// sound action lists and grace once per engine init.
//
// The sound layers cannot be finished in C++ - the samples live in CET's sound object - so the DLL
// only decides WHEN a layer starts or stops and dispatches a REDscript custom event (DAV.SndEdgeEvent)
// through the game's CallbackSystem per edge. The CET side registers a NewProxy listener straight
// on the CallbackSystem (Core:InitSoundEdgeListeners) and plays the samples. QueueEvent with a
// custom name is dropped by the game, hence the CallbackSystem.
//////////////////////////////////////////////////////////////////////////////////

// Game-thread-only state: Lua pushes from CET (game thread), OnUpdate consumes on the same thread.
static bool g_thr_enabled = false;
static float g_thr_step_rate = 12.0f;    // deg/s at full command (Lua converts its per-tick step to deg/s)
static float g_thr_restore_rate = 6.0f;  // deg/s restore toward zero
static float g_thr_max = 0.0f;           // deg
// Rotation axis in the component's local frame: 0=X+ 1=X- 2=Y+ 3=Y- 4=Z+ 5=Z-.
// The engine components' local Y happens to run along the vehicle's forward axis, so the old
// hardcoded Y yaw looked like a roll (thrusters swinging sideways); Lua picks the axis per
// vehicle with one number it can be tuned with.
static int32_t g_thr_axis = 2;
static uint32_t g_thr_setup_seq = 0;     // bumped whenever Lua re-pushes the setup
static std::atomic<float> g_thr_ext_fwd{0.0f};  // autopilot swing input (Lua pushes on change)
static std::atomic<float> g_thr_ext_back{0.0f};
static uint32_t s_thr_resolved_seq = 0xFFFFFFFFu;
static float s_thr_angle = 0.0f;
static float s_thr_written = 1e30f; // last angle actually written to the components
static std::vector<std::string> s_thr_names;
static std::vector<RED4ext::IScriptable*> s_thr_components; // owned by the vehicle entity

static std::vector<int32_t> g_snd_accel_ids;
static std::vector<int32_t> g_snd_thr_ids;
static float g_snd_grace = 0.3f;
static bool g_snd_accel_on = false;
static bool g_snd_thr_on = false;
static float g_snd_accel_idle = 0.0f;
static float g_snd_thr_idle = 0.0f;

static bool HeldAction(int32_t aId)
{
    const int idx = static_cast<int>(aId) + 1; // g_cmd_val index = Def.ActionList + 1
    return (idx >= 0 && idx < 64) && g_cmd_val[idx].load(std::memory_order_relaxed) != 0.0f;
}

static void SplitList(const char* aText, char aSep, std::vector<std::string>& aOut)
{
    if (aText == nullptr)
        return;
    const std::string s(aText);
    size_t pos = 0;
    while (pos < s.size())
    {
        size_t next = s.find(aSep, pos);
        if (next == std::string::npos)
            next = s.size();
        if (next > pos)
            aOut.emplace_back(s.substr(pos, next - pos));
        pos = next + 1;
    }
}

static void SplitIds(const char* aText, std::vector<int32_t>& aOut)
{
    std::vector<std::string> parts;
    SplitList(aText, ',', parts);
    for (const auto& p : parts)
        aOut.push_back(static_cast<int32_t>(std::strtol(p.c_str(), nullptr, 10)));
}

// Resolve the swing targets by name straight out of the entity's component storage in memory.
// Script-calling FindComponentByName from the DLL crashed the game, and CET refuses to marshal
// component handles into native function parameters (concrete ref types, IScriptable, and
// ScriptRef all failed), so neither "DLL resolves via script" nor "Lua passes handles" works.
// What remains is the plain data path: Lua passes the names (SetThrusterSetup), the DLL walks
// entEntity.componentsStorage (embedded at +0x70; components DynArray at +0x30: entries ptr,
// u32 size, u32 capacity) and matches entIComponent.name (CName at +0x40). Every read is SEH
// guarded, the walk is capped, and a failed resolve just leaves the thruster off.
static bool SafePeekPtr(void* aAddr, void** aOut)
{
    __try
    {
        *aOut = *reinterpret_cast<void**>(aAddr);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

static void ResolveThrusterComponents()
{
    s_thr_components.clear();
    if (!g_fly_vehicle || g_fly_vehicle.instance == nullptr || s_thr_names.empty())
        return;

    // Pre-resolve the wanted names to hashes once. CName is a case-insensitive FNV-1a 64 hash,
    // so the pool lookup is reproduced locally (this SDK's CNamePool has no Find).
    std::vector<RED4ext::CName> wanted;
    wanted.reserve(s_thr_names.size());
    for (const auto& name : s_thr_names)
    {
        std::string lower = name;
        for (auto& c : lower)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        wanted.push_back(RED4ext::CName(RED4ext::FNV1a64(lower.c_str())));
    }

    auto* entity = reinterpret_cast<uint8_t*>(g_fly_vehicle.instance);
    void* entries = nullptr;
    void* sizeCap = nullptr;
    if (!SafePeekPtr(entity + 0x70 + 0x30, &entries) || entries == nullptr)
    {
        if (g_sdk)
            g_sdk->logger->Info(g_plugin_handle, "DAV dbg: thruster mem-resolve: no component array");
        return;
    }
    if (!SafePeekPtr(entity + 0x70 + 0x38, &sizeCap))
        return;
    const uint32_t count = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(sizeCap) & 0xFFFFFFFFull);

    for (uint32_t i = 0; i < count && i < 1024; ++i)
    {
        void* comp = nullptr;
        if (!SafePeekPtr(reinterpret_cast<uint8_t*>(entries) + i * 16, &comp) || comp == nullptr)
            continue;
        void* nameRaw = nullptr;
        if (!SafePeekPtr(reinterpret_cast<uint8_t*>(comp) + 0x40, &nameRaw))
            continue;
        const RED4ext::CName compName = *reinterpret_cast<RED4ext::CName*>(&nameRaw);
        if (compName == RED4ext::CName())
            continue;
        for (const auto& want : wanted)
        {
            if (want != RED4ext::CName() && compName == want)
            {
                s_thr_components.push_back(reinterpret_cast<RED4ext::IScriptable*>(comp));
                break;
            }
        }
    }
    if (g_sdk)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "DAV dbg: thruster comps mem-resolved %d/%d (entity comps=%u)",
                      (int)s_thr_components.size(), (int)s_thr_names.size(), count);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
}

static void ThrusterTick(float aDt)
{
    if (g_thr_setup_seq != s_thr_resolved_seq)
    {
        s_thr_resolved_seq = g_thr_setup_seq;
        s_thr_written = 1e30f;
        ResolveThrusterComponents();
    }
    if (!g_thr_enabled || s_thr_components.empty())
        return;
    // Same order MoveThruster used: restore first, then the command, then clamp.
    const float restore = g_thr_restore_rate * aDt;
    if (s_thr_angle > restore)
        s_thr_angle -= restore;
    else if (s_thr_angle < -restore)
        s_thr_angle += restore;
    else
        s_thr_angle = 0.0f;
    if (HeldAction(1) || g_thr_ext_fwd.load(std::memory_order_relaxed) != 0.0f) // Forward (input or autopilot)
        s_thr_angle -= g_thr_step_rate * aDt;
    if (HeldAction(2) || g_thr_ext_back.load(std::memory_order_relaxed) != 0.0f) // Backward
        s_thr_angle += g_thr_step_rate * aDt;
    if (s_thr_angle > g_thr_max)
        s_thr_angle = g_thr_max;
    else if (s_thr_angle < -g_thr_max)
        s_thr_angle = -g_thr_max;
    if (s_thr_angle == s_thr_written)
        return;
    // Axis-angle quaternion (degrees). The axis is Lua-tunable (g_thr_axis) because the component
    // local frames differ from the vehicle's axes.
    const float half = s_thr_angle * 0.5f * 0.017453292519943295f;
    const float s = sinf(half);
    RED4ext::Quaternion q(0.0f, 0.0f, 0.0f, cosf(half));
    switch (g_thr_axis)
    {
    case 0: q.i = s; break;
    case 1: q.i = -s; break;
    case 2: q.j = s; break;
    case 3: q.j = -s; break;
    case 4: q.k = s; break;
    case 5: q.k = -s; break;
    default: q.j = s; break;
    }
    for (auto* comp : s_thr_components)
    {
        // SetLocalOrientation may live on a base class (entComponent/entITransformAttachable),
        // and GetFunction only sees the class's own table: walk the chain.
        RED4ext::CBaseFunction* set_orient = nullptr;
        for (auto* comp_cls = comp->GetType(); comp_cls != nullptr && set_orient == nullptr; comp_cls = comp_cls->parent)
            set_orient = comp_cls->GetFunction("SetLocalOrientation");
        if (set_orient == nullptr)
            continue;
        RED4ext::StackArgs_t args;
        args.emplace_back(nullptr, &q);
        SafeExecute(comp, set_orient, nullptr, &args);
    }
    s_thr_written = s_thr_angle;
}

// Sound edges cross to the game's action queue inside DispatchSndEdge; nothing polls them.
// Sound edges are queued straight into the game's own action event queue from the game thread:
// the edge detector (SoundEdgeTick) rides the frame clock, and playback is just
// player:QueueEvent(ActionEvent) - a plain game API the DLL may call right here. The earlier
// design shipped edges back to Lua through a shared atomic polled by the control loop purely
// because the playback code lived in Lua; that made the sound depend on the Lua control loop.
//
// game::ActionEvent (size 0x68): red::Event header, then from AI::AIEvent name CName @0x40 and
// timeToLive float @0x48, then eventAction CName @0x50. CName is the lowercase FNV-1a hash.
// ActionEvent is a script CLASS (ref type, Lua calls .new() on it), so a plain zeroed buffer is
// not enough: the instance must come from CClass::CreateInstance to carry the vtable and be
// refcount-manageable. The Handle guard mirrors what the script GC would do with Lua's .new().
// Step breadcrumbs for the sound queue path (RED4ext logger only; the unbuffered trace-file
// mirror that was used to pin down the crash is gone now that the path is stable).
static void DavLogStep(const char* aWhat)
{
    if (g_sdk != nullptr)
        g_sdk->logger->Info(g_plugin_handle, aWhat);
}

// SEH wrapper around the IGameInstance::GetSystem virtual (v1's GetGameSystem equivalent).
static RED4ext::IScriptable* SafeGetSystem(RED4ext::IGameInstance* aGi, const RED4ext::CBaseRTTIType* aType)
{
    __try
    {
        return aGi->GetSystem(aType);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return nullptr;
    }
}

// CBaseRTTIType::GetSize is a game virtual; isolate it under SEH (also: C2712 forbids __try in
// functions with objects that need unwinding, so this helper stays POD).
static uint32_t SafeTypeSize(const RED4ext::CBaseRTTIType* aType)
{
    __try
    {
        return aType->GetSize();
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0;
    }
}

// One-shot, read-only RTTI census for the sound path. Pure data reads (no script execution),
// so it cannot destabilize the game. It answers what the guessing cycle kept missing: which
// class really defines QueueEvent and with what signature, the real puppet getter names on
// cpPlayerSystem, and the real ActionEvent field offsets.
// Resolve any entity by id hash (FindEntityByID is the proven DLL-side lookup used for the
// vehicle since the beginning).
// aClassName: RTTI class to instantiate (gameActionEvent for the sound layer and the threshold
// monitor; the monitor relay lives in the MOD's REDscript).
// aTtl: timeToLive. Unknown actions ignore it, so the height monitor smuggles the measured height
// in this float - the Lua side reads event.timeToLive back (see OnDavThresholdEvent).
static void QueueDavEvent(const char* aClassName, const char* aEventName, float aTtl = 1.5f)
{
    const char* fail = nullptr;
    auto* rtti = RED4ext::CRTTISystem::Get();
    RED4ext::IScriptable* player = nullptr;
    // Primary: resolve the puppet fresh from cpPlayerSystem on every edge. The earlier DLL-side
    // attempts failed because the handle<> return (0x10 bytes) was written into an 8-byte slot,
    // smashing the stack; with a proper Handle out the call is safe. Fresh resolution also
    // survives player respawn (the entity id changes), which a Lua-pushed id cannot.
    RED4ext::ScriptGameInstance game_instance;
    auto* ps_type = (rtti != nullptr) ? rtti->GetClass("cpPlayerSystem") : nullptr;
    RED4ext::IScriptable* ps = (ps_type != nullptr && game_instance.instance != nullptr)
                                   ? SafeGetSystem(game_instance.instance, ps_type)
                                   : nullptr;
    if (ps != nullptr && ps->GetType() != nullptr)
    {
        auto* get_player = ps->GetType()->GetFunction("GetPlayer");
        if (get_player != nullptr)
        {
            RED4ext::StackArgs_t no_args;
            RED4ext::Handle<RED4ext::IScriptable> puppet_out{};
            if (SafeExecute(ps, get_player, &puppet_out, &no_args))
                player = puppet_out.instance;
        }
    }
    if (player == nullptr)
        fail = "puppet resolve failed";
    RED4ext::CBaseFunction* queue_event = nullptr;
    if (fail == nullptr)
    {
        for (auto* player_cls = player->GetType(); player_cls != nullptr && queue_event == nullptr;
             player_cls = player_cls->parent)
            queue_event = player_cls->GetFunction("QueueEvent");
        if (queue_event == nullptr)
            fail = "QueueEvent missing on puppet";
    }
    // Lua's ActionEvent.new() works because REDscript strips the package prefix: the RTTI full
    // name of that class is gameActionEvent (runtime dump confirms eventAction@0x50,
    // internalEvent@0x58, size=104). A bare "ActionEvent" does not exist in RTTI.
    auto* evt_cls = (fail == nullptr && rtti != nullptr) ? rtti->GetClass(aClassName) : nullptr;
    if (fail == nullptr && evt_cls == nullptr)
        fail = "event class missing";
    RED4ext::Handle<RED4ext::IScriptable> evt_guard(
        (fail == nullptr) ? evt_cls->CreateInstance<RED4ext::IScriptable*>(true) : nullptr);
    if (fail == nullptr && !evt_guard.instance)
        fail = "ActionEvent CreateInstance failed";
    if (fail != nullptr)
    {
        if (g_sdk)
        {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "DAV dbg: QueueDavEvent fail: %s (%s/%s)", fail, aClassName, aEventName);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
        return;
    }
    // Set the fields through RTTI properties instead of hardcoded offsets: Lua does
    // ActionEvent.new(); evt.eventAction = CName; evt.timeToLive = fade; property access mirrors
    // that exactly and cannot write to a wrong offset.
    auto* evt = static_cast<void*>(evt_guard.instance);
    // timeToLive lives on a parent class, so walk the inheritance chain for each property.
    auto find_prop = [](RED4ext::CClass* aCls, const char* aName) -> RED4ext::CProperty*
    {
        for (auto* c = aCls; c != nullptr; c = c->parent)
        {
            auto* p = c->GetProperty(aName);
            if (p != nullptr)
                return p;
        }
        return nullptr;
    };
    auto* ttl_prop = find_prop(evt_cls, "timeToLive");
    auto* act_prop = find_prop(evt_cls, "eventAction");
    if (ttl_prop == nullptr || act_prop == nullptr)
    {
        if (g_sdk)
        {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "DAV dbg: QueueDavEvent fail: evt props missing (%s)", aEventName);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
        return;
    }
    ttl_prop->SetValue<float>(evt, aTtl); // fade window (or the payload the height monitor sends)
    act_prop->SetValue<RED4ext::CName>(evt, RED4ext::CName(RED4ext::FNV1a64(aEventName)));
    // Optional 'key' field (custom event classes can carry the name in their own property).
    if (auto* key_prop = find_prop(evt_cls, "key"); key_prop != nullptr)
        key_prop->SetValue<RED4ext::CName>(evt, RED4ext::CName(RED4ext::FNV1a64(aEventName)));
    // Build every declared parameter slot, not just the first: QueueEvent may take extra
    // parameters (name/sticky style) and the VM reads whatever the stack holds for missing
    // ones - the leading suspect for the crash that survives the handle-slot fix. Slot 0 gets
    // the ActionEvent handle slot; the rest get zeroed buffers sized from RTTI.
    RED4ext::StackArgs_t args;
    // Handle-typed arguments must be the address of a real Handle object (0x10 bytes: instance +
    // ref controller). Passing the address of a bare void* slot made the VM read 8 bytes of
    // adjacent stack as the ref controller - the crash that survived every earlier fix.
    std::vector<std::vector<uint8_t>> defaults;
    for (uint32_t i = 0; i < queue_event->params.size; ++i)
    {
        if (i == 0)
        {
            args.emplace_back(nullptr, &evt_guard);
            continue;
        }
        auto* prop = queue_event->params.entries[i];
        auto* ptype = (prop != nullptr) ? prop->type : nullptr;
        size_t sz = 8;
        if (ptype != nullptr)
        {
            const uint32_t s = SafeTypeSize(ptype);
            if (s > 0)
                sz = s;
        }
        defaults.emplace_back(sz + 8, 0);
        args.emplace_back(nullptr, defaults.back().data());
    }
    // Ref-type arguments are passed as a pointer to the handle slot (a void* variable holding
    // the instance pointer), not as the instance pointer itself - passing the object directly
    // made the VM read its vftable as a handle, throw through SEH, and freeze the game.
    if (!SafeExecute(player, queue_event, nullptr, &args))
    {
        if (g_sdk)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "DAV dbg: QueueDavEvent fail: QueueEvent threw exc=0x%08X (%s)",
                          g_last_exception_code.load(std::memory_order_relaxed), aEventName);
            g_sdk->logger->Info(g_plugin_handle, buf);
        }
    }
}

static void QueueDavSound(const char* aEventName)
{
    QueueDavEvent("gameActionEvent", aEventName);
}

// ---- Threshold monitor (Phase 5) ----
// Lua pre-arms the thresholds and a per-situation enable mask (1=height, 2=player distance,
// 4=entry area). The monitor rides the frame clock on the game thread: height comes from the
// same ground raycast the hover uses, the player position from one cheap VM call per frame.
// A crossing is announced as a plain gameActionEvent (eventAction = dav_evt_*) queued on the
// puppet through the proven QueueDavEvent path; the MOD's REDscript relay cb forwards it to the
// native callback system ('DAV/Monitor') where the CET listener lives - Lua cannot hook entity
// cbs, and DLL-created custom event classes never reach a cb (registration lives in 'new').
static std::atomic<float> g_mon_height_thr{0.0f};
static std::atomic<float> g_mon_dist_thr{0.0f};
static std::atomic<float> g_mon_entry_radius{0.0f};
static std::atomic<float> g_mon_keepalive{1.0f};
static std::atomic<int32_t> g_mon_mask{0};
static std::atomic<float> g_mon_entry_ox{0.0f};
static std::atomic<float> g_mon_entry_oy{0.0f};
static std::atomic<float> g_mon_entry_oz{0.0f};
// Landing VFX slot: while the AV is inside the warning zone the DLL writes the slot's local
// position itself (Lua used to track the height with an adaptive poll).
static std::atomic<float> g_vfx_enabled{0.0f};

void SetThresholds(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    float height_thr = 0.0f, dist_thr = 0.0f, entry_radius = 0.0f, keepalive = 1.0f;
    int32_t mask = 0;
    RED4ext::Vector4 entry_off{};
    RED4ext::GetParameter(aFrame, &height_thr);
    RED4ext::GetParameter(aFrame, &dist_thr);
    RED4ext::GetParameter(aFrame, &entry_radius);
    RED4ext::GetParameter(aFrame, &keepalive);
    RED4ext::GetParameter(aFrame, &mask);
    RED4ext::GetParameter(aFrame, &entry_off);
    aFrame->code++;
    g_mon_height_thr.store(height_thr, std::memory_order_relaxed);
    g_mon_dist_thr.store(dist_thr, std::memory_order_relaxed);
    g_mon_entry_radius.store(entry_radius, std::memory_order_relaxed);
    g_mon_keepalive.store(keepalive, std::memory_order_relaxed);
    g_mon_entry_ox.store(entry_off.X, std::memory_order_relaxed);
    g_mon_entry_oy.store(entry_off.Y, std::memory_order_relaxed);
    g_mon_entry_oz.store(entry_off.Z, std::memory_order_relaxed);
    // The mask goes last: it arms the monitor, everything it reads is already in place.
    g_mon_mask.store(mask, std::memory_order_relaxed);
    *aOut = 1.0f;
}

void SetLandingVFX(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    float enabled = 0.0f;
    RED4ext::Vector4 offset{};
    RED4ext::GetParameter(aFrame, &enabled);
    RED4ext::GetParameter(aFrame, &offset);
    aFrame->code++;
    g_vfx_enabled.store(enabled, std::memory_order_relaxed);
    *aOut = 1.0f;
}

static bool SafePeekBytes(void* aAddr, void* aDst, size_t aLen)
{
    __try
    {
        std::memcpy(aDst, aAddr, aLen);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Borrowed puppet for one frame; the same proven path QueueDavSound uses.
static RED4ext::IScriptable* ResolvePuppet()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    auto* ps_type = (rtti != nullptr) ? rtti->GetClass("cpPlayerSystem") : nullptr;
    RED4ext::ScriptGameInstance gi;
    if (ps_type == nullptr || gi.instance == nullptr)
        return nullptr;
    RED4ext::IScriptable* ps = SafeGetSystem(gi.instance, ps_type);
    if (ps == nullptr || ps->GetType() == nullptr)
        return nullptr;
    auto* get_player = ps->GetType()->GetFunction("GetPlayer");
    if (get_player == nullptr)
        return nullptr;
    RED4ext::StackArgs_t no_args;
    RED4ext::Handle<RED4ext::IScriptable> out{};
    if (!SafeExecute(ps, get_player, &out, &no_args))
        return nullptr;
    return out.instance; // the game owns the puppet; the pointer outlives this frame
}

static bool PuppetWorldPos(RED4ext::IScriptable* aPuppet, RED4ext::Vector4* aPos)
{
    auto* t = aPuppet->GetType();
    if (t == nullptr)
        return false;
    auto* fn = t->GetFunction("GetWorldPosition");
    if (fn == nullptr)
        return false;
    RED4ext::StackArgs_t no_args;
    return SafeExecute(aPuppet, fn, aPos, &no_args);
}

// v' = v + w*(2 q.v) + q x (2 q.v) - rotate a local offset into world space.
static void RotateByQuat(float qw, float qx, float qy, float qz, float vx, float vy, float vz,
                         float* ox, float* oy, float* oz)
{
    const float tx = 2.0f * (qy * vz - qz * vy);
    const float ty = 2.0f * (qz * vx - qx * vz);
    const float tz = 2.0f * (qx * vy - qy * vx);
    *ox = vx + qw * tx + (qy * tz - qz * ty);
    *oy = vy + qw * ty + (qz * tx - qx * tz);
    *oz = vz + qw * tz + (qx * ty - qy * tx);
}

// Component lookup by name through the entity's component storage - the same SEH-guarded memory
// walk the thruster resolver uses (CET refuses to pass component refs across the boundary).
static void MonitorTick(float aDt)
{
    const int32_t mask = g_mon_mask.load(std::memory_order_relaxed);
    static int s_height_state = 0; // -1 inside the warning zone, +1 outside, 0 unknown
    static int s_dist_state = 0;   // -1 near, +1 far
    static int s_entry_state = 0;  // +1 in, -1 out
    static float s_keepalive_accum = 0.0f;

    // The exit movie runs unthrottled (hundreds of fps once the world is gone). Below a sane
    // gameplay frame time the monitor has nothing useful to measure anyway.
    if (aDt < 1.0f / 240.0f)
        return;

    if (mask == 0)
    {
        s_height_state = 0;
        s_dist_state = 0;
        s_entry_state = 0;
        s_keepalive_accum = 0.0f;
        return;
    }
    if (!VehicleBodyAlive())
        return;
    auto* pd = g_fly_vehicle->physicsData;
    const float avx = pd->worldPosition.X;
    const float avy = pd->worldPosition.Y;
    const float avz = pd->worldPosition.Z;

    if (mask & 1)
    {
        // Raycast every frame: the VFX tick rides on this measurement, so the ray rate sets
        // the smoothest Lua update the DLL can feed Lua.
        float gz = 0.0f;
        float height = -1.0f;
        const bool ray_ok = RaycastGroundZ(avx, avy, avz + g_query_offset.load(std::memory_order_relaxed),
                                           g_query_distance.load(std::memory_order_relaxed), &gz);
        if (ray_ok)
            height = avz - gz;
        if (height >= 0.0f)
        {
            const float thr = g_mon_height_thr.load(std::memory_order_relaxed);
            const int prev = s_height_state;
            if (height < thr - 0.5f)
                s_height_state = -1;
            else if (height > thr + 0.5f)
                s_height_state = 1;
            if (s_height_state != prev)
            {
                // The crossing carries the measured height in timeToLive (see QueueDavEvent).
                QueueDavEvent("gameActionEvent",
                              s_height_state == -1 ? "dav_evt_height_low" : "dav_evt_height_high",
                              height);
            }
            // The VFX slot position lives in Lua (it has a proven component handle there).
            // While inside the warning zone the DLL re-fires a tick every frame that carries
            // the fresh height in timeToLive, so Lua re-projects the VFX without polling.
            if (s_height_state == -1 && g_vfx_enabled.load(std::memory_order_relaxed) != 0.0f)
                QueueDavEvent("gameActionEvent", "dav_evt_height_tick", height);
        }
    }

    if (mask & 6)
    {
        RED4ext::IScriptable* puppet = ResolvePuppet();
        if (puppet == nullptr)
            return;
        RED4ext::Vector4 ppos{};
        if (!PuppetWorldPos(puppet, &ppos))
            return;
        if (mask & 2)
        {
            const float dx = ppos.X - avx, dy = ppos.Y - avy, dz = ppos.Z - avz;
            const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            const float thr = g_mon_dist_thr.load(std::memory_order_relaxed);
            const int prev = s_dist_state;
            if (dist > thr + 2.0f)
                s_dist_state = 1;
            else if (dist < thr - 2.0f)
                s_dist_state = -1;
            if (s_dist_state != prev)
                QueueDavEvent("gameActionEvent", s_dist_state == 1 ? "dav_evt_dist_far" : "dav_evt_dist_near");
        }
        if (mask & 4)
        {
            // The entry point is a local-space offset by the door; rotate it by the entity
            // quaternion (same source the idle regulator trusts). No quaternion means the
            // vehicle center - the radius still keeps the area usable.
            float ox = g_mon_entry_ox.load(std::memory_order_relaxed);
            float oy = g_mon_entry_oy.load(std::memory_order_relaxed);
            float oz = g_mon_entry_oz.load(std::memory_order_relaxed);
            const int32_t att_off = g_att_offset.load(std::memory_order_relaxed);
            if (att_off >= 0)
            {
                struct { float i, j, k, r; } q{0.0f, 0.0f, 0.0f, 1.0f};
                if (SafePeekBytes(reinterpret_cast<uint8_t*>(g_fly_vehicle.instance) + att_off, &q, sizeof(q)))
                {
                    const float n2 = q.i * q.i + q.j * q.j + q.k * q.k + q.r * q.r;
                    if (n2 > 0.5f && n2 < 1.5f)
                        RotateByQuat(q.r, q.i, q.j, q.k, ox, oy, oz, &ox, &oy, &oz);
                }
            }
            const float dx = ppos.X - (avx + ox), dy = ppos.Y - (avy + oy), dz = ppos.Z - (avz + oz);
            const float d2 = std::sqrt(dx * dx + dy * dy + dz * dz);
            const float r = g_mon_entry_radius.load(std::memory_order_relaxed);
            const int prev = s_entry_state;
            if (d2 <= r - 0.5f)
                s_entry_state = 1;
            else if (d2 >= r + 0.5f)
                s_entry_state = -1;
            if (s_entry_state != prev)
            {
                s_keepalive_accum = 0.0f;
                QueueDavEvent("gameActionEvent",
                              s_entry_state == 1 ? "dav_evt_entry_in" : "dav_evt_entry_out");
            }
            if (s_entry_state == 1)
            {
                s_keepalive_accum += aDt;
                if (s_keepalive_accum >= g_mon_keepalive.load(std::memory_order_relaxed))
                {
                    s_keepalive_accum = 0.0f;
                    QueueDavEvent("gameActionEvent", "dav_evt_entry_in"); // keep-alive re-fire (HUD choice re-show)
                }
            }
        }
    }
}

static void DispatchSndEdge(int32_t aCode)
{
    const bool heli = g_fm_buf[g_fm_idx.load(std::memory_order_acquire)].flight_mode == 1;
    const char* evt = nullptr;
    switch (aCode)
    {
    case 1: evt = heli ? "dav_heli_accel_start" : "dav_av_accel_start"; break;
    case 2: evt = heli ? "dav_heli_accel_stop" : "dav_av_accel_stop"; break;
    case 3: if (heli) evt = "dav_heli_thruster_start"; break; // thruster sounds exist for heli only
    case 4: if (heli) evt = "dav_heli_thruster_stop"; break;
    }
    if (g_sdk)
    {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "DAV dbg: snd edge code=%d evt=%s", (int)aCode,
                      evt != nullptr ? evt : "none");
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
    if (evt != nullptr)
        QueueDavSound(evt);
}

static bool HeldAny(const std::vector<int32_t>& aIds)
{
    for (const int32_t id : aIds)
    {
        if (HeldAction(id))
            return true;
    }
    return false;
}

// ControlSound as a continuous timer: latch on the first held command of a layer, and only after
// the layer has been idle for the whole grace fire the stop edge. The layers stay independent so
// acceleration cannot starve the thruster layer, exactly like the Lua original.
// Codes: 1 accel start, 2 accel stop, 3 thruster start, 4 thruster stop.
static void SoundEdgeTick(float aDt)
{
    // Manual input only - the same gate as the flight auto re-assert. While the autopilot is
    // engaged, manual held commands must not start the acceleration layer (the input itself is
    // ignored, but the sound used to blip anyway); an already-running layer fades out through the
    // grace timer. The aboard bit matters too: outside the vehicle the held state can carry stale
    // presses, and the old Lua path only dispatched sound from the in-vehicle Operate.
    const int32_t flags = g_flight_flags.load(std::memory_order_relaxed);
    const bool manual = (flags & 1) != 0 && (flags & 2) == 0;
    if (manual && HeldAny(g_snd_accel_ids))
    {
        g_snd_accel_idle = 0.0f;
        if (!g_snd_accel_on)
        {
            g_snd_accel_on = true;
            DispatchSndEdge(1);
        }
    }
    else if (g_snd_accel_on)
    {
        g_snd_accel_idle += aDt;
        if (g_snd_accel_idle >= g_snd_grace)
        {
            g_snd_accel_on = false;
            DispatchSndEdge(2);
        }
    }
    if (manual && HeldAny(g_snd_thr_ids))
    {
        g_snd_thr_idle = 0.0f;
        if (!g_snd_thr_on)
        {
            g_snd_thr_on = true;
            DispatchSndEdge(3);
        }
    }
    else if (g_snd_thr_on)
    {
        g_snd_thr_idle += aDt;
        if (g_snd_thr_idle >= g_snd_grace)
        {
            g_snd_thr_on = false;
            DispatchSndEdge(4);
        }
    }
}

// Lua hands over the swing setup: gains (per 0.01 s base tick, scaled to deg/s here) and the
// component names (';'-separated). enabled=0 releases the thruster (despawn, helicopter).
void SetThrusterSetup(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    float enabled = 0.0f, step = 0.0f, restore = 0.0f, max_angle = 0.0f;
    int32_t axis = 2;
    RED4ext::CString* engine_names = nullptr;
    RED4ext::CString* fx_names = nullptr;
    RED4ext::GetParameter(aFrame, &enabled);
    RED4ext::GetParameter(aFrame, &step);
    RED4ext::GetParameter(aFrame, &restore);
    RED4ext::GetParameter(aFrame, &max_angle);
    RED4ext::GetParameter(aFrame, &axis);
    RED4ext::GetParameter(aFrame, &engine_names);
    RED4ext::GetParameter(aFrame, &fx_names);
    aFrame->code++;

    g_thr_enabled = (enabled != 0.0f);
    g_thr_step_rate = step;
    g_thr_restore_rate = restore;
    g_thr_max = max_angle;
    g_thr_axis = (axis >= 0 && axis <= 5) ? axis : 2;
    s_thr_names.clear();
    SplitList(engine_names ? engine_names->c_str() : nullptr, ';', s_thr_names);
    SplitList(fx_names ? fx_names->c_str() : nullptr, ';', s_thr_names);
    // Components are re-pushed by Lua right after every setup; drop the previous vehicle's handles now.
    s_thr_components.clear();
    if (!g_thr_enabled)
        s_thr_angle = 0.0f;
    g_thr_ext_fwd.store(0.0f, std::memory_order_relaxed);
    g_thr_ext_back.store(0.0f, std::memory_order_relaxed);
    ++g_thr_setup_seq;
    if (g_sdk)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "DAV dbg: thruster setup en=%d step=%.4f restore=%.4f max=%.1f axis=%d names=%d",
                      (int)enabled, step, restore, max_angle, (int)g_thr_axis, (int)s_thr_names.size());
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
    *aOut = 1.0f;
}

// Autopilot swing input: the Lua autopilot synthesizes Forward/Backward from the vehicle attitude
// and pushes the pair whenever it changes. Manual flight needs nothing here - the held input
// commands already drive the same axes.
void SetThrusterCommand(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    float fwd = 0.0f, back = 0.0f;
    RED4ext::GetParameter(aFrame, &fwd);
    RED4ext::GetParameter(aFrame, &back);
    aFrame->code++;
    g_thr_ext_fwd.store(fwd, std::memory_order_relaxed);
    g_thr_ext_back.store(back, std::memory_order_relaxed);
    *aOut = 1.0f;
}

// Lua hands over the sound layers once: grace seconds plus the comma-separated action id lists
// (Def.AccelerationActionList / Def.ThrusterActionList).
void SetSoundParams(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);
    float grace = 0.0f;
    RED4ext::CString* accel_ids = nullptr;
    RED4ext::CString* thr_ids = nullptr;
    RED4ext::GetParameter(aFrame, &grace);
    RED4ext::GetParameter(aFrame, &accel_ids);
    RED4ext::GetParameter(aFrame, &thr_ids);
    aFrame->code++;

    g_snd_grace = (grace > 0.0f) ? grace : 0.3f;
    g_snd_accel_ids.clear();
    g_snd_thr_ids.clear();
    SplitIds(accel_ids ? accel_ids->c_str() : nullptr, g_snd_accel_ids);
    SplitIds(thr_ids ? thr_ids->c_str() : nullptr, g_snd_thr_ids);
    if (g_sdk)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "DAV dbg: sound params grace=%.2f accel=[%s] thr=[%s]", g_snd_grace,
                      accel_ids ? accel_ids->c_str() : "", thr_ids ? thr_ids->c_str() : "");
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
    *aOut = 1.0f;
}

// Lua polls the pending sound edge (0 = none). Reading clears it, so each edge is consumed once.
RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetThrusterSetup()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetThrusterSetup", "SetThrusterSetup", &SetThrusterSetup,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Float", "enabled");
    func->AddParam("Float", "step");
    func->AddParam("Float", "restore");
    func->AddParam("Float", "maxAngle");
    func->AddParam("Int32", "axis");
    func->AddParam("String", "engineNames");
    func->AddParam("String", "fxNames");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetThrusterCommand()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetThrusterCommand", "SetThrusterCommand", &SetThrusterCommand,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Float", "forward");
    func->AddParam("Float", "backward");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetSoundParams()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    auto func = RED4ext::CClassFunction::Create(&cls, "SetSoundParams", "SetSoundParams", &SetSoundParams,
                                                {.isNative = true});
    func->flags = RED4ext::CBaseFunction::Flags{.isNative = true};
    func->SetReturnType("Float");
    func->AddParam("Float", "graceSeconds");
    func->AddParam("String", "accelIds");
    func->AddParam("String", "thrusterIds");
    cls.RegisterFunction(func);
}

//////////////////////////////////////////////////////////////////////////////////
// A game-thread per-frame callback.
//
// The physics hook lives on the physics thread: RED4ext::TLS::Get() reads the thread's own TEB slot,
// so the world pointer is only valid on the game thread, script handles must not be touched there,
// and a scene query during the step re-enters the physics engine. Anything needing the world, the
// player, or a raycast therefore fell to CET, polling from Lua.
//
// RED4ext game states give the missing cadence: OnUpdate runs on the game thread once per frame.
// The ground reference is published from here, which is what lets Lua stop raycasting for height.
// Contact is enough to learn it: while the vehicle touches something, the ground is right under it.
//////////////////////////////////////////////////////////////////////////////////
static bool RED4EXT_CALL DavGameState_OnEnter(RED4ext::CGameApplication* aApp)
{
    RED4EXT_UNUSED_PARAMETER(aApp);
    return true;
}

static bool RED4EXT_CALL DavGameState_OnUpdate(RED4ext::CGameApplication* aApp)
{
    RED4EXT_UNUSED_PARAMETER(aApp);
    // Quit to desktop: once the Running state is exiting, the world is going away under us. Do
    // nothing at all - the exit movie runs hundreds of frames per second and every world touch
    // there is a use-after-free waiting to happen.
    if (g_shutting_down.load(std::memory_order_relaxed))
        return false;
    g_game_thread_id.store(static_cast<uint32_t>(GetCurrentThreadId()), std::memory_order_relaxed);
    const uint64_t n = g_game_frame_count.fetch_add(1, std::memory_order_relaxed) + 1;
    // Resolve the query's shape as soon as a world exists, whether or not the vehicle is flying, so
    // the diagnostic lands in the log instead of waiting for someone to take off.
    if (!g_ray.usable && !g_ray.dead && (n % 30) == 0)
    {
        ResolveGroundRay();
    }
    if (n == 1 && g_sdk)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "DAV dbg: game state OnUpdate live (tid=%lu, physics tid=%lu)",
                      (unsigned long)GetCurrentThreadId(), (unsigned long)g_hook_thread_id.load());
        g_sdk->logger->Info(g_plugin_handle, buf);
    }
        if ((n % 600) == 300 && g_sdk)
    {
        // Heartbeat proving OnUpdate (and therefore ThrusterTick/SoundEdgeTick) still runs; also
        // samples the physics state so a game-side disable becomes visible in the log.
        char buf[96];
        const int phys = VehicleBodyAlive() ? (int)g_fly_vehicle->physicsState : -1;
        std::snprintf(buf, sizeof(buf), "DAV dbg: OnUpdate heartbeat #%llu phys=%d", (unsigned long long)n, phys);
        g_sdk->logger->Info(g_plugin_handle, buf);
    }

    if (VehicleBodyAlive())
    {
        // The game disables physics (physicsState != 0) when the AV lands or touches something
        // after dismount. A disabled body is never simulated, so the physics hook stops running
        // and every mode push is silently dropped - the "recall does not climb" bug. The hook's
        // own safety net cannot fire in that case (it lives inside the physics step), so physics
        // is re-asserted here on the frame clock. Lua clears the vehicle handle on despawn
        // (SetVehicle(0)), so a deleted entity can never be touched here.
        if (g_fly_vehicle->physicsState != 0)
        {
            const uint64_t rn = g_native_phys_enable.fetch_add(1, std::memory_order_relaxed);
            // Log BEFORE the call: ForceEnablePhysics is a hash-resolved static entry point that has
            // never been observed firing; if it ever faults, the log shows it was reached.
            if (rn < 3 && g_sdk)
            {
                char pbuf[128];
                std::snprintf(pbuf, sizeof(pbuf), "DAV dbg: OnUpdate re-enabling physics (state=%d)",
                              (int)g_fly_vehicle->physicsState);
                g_sdk->logger->Info(g_plugin_handle, pbuf);
            }
            g_fly_vehicle->ForceEnablePhysics();
        }
        static bool s_was_on_ground = false;
        const bool on_ground = g_fly_vehicle->isOnGround;
        g_av_on_ground.store(on_ground ? 1.0f : 0.0f, std::memory_order_relaxed);
        auto* pd = g_fly_vehicle->physicsData;
        if (pd != nullptr)
        {
            const float z = pd->worldPosition.Z;
            g_av_world_z.store(z, std::memory_order_relaxed);
            // Learn the ground only on the frame contact starts. Re-learning it on every frame the
            // vehicle rests on it kept the measured height at zero forever, so the regulator pushed
            // up, the vehicle left the ground, the stale reference pulled it back down, and the loop
            // shook the vehicle apart right before a despawn.
            if (on_ground && !s_was_on_ground)
            {
                g_ground_z.store(z, std::memory_order_relaxed);
                g_ground_valid.store(1.0f, std::memory_order_relaxed);
            }
            // Query on the cadence the hover actually consumes the answer on: twice a second, and
            // only while the native hover is running. Airborne is the case contact cannot cover -
            // over a rooftop edge or a pit the last contact is the wrong floor - but querying while
            // landed too keeps the published value honest and makes the whole path observable from
            // the probe without anyone having to take off. A miss leaves the last known ground alone.
            const bool hover_active = (g_fc_buf[g_fc_idx.load(std::memory_order_acquire)].mode == 4);
            if (hover_active && (n % 30) == 0)
            {
                float gz = 0.0f;
                if (RaycastGroundZ(pd->worldPosition.X, pd->worldPosition.Y, z + g_query_offset.load(std::memory_order_relaxed),
                                   g_query_distance.load(std::memory_order_relaxed), &gz))
                {
                    g_ground_z.store(gz, std::memory_order_relaxed);
                    g_ground_valid.store(1.0f, std::memory_order_relaxed);
                }
            }
        }
        s_was_on_ground = on_ground;
    }
    // Stall probe: if the physics hook stops running for our vehicle (park/dismount states), every
    // mode push is silently dropped. Warn once per stall so the recall bug can be pinned down.
    {
        static uint64_t s_last_native_ticks = 0;
        static int s_stall_frames = 0;
        const uint64_t now_ticks = g_native_tick_count.load(std::memory_order_relaxed);
        const bool have_vehicle = g_fly_vehicle_raw.load(std::memory_order_acquire) != 0;
        if (have_vehicle && now_ticks == s_last_native_ticks)
        {
            if (++s_stall_frames == 180 && g_sdk)
            {
                char sbuf[160];
                std::snprintf(sbuf, sizeof(sbuf), "DAV dbg: hook stalled - no physics ticks for ~3s (phys=%d)",
                              VehicleBodyAlive() ? (int)g_fly_vehicle->physicsState : -1);
                g_sdk->logger->Info(g_plugin_handle, sbuf);
            }
        }
        else
        {
            if (s_stall_frames >= 180 && g_sdk)
            {
                g_sdk->logger->Info(g_plugin_handle, "DAV dbg: hook resumed after stall");
            }
            s_last_native_ticks = now_ticks;
            s_stall_frames = 0;
        }
    }
    // Thruster swing and sound edges ran in the Lua control loop; they consume the held commands,
    // so they ride the frame clock here. Clamped dt: a tab-stop frame must not teleport the angle.
    {
        static auto s_thr_last = std::chrono::steady_clock::now();
        const auto thr_now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(thr_now - s_thr_last).count();
        s_thr_last = thr_now;
        if (dt < 0.0f)
            dt = 0.0f;
        if (dt > 0.1f)
            dt = 0.1f;
        ThrusterTick(dt);
        SoundEdgeTick(dt);
        // Threshold monitor: height / player distance / entry-area edges announced as events.
        if (g_mon_mask.load(std::memory_order_relaxed) != 0)
            MonitorTick(dt);
    }
    // MUST return false: RED4ext treats a state's OnUpdate returning true as "finished" and stops
    // calling it next frame. Returning true here once made the whole frame clock (thruster swing,
    // sound edges, ground raycast) run exactly one frame per game start.
    return false;
}

static bool RED4EXT_CALL DavGameState_OnExit(RED4ext::CGameApplication* aApp)
{
    RED4EXT_UNUSED_PARAMETER(aApp);
    // Shutdown etiquette: this fires when the Running state pops (quit to desktop). The teardown
    // that follows frees the vehicle entity while our handle still points at it, so drop every
    // piece of world state the frame clock touches. Lua's despawn path (SetVehicle(0)) never runs
    // on a quit; this is the only reliable place to let go.
    g_shutting_down.store(true, std::memory_order_relaxed);
    g_mon_mask.store(0, std::memory_order_relaxed);
    g_fly_vehicle = RED4ext::Handle<RED4ext::vehicle::BaseObject>();
    g_fly_vehicle_entity_id_hash = 0;
    return true;
}

static RED4ext::v0::GameState g_dav_game_state = {
    &DavGameState_OnEnter,
    &DavGameState_OnUpdate,
    &DavGameState_OnExit,
};

// Seed or correct the ground reference from CET (e.g. right after a spawn, before first contact).
void SetGroundReference(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    float z;
    RED4ext::GetParameter(aFrame, &z);
    aFrame->code++; // skip ParamEnd

    g_ground_z.store(z, std::memory_order_relaxed);
    g_ground_valid.store(1.0f, std::memory_order_relaxed);
    *aOut = 1;
}

// (ground_z, av_z, on_ground, valid) - one call replaces CET's raycast for every height decision.
void GetGroundInfo(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut,
                   int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = RED4ext::Vector4{g_ground_z.load(std::memory_order_relaxed),
                             g_av_world_z.load(std::memory_order_relaxed),
                             g_av_on_ground.load(std::memory_order_relaxed),
                             g_ground_valid.load(std::memory_order_relaxed)};
}

// What RTTI cannot tell us: the search box and the collision groups, both of which are user settings.
void SetGroundQuery(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    float offset;
    float distance;
    uint64_t mask1;
    uint64_t mask2;
    RED4ext::GetParameter(aFrame, &offset);
    RED4ext::GetParameter(aFrame, &distance);
    RED4ext::GetParameter(aFrame, &mask1);
    RED4ext::GetParameter(aFrame, &mask2);
    aFrame->code++; // skip ParamEnd

    g_query_offset.store(offset, std::memory_order_relaxed);
    g_query_distance.store(distance, std::memory_order_relaxed);
    g_query_mask1.store(mask1, std::memory_order_relaxed);
    g_query_mask2.store(mask2, std::memory_order_relaxed);
    *aOut = 1;
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetGroundReference()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetGroundReference", "SetGroundReference", &SetGroundReference, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Float", "z");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetGroundInfo()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "GetGroundInfo", "GetGroundInfo", &GetGroundInfo, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Vector4");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterSetGroundQuery()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "SetGroundQuery", "SetGroundQuery", &SetGroundQuery, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Float", "offset");
    func->AddParam("Float", "distance");
    func->AddParam("Uint64", "mask1");
    func->AddParam("Uint64", "mask2");
    cls.RegisterFunction(func);
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
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetThresholds);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetLandingVFX);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightFlags);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightControl);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams2);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetFlightParams3);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetIdleHoverParams);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetGroundInfo);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetGroundReference);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetGroundQuery);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetThrusterSetup);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetThrusterCommand);
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetSoundParams);
        // Game-thread cadence: OnUpdate runs once per frame on the game thread.
        g_sdk->gameStates->Add(aHandle, RED4ext::EGameStateType::Running, &g_dav_game_state);
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
