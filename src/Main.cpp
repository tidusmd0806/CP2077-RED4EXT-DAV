#include <RED4ext/RED4ext.hpp>
#include <RED4ext/RTTITypes.hpp>
#include <RED4ext/Scripting/Natives/vehicleBaseObject.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysicsData.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysics.hpp>

static bool is_enable_original_physics = true;

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
    if (is_enable_original_physics)
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
    if (is_enable_original_physics)
    {
        ApplyForceAtPosition_Original(physicsData, offset, force);
    }
    else
    {
        return;
    }
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

static RED4ext::Handle<RED4ext::vehicle::BaseObject> vehicle;

void SetVehicle(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    RED4ext::WeakHandle<RED4ext::vehicle::BaseObject> wvehicle;
    uint32_t entity_id_hash;
    RED4ext::EntityID entity_id;

    RED4ext::GetParameter(aFrame, &entity_id_hash);
    aFrame->code++; // skip ParamEnd

    entity_id.hash = entity_id_hash;

    RED4ext::ScriptGameInstance gameInstance;
    RED4ext::ExecuteFunction("ScriptGameInstance", "FindEntityByID", &wvehicle, gameInstance, entity_id);

    vehicle = wvehicle.Lock();
    is_enable_original_physics = true;

    *aOut = 0;

    if (vehicle)
    {
        *aOut = 1;
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
    *aOut = 0;
    if (vehicle)
    {
        is_enable_original_physics = is_enable;
        *aOut = 1;
    }
}

void GetMass(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = 0;

    if (vehicle)
    {
        *aOut = vehicle->physicsData->total_mass;
    }
}

void HasGravity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = false;

    if (vehicle)
    {
        *aOut = vehicle->physicsData->unk1B0;
    }
}

void EnableGravity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(a4);

    bool is_enable;

    RED4ext::GetParameter(aFrame, &is_enable);
    aFrame->code++; // skip ParamEnd

    *aOut = 0;

    if (vehicle)
    {
        vehicle->physicsData->unk1B0 = is_enable;
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

    if (vehicle)
    {
        vehicle->physicsData->velocity += velocity;
        vehicle->physicsData->angularVelocity += angularVelocity;
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

    if (vehicle)
    {
        vehicle->physicsData->force += force;
        vehicle->physicsData->torque += torque;
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

    if (vehicle)
    {
        if (switchIndex == 1)
        {
            vehicle->physicsData->velocity = velocity;
        }
        else if (switchIndex == 2)
        {
            vehicle->physicsData->angularVelocity = angularVelocity;
        }
        else
        {
            vehicle->physicsData->velocity = velocity;
            vehicle->physicsData->angularVelocity = angularVelocity;
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
    if (vehicle)
    {
        if (switchIndex == 1)
        {
            vehicle->physicsData->force = force;
        }
        else if (switchIndex == 2)
        {
            vehicle->physicsData->torque = torque;
        }
        else
        {
            vehicle->physicsData->force = force;
            vehicle->physicsData->torque = torque;
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

    if (vehicle)
    {
        *aOut = vehicle->physicsData->velocity;
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

    if (vehicle)
    {
        *aOut = vehicle->physicsData->angularVelocity;
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
    if (vehicle)
    {
        *aOut = vehicle->physicsData->force;
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
    if (vehicle)
    {
        *aOut = vehicle->physicsData->torque;
    }
}

void GetPhysicsState(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = -1;

    if (vehicle)
    {
        *aOut = vehicle->physicsState;
    }
}

void UnsetPhysicsState(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = -1;

    if (vehicle)
    {
        vehicle->ForceEnablePhysics();
        *aOut = 1;
    }
}

void IsOnGround(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = false;

    if (vehicle)
    {
        *aOut = vehicle->isOnGround;
    }
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
#ifdef FLY_TANK_MOD
        RED4ext::CRTTISystem::Get()->AddRegisterCallback(RegisterFlyTankSystem);
#else
        RED4ext::CRTTISystem::Get()->AddRegisterCallback(RegisterFlyAVSystem);
#endif
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterSetVehicle);
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
    aInfo->author = L"tidus";
    aInfo->version = RED4EXT_SEMVER(1, 2, 0);
    aInfo->runtime = RED4EXT_RUNTIME_INDEPENDENT;
    aInfo->sdk = RED4EXT_SDK_LATEST;
#else
    aInfo->name = L"DAV API";
    aInfo->author = L"tidus";
    aInfo->version = RED4EXT_SEMVER(3, 0, 0);
    aInfo->runtime = RED4EXT_RUNTIME_INDEPENDENT;
    aInfo->sdk = RED4EXT_SDK_LATEST;
#endif
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_LATEST;
}
