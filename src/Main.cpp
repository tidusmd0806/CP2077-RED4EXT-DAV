// fix to compile error in physicsProxyManager-inl.hpp
#define physics_proxyManager_Addr 0

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/RTTITypes.hpp>
#include <RED4ext/Scripting/Natives/vehicleBaseObject.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysicsData.hpp>
#include <RED4ext/Definitions.hpp>

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

    *aOut = 0;

    if (vehicle)
    {
        *aOut = 1;
    }
}

void GetInverseMass(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
{
    RED4EXT_UNUSED_PARAMETER(aContext);
    RED4EXT_UNUSED_PARAMETER(aFrame);
    RED4EXT_UNUSED_PARAMETER(a4);

    *aOut = 0;

    if (vehicle && vehicle->physicsData)
    {
        *aOut = vehicle->physicsData->inverseMass;
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
        vehicle->UnsetPhysicsStates();
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

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterGetInverseMass()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;
    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func =
        RED4ext::CClassFunction::Create(&cls, "GetInverseMass", "GetInverseMass", &GetInverseMass, {.isNative = true});
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

RED4EXT_C_EXPORT bool RED4EXT_CALL Main(RED4ext::PluginHandle aHandle, RED4ext::EMainReason aReason,
                                        const RED4ext::Sdk* aSdk)
{
    RED4EXT_UNUSED_PARAMETER(aHandle);
    RED4EXT_UNUSED_PARAMETER(aSdk);

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
        RED4ext::CRTTISystem::Get()->AddPostRegisterCallback(PostRegisterGetInverseMass);
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
        break;
    }
    case RED4ext::EMainReason::Unload:
    {
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
