#include <RED4ext/RED4ext.hpp>
#include <RED4ext/RTTITypes.hpp>
#include <RED4ext/Scripting/Natives/vehicleBaseObject.hpp>
#include <RED4ext/Scripting/Natives/vehiclePhysicsData.hpp>

struct FlyAVSystem : RED4ext::IScriptable
{
    RED4ext::CClass* GetNativeType();
};

RED4ext::TTypedClass<FlyAVSystem> cls("FlyAVSystem");

RED4ext::CClass* FlyAVSystem::GetNativeType()
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

void AddLinelyVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
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

void ChangeLinelyVelocity(RED4ext::IScriptable* aContext, RED4ext::CStackFrame* aFrame, float* aOut, int64_t a4)
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

RED4EXT_C_EXPORT void RED4EXT_CALL RegisterFlyAVSystem()
{
    RED4ext::CNamePool::Add("FlyAVSystem");

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

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterAddLinelyVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "AddLinelyVelocity", "AddLinelyVelocity", &AddLinelyVelocity,
                                                {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "velocity");
    func->AddParam("Vector3", "angularVelocity");
    cls.RegisterFunction(func);
}

RED4EXT_C_EXPORT void RED4EXT_CALL PostRegisterChangeLinelyVelocity()
{
    auto rtti = RED4ext::CRTTISystem::Get();
    auto scriptable = rtti->GetClass("IScriptable");
    cls.parent = scriptable;

    RED4ext::CBaseFunction::Flags flags = {.isNative = true};
    auto func = RED4ext::CClassFunction::Create(&cls, "ChangeLinelyVelocity", "ChangeLinelyVelocity",
                                                &ChangeLinelyVelocity, {.isNative = true});
    func->flags = flags;
    func->SetReturnType("Float");
    func->AddParam("Vector3", "velocity");
    func->AddParam("Vector3", "angularVelocity");
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
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterSetVehicle);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterHasGravity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterEnableGravity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterAddLinelyVelocity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterChangeLinelyVelocity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterGetVelocity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterGetAngularVelocity);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterGetPhysicsState);
        RED4ext::RTTIRegistrator::Add(RegisterFlyAVSystem, PostRegisterIsOnGround);
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
    aInfo->name = L"DAV API";
    aInfo->author = L"tidus";
    aInfo->version = RED4EXT_SEMVER(2, 6, 0);
    aInfo->runtime = RED4EXT_RUNTIME_INDEPENDENT;
    aInfo->sdk = RED4EXT_SDK_LATEST;
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_LATEST;
}
