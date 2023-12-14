#include <Havok.h>
#include <MathUtils.h>
#include <SimpleIni.h>
#include <Utilities.h>
#include <Windows.h>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Xinput.h>
using namespace F4SE;
#define MATH_PI 3.14159265358979323846
#define HAVOKTOFO4 69.99124f

using std::vector;
using namespace RE;

#pragma region Variables

class hknpDynamicCompoundShape;
typedef bool (*_updateAabb)(hknpDynamicCompoundShape*);
REL::Relocation<_updateAabb*> updateAabb{ VTABLE::hknpDynamicCompoundShape[0], 0xF0 };

REL::Relocation<uintptr_t> ptr_ADS_DistanceCheck{ REL::ID(1278889), 0x31 };
/*REL::Relocation<uintptr_t> ptr_PCUpdateMainThread{REL::ID(633524), 0x22D};
static uintptr_t PCUpdateMainThreadOrig;
REL::Relocation<uintptr_t> ptr_UpdateSceneGraph{REL::ID(1318162), 0xD5};
static uintptr_t UpdateSceneGraphOrig;*/
REL::Relocation<uintptr_t> ptr_RunActorUpdates{ REL::ID(556439), 0x17 };
uintptr_t RunActorUpdatesOrig;
REL::Relocation<uintptr_t> ptr_bhkWorldUpdate{ REL::ID(1395106) };
const F4SE::TaskInterface* taskInterface;
//vector<hkVector4f> cachedShape[5];
PlayerCharacter* p;
PlayerCamera* pcam;
PlayerControls* pc;
bhkPickData* pick;
NiNode* lastRoot;
BSBound* bbx;
ActorValueInfo* rightAttackCondition;
ActorValueInfo* leftAttackCondition;
BGSProjectile* colCheckProj;
TESQuest* MQ102;
float rotX, rotY, rotZ, transZ, targetRotX, targetRotZ;
NiMatrix3 lastCamRot;
float rotLimitX = 10.0f;
float rotLimitY = 5.0f;
float rotDivX = 10.0f;
float rotDivY = 20.0f;
float rotDivXADS = 4.0f;
float rotDivYADS = 8.0f;
float rotADSConditionMult = 5.0f;
float rotReturnDiv = 1.5f;
float rotReturnStep = 0.1f;
float rotReturnDivMin = 1.02f;
bool rotDisableInADS = false;
float lastRun;
float playerLastLoaded;
float lastSkeletonUpdate;
int leanState = 0;
int lastLeanState = 0;
float leanWeight;
float leanTime;
float leanTimeCost = 1.0f;
float leanMax = 15.0f;
float leanMax3rd = 30.0f;
float leanCumulativeMouseX = 0;
float leanCumulativeMouseY = 0;
NiPoint3 leanLastDir;
bool toggleLean = false;
bool disableLean = false;
bool vanillaPeekPatched = false;
uint32_t leanLeft = 0xFFFF;
uint32_t leanRight = 0xFFFF;
bool leanADSOnly = false;
bool leanR6Style = false;
bool leanRotateInput = false;
bool collisionDevMode = false;
bool buttonDevMode = false;
float lastiniUpdate;
float lastHeight = 120.0f;
float heightDiffThreshold = 5.0f;
float heightBuffer = 16.0f;
float minHeight = 20.f;
bool dynamicHeight = false;
float realismDefaultRot = 12.f;
float realismRatio = 1.f;
float realismYawRatio = 0.8f;
float realismRatioADSMult = 0.2f;
float realismRotLimit = 30.f;
float realismYawRotLimit = 7.5f;
float realismReturnStep = 0.0001f;
float realismReturnStepADSMult = 4.f;
float realismElasticity = 0.3f;
float realismWeight, realismWeightNoElastic, realismWeightSpring, realismRotZ, realismRotX, realismReturnStepWeight;
bool realismDisableInPA = false;
bool realism = false;
bool isLoading = false;
bool inWorkShopMenu = false;
bool inLooksMenu = false;
bool inDialogueMenu = false;
bool inPipboyMenu = false;

#define REALISM_RATIO_CONSTANT 0.45f
#define REALISM_RETURNWEIGHT_START 0.75f
#define REALISM_RETURNWEIGHT_TIME 0.1f

#pragma endregion

#pragma region Utilities

namespace Translation
{
	uint32_t ReadLine_w(BSResourceNiBinaryStream& stream, wchar_t* dst, uint32_t dstLen, uint32_t terminator)
	{
		wchar_t* iter = dst;

		if (dstLen == 0)
			return 0;

		for (uint32_t i = 0; i < dstLen - 1; i++) {
			wchar_t data;

			if (stream.binary_read(&data, sizeof(data)) != sizeof(data))
				break;

			if (data == terminator)
				break;

			*iter++ = data;
		}

		// null terminate
		*iter = 0;

		return iter - dst;
	}

	void ParseTranslation(BSScaleformTranslator* translator, std::string name)
	{
		Setting* setting = INISettingCollection::GetSingleton()->GetSetting("sLanguage:General");
		if (!setting)
			setting = INIPrefSettingCollection::GetSingleton()->GetSetting("sLanguage:General");
		std::string path = "Interface\\Translations\\";

		// Construct translation filename
		path += name;
		path += "_";
		path += (setting && setting->GetType() == Setting::SETTING_TYPE::kString) ? setting->GetString() : "en";
		path += ".txt";

		BSResourceNiBinaryStream fileStream(path.c_str());
		_MESSAGE("Loading translations %s", path.c_str());

		// Check if file is empty, if not check if the BOM is UTF-16
		uint16_t bom = 0;
		uint32_t ret = (uint32_t)fileStream.binary_read(&bom, sizeof(uint16_t));
		if (ret == 0) {
			_MESSAGE("Empty translation file.");
			return;
		}
		if (bom != 0xFEFF) {
			_MESSAGE("BOM Error, file must be encoded in UCS-2 LE.");
			return;
		}

		while (true) {
			wchar_t buf[512];
			uint32_t len = ReadLine_w(fileStream, buf, sizeof(buf) / sizeof(buf[0]), '\n');
			if (len == 0)  // End of file
				return;

			// at least $ + wchar_t + \t + wchar_t
			if (len < 4 || buf[0] != '$')
				continue;

			wchar_t last = buf[len - 1];
			if (last == '\r')
				len--;

			// null terminate
			buf[len] = 0;

			uint32_t delimIdx = 0;
			for (uint32_t i = 0; i < len; i++)
				if (buf[i] == '\t')
					delimIdx = i;

			// at least $ + wchar_t
			if (delimIdx < 2)
				continue;

			// replace \t by \0
			buf[delimIdx] = 0;

			BSFixedStringWCS key(buf);
			BSFixedStringWCS translation(&buf[delimIdx + 1]);

			BSTTuple<BSFixedStringWCS, BSFixedStringWCS> item(key, translation);
			translator->translator.translationMap.insert(item);
		}
	}
}

NiNode* InsertBone(NiAVObject* root, NiNode* node, const char* name)
{
	NiNode* parent = node->parent;
	NiNode* inserted = (NiNode*)root->GetObjectByName(name);
	if (!inserted) {
		inserted = CreateBone(name);
		//_MESSAGE("%s (%llx) created.", name, inserted);
		if (parent) {
			parent->AttachChild(inserted, true);
			inserted->parent = parent;
		} else {
			parent = node;
		}
		inserted->local.translate = NiPoint3();
		inserted->local.rotate.MakeIdentity();
		inserted->AttachChild(node, true);
		//_MESSAGE("%s (%llx) inserted to %s (%llx).", name, inserted, parent->name.c_str(), parent);
		return inserted;
	} else {
		if (!inserted->GetObjectByName(node->name)) {
			_MESSAGE("%s structure mismatch. Reinserting... (%f)", inserted->name.c_str(), *F4::ptr_engineTime);
			//_MESSAGE("%s (%llx) created.", name, inserted);
			if (parent) {
				parent->AttachChild(inserted, true);
				inserted->parent = parent;
			} else {
				parent = node;
			}
			inserted->AttachChild(node, true);
			return inserted;
		}
	}
	return nullptr;
}

bool IsInPowerArmor(Actor* a)
{
	if (!a->extraList) {
		return false;
	}
	return a->extraList->HasType(RE::EXTRA_DATA_TYPE::kPowerArmor);
}

bool IsInADS(Actor* a)
{
	return ((int)a->gunState == 0x8 || (int)a->gunState == 0x6);
}

bool IsFirstPerson()
{
	return pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson];
}

float Sign(float f)
{
	if (f == 0)
		return 0;
	return abs(f) / f;
}

#pragma endregion

#pragma region Functions

void LoadConfigs()
{
	std::string path = "Data\\MCM\\Config\\UneducatedShooter\\settings.ini";
	if (std::filesystem::exists(path)) {
		path = "Data\\MCM\\Settings\\UneducatedShooter.ini";
	}
	CSimpleIniA ini(true, false, false);
	SI_Error result = ini.LoadFile(path.c_str());
	if (result >= 0) {
		rotLimitX = std::stof(ini.GetValue("Inertia", "frotLimitX", "24.0"));
		rotLimitY = std::stof(ini.GetValue("Inertia", "frotLimitY", "8.0"));
		rotDivX = std::stof(ini.GetValue("Inertia", "frotDivX", "24.0"));
		rotDivY = std::stof(ini.GetValue("Inertia", "frotDivY", "32.0"));
		rotDivXADS = std::stof(ini.GetValue("Inertia", "frotDivXADS", "24.0"));
		rotDivYADS = std::stof(ini.GetValue("Inertia", "frotDivYADS", "32.0"));
		rotADSConditionMult = std::stof(ini.GetValue("Inertia", "frotADSConditionMult", "3.0"));
		rotReturnDiv = std::stof(ini.GetValue("Inertia", "frotReturnDiv", "1.25"));
		//rotReturnDivMin = std::stof(ini.GetValue("Inertia", "frotReturnDivMin", "1.05"));
		rotReturnDivMin = 1.12f;
		rotReturnStep = std::stof(ini.GetValue("Inertia", "frotReturnStep", "0.0"));
		rotDisableInADS = std::stoi(ini.GetValue("Inertia", "brotDisableInADS", "0")) > 0;
		realismDefaultRot = std::stof(ini.GetValue("Realism", "fRealismDefaultRotation", "-2.5"));
		realismRatio = std::stof(ini.GetValue("Realism", "fRealismRatio", "1.2"));
		realismYawRatio = std::stof(ini.GetValue("Realism", "realismYawRatio", "0.8"));
		realismRatioADSMult = std::stof(ini.GetValue("Realism", "fRealismRatioADS", "0.2"));
		realismRotLimit = std::stof(ini.GetValue("Realism", "fRealismRotLimit", "10.0"));
		realismYawRotLimit = std::stof(ini.GetValue("Realism", "fRealismYawRotLimit", "12.5"));
		realismReturnStep = std::stof(ini.GetValue("Realism", "fRealismReturnStep", "0.05"));
		realismReturnStepADSMult = std::stof(ini.GetValue("Realism", "fRealismReturnStepADS", "4.0"));
		//realismElasticity = std::stof(ini.GetValue("Realism", "fRealismElasticity", "0.3"));
		realismElasticity = 0.f;
		realismDisableInPA = std::stoi(ini.GetValue("Realism", "bRealismDisableInPA", "0")) > 0;
		realism = std::stoi(ini.GetValue("Realism", "bRealism", "0")) > 0;
		disableLean = std::stoi(ini.GetValue("Leaning", "bleanDisable", "0")) > 0;
		leanTimeCost = std::stof(ini.GetValue("Leaning", "fleanTimeCost", "0.2"));
		leanMax = std::stof(ini.GetValue("Leaning", "fleanMax", "15.0"));
		leanMax3rd = std::stof(ini.GetValue("Leaning", "fleanMax3rd", "30.0"));
		toggleLean = std::stoi(ini.GetValue("Leaning", "bToggleLean", "1")) > 0;
		leanADSOnly = std::stoi(ini.GetValue("Leaning", "bADSOnly", "0")) > 0;
		leanR6Style = std::stoi(ini.GetValue("Leaning", "bR6Style", "0")) > 0;
		leanRotateInput = std::stoi(ini.GetValue("Leaning", "bRotateInput", "1")) > 0;
		dynamicHeight = std::stoi(ini.GetValue("Height", "bdynamicHeight", "1")) > 0;
		heightDiffThreshold = std::stof(ini.GetValue("Height", "fheightDiffThreshold", "5.0"));
		heightBuffer = std::stof(ini.GetValue("Height", "fheightBuffer", "4.0"));
		minHeight = std::stof(ini.GetValue("Height", "fminHeight", "20.0"));
		collisionDevMode = std::stoi(ini.GetValue("Dev", "bcollisionDevMode", "0")) > 0;
		buttonDevMode = std::stoi(ini.GetValue("Dev", "bbuttonDevMode", "0")) > 0;
	}
	ini.Reset();
	std::string hotkeyPath = "Data\\MCM\\Settings\\Keybinds.json";
	if (std::filesystem::exists(hotkeyPath)) {
		std::ifstream reader;
		reader.open(hotkeyPath);
		nlohmann::json keyBinds;
		reader >> keyBinds;
		reader.close();
		if (keyBinds.contains("keybinds")) {
			for (auto& key : keyBinds["keybinds"]) {
				try {
					if (key["modName"].get<std::string>() == "UneducatedShooter") {
						if (key["id"].get<std::string>() == "keyLeanLeft") {
							leanLeft = key["keycode"].get<int>();
						} else if (key["id"].get<std::string>() == "keyLeanRight") {
							leanRight = key["keycode"].get<int>();
						}
					}
				} catch (...) {
					continue;
				}
			}
		}
	}

	if (!vanillaPeekPatched && !disableLean) {
		uint8_t bytes[] = { 0xE9, 0x1C, 0x06, 0x00, 0x00, 0x90 };
		REL::safe_write<uint8_t>(ptr_ADS_DistanceCheck.address(), std::span{ bytes });
		for (auto it = GameSettingCollection::GetSingleton()->settings.begin(); it != GameSettingCollection::GetSingleton()->settings.end(); ++it) {
			if (it->first == "fPlayerCoverPeekTime") {
				it->second->SetFloat(0.0f);
				_MESSAGE("%s changed to %f", it->first.c_str(), it->second->GetFloat());
			}
		}
		vanillaPeekPatched = true;
	}
}

void PreparePlayerSkeleton(NiAVObject* node = nullptr)
{
	NiAVObject* fpNode = p->Get3D(true);
	if (fpNode && (!node || node == fpNode)) {
		//_MESSAGE("First person skeleton found.");

		NiNode* chest = (NiNode*)fpNode->GetObjectByName("Chest");
		if (chest) {
			InsertBone(fpNode, chest, "ChestInserted1st");
		}

		NiNode* com = (NiNode*)fpNode->GetObjectByName("COM");
		if (com) {
			InsertBone(fpNode, com, "COMInserted1st");
		}

		NiNode* camera = (NiNode*)fpNode->GetObjectByName("Camera");
		if (camera) {
			InsertBone(fpNode, camera, "CameraInserted1st");
		}
	}
	NiAVObject* tpNode = p->Get3D(false);
	if (tpNode && (!node || node == tpNode)) {
		//_MESSAGE("Third person skeleton found.");

		NiNode* com = (NiNode*)tpNode->GetObjectByName("COM");
		if (com) {
			InsertBone(tpNode, com, "COMInserted");
		}

		NiNode* camera = (NiNode*)tpNode->GetObjectByName("Camera");
		if (camera) {
			InsertBone(tpNode, camera, "CameraInserted");
		}

		NiNode* spine1 = (NiNode*)tpNode->GetObjectByName("SPINE1");
		if (spine1) {
			InsertBone(tpNode, spine1, "Spine1Inserted");
		}

		NiNode* spine2 = (NiNode*)tpNode->GetObjectByName("SPINE2");
		if (spine2) {
			InsertBone(tpNode, spine2, "Spine2Inserted");
		}

		NiNode* chest = (NiNode*)tpNode->GetObjectByName("Chest");
		if (chest) {
			InsertBone(tpNode, chest, "ChestInserted");
		}

		NiNode* pelvis = (NiNode*)tpNode->GetObjectByName("Pelvis");
		if (pelvis) {
			InsertBone(tpNode, pelvis, "PelvisInserted");
		}

		NiNode* llegCalf = (NiNode*)tpNode->GetObjectByName("LLeg_Calf");
		if (llegCalf) {
			InsertBone(tpNode, llegCalf, "LLeg_CalfInserted");
		}

		NiNode* rlegCalf = (NiNode*)tpNode->GetObjectByName("RLeg_Calf");
		if (rlegCalf) {
			InsertBone(tpNode, rlegCalf, "RLeg_CalfInserted");
		}
	}
}

void ResizeCollisionPolytopeShapeX(hknpShape* shape, vector<hkVector4f>& cache, float ratio, float bumperHalf, float bumperDiff)
{
	uint16_t vertexSize = *(uint16_t*)((uintptr_t)shape + 0x30);
	uint16_t vertexOffset = *(uint16_t*)((uintptr_t)shape + 0x32);
	if (cache.size() == 0) {
		for (int i = 0; i < vertexSize; ++i) {
			cache.push_back(*(hkVector4f*)((uintptr_t)shape + 0x30 + vertexOffset + 0x10 * i));
		}
	}
	for (int i = 0; i < cache.size(); ++i) {
		((hkVector4f*)((uintptr_t)shape + 0x30 + vertexOffset + 0x10 * i))->x = cache[i].x + cache[i].x * ratio + bumperDiff;
	}
}

void ResizeCollisionShapeX(CompoundShapeData* data, vector<hkVector4f>& cache, float ratio, float bumperHalf, float bumperDiff)
{
	ResizeCollisionPolytopeShapeX(data->shape, cache, ratio, bumperHalf, bumperDiff);
}

void SetLeanState(int state)
{
	lastLeanState = leanState;
	leanState = state;
}

#pragma endregion

#pragma region Events

class LeanLookHandler
{
public:
	typedef void (LeanLookHandler::*FnOnMouseMoveEvent)(MouseMoveEvent* evn);
	typedef void (LeanLookHandler::*FnOnThumbstickEvent)(ThumbstickEvent* evn);

	void RotateXY(float x, float y, float rad, std::vector<float>& v)
	{
		v.push_back(cos(rad) * x - sin(rad) * y);
		v.push_back(sin(rad) * x + cos(rad) * y);
	}

	void HookedMouseMoveEvent(MouseMoveEvent* evn)
	{
		if (pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson] && !leanR6Style && leanRotateInput) {
			std::vector<float> input;
			RotateXY((float)evn->mouseInputX, (float)evn->mouseInputY, -rotZ * toRad, input);
			leanCumulativeMouseX += input[0];
			leanCumulativeMouseY += input[1];
			evn->mouseInputX = (int32_t)leanCumulativeMouseX;
			evn->mouseInputY = (int32_t)leanCumulativeMouseY;
			leanCumulativeMouseX -= (int32_t)leanCumulativeMouseX;
			leanCumulativeMouseY -= (int32_t)leanCumulativeMouseY;
		}
		(this->*mouseEvn)(evn);
	}

	void HookedThumbstickEvent(ThumbstickEvent* evn)
	{
		if (evn->idCode == ThumbstickEvent::kRight && pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson] && !leanR6Style && leanRotateInput) {
			std::vector<float> input;
			RotateXY(evn->xValue, -evn->yValue, -rotZ * toRad, input);
			evn->xValue = input[0];
			evn->yValue = -input[1];
		}
		(this->*thumbstickEvn)(evn);
	}

	void HookEvents()
	{
		uintptr_t vtable = *(uintptr_t*)this;
		mouseEvn = SafeWrite64Function(vtable + 0x30, &LeanLookHandler::HookedMouseMoveEvent);
		thumbstickEvn = SafeWrite64Function(vtable + 0x20, &LeanLookHandler::HookedThumbstickEvent);
	}

protected:
	static FnOnMouseMoveEvent mouseEvn;
	static FnOnThumbstickEvent thumbstickEvn;
};
LeanLookHandler::FnOnMouseMoveEvent LeanLookHandler::mouseEvn;
LeanLookHandler::FnOnThumbstickEvent LeanLookHandler::thumbstickEvn;

void HookedActorUpdate(F4::ProcessLists* list, float dt, bool instant)
{
	float curTime = *F4::ptr_engineTime;
	if (curTime - lastSkeletonUpdate > 1.f) {
		PreparePlayerSkeleton(p->Get3D());
		lastSkeletonUpdate = curTime;
	}
	NiAVObject* node = p->Get3D();
	if (node && node != lastRoot) {
		bbx = (BSBound*)p->Get3D(false)->GetExtraData("BBX");
		lastRoot = node->IsNode();
		PreparePlayerSkeleton();
	}

	typedef void (*FnUpdate)(F4::ProcessLists*, float, bool);
	FnUpdate fn = (FnUpdate)RunActorUpdatesOrig;
	if (fn)
		(*fn)(list, dt, instant);

	Actor* a = p;
	NiAVObject* tpNode = a->Get3D(false);
	if (node && tpNode && (a->moreFlags & 0x20) == 0) {
		NiNode* head = (NiNode*)tpNode->GetObjectByName("HEAD");
		NiNode* pelvis = (NiNode*)tpNode->GetObjectByName("Pelvis");
		NiNode* root = (NiNode*)tpNode->GetObjectByName("Root");
		float deltaTime = min(curTime - lastRun, 5.0f);
		float timeMult = deltaTime * 30.0f;
		bool isFP = IsFirstPerson();
		float pcScale = GetActorScale(p);
		float heightRatio = 1;
		float transDist = 0;
		NiPoint3 pos, dir;
		NiPoint3 right = NiPoint3(-1, 0, 0);
		a->GetEyeVector(pos, dir, true);
		NiPoint3 heading = Normalize(NiPoint3(dir.x, dir.y, 0));
		if (root) {
			right = ToRightVector(root->world.rotate);
		}

		bhkCharacterController* con = nullptr;
		if (a->currentProcess) {
			con = a->currentProcess->middleHigh->charController.get();
		}

		float eulerX, eulerY, eulerZ;
		NiMatrix3 playerRot = GetRotationMatrix33(0, -a->data.angle.x, -a->data.angle.z);
		(playerRot * Transpose(lastCamRot)).ToEulerAnglesXYZ(eulerX, eulerY, eulerZ);
		lastCamRot = playerRot;

#pragma region Inertia
		float tempRotDivX = rotDivX;
		float tempRotDivY = rotDivY;
		float conditionalMultiplier = 1.0f;
		if (a->GetActorValue(*leftAttackCondition) == 0) {
			conditionalMultiplier -= 0.2f;
		}
		if (a->GetActorValue(*rightAttackCondition) == 0) {
			conditionalMultiplier -= 0.2f;
		}
		if (IsInADS(a)) {
			conditionalMultiplier *= rotADSConditionMult;
			tempRotDivX = rotDivXADS;
			tempRotDivY = rotDivYADS;
		}
		float step = rotReturnStep * conditionalMultiplier * timeMult;
		float retDiv = pow(pow(max(rotReturnDiv * conditionalMultiplier, rotReturnDivMin), 30.f), deltaTime);
		float followDiv = pow(pow(3.0f, 30.f), deltaTime);
		targetRotZ -= eulerZ / toRad / tempRotDivY * 5.f;
		targetRotX -= eulerX / toRad / tempRotDivX * 5.f;
		rotX += (targetRotX - rotX) / followDiv;
		rotX = max(min(rotX, rotLimitX), -rotLimitX);
		rotY += (targetRotZ - rotY) / followDiv;
		rotY = max(min(rotY, rotLimitY), -rotLimitY);
		targetRotZ /= retDiv;
		targetRotX /= retDiv;
		//_MESSAGE("retDiv %f rotX %f rotY %f", retDiv, rotX, rotY);
		if (abs(rotX) * (abs(rotX) - step) <= 0) {
			rotX = 0;
		} else {
			if (rotX > 0) {
				rotX -= step;
			} else {
				rotX += step;
			}
		}
		if (abs(rotY) * (abs(rotY) - step) <= 0) {
			rotY = 0;
		} else {
			if (rotY > 0) {
				rotY -= step;
			} else {
				rotY += step;
			}
		}
#pragma endregion

#pragma region Realism

		if (realism && (!realismDisableInPA || !IsInPowerArmor(a))) {
			float tempRealismRatio = realismRatio;
			float tempRealismYawRatio = realismYawRatio;
			float tempRealismReturnStep = realismReturnStep;
			if (IsInADS(a)) {
				tempRealismRatio *= realismRatioADSMult;
				tempRealismYawRatio *= realismRatioADSMult;
				tempRealismReturnStep *= realismReturnStepADSMult;
			}

			float tempRealismWeight = realismWeight;
			realismWeight = max(min(realismWeight - eulerZ * tempRealismRatio * REALISM_RATIO_CONSTANT, 1.f), -1.f);
			realismWeightNoElastic = max(min(realismWeightNoElastic - eulerZ * tempRealismYawRatio * REALISM_RATIO_CONSTANT, 1.f), -1.f);
			realismReturnStepWeight = min(realismReturnStepWeight + deltaTime / REALISM_RETURNWEIGHT_TIME, 1.f);

			realismRotZ = easeInOutCubic(abs(realismWeight), 0.f, 1.f, 1.f) * Sign(realismWeight) * realismRotLimit + realismDefaultRot;
			realismRotX = easeInOutQuad(abs(realismWeightNoElastic)) * Sign(realismWeightNoElastic) * realismYawRotLimit;

			if (abs(eulerZ) > 0.01f && tempRealismWeight * realismWeight > 0) {
				realismReturnStepWeight = REALISM_RETURNWEIGHT_START;
				if (abs(realismWeight) >= abs(realismWeightSpring)) {
					realismWeightSpring = realismWeight * realismElasticity;
				}
			}

			float realismStep = tempRealismReturnStep * realismReturnStepWeight * min(timeMult, 2.f);
			if (realismWeightSpring != 0.f) {
				realismWeight = realismWeight - realismStep * Sign(realismWeightSpring);
				if (realismWeight * realismWeightSpring < 0) {
					realismWeight = max(min(realismWeight, abs(realismWeightSpring)), -abs(realismWeightSpring));
				}
			} else {
				if (abs(realismWeight) >= realismStep) {
					realismWeight -= realismStep * Sign(realismWeight);
				} else {
					realismWeight = 0.f;
				}
			}
			realismWeightNoElastic = realismWeightNoElastic - min(realismStep, abs(realismWeightNoElastic)) * Sign(realismWeightNoElastic);

			if ((realismWeight * realismWeightSpring < 0 && abs(realismWeight) >= abs(realismWeightSpring))) {
				realismReturnStepWeight = 0.f;
				realismWeightSpring *= -realismElasticity;
			}

			//_MESSAGE("realismWeight %f realismWeightSpring %f realismRotX %f realismRotZ %f", realismWeight, realismWeightSpring, realismRotX, realismRotZ);

		} else {
			realismWeight = 0.f;
			realismRotZ = 0.f;
		}

#pragma endregion

		if ((IsInADS(a) && rotDisableInADS)) {
			rotX = 0;
			rotY = 0;
			targetRotZ = 0;
			targetRotX = 0;
		} else if (inPipboyMenu || a->interactingState != INTERACTING_STATE::kNotInteracting) {
			rotX = 0;
			rotY = 0;
			targetRotZ = 0;
			targetRotX = 0;
			realismWeight = 0.f;
			realismRotZ = 0.f;
		}

		if (leanADSOnly && !IsInADS(a)) {
			SetLeanState(0);
		}

		if (leanState != 0) {
			if (inWorkShopMenu || inDialogueMenu || inLooksMenu || a->interactingState != INTERACTING_STATE::kNotInteracting || (a->moveMode & 0x100) == 0x100 || (*(uint32_t*)((uintptr_t)a + 0x130) & 0x1E00000) == 0xE00000) {
				SetLeanState(0);
			}
		}
		if (!isFP && head) {
			pos = head->world.translate;
		}
		if (leanState != 0) {
			GetPickDataCELL(pos, pos - right * 30.f * (float)leanState, a, *pick);
			if (!pick->HasHit()) {
				if (leanState == 1) {
					leanTime = min(leanTime + deltaTime, leanTimeCost);
				} else if (leanState == -1) {
					leanTime = max(leanTime - deltaTime, -leanTimeCost);
				}
			} else {
				if (collisionDevMode) {
					_MESSAGE("Camera colliding! No Lean");
				}
				pick->Reset();
				GetPickDataCELL(pos, pos - right * 22.5f * (float)leanState, a, *pick);
				if (pick->HasHit()) {
					if (leanState == 1) {
						leanTime = max(leanTime - deltaTime, 0);
					} else if (leanState == -1) {
						leanTime = min(leanTime + deltaTime, 0);
					}
					if (collisionDevMode) {
						_MESSAGE("Camera hard collide! Reverting Lean");
					}
				}
			}
		} else {
			if (abs(leanTime) <= deltaTime) {
				leanTime = 0;
			} else {
				leanTime = max(min(leanTime - Sign(leanTime) * deltaTime, leanTimeCost), -leanTimeCost);
			}
			leanCumulativeMouseX = 0.f;
			leanCumulativeMouseY = 0.f;
		}
		float deltaLeanWeight = leanTime / leanTimeCost - leanWeight;
		leanWeight += deltaLeanWeight;

		rotZ = isFP ? leanMax * leanWeight : leanMax3rd * leanWeight;
		transZ = isFP ? leanMax * leanWeight / 3.0f : leanMax3rd * leanWeight / 3.0f;
		float rotXRadByThree = rotX * toRad / 3.0f;
		float rotYRadByTwo = rotY * toRad / 2.0f;
		float rotZRadByThree = rotZ * toRad / 3.0f;
		float transZRad = transZ * toRad;
		float transZByTwo = transZ / 2.0f;
		float transX = 0.f;

		if (con) {
			if (head && pelvis) {
				float height = max(max(head->world.translate.z, pelvis->world.translate.z) + heightBuffer - a->data.location.z, minHeight);
				//_MESSAGE("Current height %f", height);
				if (bbx) {
					float optimalHeight = bbx->extents.z * 2.0f;
					heightRatio = height / optimalHeight;
					transDist = bbx->extents.z * sin(leanMax3rd * toRad);
					transDist *= pcScale;
					transX = isFP ? transDist * leanWeight * heightRatio : transDist * leanWeight;
					hknpDynamicCompoundShape* colShape = (hknpDynamicCompoundShape*)con->shapes[1]._ptr;
					//_MESSAGE("Optimal height %f", optimalHeight);
					if (deltaLeanWeight != 0) {
						if (collisionDevMode) {
							_MESSAGE("transDist %f", transDist);
						}
						if (a->interactingState == INTERACTING_STATE::kNotInteracting && colShape) {
							float ratio = 1.f + abs(transX) / (bbx->extents.x * pcScale) / 2.f;
							MultiCompoundShape* multiShape = *(MultiCompoundShape**)((uintptr_t)colShape + 0x60);
							if (multiShape) {
								int32_t numShape = *(int32_t*)((uintptr_t)colShape + 0x68);
								multiShape->data[0].translate.x = (ratio - 1.f) * Sign(transX) / 2.f;
								multiShape->data[0].scale.x = ratio;
							}
						}
						hkTransform charProxyTransform;
						con->GetTransformImpl(charProxyTransform);
						float deltaDist = transDist * deltaLeanWeight / HAVOKTOFO4;
						pick->Reset();
						GetPickDataCELL(a->data.location + NiPoint3(0, 0, 25.f), a->data.location + NiPoint3(0, 0, 25.f) - right * transDist * Sign(deltaLeanWeight), a, *pick);
						hkVector4f displacement = right * -deltaDist;
						if (!pick->HasHit()) {
							charProxyTransform.m_translation.v.x += displacement.x;
							charProxyTransform.m_translation.v.y += displacement.y;
							charProxyTransform.m_translation.v.z += displacement.z;
							con->SetTransformImpl(charProxyTransform);
							if (collisionDevMode) {
								_MESSAGE("Translating charProxy");
							}
						}
					}

					if (dynamicHeight && !isLoading) {
						if (abs(height - lastHeight) > heightDiffThreshold) {
							if (colShape) {
								CompoundShapeData* data = *(CompoundShapeData**)((uintptr_t)colShape + 0x60);
								if (data) {
									data->translate.z = heightRatio - 0.98f;
									data->scale.z = heightRatio;
								}
							}
							//_MESSAGE("Changed height");
							lastHeight = height;
						}
					}

					if (colShape) {
						(*updateAabb)(colShape);
					}
				}
			}

			NiPoint3 colTransX = NiPoint3(transX, 0, 0);
			NiNode* comInserted = (NiNode*)tpNode->GetObjectByName("COMInserted");
			if (comInserted) {
				comInserted->local.translate = colTransX;
			}

			NiNode* cameraInserted = (NiNode*)tpNode->GetObjectByName("CameraInserted");
			if (cameraInserted) {
				cameraInserted->local.translate = colTransX;
			}

			NiNode* chestInserted = (NiNode*)tpNode->GetObjectByName("ChestInserted");
			if (chestInserted) {
				NiMatrix3 rot = chestInserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(chestInserted->parent->world.rotate);
				chestInserted->local.rotate = rot;
			}

			NiNode* spine2Inserted = (NiNode*)tpNode->GetObjectByName("Spine2Inserted");
			if (spine2Inserted) {
				spine2Inserted->local.rotate = spine2Inserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(spine2Inserted->parent->world.rotate);
				//_MESSAGE("spine1Inserted");
			}

			NiNode* spine1Inserted = (NiNode*)tpNode->GetObjectByName("Spine1Inserted");
			if (spine1Inserted) {
				spine1Inserted->local.rotate = spine1Inserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(spine1Inserted->parent->world.rotate);
				spine1Inserted->local.translate.z = transZ;
				//_MESSAGE("spine1Inserted");
			}

			NiNode* pelvisInserted = (NiNode*)tpNode->GetObjectByName("PelvisInserted");
			if (pelvisInserted) {
				NiMatrix3 rot = GetRotationMatrix33(rotZRadByThree, 0, 0);
				pelvisInserted->local.rotate = rot;
				pelvisInserted->local.translate.z = transZ;
				//_MESSAGE("pelvisInserted");
			}

			NiNode* llegCalfInserted = (NiNode*)tpNode->GetObjectByName("LLeg_CalfInserted");
			if (llegCalfInserted) {
				NiMatrix3 rot = GetRotationMatrix33(-rotZRadByThree, 0, 0);
				llegCalfInserted->local.rotate = rot;
				llegCalfInserted->local.translate.z = transZByTwo;
				//_MESSAGE("llegFootInserted");
			}

			NiNode* rlegCalfInserted = (NiNode*)tpNode->GetObjectByName("RLeg_CalfInserted");
			if (rlegCalfInserted) {
				NiMatrix3 rot = GetRotationMatrix33(-rotZRadByThree, 0, 0);
				rlegCalfInserted->local.rotate = rot;
				rlegCalfInserted->local.translate.z = transZByTwo;
				//_MESSAGE("rlegFootInserted");
			}

			if (isFP) {
				NiNode* cameraInserted1st = (NiNode*)node->GetObjectByName("CameraInserted1st");
				NiNode* camera = (NiNode*)node->GetObjectByName("Camera");
				NiPoint3 targetCamPos = NiPoint3();
				if (camera && cameraInserted1st) {
					NiPoint3 camLocal = camera->local.translate;
					NiMatrix3 rot = GetRotationMatrix33(rotZ * toRad, 0, 0);
					targetCamPos = NiPoint3(camLocal.x, camLocal.y, camLocal.z / 2.f) + Transpose(rot) * NiPoint3(0, 0, camLocal.z / 2.f);
					if (!leanR6Style) {
						cameraInserted1st->local.translate = targetCamPos - Transpose(rot) * camLocal + colTransX;
						cameraInserted1st->local.rotate = rot;
						if (realism) {
							rot = rot * GetRotationMatrix33(realismRotZ * toRad, 0, 0) * GetRotationMatrix33(0, 0, -realismRotX * toRad);
							cameraInserted1st->local.translate = targetCamPos - Transpose(rot) * camLocal + colTransX;
							cameraInserted1st->local.rotate = rot;
						}
					} else {
						NiPoint3 zoomData = NiPoint3();
						if (p->currentProcess && p->currentProcess->middleHigh) {
							p->currentProcess->middleHigh->equippedItemsLock.lock();
							BSTArray<EquippedItem>& equipped = p->currentProcess->middleHigh->equippedItems;
							if (equipped.size() != 0 && equipped[0].item.instanceData) {
								TESObjectWEAP::InstanceData* instance = (TESObjectWEAP::InstanceData*)equipped[0].item.instanceData.get();
								if (instance->type == WEAPON_TYPE::kGun && instance->zoomData) {
									zoomData = instance->zoomData->zoomData.cameraOffset;
								}
							}
							p->currentProcess->middleHigh->equippedItemsLock.unlock();
						}
						NiPoint3 camPos = camera->local.translate + zoomData;
						cameraInserted1st->local.translate = targetCamPos - camLocal + colTransX;
						cameraInserted1st->local.rotate.MakeIdentity();
						if (realism) {
							rot = GetRotationMatrix33(realismRotZ * toRad, 0, 0) * GetRotationMatrix33(0, 0, -realismRotX * toRad);
							cameraInserted1st->local.translate = targetCamPos - Transpose(rot) * camLocal + colTransX;
							cameraInserted1st->local.rotate = rot;
						}
					}
				}

				NiNode* chestInserted1st = (NiNode*)node->GetObjectByName("ChestInserted1st");
				if (camera && chestInserted1st) {
					NiMatrix3 rot = chestInserted1st->parent->world.rotate * GetRotationMatrix33(ToRightVector(camera->world.rotate), rotX * toRad) * GetRotationMatrix33(ToUpVector(camera->world.rotate), rotY * toRad) * Transpose(chestInserted1st->parent->world.rotate);  //GetRotationMatrix33(0, rotY * toRad, -rotX * toRad);
					chestInserted1st->local.rotate = rot;
				}

				NiNode* comInserted1st = (NiNode*)node->GetObjectByName("COMInserted1st");
				if (comInserted1st) {
					NiMatrix3 rot = GetRotationMatrix33(rotZ * toRad, 0, 0);
					comInserted1st->local.translate = targetCamPos - Transpose(rot) * camera->local.translate + colTransX;
					comInserted1st->local.rotate = rot;
				}
			}
		}
	} else {
		SetLeanState(0);
	}
	lastRun = curTime;
}

class InputEventReceiverOverride : public BSInputEventReceiver
{
protected:
	enum
	{
		// first 256 for keyboard, then 8 mouse buttons, then mouse wheel up, wheel down, then 16 gamepad buttons
		kMacro_KeyboardOffset = 0,  // not actually used, just for self-documentation
		kMacro_NumKeyboardKeys = 256,

		kMacro_MouseButtonOffset = kMacro_NumKeyboardKeys,  // 256
		kMacro_NumMouseButtons = 8,

		kMacro_MouseWheelOffset = kMacro_MouseButtonOffset + kMacro_NumMouseButtons,  // 264
		kMacro_MouseWheelDirections = 2,

		kMacro_GamepadOffset = kMacro_MouseWheelOffset + kMacro_MouseWheelDirections,  // 266
		kMacro_NumGamepadButtons = 16,

		kMaxMacros = kMacro_GamepadOffset + kMacro_NumGamepadButtons  // 282
	};

	enum
	{
		kGamepadButtonOffset_DPAD_UP = kMacro_GamepadOffset,  // 266
		kGamepadButtonOffset_DPAD_DOWN,
		kGamepadButtonOffset_DPAD_LEFT,
		kGamepadButtonOffset_DPAD_RIGHT,
		kGamepadButtonOffset_START,
		kGamepadButtonOffset_BACK,
		kGamepadButtonOffset_LEFT_THUMB,
		kGamepadButtonOffset_RIGHT_THUMB,
		kGamepadButtonOffset_LEFT_SHOULDER,
		kGamepadButtonOffset_RIGHT_SHOULDER,
		kGamepadButtonOffset_A,
		kGamepadButtonOffset_B,
		kGamepadButtonOffset_X,
		kGamepadButtonOffset_Y,
		kGamepadButtonOffset_LT,
		kGamepadButtonOffset_RT  // 281
	};

	uint32_t GamepadMaskToKeycode(uint32_t keyMask)
	{
		switch (keyMask) {
		case XINPUT_GAMEPAD_DPAD_UP:
			return kGamepadButtonOffset_DPAD_UP;
		case XINPUT_GAMEPAD_DPAD_DOWN:
			return kGamepadButtonOffset_DPAD_DOWN;
		case XINPUT_GAMEPAD_DPAD_LEFT:
			return kGamepadButtonOffset_DPAD_LEFT;
		case XINPUT_GAMEPAD_DPAD_RIGHT:
			return kGamepadButtonOffset_DPAD_RIGHT;
		case XINPUT_GAMEPAD_START:
			return kGamepadButtonOffset_START;
		case XINPUT_GAMEPAD_BACK:
			return kGamepadButtonOffset_BACK;
		case XINPUT_GAMEPAD_LEFT_THUMB:
			return kGamepadButtonOffset_LEFT_THUMB;
		case XINPUT_GAMEPAD_RIGHT_THUMB:
			return kGamepadButtonOffset_RIGHT_THUMB;
		case XINPUT_GAMEPAD_LEFT_SHOULDER:
			return kGamepadButtonOffset_LEFT_SHOULDER;
		case XINPUT_GAMEPAD_RIGHT_SHOULDER:
			return kGamepadButtonOffset_RIGHT_SHOULDER;
		case XINPUT_GAMEPAD_A:
			return kGamepadButtonOffset_A;
		case XINPUT_GAMEPAD_B:
			return kGamepadButtonOffset_B;
		case XINPUT_GAMEPAD_X:
			return kGamepadButtonOffset_X;
		case XINPUT_GAMEPAD_Y:
			return kGamepadButtonOffset_Y;
		case 0x9:
			return kGamepadButtonOffset_LT;
		case 0xA:
			return kGamepadButtonOffset_RT;
		default:
			return kMaxMacros;  // Invalid
		}
	}

public:
	typedef void (InputEventReceiverOverride::*FnPerformInputProcessing)(InputEvent* a_queueHead);

	void ProcessButtonEvent(const ButtonEvent* evn)
	{
		uint32_t id = evn->idCode;
		if (evn->device == INPUT_DEVICE::kMouse)
			id += kMacro_MouseButtonOffset;
		else if (evn->device == INPUT_DEVICE::kGamepad)
			id = GamepadMaskToKeycode(id);

		if (buttonDevMode) {
			_MESSAGE("Button event fired id %d held %f", id, evn->heldDownSecs);
		}

		if (!leanADSOnly || (leanADSOnly && IsInADS(p))) {
			if (toggleLean) {
				if (evn->value && evn->heldDownSecs == 0) {
					if ((leanState == 1 && id == leanLeft) || (leanState == -1 && id == leanRight)) {
						SetLeanState(0);
					} else {
						if (id == leanLeft) {
							SetLeanState(1);
						} else if (id == leanRight) {
							SetLeanState(-1);
						}
					}
				}
			} else {
				if (evn->value && evn->heldDownSecs == 0) {
					if (id == leanRight) {
						SetLeanState(-1);
					} else if (id == leanLeft) {
						SetLeanState(1);
					}
				} else if (!evn->value) {
					if ((id == leanLeft && leanState == 1) || (id == leanRight && leanState == -1)) {
						SetLeanState(0);
					}
				}
			}
		}
	}

	void HookedPerformInputProcessing(InputEvent* a_queueHead)
	{
		if (!UI::GetSingleton()->menuMode && !disableLean) {
			InputEvent* evn = a_queueHead;
			while (evn) {
				if (evn->eventType == INPUT_EVENT_TYPE::kButton) {
					ProcessButtonEvent((ButtonEvent*)evn);
				}
				evn = evn->next;
			}
		}
		FnPerformInputProcessing fn = fnHash.at(*(uint64_t*)this);
		if (fn) {
			(this->*fn)(a_queueHead);
		}
	}

	void HookSink()
	{
		uint64_t vtable = *(uint64_t*)this;
		auto it = fnHash.find(vtable);
		if (it == fnHash.end()) {
			FnPerformInputProcessing fn = SafeWrite64Function(vtable, &InputEventReceiverOverride::HookedPerformInputProcessing);
			fnHash.insert(std::pair<uint64_t, FnPerformInputProcessing>(vtable, fn));
		}
	}

	void UnHookSink()
	{
		uint64_t vtable = *(uint64_t*)this;
		auto it = fnHash.find(vtable);
		if (it == fnHash.end())
			return;
		SafeWrite64Function(vtable, it->second);
		fnHash.erase(it);
	}

protected:
	static std::unordered_map<uint64_t, FnPerformInputProcessing> fnHash;
};
std::unordered_map<uint64_t, InputEventReceiverOverride::FnPerformInputProcessing> InputEventReceiverOverride::fnHash;

class ObjectLoadWatcher : public BSTEventSink<TESObjectLoadedEvent>
{
public:
	virtual BSEventNotifyControl ProcessEvent(const TESObjectLoadedEvent& evn, BSTEventSource<TESObjectLoadedEvent>* a_source)
	{
		if (!evn.loaded) {
			if (evn.formId == 0x14) {
				_MESSAGE("Player unloaded");
			}
			return BSEventNotifyControl::kContinue;
		} else {
			if (evn.formId == 0x14) {
				_MESSAGE("Player loaded");
				playerLastLoaded = *F4::ptr_engineTime;
				lastSkeletonUpdate = *F4::ptr_engineTime;
				PreparePlayerSkeleton();
			}
		}
		return BSEventNotifyControl::kContinue;
	}
	F4_HEAP_REDEFINE_NEW(ObjectLoadWatcher);
};

class MenuWatcher : public BSTEventSink<MenuOpenCloseEvent>
{
	virtual BSEventNotifyControl ProcessEvent(const MenuOpenCloseEvent& evn, BSTEventSource<MenuOpenCloseEvent>* src) override
	{
		if (evn.menuName == "LoadingMenu") {
			if (evn.opening) {
				isLoading = true;
				leanState = 0;
			} else {
				isLoading = false;
			}
		} else if (evn.menuName == "WorkshopMenu") {
			if (evn.opening) {
				inWorkShopMenu = true;
			} else {
				inWorkShopMenu = false;
			}
		} else if (evn.menuName == "DialogueMenu") {
			if (evn.opening) {
				inDialogueMenu = true;
			} else {
				inDialogueMenu = false;
			}
		} else if (evn.menuName == "LooksMenu") {
			if (evn.opening) {
				inLooksMenu = true;
			} else {
				inLooksMenu = false;
			}
		} else if (evn.menuName == "PipboyMenu") {
			if (evn.opening) {
				inPipboyMenu = true;
			} else {
				inPipboyMenu = false;
			}
		} else if (!evn.opening && evn.menuName == "PauseMenu") {
			LoadConfigs();
		}
		return BSEventNotifyControl::kContinue;
	}
};

#pragma endregion

#pragma region Initializers

void InitializePlugin()
{
	p = PlayerCharacter::GetSingleton();
	pcam = PlayerCamera::GetSingleton();
	((InputEventReceiverOverride*)((uint64_t)pcam + 0x38))->HookSink();
	_MESSAGE("PlayerCharacter %llx PlayerCamera %llx", p, pcam);
	pc = PlayerControls::GetSingleton();
	((LeanLookHandler*)pc->lookHandler)->HookEvents();
	leftAttackCondition = (ActorValueInfo*)TESForm::GetFormByID(0x00036E);
	rightAttackCondition = (ActorValueInfo*)TESForm::GetFormByID(0x00036F);
	colCheckProj = (BGSProjectile*)TESForm::GetFormByID(0xE942D);
	MQ102 = (TESQuest*)TESForm::GetFormByID(0x1CC2A);
	_MESSAGE("Hooked to event");
	ObjectLoadWatcher* olw = new ObjectLoadWatcher();
	ObjectLoadedEventSource::GetSingleton()->RegisterSink(olw);
	MenuWatcher* mw = new MenuWatcher();
	UI::GetSingleton()->GetEventSource<MenuOpenCloseEvent>()->RegisterSink(mw);
	pick = new bhkPickData();
}

#pragma endregion

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Query(const F4SE::QueryInterface* a_f4se, F4SE::PluginInfo* a_info)
{
#ifndef NDEBUG
	auto sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
#else
	auto path = logger::log_directory();
	if (!path) {
		return false;
	}

	*path /= fmt::format(FMT_STRING("{}.log"), Version::PROJECT);
	auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
#endif

	auto log = std::make_shared<spdlog::logger>("global log"s, std::move(sink));

#ifndef NDEBUG
	log->set_level(spdlog::level::trace);
#else
	log->set_level(spdlog::level::info);
	log->flush_on(spdlog::level::warn);
#endif

	spdlog::set_default_logger(std::move(log));
	spdlog::set_pattern("%g(%#): [%^%l%$] %v"s);

	logger::info(FMT_STRING("{} v{}"), Version::PROJECT, Version::NAME);

	a_info->infoVersion = F4SE::PluginInfo::kVersion;
	a_info->name = Version::PROJECT.data();
	a_info->version = Version::MAJOR;

	if (a_f4se->IsEditor()) {
		logger::critical(FMT_STRING("loaded in editor"));
		return false;
	}

	const auto ver = a_f4se->RuntimeVersion();
	if (ver < F4SE::RUNTIME_1_10_162) {
		logger::critical(FMT_STRING("unsupported runtime v{}"), ver.string());
		return false;
	}

	F4SE::AllocTrampoline(8 * 8);

	return true;
}

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Load(const F4SE::LoadInterface* a_f4se)
{
	F4SE::Init(a_f4se);

	F4SE::Trampoline& trampoline = F4SE::GetTrampoline();
	RunActorUpdatesOrig = trampoline.write_call<5>(ptr_RunActorUpdates.address(), &HookedActorUpdate);
	//PCUpdateMainThreadOrig = trampoline.write_call<5>(ptr_PCUpdateMainThread.address(), &HookedPCUpdate);
	//UpdateSceneGraphOrig = trampoline.write_call<5>(ptr_UpdateSceneGraph.address(), &HookedUpdateSceneGraph);

	taskInterface = F4SE::GetTaskInterface();

	const F4SE::MessagingInterface* message = F4SE::GetMessagingInterface();
	message->RegisterListener([](F4SE::MessagingInterface::Message* msg) -> void {
		if (msg->type == F4SE::MessagingInterface::kGameDataReady) {
			InitializePlugin();
			LoadConfigs();
		} else if (msg->type == F4SE::MessagingInterface::kGameLoaded) {
			BSScaleformTranslator* translator = (BSScaleformTranslator*)BSScaleformManager::GetSingleton()->loader->GetStateAddRef(Scaleform::GFx::State::StateType::kTranslator);
			if (translator) {
				Translation::ParseTranslation(translator, "UneducatedShooter");
				_MESSAGE("Translation injected");
			}
		} else if (msg->type == F4SE::MessagingInterface::kPostLoadGame) {
			LoadConfigs();
		} else if (msg->type == F4SE::MessagingInterface::kNewGame) {
			LoadConfigs();
		}
	});

	return true;
}
